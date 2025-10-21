import mujoco 
import grpc
import time
import threading
import http.server
import socketserver
import os
import gzip
import tempfile
from pathlib import Path
import numpy as np
from copy import deepcopy
# Import generated gRPC classes
from .generated import mujoco_ar_pb2, mujoco_ar_pb2_grpc

# Constants and helper functions for hand tracking data processing
YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype=np.float64)

def process_matrix(message):
    """Convert protobuf matrix to numpy array with proper shape"""
    m = np.array([[[message.m00, message.m01, message.m02, message.m03],
                    [message.m10, message.m11, message.m12, message.m13],
                    [message.m20, message.m21, message.m22, message.m23],
                    [0, 0, 0, 1]]])
    return m 

def process_matrices(skeleton, matrix=np.eye(4)):
    """Process multiple joint matrices from skeleton"""
    return np.concatenate([matrix @ process_matrix(joint) for joint in skeleton], axis=0)

def rotate_head(R, degrees=-90):
    """Rotate head matrix around x-axis"""
    # Convert degrees to radians
    theta = np.radians(degrees)
    # Create the rotation matrix for rotating around the x-axis
    R_x = np.array([[
        [1, 0, 0, 0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta), np.cos(theta), 0],
        [0, 0, 0, 1]
    ]])
    R_rotated = R @ R_x 
    return R_rotated

def get_pinch_distance(finger_messages): 
    """Calculate distance between thumb and index finger"""
    fingers = process_matrices(finger_messages)
    thumb = fingers[4, :3, 3]
    index = fingers[9, :3, 3]
    return np.linalg.norm(thumb - index)

def get_wrist_roll(mat):
    """Calculate wrist roll angle in radians"""
    R = mat[0, :3, :3]

    # Calculate angles for rotation around z and y axis to align the first column with [1, 0, 0]
    # Angle to rotate around z-axis to align the projection on the XY plane
    theta_z = np.arctan2(R[1, 0], R[0, 0])  # arctan2(y, x)

    # Rotate R around the z-axis by -theta_z to align its x-axis on the XY plane
    Rz = np.array([
        [np.cos(-theta_z), -np.sin(-theta_z), 0],
        [np.sin(-theta_z), np.cos(-theta_z), 0],
        [0, 0, 1]
    ])
    R_after_z = Rz @ R

    # Angle to rotate around y-axis to align the x-axis with the global x-axis
    theta_y = np.arctan2(R_after_z[0, 2], R_after_z[0, 0])  # arctan2(z, x)

    Ry = np.array([
        [np.cos(-theta_y), 0, np.sin(-theta_y)],
        [0, 1, 0],
        [-np.sin(-theta_y), 0, np.cos(-theta_y)]
    ])
    R_after_y = Ry @ R_after_z

    # Angle to rotate around x-axis to align the y-axis and z-axis properly with the global y-axis and z-axis
    theta_x = np.arctan2(R_after_y[1, 2], R_after_y[1, 1])  # arctan2(z, y) of the second row

    return theta_x

class MJARViewer: 

    def __init__(self, avp_ip, grpc_port = 50051, enable_hand_tracking=False): 
        self.avp_ip = avp_ip
        self.grpc_port = grpc_port
        self.enable_hand_tracking = enable_hand_tracking
        
        # Hand tracking state
        self.hand_tracking_data = None
        self.hand_tracking_thread = None
        self.hand_tracking_running = False
        
        # Pose streaming state
        self.pose_streaming_enabled = False
        self.pose_stream_thread = None
        self.current_poses = {}
        self.pose_stream_running = False
        self.pose_stream_lock = threading.Lock()
        
        self._setup_grpc_client()
        
        # Auto-start hand tracking if enabled
        if self.enable_hand_tracking:
            self.start_hand_tracking()
            
        # Auto-start pose streaming
        self.start_pose_streaming()
        
    def _setup_grpc_client(self):
        """Setup gRPC client connection"""
        try:
            target = f"{self.avp_ip}:{self.grpc_port}"
            
            # Configure gRPC options for large message handling
            options = [
                ('grpc.max_send_message_length', 100 * 1024 * 1024),  # 100MB
                ('grpc.max_receive_message_length', 100 * 1024 * 1024),  # 100MB
                ('grpc.keepalive_time_ms', 30000),  # 30 seconds
                ('grpc.keepalive_timeout_ms', 5000),  # 5 seconds
                ('grpc.keepalive_permit_without_calls', True),
                ('grpc.http2.max_pings_without_data', 0),
                ('grpc.http2.min_time_between_pings_ms', 10000),
                ('grpc.http2.min_ping_interval_without_data_ms', 300000)
            ]
            
            self.grpc_channel = grpc.insecure_channel(target, options=options)
            self.grpc_stub = mujoco_ar_pb2_grpc.MuJoCoARServiceStub(self.grpc_channel)
            print(f"🔗 gRPC client connected to {target}")
            self.session_id = f"mjarview_{int(time.time())}"

        except Exception as e:
            print(f"❌ Failed to setup gRPC client: {e}")


    def load_scene(self, model_path, attach_to=None):
        """
        model_path: str, either XML or USDZ file path
        attach_to: array-like of length 7 with [x, y, z, qw, qx, qy, qz] in ZUP coordinates
                  where [x,y,z] is position and [qw,qx,qy,qz] is quaternion in wxyz order
        """

        # if XML, convert to USDZ first
        if model_path.endswith('.xml'):
            usdz_path = self._convert_to_usdz(model_path)
            
        else: 
            usdz_path = model_path

        # if attach_to is 4 dim 
        if len(attach_to) == 4: 
            # assume it's [x,y,z, rotation around z in degrees], convert to quaternion
            yaw_deg = attach_to[3]
            yaw_rad = np.radians(yaw_deg / 2)
            qw = np.cos(yaw_rad)
            qx = 0.0
            qy = 0.0
            qz = np.sin(yaw_rad)
            attach_to = [attach_to[0], attach_to[1], attach_to[2], qw, qx, qy, qz]

        self._send_usdz_data(usdz_path, attach_to=attach_to)

    def send_model(self, model_path, attach_to=None):
        """
        Alias for load_scene for backwards compatibility
        model_path: str, either XML or USDZ file path  
        attach_to: array-like of length 7 with [x, y, z, qw, qx, qy, qz] in ZUP coordinates
                  where [x,y,z] is position and [qw,qx,qy,qz] is quaternion in wxyz order
        """
        return self.load_scene(model_path, attach_to=attach_to)


    def _test_small_data_transfer(self):
        """Test gRPC connection with a small dummy file"""
        try:
            # Create a small test file (1KB)
            test_data = b"This is a test USDZ file content. " * 30  # ~1KB
            test_filename = "test_small.usdz"
            
            request = mujoco_ar_pb2.UsdzDataRequest(
                usdz_data=test_data,
                filename=test_filename,
                session_id=self.session_id
            )
            
            print(f"🧪 Testing with small file: {len(test_data)} bytes, filename: {test_filename}")
            
            # Test with short timeout first
            response = self.grpc_stub.SendUsdzData(request, timeout=10.0)
            
            if response.success:
                print(f"✅ Small test file sent successfully!")
                print(f"   Server saved to: {response.local_file_path}")
                return True
            else:
                print(f"❌ Failed to send small test file: {response.message}")
                return False
                
        except grpc.RpcError as e:
            print(f"❌ gRPC Error with small test file: {e}")
            print(f"   Status code: {e.code()}")
            print(f"   Details: {e.details()}")
            return False
        except Exception as e:
            print(f"❌ Error with small test file: {e}")
            return False

    def _send_usdz_data_chunked(self, usdz_data, usdz_filename, attach_to=None):
        """Send USDZ file data in chunks via gRPC streaming"""
        try:
            chunk_size = 1024 * 1024  # 1MB chunks
            total_size = len(usdz_data)
            total_chunks = (total_size + chunk_size - 1) // chunk_size
            
            print(f"📦 Sending {total_size} bytes in {total_chunks} chunks of {chunk_size} bytes each")
            
            # Parse attach_to parameter
            attach_position = None
            attach_rotation = None
            if attach_to is not None:
                # Convert to numpy array for easier handling
                attach_array = np.array(attach_to)
                if len(attach_array) != 7:
                    raise ValueError(f"attach_to must be a 7-element array [x,y,z,qw,qx,qy,qz], got {len(attach_array)} elements")
                
                # Extract position (first 3 elements)
                position = attach_array[:3]
                
                # Extract quaternion (last 4 elements) in wxyz order and convert to xyzw order for protobuf
                quat_wxyz = attach_array[3:]
                quaternion = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # Convert wxyz -> xyzw
                
                attach_position = mujoco_ar_pb2.Vector3(x=position[0], y=position[1], z=position[2])
                attach_rotation = mujoco_ar_pb2.Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                print(f"📍 Attach to position: {position}, rotation (wxyz): {quat_wxyz}")
            
            def chunk_generator():
                for i in range(total_chunks):
                    start = i * chunk_size
                    end = min(start + chunk_size, total_size)
                    chunk_data = usdz_data[start:end]
                    
                    chunk_request = mujoco_ar_pb2.UsdzChunkRequest(
                        chunk_data=chunk_data,
                        filename=usdz_filename,
                        session_id=self.session_id,
                        chunk_index=i,
                        total_chunks=total_chunks,
                        total_size=total_size,
                        is_last_chunk=(i == total_chunks - 1)
                    )
                    
                    # Add attach_to information only to the first chunk
                    if i == 0 and attach_position is not None and attach_rotation is not None:
                        chunk_request.attach_to_position.CopyFrom(attach_position)
                        chunk_request.attach_to_rotation.CopyFrom(attach_rotation)
                    
                    print(f"📤 Sending chunk {i+1}/{total_chunks} ({len(chunk_data)} bytes)")
                    yield chunk_request
            
            # Send chunks via streaming RPC
            response = self.grpc_stub.SendUsdzDataChunked(chunk_generator(), timeout=120.0)
            
            if response.success:
                print(f"✅ Chunked USDZ data sent successfully, saved to: {response.local_file_path}")
                return True
            else:
                print(f"❌ Failed to send chunked USDZ data: {response.message}")
                return False
                
        except grpc.RpcError as e:
            print(f"❌ gRPC Error sending chunked USDZ data: {e}")
            print(f"   Status code: {e.code()}")
            print(f"   Details: {e.details()}")
            return False
        except Exception as e:
            print(f"❌ Error sending chunked USDZ data: {e}")
            return False

    def _send_usdz_data(self, usdz_path, attach_to=None):
        """Send USDZ file data directly via gRPC"""
        try:
            # First test with a small file to verify connection
            print("🔍 Testing gRPC connection with small file first...")
            if not self._test_small_data_transfer():
                print("❌ Small file test failed, skipping large file transfer")
                return
            
            print("✅ Small file test passed, proceeding with actual USDZ file...")
            
            # Read the USDZ file as binary data
            with open(usdz_path, 'rb') as f:
                usdz_data = f.read()

            usdz_filename = os.path.basename(usdz_path)
            
            # Check file size and decide transfer method
            file_size_mb = len(usdz_data) / (1024 * 1024)
            print(f"📊 File size: {file_size_mb:.2f} MB")
            
            # Parse attach_to parameter
            attach_position = None
            attach_rotation = None
            if attach_to is not None:
                # Convert to numpy array for easier handling
                attach_array = np.array(attach_to)
                if len(attach_array) != 7:
                    raise ValueError(f"attach_to must be a 7-element array [x,y,z,qw,qx,qy,qz], got {len(attach_array)} elements")
                
                # Extract position (first 3 elements)
                position = attach_array[:3]
                
                # Extract quaternion (last 4 elements) in wxyz order and convert to xyzw order for protobuf
                quat_wxyz = attach_array[3:]
                quaternion = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # Convert wxyz -> xyzw
                
                attach_position = mujoco_ar_pb2.Vector3(x=position[0], y=position[1], z=position[2])
                attach_rotation = mujoco_ar_pb2.Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                print(f"📍 Attach to position: {position}, rotation (wxyz): {quat_wxyz}")
            
            # Use chunked transfer for files larger than 5MB
            if file_size_mb > 5.0:
                print("📦 File is large, using chunked transfer...")
                success = self._send_usdz_data_chunked(usdz_data, usdz_filename, attach_to=attach_to)
                if success:
                    return
                else:
                    print("❌ Chunked transfer failed, falling back to single message...")
            
            # Try single message transfer (for smaller files or as fallback)
            print(f"📤 Sending USDZ data as single message: {len(usdz_data)} bytes")
            
            request = mujoco_ar_pb2.UsdzDataRequest(
                usdz_data=usdz_data,
                filename=usdz_filename,
                session_id=self.session_id
            )
            
            # Add attach_to information if provided
            if attach_position is not None and attach_rotation is not None:
                request.attach_to_position.CopyFrom(attach_position)
                request.attach_to_rotation.CopyFrom(attach_rotation)
            
            # Use longer timeout for large files
            timeout_seconds = max(60.0, file_size_mb * 2)  # 2 seconds per MB, minimum 60s
            print(f"⏱️  Using timeout: {timeout_seconds} seconds")
            
            response = self.grpc_stub.SendUsdzData(request, timeout=timeout_seconds)
            
            if response.success:
                print(f"✅ USDZ data sent successfully, saved to: {response.local_file_path}")
            else:
                print(f"❌ Failed to send USDZ data: {response.message}")
                
        except grpc.RpcError as e:
            print(f"❌ gRPC Error sending USDZ data: {e}")
            print(f"   Status code: {e.code()}")
            print(f"   Details: {e.details()}")
            print(f"   Debug string: {e.debug_error_string()}")
            
            # Suggest fallback to HTTP method
            print("\n💡 Suggestion: Try using HTTP transfer method instead:")
            print("   ar_view = MJARView(..., use_grpc_data_transfer=False)")
            
        except Exception as e:
            print(f"❌ Error sending USDZ data: {e}")


    def _convert_to_usdz(self, xml_path):
        """Convert MuJoCo XML to USDZ file"""

        import mujoco_usd_converter, usdex.core
        from pxr import Sdf, Usd, UsdUtils

        converter = mujoco_usd_converter.Converter()
        
        # Generate USDZ file path
        usd_output_path = xml_path.replace('.xml', '_usd')
        usdz_output_path = xml_path.replace('.xml', '.usdz')
        
        # Convert to USD first
        asset = converter.convert(xml_path, usd_output_path)
        stage = Usd.Stage.Open(asset.path)
        usdex.core.saveStage(stage, comment="modified after conversion")
        
        # Create USDZ package
        UsdUtils.CreateNewUsdzPackage(asset.path, usdz_output_path)
        print(f"✅ USDZ file created: {usdz_output_path}")

        return usdz_output_path


    def register(self, model, data): 
        self.model = model 
        self.data = data 

        # bodies 
        self.bodies = {self.model.body(i).name: i for i in range(self.model.nbody)}

    def get_poses(self): 
        """
        construct a dictionary of body names and their xpos / xquat
        """
        body_dict = {}
        for body_name, body_id in self.bodies.items(): 
            xpos = deepcopy(self.data.body(body_id).xpos.tolist())
            xquat = deepcopy(self.data.body(body_id).xquat.tolist())
            # Remove slashes from body name when storing in dictionary
            clean_body_name = body_name.replace('/', '').replace('-', '') if body_name else body_name
            body_dict[clean_body_name] = {
                "xpos": xpos, 
                "xquat": xquat
            }
        return body_dict
    
    
    def sync(self): 
        """Update the current poses that will be sent via the streaming connection"""
        try:
            poses = self.get_poses()
            
            # Update the current poses that the streaming thread will send
            with self.pose_stream_lock:
                self.current_poses = poses
                
        except Exception as e:
            print(f"❌ Error in sync: {e}")

    def start_pose_streaming(self):
        """Start pose streaming in background thread"""
        if self.pose_stream_running:
            print("⚠️ Pose streaming already running")
            return
        
        print("🔄 Starting pose streaming...")
        self.pose_stream_running = True
        self.pose_stream_thread = threading.Thread(target=self._pose_streaming_loop, daemon=True)
        self.pose_stream_thread.start()
    
    def stop_pose_streaming(self):
        """Stop pose streaming"""
        if not self.pose_stream_running:
            return
        
        print("🛑 Stopping pose streaming...")
        self.pose_stream_running = False
        if self.pose_stream_thread:
            self.pose_stream_thread.join(timeout=2.0)
    
    def _pose_streaming_loop(self):
        """Background thread to continuously stream poses"""
        try:
            print(f"🔌 Connecting to pose stream at {self.avp_ip}:{self.grpc_port}")
            
            def pose_generator():
                while self.pose_stream_running:
                    # Get current poses (thread-safe)
                    with self.pose_stream_lock:
                        current_poses = self.current_poses.copy()
                    
                    if current_poses:
                        body_poses = []
                        
                        for body_name, pose_data in current_poses.items():
                            if body_name:  # Skip empty body names
                                # Create protobuf objects
                                position = mujoco_ar_pb2.Vector3(
                                    x=pose_data["xpos"][0],
                                    y=pose_data["xpos"][1], 
                                    z=pose_data["xpos"][2]
                                )
                                
                                rotation = mujoco_ar_pb2.Quaternion(
                                    x=pose_data["xquat"][1],  # Note: MuJoCo uses w,x,y,z order
                                    y=pose_data["xquat"][2],
                                    z=pose_data["xquat"][3],
                                    w=pose_data["xquat"][0]   # w comes first in MuJoCo
                                )
                                
                                body_pose = mujoco_ar_pb2.BodyPose(
                                    position=position,
                                    rotation=rotation,
                                    body_name=body_name
                                )
                                
                                body_poses.append(body_pose)
                        
                        if body_poses:
                            # Create the update request
                            request = mujoco_ar_pb2.PoseUpdateRequest(
                                body_poses=body_poses,
                                session_id=self.session_id,
                                timestamp=time.time()
                            )
                            
                            yield request
                    
                    # Stream at ~100 Hz (10ms delay) - adjust as needed
                    time.sleep(0.01)
            
            # Start streaming poses
            responses = self.grpc_stub.StreamPoses(pose_generator())
            
            print("✅ Pose streaming connected!")
            
            for response in responses:
                if not self.pose_stream_running:
                    break
                    
                if not response.success:
                    print(f"⚠️ Pose stream response: {response.message}")
                
        except grpc.RpcError as e:
            if self.pose_stream_running:  # Only print error if we didn't intentionally stop
                if e.code() == grpc.StatusCode.UNAVAILABLE:
                    print(f"❌ Pose streaming server unavailable")
                else:
                    print(f"❌ Pose streaming gRPC Error: {e}")
            self.pose_stream_running = False
        except Exception as e:
            if self.pose_stream_running:
                print(f"❌ Pose streaming error: {e}")
            self.pose_stream_running = False

    # MARK: - Hand Tracking Methods
    
    def start_hand_tracking(self):
        """Start hand tracking data stream in background thread"""
        if self.hand_tracking_running:
            print("⚠️ Hand tracking already running")
            return
        
        print("🖐️ Starting hand tracking stream...")
        self.hand_tracking_running = True
        self.hand_tracking_thread = threading.Thread(target=self._hand_tracking_stream, daemon=True)
        self.hand_tracking_thread.start()
    
    def stop_hand_tracking(self):
        """Stop hand tracking data stream"""
        if not self.hand_tracking_running:
            return
        
        print("🛑 Stopping hand tracking stream...")
        self.hand_tracking_running = False
        if self.hand_tracking_thread:
            self.hand_tracking_thread.join(timeout=2.0)
    
    def _hand_tracking_stream(self):
        """Background thread to continuously receive hand tracking data"""
        request = mujoco_ar_pb2.HandTrackingRequest()
        request.session_id = self.session_id
        
        try:
            print(f"🔌 Connecting to hand tracking stream at {self.avp_ip}:{self.grpc_port}")
            
            # Stream hand tracking updates
            responses = self.grpc_stub.StreamHandTracking(request)
            
            print("✅ Hand tracking stream connected!")
            
            for response in responses:
                if not self.hand_tracking_running:
                    break
                
                # Process and store the latest data
                self.hand_tracking_data = self._process_hand_tracking_update(response)
                
        except grpc.RpcError as e:
            if self.hand_tracking_running:  # Only print error if we didn't intentionally stop
                if e.code() == grpc.StatusCode.UNAVAILABLE:
                    print(f"❌ Hand tracking server unavailable")
                else:
                    print(f"❌ Hand tracking gRPC Error: {e}")
            self.hand_tracking_running = False
        except Exception as e:
            if self.hand_tracking_running:
                print(f"❌ Hand tracking error: {e}")
            self.hand_tracking_running = False
    
    def _process_hand_tracking_update(self, update):
        """Process a hand tracking update and convert to dictionary matching python_client.py format"""
        # Process left hand data
        left_wrist = None
        left_fingers = None
        left_pinch_distance = None
        left_wrist_roll = None
        
        if update.left_hand.HasField('wrist_matrix'):
            left_wrist = YUP2ZUP @ process_matrix(update.left_hand.wrist_matrix)
            left_wrist_roll = get_wrist_roll(left_wrist)
            
        if update.left_hand.skeleton.joint_matrices:
            left_fingers = process_matrices(update.left_hand.skeleton.joint_matrices)
            left_pinch_distance = get_pinch_distance(update.left_hand.skeleton.joint_matrices)

        # Process right hand data  
        right_wrist = None
        right_fingers = None
        right_pinch_distance = None
        right_wrist_roll = None
        
        if update.right_hand.HasField('wrist_matrix'):
            right_wrist = YUP2ZUP @ process_matrix(update.right_hand.wrist_matrix)
            right_wrist_roll = get_wrist_roll(right_wrist)
            
        if update.right_hand.skeleton.joint_matrices:
            right_fingers = process_matrices(update.right_hand.skeleton.joint_matrices)
            right_pinch_distance = get_pinch_distance(update.right_hand.skeleton.joint_matrices)

        # Process head data
        head = None
        if update.HasField('head'):
            head = rotate_head(YUP2ZUP @ process_matrix(update.head))

        # Create data dictionary matching python_client.py format
        data = {
            "left_wrist": left_wrist,
            "right_wrist": right_wrist,
            "left_fingers": left_fingers,
            "right_fingers": right_fingers,
            "head": head,
            "left_pinch_distance": left_pinch_distance,
            "right_pinch_distance": right_pinch_distance,
            "left_wrist_roll": left_wrist_roll,
            "right_wrist_roll": right_wrist_roll,
        }
        
        return data
    
    def get_hand_tracking(self):
        """Get the most recent hand tracking data in python_client.py format
        
        Returns:
            dict: Dictionary containing:
                - left_wrist: 4x4 numpy array or None (with YUP2ZUP transformation)
                - right_wrist: 4x4 numpy array or None (with YUP2ZUP transformation)
                - left_fingers: numpy array of joint matrices or None
                - right_fingers: numpy array of joint matrices or None  
                - head: 4x4 numpy array or None (with rotation and YUP2ZUP transformation)
                - left_pinch_distance: float or None (distance between thumb and index)
                - right_pinch_distance: float or None (distance between thumb and index)
                - left_wrist_roll: float or None (wrist roll angle in radians)
                - right_wrist_roll: float or None (wrist roll angle in radians)
        """
        if not self.hand_tracking_running:
            print("⚠️ Hand tracking is not running. Call start_hand_tracking() first or set enable_hand_tracking=True")
            return None
        
        return self.hand_tracking_data
    
    def close(self):
        """Clean up resources"""
        self.stop_hand_tracking()
        self.stop_pose_streaming()
        if hasattr(self, 'grpc_channel'):
            self.grpc_channel.close()


if __name__ == "__main__":
    # Example usage
    usdz_path = "scenes/franka_emika_panda/scene.usdz"  # Replace with your MuJoCo XML file path
    xml_path = "scenes/franka_emika_panda/scene.xml"
    avp_ip = "10.29.194.74"

    arviewer = MJARViewer(avp_ip = avp_ip) 
    
    # Example with attach_to offset (7-element array: [x,y,z,qw,qx,qy,qz] in ZUP coordinates)
    attach_to = [0.5, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]  # 0.5m right, 1m up, no rotation
    
    arviewer.send_model(usdz_path, attach_to=attach_to)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    arviewer.register(model, data)

    import mujoco.viewer 
    viewer = mujoco.viewer.launch_passive(model, data)

    while True: 
        mujoco.mj_step(model, data)
        arviewer.sync()
        viewer.sync()
