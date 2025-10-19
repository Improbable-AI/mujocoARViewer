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
# Import generated gRPC classes
from .generated import mujoco_ar_pb2, mujoco_ar_pb2_grpc

class MJARViewer: 

    def __init__(self, avp_ip, grpc_port = 50051 ): 
        self.avp_ip = avp_ip
        self.grpc_port = grpc_port
        self._setup_grpc_client()
        
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


    def load_scene(self, model_path):
        """
        model_path: str, either XML or USDZ file path
        """

        # if XML, convert to USDZ first
        if model_path.endswith('.xml'):
            usdz_path = self._convert_to_usdz(model_path)
            
        else: 
            usdz_path = model_path

        self._send_usdz_data(usdz_path)


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

    def _send_usdz_data_chunked(self, usdz_data, usdz_filename):
        """Send USDZ file data in chunks via gRPC streaming"""
        try:
            chunk_size = 1024 * 1024  # 1MB chunks
            total_size = len(usdz_data)
            total_chunks = (total_size + chunk_size - 1) // chunk_size
            
            print(f"📦 Sending {total_size} bytes in {total_chunks} chunks of {chunk_size} bytes each")
            
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

    def _send_usdz_data(self, usdz_path):
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
            
            # Use chunked transfer for files larger than 5MB
            if file_size_mb > 5.0:
                print("📦 File is large, using chunked transfer...")
                success = self._send_usdz_data_chunked(usdz_data, usdz_filename)
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
            xpos = self.data.body(body_id).xpos.tolist()
            xquat = self.data.body(body_id).xquat.tolist()
            body_dict[body_name] = {
                "xpos": xpos, 
                "xquat": xquat
            }
        return body_dict
    
    
    def sync(self): 

        try:
            poses = self.get_poses()
            body_poses = []
            
            for body_name, pose_data in poses.items():
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
            
            # Create the update request
            request = mujoco_ar_pb2.PoseUpdateRequest(
                body_poses=body_poses,
                session_id=self.session_id,
                timestamp=time.time()
            )
            
            # Send the update
            if self.grpc_stub:
                response = self.grpc_stub.UpdatePoses(request)
                if response.success:
                    pass 
                else:
                    print(f"❌ Pose update failed: {response.message}")
            else:
                print("❌ gRPC connection not available")
                
        except Exception as e:
            print(f"❌ Error in update: {e}")


if __name__ == "__main__":
    # Example usage
    usdz_path = "scenes/franka_emika_panda/scene.usdz"  # Replace with your MuJoCo XML file path
    xml_path = "scenes/franka_emika_panda/scene.xml"
    avp_ip = "10.29.194.74"

    arviewer = MJARViewer(avp_ip = avp_ip) 
    arviewer.send_model(usdz_path)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    arviewer.register(model, data)

    import mujoco.viewer 
    viewer = mujoco.viewer.launch_passive(model, data)

    while True: 
        mujoco.mj_step(model, data)
        arviewer.sync()
        viewer.sync() 
