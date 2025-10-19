import mujoco 
import mujoco_usd_converter, usdex.core
from pxr import Sdf, Usd, UsdUtils
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

class MJARView:
    """
    Legacy class for backward compatibility.
    
    This class provides the original API for MuJoCo AR visualization.
    For new projects, consider using MJARViewer instead.
    """

    def __init__(self, xml_path, avp_ip="localhost"): 

        # Store connection parameters
        self.xml_path = xml_path
        self.avp_ip = avp_ip
        self.grpc_port = 50051
        self.http_port = 8083
        self.use_grpc_data_transfer = True

        # Load the MuJoCo model from the XML file
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # parse the number of bodies 
        self.bodies = {self.model.body(i).name: i for i in range(self.model.nbody)}
        
        # gRPC client setup
        self.grpc_channel = None
        self.grpc_stub = None
        self.session_id = f"mjarview_{int(time.time())}"
        
        # HTTP server for USDZ file serving
        self.http_server = None
        self.http_thread = None
        
        # Convert to USDZ on initialization
        self._convert_to_usdz()
        
        # Start HTTP server if using URL transfer method
        if not self.use_grpc_data_transfer:
            self._start_http_server()
        
        self._setup_grpc_client()


    def _convert_to_usdz(self):
        """Convert MuJoCo XML to USDZ file"""
        converter = mujoco_usd_converter.Converter()
        
        # Generate USDZ file path
        self.usd_output_path = self.xml_path.replace('.xml', '_usd')
        self.usdz_output_path = self.xml_path.replace('.xml', '.usdz')
        
        # Convert to USD first
        asset = converter.convert(self.xml_path, self.usd_output_path)
        stage = Usd.Stage.Open(asset.path)
        usdex.core.saveStage(stage, comment="modified after conversion")
        
        # Create USDZ package
        UsdUtils.CreateNewUsdzPackage(asset.path, self.usdz_output_path)
        print(f"✅ USDZ file created: {self.usdz_output_path}")

    def _start_http_server(self):
        """Start HTTP server to serve USDZ file"""
        try:
            # Change to directory containing USDZ file
            usdz_dir = os.path.dirname(os.path.abspath(self.usdz_output_path))
            os.chdir(usdz_dir)
            
            handler = http.server.SimpleHTTPRequestHandler
            self.http_server = socketserver.TCPServer(("", self.http_port), handler)
            
            def serve_forever():
                print(f"🌐 HTTP server started on port {self.http_port}")
                self.http_server.serve_forever()
            
            self.http_thread = threading.Thread(target=serve_forever, daemon=True)
            self.http_thread.start()
            
        except Exception as e:
            print(f"❌ Failed to start HTTP server: {e}")

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
            
            # Wait a moment for connection to establish
            time.sleep(0.5)
            
            # Send initial USDZ data via gRPC or URL
            if self.use_grpc_data_transfer:
                self._send_usdz_data()
            else:
                self._send_usdz_url()


            
        except Exception as e:
            print(f"❌ Failed to setup gRPC client: {e}")

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

    def _send_usdz_data(self):
        """Send USDZ file data directly via gRPC"""
        try:
            # First test with a small file to verify connection
            print("🔍 Testing gRPC connection with small file first...")
            if not self._test_small_data_transfer():
                print("❌ Small file test failed, skipping large file transfer")
                return
            
            print("✅ Small file test passed, proceeding with actual USDZ file...")
            
            # Read the USDZ file as binary data
            with open(self.usdz_output_path, 'rb') as f:
                usdz_data = f.read()
            
            usdz_filename = os.path.basename(self.usdz_output_path)
            
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

    def _send_usdz_url(self):
        """Send USDZ URL to VR device (legacy method)"""
        try:
            usdz_filename = os.path.basename(self.usdz_output_path)
            usdz_url = f"http://{self._get_local_ip()}:{self.http_port}/{usdz_filename}"
            
            request = mujoco_ar_pb2.UsdzUrlRequest(
                usdz_url=usdz_url,
                session_id=self.session_id
            )
            
            response = self.grpc_stub.SendUsdzUrl(request)
            if response.success:
                print(f"✅ USDZ URL sent successfully: {usdz_url}")
            else:
                print(f"❌ Failed to send USDZ URL: {response.message}")
                
        except Exception as e:
            print(f"❌ Error sending USDZ URL: {e}")

    def _get_local_ip(self):
        """Get local IP address"""
        import socket
        try:
            # Connect to a remote server to determine local IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except:
            return "localhost"

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

    def update(self): 
        """
        Generate a pose dictionary and send it to the VR device over gRPC. 
        """
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
                    print(f"📍 Updated {response.bodies_updated} bodies")
                else:
                    print(f"❌ Pose update failed: {response.message}")
            else:
                print("❌ gRPC connection not available")
                
        except Exception as e:
            print(f"❌ Error in update: {e}")

    def close(self):
        """Clean up resources"""
        if self.grpc_channel:
            self.grpc_channel.close()
        if self.http_server:
            self.http_server.shutdown()
            self.http_server.server_close()
        print("🔌 MJARView closed")


class MJARViewer:
    """
    MuJoCo AR Viewer - Main class for AR visualization of MuJoCo simulations.
    
    This class provides a clean API for visualizing MuJoCo physics simulations
    in Augmented Reality using Apple Vision Pro and other AR devices.
    
    Args:
        avp_ip (str): IP address of the AR device (e.g., Apple Vision Pro)
        grpc_port (int, optional): gRPC port for communication. Defaults to 50051.
        
    Example:
        >>> from mujoco_arviewer import MJARViewer
        >>> import mujoco
        >>> 
        >>> # Initialize the AR viewer
        >>> viewer = MJARViewer(avp_ip="192.168.1.100")
        >>> 
        >>> # Send a MuJoCo model to the AR device
        >>> viewer.send_model("path/to/your/model.xml")
        >>> 
        >>> # Register MuJoCo model and data for pose updates
        >>> model = mujoco.MjModel.from_xml_path("path/to/your/model.xml")
        >>> data = mujoco.MjData(model)
        >>> viewer.register(model, data)
        >>> 
        >>> # Simulation loop
        >>> while True:
        >>>     mujoco.mj_step(model, data)
        >>>     viewer.sync()  # Send pose updates to AR device
    """

    def __init__(self, avp_ip, grpc_port=50051): 
        """
        Initialize MJARViewer.
        
        Args:
            avp_ip (str): IP address of the AR device
            grpc_port (int, optional): gRPC port for communication. Defaults to 50051.
        """
        self.avp_ip = avp_ip
        self.grpc_port = grpc_port
        self.model = None
        self.data = None
        self.bodies = {}
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

    def send_model(self, model_path):
        """
        Send a MuJoCo model to the AR device.
        
        Args:
            model_path (str): Path to either a MuJoCo XML file or USDZ file.
                              If XML is provided, it will be automatically converted to USDZ.
        
        Example:
            >>> viewer.send_model("scenes/robot.xml")
            >>> # or
            >>> viewer.send_model("scenes/robot.usdz")
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
        import mujoco_usd_converter
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
        """
        Register MuJoCo model and data for pose updates.
        
        Args:
            model: MuJoCo MjModel instance
            data: MuJoCo MjData instance
            
        Example:
            >>> model = mujoco.MjModel.from_xml_path("robot.xml")
            >>> data = mujoco.MjData(model)
            >>> viewer.register(model, data)
        """
        self.model = model 
        self.data = data 

        # bodies 
        self.bodies = {self.model.body(i).name: i for i in range(self.model.nbody)}

    def get_poses(self): 
        """
        Get poses of all bodies in the simulation.
        
        Returns:
            dict: Dictionary mapping body names to their position and quaternion data.
                  Format: {body_name: {"xpos": [x, y, z], "xquat": [w, x, y, z]}}
        """
        if self.model is None or self.data is None:
            raise RuntimeError("Model and data must be registered first using register() method")
            
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
        """
        Synchronize the current simulation state with the AR device.
        
        This method sends the current poses of all bodies to the AR device.
        Call this method regularly in your simulation loop to keep the AR
        visualization updated.
        
        Example:
            >>> while True:
            >>>     mujoco.mj_step(model, data)
            >>>     viewer.sync()  # Update AR visualization
        """
        if self.model is None or self.data is None:
            raise RuntimeError("Model and data must be registered first using register() method")

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
                if not response.success:
                    print(f"❌ Pose update failed: {response.message}")
            else:
                print("❌ gRPC connection not available")
                
        except Exception as e:
            print(f"❌ Error in sync: {e}")

    def close(self):
        """
        Close the viewer and clean up resources.
        
        Call this method when you're done with the AR viewer to properly
        close the gRPC connection and free resources.
        """
        if hasattr(self, 'grpc_channel') and self.grpc_channel:
            self.grpc_channel.close()
        print("🔌 MJARViewer closed")