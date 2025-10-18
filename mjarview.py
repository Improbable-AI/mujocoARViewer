import mujoco 
import mujoco_usd_converter, usdex.core
from pxr import Sdf, Usd, UsdUtils
import grpc
import threading
import time
import http.server
import socketserver
import os
from concurrent import futures

# Import generated gRPC stubs
import mujoco_ar_pb2
import mujoco_ar_pb2_grpc

class MJARView:

    def __init__(self, xml_path, vr_device_ip="localhost", grpc_port=50051, http_port=8083): 

        # Store connection parameters
        self.xml_path = xml_path
        self.vr_device_ip = vr_device_ip
        self.grpc_port = grpc_port
        self.http_port = http_port

        # Load the MuJoCo model from the XML file
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # parse the number of bodies 
        self.bodies = {self.model.body(i).name: i for i in range(self.model.nbody)}

        # Convert to USDZ
        self._convert_to_usdz()
        
        # Start HTTP server for USDZ file serving
        self._start_http_server()
        
        # Initialize gRPC client
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
        def serve_files():
            class CustomHandler(http.server.SimpleHTTPRequestHandler):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, directory=os.path.dirname(self.usdz_output_path), **kwargs)
                    
                def end_headers(self):
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                    self.send_header('Access-Control-Allow-Headers', '*')
                    super().end_headers()

            with socketserver.TCPServer(("", self.http_port), CustomHandler) as httpd:
                print(f"🌐 HTTP server started on port {self.http_port}")
                httpd.serve_forever()

        # Start HTTP server in background thread
        self.http_thread = threading.Thread(target=serve_files, daemon=True)
        self.http_thread.start()
        
        # Generate the USDZ URL
        usdz_filename = os.path.basename(self.usdz_output_path)
        self.usdz_url = f"http://localhost:{self.http_port}/{usdz_filename}"
        print(f"📦 USDZ available at: {self.usdz_url}")

    def _setup_grpc_client(self):
        """Setup gRPC client connection"""
        self.channel = grpc.insecure_channel(f'{self.vr_device_ip}:{self.grpc_port}')
        self.stub = mujoco_ar_pb2_grpc.MuJoCoARServiceStub(self.channel)
        print(f"🔗 gRPC client connected to {self.vr_device_ip}:{self.grpc_port}")

    def send_handshake(self):
        """Send initial handshake with USDZ URL to VR device"""
        try:
            request = mujoco_ar_pb2.HandshakeRequest(
                usdz_url=self.usdz_url,
                model_name=os.path.basename(self.xml_path).replace('.xml', ''),
                num_bodies=self.model.nbody
            )
            
            response = self.stub.SendHandshake(request)
            if response.success:
                print(f"✅ Handshake successful: {response.message}")
            else:
                print(f"❌ Handshake failed: {response.message}")
            return response.success
        except grpc.RpcError as e:
            print(f"❌ gRPC error during handshake: {e}")
            return False

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

    def _convert_poses_to_proto(self, poses_dict):
        """Convert pose dictionary to protobuf format"""
        proto_poses = {}
        for body_name, pose_data in poses_dict.items():
            position = mujoco_ar_pb2.Vector3(
                x=pose_data["xpos"][0],
                y=pose_data["xpos"][1], 
                z=pose_data["xpos"][2]
            )
            rotation = mujoco_ar_pb2.Quaternion(
                w=pose_data["xquat"][0],
                x=pose_data["xquat"][1],
                y=pose_data["xquat"][2], 
                z=pose_data["xquat"][3]
            )
            proto_poses[body_name] = mujoco_ar_pb2.BodyPose(
                position=position,
                rotation=rotation
            )
        return proto_poses

    def update(self): 
        """
        Generate a pose dictionary and send it to the VR device over gRPC. 
        """
        try:
            # Get current poses
            poses = self.get_poses()
            
            # Convert to protobuf format
            proto_poses = self._convert_poses_to_proto(poses)
            
            # Create update request
            request = mujoco_ar_pb2.PoseUpdateRequest(
                body_poses=proto_poses,
                timestamp=time.time()
            )
            
            # Send update via gRPC
            response = self.stub.UpdatePoses(request)
            
            if response.success:
                print(f"✅ Pose update sent successfully")
            else:
                print(f"❌ Pose update failed: {response.message}")
            
            return response.success
            
        except grpc.RpcError as e:
            print(f"❌ gRPC error during pose update: {e}")
            return False

    def close(self):
        """Clean up resources"""
        if hasattr(self, 'channel'):
            self.channel.close()
        print("🔌 gRPC connection closed")