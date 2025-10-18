import mujoco 
import mujoco_usd_converter, usdex.core
from pxr import Sdf, Usd, UsdUtils
import grpc
import time
import threading
import http.server
import socketserver
import os
from pathlib import Path

# Import generated gRPC classes
from generated import mujoco_ar_pb2, mujoco_ar_pb2_grpc

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
        
        # gRPC client setup
        self.grpc_channel = None
        self.grpc_stub = None
        self.session_id = f"mjarview_{int(time.time())}"
        
        # HTTP server for USDZ file serving
        self.http_server = None
        self.http_thread = None
        
        # Convert to USDZ on initialization
        self._convert_to_usdz()
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
            target = f"{self.vr_device_ip}:{self.grpc_port}"
            self.grpc_channel = grpc.insecure_channel(target)
            self.grpc_stub = mujoco_ar_pb2_grpc.MuJoCoARServiceStub(self.grpc_channel)
            print(f"🔗 gRPC client connected to {target}")
            
            # Send initial handshake with USDZ URL
            self._send_usdz_url()
            
        except Exception as e:
            print(f"❌ Failed to setup gRPC client: {e}")

    def _send_usdz_url(self):
        """Send USDZ URL to VR device"""
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

if __name__ == "__main__":
    # Example usage
    xml_path = "scenes/agilex_piper/scene.xml"  # Replace with your MuJoCo XML file path
    vr_device_ip = "128.30.227.76"