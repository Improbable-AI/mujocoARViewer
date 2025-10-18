import Foundation
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf
import NIOCore
import NIOPosix

// Import the generated proto files
// Note: Make sure the generated files are included in your Xcode target

@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
final class GRPCServerManager: ObservableObject {
    private var grpcServer: GRPCServer<HTTP2ServerTransport.Posix>?
    private let port = 50051
    
    func startServer(immersiveView: ImmersiveView) async {
        do {
            // Create the service implementation
            let mujocoService = MuJoCoARServiceImpl(immersiveView: immersiveView)
            
            // Create the gRPC server with NIO transport
            let transport = HTTP2ServerTransport.Posix(
                address: .ipv4(host: "0.0.0.0", port: port),
                transportSecurity: .plaintext
            )
            
            let server = GRPCServer(
                transport: transport,
                services: [mujocoService]
            )
            
            self.grpcServer = server
            
            try await server.serve()
            print("🚀 gRPC server started on port \(port)")
            
        } catch {
            print("❌ Failed to start gRPC server: \(error)")
        }
    }
    
    func stopServer() async {
        if let server = self.grpcServer {
//            await server.k()
            print("🛑 gRPC server stopped")
            self.grpcServer = nil
        } else {
            print("ℹ️ gRPC server was not running")
        }
    }
}

@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
struct MuJoCoARServiceImpl: MujocoAr_MuJoCoARService.SimpleServiceProtocol {
    var immersiveView: ImmersiveView?
    
    init(immersiveView: ImmersiveView) {
        self.immersiveView = immersiveView
    }
    
    // MARK: - Service Implementation
    
    func sendUsdzUrl(
        request: MujocoAr_UsdzUrlRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzUrlResponse {
        print("📨 Received USDZ URL: \(request.usdzURL)")
        
        // Update the immersive view with the new URL
        await MainActor.run {
            immersiveView?.updateUsdzURL(request.usdzURL)
        }
        
        var response = MujocoAr_UsdzUrlResponse()
        response.success = true
        response.message = "USDZ URL received successfully"
        return response
    }
    
    func updatePoses(
        request: MujocoAr_PoseUpdateRequest,
        context: ServerContext
    ) async throws -> MujocoAr_PoseUpdateResponse {
        
        // Convert gRPC poses to a format the immersive view can use
        var poses: [String: MujocoAr_BodyPose] = [:]
        
        for bodyPose in request.bodyPoses {
            poses[bodyPose.bodyName] = bodyPose
        }
        
        // Update poses in the immersive view
        await MainActor.run {
            immersiveView?.updatePoses(poses)
        }
        
        var response = MujocoAr_PoseUpdateResponse()
        response.success = true
        response.message = "Poses updated successfully"
        response.bodiesUpdated = Int32(request.bodyPoses.count)
        return response
    }
    
    func streamPoses(
        request: RPCAsyncSequence<MujocoAr_PoseUpdateRequest, any Error>,
        response: RPCWriter<MujocoAr_PoseUpdateResponse>,
        context: ServerContext
    ) async throws {
        
        do {
            for try await poseRequest in request {
                // Process each pose update
                var poses: [String: MujocoAr_BodyPose] = [:]
                
                for bodyPose in poseRequest.bodyPoses {
                    poses[bodyPose.bodyName] = bodyPose
                }
                
                // Update poses in the immersive view
                await MainActor.run {
                    immersiveView?.updatePoses(poses)
                }
                
                // Send response
                var responseMsg = MujocoAr_PoseUpdateResponse()
                responseMsg.success = true
                responseMsg.message = "Stream poses updated"
                responseMsg.bodiesUpdated = Int32(poseRequest.bodyPoses.count)
                
                try await response.write(responseMsg)
            }
        } catch {
            print("❌ Error in pose streaming: \(error)")
            throw error
        }
    }
}
