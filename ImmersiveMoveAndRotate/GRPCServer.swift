import Foundation
import GRPC
import NIOCore
import NIOPosix
import SwiftUI

// MARK: - gRPC Server Implementation
final class MuJoCoARServer: Mujoco_arMuJoCoARServiceAsyncProvider {
    weak var immersiveView: ImmersiveView?
    
    init(immersiveView: ImmersiveView) {
        self.immersiveView = immersiveView
    }
    
    func sendHandshake(
        request: Mujoco_arHandshakeRequest,
        context: GRPCAsyncServerCallContext
    ) async throws -> Mujoco_arHandshakeResponse {
        print("📥 Received handshake: \(request.usdzURL)")
        
        // Update the USDZ URL in the immersive view
        DispatchQueue.main.async {
            self.immersiveView?.updateUsdzURL(request.usdzURL)
        }
        
        return Mujoco_arHandshakeResponse.with {
            $0.success = true
            $0.message = "Handshake received successfully"
        }
    }
    
    func updatePoses(
        request: Mujoco_arPoseUpdateRequest,
        context: GRPCAsyncServerCallContext
    ) async throws -> Mujoco_arPoseUpdateResponse {
        // Update poses in the AR scene
        DispatchQueue.main.async {
            self.immersiveView?.updatePoses(request.bodyPoses)
        }
        
        return Mujoco_arPoseUpdateResponse.with {
            $0.success = true
            $0.message = "Poses updated successfully"
        }
    }
    
    func streamPoseUpdates(
        requestStream: GRPCAsyncRequestStream<Mujoco_arPoseUpdateRequest>,
        responseStream: GRPCAsyncResponseStreamWriter<Mujoco_arPoseUpdateResponse>,
        context: GRPCAsyncServerCallContext
    ) async throws {
        // Handle streaming pose updates
        for try await request in requestStream {
            DispatchQueue.main.async {
                self.immersiveView?.updatePoses(request.bodyPoses)
            }
            
            let response = Mujoco_arPoseUpdateResponse.with {
                $0.success = true
                $0.message = "Streaming pose update received"
            }
            
            try await responseStream.send(response)
        }
    }
}

// MARK: - gRPC Server Manager
@MainActor
final class GRPCServerManager: ObservableObject {
    private var server: GRPCServer?
    private let eventLoopGroup = MultiThreadedEventLoopGroup(numberOfThreads: 1)
    
    func startServer(port: Int = 50051, immersiveView: ImmersiveView) async {
        do {
            let provider = MuJoCoARServer(immersiveView: immersiveView)
            
            server = try await GRPCServer.start(
                configuration: .init(
                    target: .hostAndPort("0.0.0.0", port),
                    eventLoopGroup: eventLoopGroup,
                    serviceProviders: [provider]
                )
            )
            
            print("🚀 gRPC server started on port \(port)")
            
        } catch {
            print("❌ Failed to start gRPC server: \(error)")
        }
    }
    
    func stopServer() async {
        server?.initiateGracefulShutdown()
        server = nil
        print("🛑 gRPC server stopped")
    }
    
    deinit {
        Task {
            await stopServer()
            try await eventLoopGroup.shutdownGracefully()
        }
    }
}