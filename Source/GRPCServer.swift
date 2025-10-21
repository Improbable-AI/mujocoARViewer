import Foundation
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf
import NIOCore
import NIOPosix
import simd
import QuartzCore

// Import the generated proto files
// Note: Make sure the generated files are included in your Xcode target

// Simple thread-safe rate limiter
private class RateLimiter {
    private var lastUpdateTime: CFTimeInterval = 0
    private let minInterval: CFTimeInterval
    private let queue = DispatchQueue(label: "com.mujocoAR.rateLimiter")
    
    init(maxFPS: Double) {
        self.minInterval = 1.0 / maxFPS
    }
    
    func shouldAllow() -> Bool {
        return queue.sync {
            let currentTime = CACurrentMediaTime()
            if currentTime - lastUpdateTime >= minInterval {
                lastUpdateTime = currentTime
                return true
            }
            return false
        }
    }
}

@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
final class GRPCServerManager: ObservableObject {
    private var grpcServer: GRPCServer<HTTP2ServerTransport.Posix>?
    private var currentPort: Int = 50051
    
    func startServer(immersiveView: ImmersiveView, port: Int = 50051) async {
        self.currentPort = port
        print("🚀 Starting gRPC server...")
        do {
            // Create the service implementation
            let mujocoService = MuJoCoARServiceImpl(immersiveView: immersiveView)
            print("✅ Created MuJoCoARServiceImpl")
            
            // Create the gRPC server with NIO transport
            let transport = HTTP2ServerTransport.Posix(
                address: .ipv4(host: "0.0.0.0", port: currentPort),
                transportSecurity: .plaintext
            )
            print("✅ Created HTTP2ServerTransport on port \(currentPort)")
            
            let server = GRPCServer(
                transport: transport,
                services: [mujocoService]
            )
            print("✅ Created GRPCServer with service")
            
            self.grpcServer = server
            
            print("🎯 Starting server.serve()...")
            try await server.serve()
            print("🚀 gRPC server started successfully on port \(currentPort)")
            
        } catch {
            print("❌ Failed to start gRPC server: \(error)")
            print("🔍 Error details: \(error.localizedDescription)")
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
    
    // Add a serial queue for pose processing to prevent race conditions
    private let poseProcessingQueue = DispatchQueue(label: "com.mujocoAR.poseProcessing", qos: .userInteractive)
    
    // Rate limiter to prevent overwhelming the rendering system
    private let rateLimiter = RateLimiter(maxFPS: 60)
    
    init(immersiveView: ImmersiveView) {
        self.immersiveView = immersiveView
        print("🎯 MuJoCoARServiceImpl initialized")
        print("📋 Available methods:")
        print("   - sendUsdzUrl")
        print("   - sendUsdzData")
        print("   - sendUsdzDataChunked")
        print("   - updatePoses")
        print("   - streamPoses")
        print("   - streamHandTracking (NEW!)")
    }
    
    // MARK: - Service Implementation
    
    func sendUsdzUrl(
        request: MujocoAr_UsdzUrlRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzUrlResponse {
        print("📨 [sendUsdzUrl] Received request")
        print("📨 [sendUsdzUrl] USDZ URL: \(request.usdzURL)")
        print("📨 [sendUsdzUrl] Session ID: \(request.sessionID)")
        
        // Update the immersive view with the new URL
        await MainActor.run {
            print("📨 [sendUsdzUrl] Updating immersive view...")
            immersiveView?.updateUsdzURL(request.usdzURL)
            print("📨 [sendUsdzUrl] Immersive view updated")
        }
        
        var response = MujocoAr_UsdzUrlResponse()
        response.success = true
        response.message = "USDZ URL received successfully"
        print("📨 [sendUsdzUrl] Sending success response")
        return response
    }
    
    func sendUsdzData(
        request: MujocoAr_UsdzDataRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📨 [sendUsdzData] *** METHOD CALLED ***")
        print("📨 [sendUsdzData] Received USDZ data: \(request.usdzData.count) bytes")
        print("📨 [sendUsdzData] Filename: \(request.filename)")
        print("📨 [sendUsdzData] Session ID: \(request.sessionID)")
        
        // Extract attach_to information
        var attachToPosition: SIMD3<Float>? = nil
        var attachToRotation: simd_quatf? = nil
        
        if request.hasAttachToPosition && request.hasAttachToRotation {
            attachToPosition = SIMD3<Float>(request.attachToPosition.x, request.attachToPosition.y, request.attachToPosition.z)
            attachToRotation = simd_quatf(ix: request.attachToRotation.x, iy: request.attachToRotation.y, iz: request.attachToRotation.z, r: request.attachToRotation.w)
            print("📨 [sendUsdzData] Attach to position: \(attachToPosition!), rotation: \(attachToRotation!)")
        }
        
        var response = MujocoAr_UsdzDataResponse()
        
        do {
            print("📨 [sendUsdzData] Creating temporary file...")
            // Create a temporary file to store the USDZ data
            let tempDir = FileManager.default.temporaryDirectory
            let fileName = request.filename.isEmpty ? "\(UUID().uuidString).usdz" : request.filename
            let localURL = tempDir.appendingPathComponent(fileName)
            
            print("📨 [sendUsdzData] Writing to: \(localURL.path)")
            // Write the data to the local file
            try request.usdzData.write(to: localURL)
            
            print("💾 [sendUsdzData] Saved USDZ data to: \(localURL.path)")
            
            // Update the immersive view with the local file URL and attach_to information
            await MainActor.run {
                print("📨 [sendUsdzData] Updating immersive view with local file...")
                immersiveView?.updateUsdzURL(localURL.absoluteString, attachToPosition: attachToPosition, attachToRotation: attachToRotation)
                print("📨 [sendUsdzData] Immersive view updated")
            }
            
            response.success = true
            response.message = "USDZ data received and saved successfully"
            response.localFilePath = localURL.path
            print("📨 [sendUsdzData] Sending success response")
            
        } catch {
            print("❌ [sendUsdzData] Failed to save USDZ data: \(error)")
            print("❌ [sendUsdzData] Error details: \(error.localizedDescription)")
            response.success = false
            response.message = "Failed to save USDZ data: \(error.localizedDescription)"
        }
        
        print("📨 [sendUsdzData] Returning response (success: \(response.success))")
        return response
    }
    
    func sendUsdzDataChunked(
        request: RPCAsyncSequence<MujocoAr_UsdzChunkRequest, any Error>,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📦 [sendUsdzDataChunked] *** CHUNKED TRANSFER STARTED ***")
        
        var response = MujocoAr_UsdzDataResponse()
        var chunkData = Data()
        var fileName = ""
        var sessionID = ""
        var totalExpectedSize: Int64 = 0
        var receivedChunks = 0
        var totalChunks = 0
        var attachToPosition: SIMD3<Float>? = nil
        var attachToRotation: simd_quatf? = nil
        
        do {
            // Process each chunk
            for try await chunkRequest in request {
                print("📦 [sendUsdzDataChunked] Received chunk \(chunkRequest.chunkIndex + 1)/\(chunkRequest.totalChunks)")
                print("📦 [sendUsdzDataChunked] Chunk size: \(chunkRequest.chunkData.count) bytes")
                
                // Store metadata from first chunk
                if receivedChunks == 0 {
                    fileName = chunkRequest.filename
                    sessionID = chunkRequest.sessionID
                    totalExpectedSize = chunkRequest.totalSize
                    totalChunks = Int(chunkRequest.totalChunks)
                    
                    // Extract attach_to information from first chunk
                    if chunkRequest.hasAttachToPosition && chunkRequest.hasAttachToRotation {
                        attachToPosition = SIMD3<Float>(chunkRequest.attachToPosition.x, chunkRequest.attachToPosition.y, chunkRequest.attachToPosition.z)
                        attachToRotation = simd_quatf(ix: chunkRequest.attachToRotation.x, iy: chunkRequest.attachToRotation.y, iz: chunkRequest.attachToRotation.z, r: chunkRequest.attachToRotation.w)
                        print("📦 [sendUsdzDataChunked] Attach to position: \(attachToPosition!), rotation: \(attachToRotation!)")
                    }
                    
                    print("📦 [sendUsdzDataChunked] Filename: \(fileName)")
                    print("📦 [sendUsdzDataChunked] Session ID: \(sessionID)")
                    print("📦 [sendUsdzDataChunked] Total expected size: \(totalExpectedSize) bytes")
                    print("📦 [sendUsdzDataChunked] Total chunks: \(totalChunks)")
                }
                
                // Append chunk data
                chunkData.append(chunkRequest.chunkData)
                receivedChunks += 1
                
                print("📦 [sendUsdzDataChunked] Progress: \(chunkData.count)/\(totalExpectedSize) bytes (\(receivedChunks)/\(totalChunks) chunks)")
                
                // Check if this is the last chunk
                if chunkRequest.isLastChunk {
                    print("📦 [sendUsdzDataChunked] Received last chunk!")
                    break
                }
            }
            
            // Verify we received all expected data
            if chunkData.count != totalExpectedSize {
                print("⚠️ [sendUsdzDataChunked] Warning: Received \(chunkData.count) bytes, expected \(totalExpectedSize)")
            }
            
            if receivedChunks != totalChunks {
                print("⚠️ [sendUsdzDataChunked] Warning: Received \(receivedChunks) chunks, expected \(totalChunks)")
            }
            
            print("📦 [sendUsdzDataChunked] All chunks received, assembling file...")
            
            // Create temporary file to store the complete data
            let tempDir = FileManager.default.temporaryDirectory
            let finalFileName = fileName.isEmpty ? "\(UUID().uuidString).usdz" : fileName
            let localURL = tempDir.appendingPathComponent(finalFileName)
            
            print("📦 [sendUsdzDataChunked] Writing assembled file to: \(localURL.path)")
            
            // Write the complete data to file
            try chunkData.write(to: localURL)
            
            print("💾 [sendUsdzDataChunked] Successfully saved complete USDZ file (\(chunkData.count) bytes)")
            
            // Update the immersive view with the local file URL and attach_to information
            await MainActor.run {
                print("📦 [sendUsdzDataChunked] Updating immersive view with assembled file...")
                immersiveView?.updateUsdzURL(localURL.absoluteString, attachToPosition: attachToPosition, attachToRotation: attachToRotation)
                print("📦 [sendUsdzDataChunked] Immersive view updated")
            }
            
            response.success = true
            response.message = "Chunked USDZ data received and assembled successfully (\(receivedChunks) chunks, \(chunkData.count) bytes)"
            response.localFilePath = localURL.path
            print("📦 [sendUsdzDataChunked] Sending success response")
            
        } catch {
            print("❌ [sendUsdzDataChunked] Failed to process chunked data: \(error)")
            print("❌ [sendUsdzDataChunked] Error details: \(error.localizedDescription)")
            response.success = false
            response.message = "Failed to process chunked data: \(error.localizedDescription)"
        }
        
        print("📦 [sendUsdzDataChunked] Returning response (success: \(response.success))")
        return response
    }
    
    func updatePoses(
        request: MujocoAr_PoseUpdateRequest,
        context: ServerContext
    ) async throws -> MujocoAr_PoseUpdateResponse {
        print("📨 [updatePoses] Received pose update with \(request.bodyPoses.count) bodies")
        print("📨 [updatePoses] Session ID: \(request.sessionID)")
        print("📨 [updatePoses] Timestamp: \(request.timestamp)")
        
        // Convert gRPC poses to a format for computation
        var posesMutable: [String: MujocoAr_BodyPose] = [:]
        for bodyPose in request.bodyPoses {
            posesMutable[bodyPose.bodyName] = bodyPose
        }
        let poses = posesMutable  // make immutable snapshot
        
        // Process poses on serial queue to prevent race conditions
        await withCheckedContinuation { continuation in
            poseProcessingQueue.async {
                Task { @MainActor in
                    guard let immersiveView = immersiveView else { 
                        continuation.resume()
                        return 
                    }
                    
                    // Compute the final transforms here
                    let finalTransforms = immersiveView.computeFinalTransforms(poses)
                    
                    // Apply the pre-computed transforms atomically
                    immersiveView.updatePosesWithTransforms(finalTransforms)
                    
                    continuation.resume()
                }
            }
        }
        
        var response = MujocoAr_PoseUpdateResponse()
        response.success = true
        response.message = "Poses updated successfully"
        response.bodiesUpdated = Int32(request.bodyPoses.count)
        print("📨 [updatePoses] Sending success response for \(response.bodiesUpdated) bodies")
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
                var posesMutable: [String: MujocoAr_BodyPose] = [:]
                for bodyPose in poseRequest.bodyPoses {
                    posesMutable[bodyPose.bodyName] = bodyPose
                }
                let poses = posesMutable  // immutable snapshot
                
                // Process all pose updates - let SwiftUI handle the rendering rate
                
                // Process poses on serial queue to prevent race conditions 
                await withCheckedContinuation { continuation in
                    poseProcessingQueue.async {
                        Task { @MainActor in
                            guard let immersiveView = immersiveView else { 
                                continuation.resume()
                                return 
                            }
                            
                            // Compute the final transforms here
                            let finalTransforms = immersiveView.computeFinalTransforms(poses)
                            
                            // Apply the pre-computed transforms atomically
                            immersiveView.updatePosesWithTransforms(finalTransforms)
                            
                            continuation.resume()
                        }
                    }
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
    
    func streamHandTracking(
        request: MujocoAr_HandTrackingRequest,
        response: RPCWriter<MujocoAr_HandTrackingUpdate>,
        context: ServerContext
    ) async throws {
        print("🖐️ [streamHandTracking] Client connected for hand tracking stream")
        print("🖐️ [streamHandTracking] Session ID: \(request.sessionID)")
        
        do {
            // Stream hand tracking data continuously
            while !Task.isCancelled {
                // Get the latest hand tracking data from ImmersiveView
                if let handTrackingData = await MainActor.run(body: {
                    return immersiveView?.getHandTrackingData()
                }) {
                    try await response.write(handTrackingData)
                }
                
                // Stream at ~100 Hz (10ms delay)
                try await Task.sleep(nanoseconds: 10_000_000)
            }
        } catch {
            print("❌ [streamHandTracking] Error in hand tracking streaming: \(error)")
            throw error
        }
        
        print("🖐️ [streamHandTracking] Client disconnected from hand tracking stream")
    }
}
