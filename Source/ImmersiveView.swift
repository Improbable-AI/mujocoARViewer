import SwiftUI
import RealityKit
import Network

// Import generated protobuf types
// Note: These should be generated from your proto file
// Make sure the generated files are included in your Xcode target

// MARK: - TCP Server to Receive Handshake (Legacy - can be removed once gRPC is working)
final class HandshakeServer: ObservableObject {
    @Published var usdzURL: String? = "http://localhost:8083/scene.usdz"
    private var listener: NWListener?

    func startListening(port: UInt16 = 9999) {
        do {
            let listener = try NWListener(using: .tcp, on: NWEndpoint.Port(rawValue: port)!)
            listener.newConnectionHandler = { connection in
                connection.start(queue: .main)
                connection.receive(minimumIncompleteLength: 1, maximumLength: 2048) { data, _, _, _ in
                    if let data = data,
                       let json = try? JSONSerialization.jsonObject(with: data) as? [String: String],
                       let url = json["usdz_url"] {
                        DispatchQueue.main.async {
                            print("✅ Received USDZ URL: \(url)")
                            self.usdzURL = url
                        }
                    }
                    connection.cancel()
                }
            }
            listener.start(queue: .main)
            self.listener = listener
            print("👂 Listening for PC handshake on port \(port)...")
        } catch {
            print("⚠️ Failed to start handshake listener: \(error)")
        }
    }
}

// MARK: - Main Immersive View
struct ImmersiveView: View {
    @StateObject private var server = HandshakeServer()
    @StateObject private var grpcManager = GRPCServerManager()
    @State private var entity: ModelEntity?
    @State private var bodyEntities: [String: ModelEntity] = [:]
    @State private var usdzURL: String? = nil
    @State private var initialLocalTransforms: [String: Transform] = [:]
    let spatialGen = SpatialGenHelper()
    

    var body: some View {
        Group {
            if let entity {
                RealityView { content in
                    content.add(entity)
                }
            } else {
                VStack {
                    ProgressView("Waiting for USDZ file from PC…")
                        .padding()
                    Text("Ensure the PC script is running and sending handshake.")
                        .font(.footnote)
                        .foregroundStyle(.secondary)
                }
            }
        }
        .task {
            print("🚀 [ImmersiveView] Starting initialization...")
            // Start gRPC server
            print("🚀 [ImmersiveView] Starting gRPC server...")
            await grpcManager.startServer(immersiveView: self)
            print("✅ [ImmersiveView] gRPC server start completed")
            
            // Legacy TCP server (can be removed once gRPC is working)
            // server.startListening()
        }
        .onChange(of: usdzURL ?? server.usdzURL) { _, newValue in
            if let urlStr = newValue, let url = URL(string: urlStr) {
                Task {
                    await loadUsdzModel(from: url)
                }
            }
        }
        .onDisappear {
            Task {
                await grpcManager.stopServer()
            }
        }
    }
    
    // MARK: - gRPC Integration Methods
    func updateUsdzURL(_ url: String) {
        print("🔄 [updateUsdzURL] Called with URL: \(url)")

        var finalURLString = url

        // Handle both URL strings and local file paths
        if url.hasPrefix("http://") || url.hasPrefix("https://") {
            // 🧩 Append a timestamp to force RealityKit to reload the asset
            let timestamp = Int(Date().timeIntervalSince1970)
            if url.contains("?") {
                finalURLString += "&t=\(timestamp)"
            } else {
                finalURLString += "?t=\(timestamp)"
            }
            print("🧩 [updateUsdzURL] Cache-busting URL: \(finalURLString)")
            self.usdzURL = finalURLString

        } else if url.hasPrefix("file://") {
            // Still bust cache by adding ?t=
            let timestamp = Int(Date().timeIntervalSince1970)
            finalURLString = "\(url)?t=\(timestamp)"
            print("🧩 [updateUsdzURL] Cache-busting file URL: \(finalURLString)")
            self.usdzURL = finalURLString

        } else {
            // Convert local path → file URL
            let fileURL = URL(fileURLWithPath: url)
            let timestamp = Int(Date().timeIntervalSince1970)
            let busted = "\(fileURL.absoluteString)?t=\(timestamp)"
            print("🧩 [updateUsdzURL] Cache-busting path URL: \(busted)")
            self.usdzURL = busted
        }
    }
    
    func updatePoses(_ poses: [String: MujocoAr_BodyPose]) {
        print("🔄 [updatePoses] Called with \(poses.count) poses")

        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0)) // Z-up → Y-up

        for (bodyName, pose) in poses {
            guard let bodyEntity = bodyEntities[bodyName] else {
                print("⚠️ Body entity '\(bodyName)' not found")
                continue
            }

            let mjPos = SIMD3<Float>(pose.position.x, pose.position.y, pose.position.z)
            let mjRot = simd_quatf(ix: pose.rotation.x, iy: pose.rotation.y, iz: pose.rotation.z, r: pose.rotation.w)

            // Convert from MuJoCo (Z-up) to RealityKit (Y-up)
            let rkRot = axisCorrection * mjRot
            let rkPos = axisCorrection.act(mjPos)
            
//            let rkRot = mjRot
//            let rkPos = mjPos

            // Build world-space transform from MuJoCo pose
            var worldTransform = matrix_identity_float4x4
            worldTransform = simd_mul(matrix_float4x4(rkRot), worldTransform)
            worldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)
            
            // Retrieve the initial local transform offset (geometry alignment)
            let localOffset = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4
            
            // define inverse of local Offset
            let invlocalOffset = localOffset.inverse

            // Combine: MuJoCo world * local offset
            let finalTransform = simd_mul(worldTransform, invlocalOffset)

            bodyEntity.setTransformMatrix(finalTransform, relativeTo: nil)
        }
    }
    
    private func loadUsdzModel(from url: URL) async {
        do {
            // --- Reset state before loading new model ---
            print("♻️ Resetting previous model and cached data...")
            entity = nil
            bodyEntities.removeAll()
            initialLocalTransforms.removeAll()
            
            // Optional: if you want to remove from RealityView entirely
            // (useful when RealityKit otherwise keeps entities alive)
            await MainActor.run {
                print("🧹 Removing all previous RealityKit entities")
            }

            // --- Load new model ---
            print("📦 Loading USDZ from \(url.absoluteString)")
            let loadedEntity = try await spatialGen.loadEntity(from: url)
            
            // Wrap or assign
            if let model = loadedEntity as? ModelEntity {
                entity = model
                indexBodyEntities(model)
            } else {
                let wrapper = ModelEntity()
                wrapper.addChild(loadedEntity)
                entity = wrapper
                indexBodyEntities(wrapper)
            }

            print("✅ Model loaded and indexed successfully")

        } catch {
            print("❌ Failed to load USDZ: \(error)")
        }
    }
    
    
    private func indexBodyEntities(_ rootEntity: Entity) {
        bodyEntities.removeAll()
        initialLocalTransforms.removeAll()
        print("🔍 [indexBodyEntities] Starting recursive indexing...")

        func indexChildren(_ entity: Entity) {
            if !entity.name.isEmpty {
                if let modelEntity = entity as? ModelEntity {
                    bodyEntities[entity.name] = modelEntity
                    initialLocalTransforms[entity.name] = modelEntity.transform
                    print("✅ Added ModelEntity: '\(entity.name)'")
                } else {
                    let wrapper = ModelEntity()
                    wrapper.name = entity.name
                    wrapper.transform = entity.transform   // preserve local transform
                    if let parent = entity.parent {
                        parent.addChild(wrapper)
                        entity.removeFromParent()
                        wrapper.addChild(entity)
                    }
                    bodyEntities[wrapper.name] = wrapper
                    initialLocalTransforms[wrapper.name] = wrapper.transform
                    print("✅ Added wrapper: '\(wrapper.name)' (cached local offset)")
                }
            }
            for child in entity.children {
                indexChildren(child)
            }
        }

        indexChildren(rootEntity)
        print("📝 Indexed \(bodyEntities.count) entities with cached local transforms")
    }
    
    private func setupInteraction(for model: ModelEntity) {
        let bounds = model.model?.mesh.bounds.extents ?? .one
        model.components.set(CollisionComponent(shapes: [.generateBox(size: bounds)]))
        model.components.set(HoverEffectComponent())
        model.components.set(InputTargetComponent())
//        model.components.physics
    }
}
