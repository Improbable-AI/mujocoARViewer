import SwiftUI
import RealityKit
import Network

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
    
    let spatialGen = SpatialGenHelper()
    

    var body: some View {
        Group {
            if let entity {
                RealityView { content in
                    let bounds = entity.model?.mesh.bounds.extents ?? .zero
                    entity.position = SIMD3(0, bounds.y / 2, -2)
                    content.add(entity)
                }
                .enableMovingEntity(entity)
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
            // Start gRPC server
            await grpcManager.startServer(immersiveView: self)
            
            // Legacy TCP server (can be removed once gRPC is working)
            // server.startListening()
            
            // Observe URL updates from both gRPC and legacy TCP
            await observeUsdzUpdates()
        }
        .onDisappear {
            Task {
                await grpcManager.stopServer()
            }
        }
    }
    
    // MARK: - gRPC Integration Methods
    func updateUsdzURL(_ url: String) {
        self.usdzURL = url
    }
    
    func updatePoses(_ poses: [String: Mujoco_arBodyPose]) {
        for (bodyName, pose) in poses {
            if let bodyEntity = bodyEntities[bodyName] {
                // Convert gRPC pose to RealityKit Transform
                let position = SIMD3<Float>(
                    pose.position.x,
                    pose.position.y, 
                    pose.position.z
                )
                
                let rotation = simd_quatf(
                    ix: pose.rotation.x,
                    iy: pose.rotation.y,
                    iz: pose.rotation.z,
                    r: pose.rotation.w
                )
                
                // Update entity transform
                bodyEntity.transform.translation = position
                bodyEntity.transform.rotation = rotation
            }
        }
    }
    
    private func observeUsdzUpdates() async {
        // Monitor usdzURL changes
        for await urlStr in $usdzURL.values {
            guard let urlStr,
                  let url = URL(string: urlStr) else { continue }
            await loadUsdzModel(from: url)
        }
        
        // Also monitor legacy server updates
        for await urlStr in server.$usdzURL.values {
            guard let urlStr,
                  let url = URL(string: urlStr) else { continue }
            await loadUsdzModel(from: url)
        }
    }
    
    private func loadUsdzModel(from url: URL) async {
        do {
            print("📦 Loading USDZ from \(url.absoluteString)")
            let loadedEntity = try await self.spatialGen.loadEntity(from: url)
            
            if let model = loadedEntity as? ModelEntity {
                entity = model
                setupInteraction(for: model)
                indexBodyEntities(model)
            } else {
                // Wrap generic Entity in ModelEntity
                let wrapper = ModelEntity()
                wrapper.addChild(loadedEntity)
                entity = wrapper
                setupInteraction(for: wrapper)
                indexBodyEntities(wrapper)
            }
            
        } catch {
            print("❌ Failed to load USDZ: \(error)")
        }
    }
    
    private func indexBodyEntities(_ rootEntity: Entity) {
        // Recursively find and index all child entities that represent MuJoCo bodies
        func indexChildren(_ entity: Entity) {
            if !entity.name.isEmpty {
                if let modelEntity = entity as? ModelEntity {
                    bodyEntities[entity.name] = modelEntity
                } else {
                    // Create a ModelEntity wrapper if needed
                    let wrapper = ModelEntity()
                    wrapper.transform = entity.transform
                    wrapper.name = entity.name
                    if let parent = entity.parent {
                        parent.addChild(wrapper)
                        entity.removeFromParent()
                        wrapper.addChild(entity)
                    }
                    bodyEntities[entity.name] = wrapper
                }
            }
            
            for child in entity.children {
                indexChildren(child)
            }
        }
        
        indexChildren(rootEntity)
        print("📝 Indexed \(bodyEntities.count) body entities")
    }
    }

    private func setupInteraction(for model: ModelEntity) {
        let bounds = model.model?.mesh.bounds.extents ?? .one
        model.components.set(CollisionComponent(shapes: [.generateBox(size: bounds)]))
        model.components.set(HoverEffectComponent())
        model.components.set(InputTargetComponent())
//        model.components.physics
    }
}

