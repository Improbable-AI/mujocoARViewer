import SwiftUI
import RealityKit
import Network
import ARKit
import Foundation
import QuartzCore

// Import generated protobuf types
// Note: These should be generated from your proto file
// Make sure the generated files are included in your Xcode target

// MARK: - Network Information Helper
final class NetworkInfoManager: ObservableObject {
    @Published var ipAddress: String = "Getting IP..."
    @Published var connectionStatus: String = "Server Stopped"
    @Published var grpcPort: Int = 50051
    @Published var isServerRunning: Bool = false
    
    init() {
        updateNetworkInfo()
    }
    
    func updateNetworkInfo() {
        DispatchQueue.global(qos: .background).async {
            let ip = self.getWiFiAddress() ?? self.getLocalIPAddress() ?? "No IP Found"
            DispatchQueue.main.async {
                self.ipAddress = ip
            }
        }
    }
    
    func updateConnectionStatus(_ status: String) {
        DispatchQueue.main.async {
            self.connectionStatus = status
        }
    }
    
    private func getWiFiAddress() -> String? {
        var address: String?
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        
        guard getifaddrs(&ifaddr) == 0 else { return nil }
        guard let firstAddr = ifaddr else { return nil }
        
        for ifptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
            let interface = ifptr.pointee
            let addrFamily = interface.ifa_addr.pointee.sa_family
            
            if addrFamily == UInt8(AF_INET) {
                let name = String(cString: interface.ifa_name)
                if name == "en0" {  // WiFi interface
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t(interface.ifa_addr.pointee.sa_len),
                               &hostname, socklen_t(hostname.count),
                               nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                }
            }
        }
        
        freeifaddrs(ifaddr)
        return address
    }
    
    private func getLocalIPAddress() -> String? {
        let host = CFHostCreateWithName(nil, "google.com" as CFString).takeRetainedValue()
        CFHostStartInfoResolution(host, .addresses, nil)
        var success: DarwinBoolean = false
        if let addresses = CFHostGetAddressing(host, &success)?.takeUnretainedValue() as NSArray?,
           let theAddress = addresses.firstObject as? NSData {
            var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            if getnameinfo(theAddress.bytes.assumingMemoryBound(to: sockaddr.self), socklen_t(theAddress.length),
                          &hostname, socklen_t(hostname.count), nil, 0, NI_NUMERICHOST) == 0 {
                let numAddress = String(cString: hostname)
                return numAddress
            }
        }
        return nil
    }
}

// MARK: - Head Following Status Component
struct StatusDisplayComponent: Component {
    var networkManager: NetworkInfoManager
}

// MARK: - Head Following Status System
public struct StatusDisplaySystem: System {
    static let query = EntityQuery(where: .has(StatusDisplayComponent.self))
    
    public init(scene: RealityKit.Scene) {
        // System initialization
    }
    
    public func update(context: SceneUpdateContext) {
        let entities = context.entities(matching: Self.query, updatingSystemWhen: .rendering)
        
        for entity in entities {
            guard let statusComponent = entity.components[StatusDisplayComponent.self] else { continue }
            
            // Update the status display periodically using a simple timer check
            let currentTime = CACurrentMediaTime()
            if Int(currentTime) % 5 == 0 {  // Update every 5 seconds
                statusComponent.networkManager.updateNetworkInfo()
            }
        }
    }
}

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
    @StateObject private var networkManager = NetworkInfoManager()
    @State private var entity: ModelEntity?
    @State private var bodyEntities: [String: ModelEntity] = [:]
    @State private var usdzURL: String? = nil
    @State private var initialLocalTransforms: [String: Transform] = [:]
    @State private var statusEntity: Entity?
    @State private var realityContent: RealityViewContent?
    @State private var inputPort: String = "50051"
    let spatialGen = SpatialGenHelper()
    

    var body: some View {
        Group {
            RealityView { content, attachments in
                // Store reference to content for manual updates
                realityContent = content
                
                // Create head-following status display
                createStatusDisplay(content: content, attachments: attachments)
                
                // Add initial entity if available
                if let entity = entity {
                    content.add(entity)
                    print("✅ Added initial entity to RealityView: \(entity.name)")
                }
                
            } attachments: {
                Attachment(id: "status") {
                    VStack(alignment: .leading, spacing: 12) {
                        HStack {
                            Image(systemName: "network")
                                .foregroundColor(.blue)
                            Text("gRPC Server")
                                .font(.headline)
                                .foregroundColor(.primary)
                        }
                        
                        VStack(alignment: .leading, spacing: 8) {
                            Text("IP: \(networkManager.ipAddress)")
                                .font(.system(.body, design: .monospaced))
                                .foregroundColor(.primary)
                            
                            if networkManager.isServerRunning {
                                Text("Port: \(networkManager.grpcPort, format: .number.grouping(.never))")
                                    .font(.system(.body, design: .monospaced))
                                    .foregroundColor(.primary)
                            } else {
                                HStack {
                                    Text("Port:")
                                        .font(.system(.body, design: .monospaced))
                                        .foregroundColor(.primary)
                                    
                                    TextField("Port", text: $inputPort)
                                        .keyboardType(.numberPad)
                                        .textFieldStyle(.roundedBorder)
                                        .frame(maxWidth: 100)
                                        .font(.system(.body, design: .monospaced))
                                        .onChange(of: inputPort) { _, newValue in
                                            // Validate port number
                                            let filtered = newValue.filter { $0.isNumber }
                                            if filtered != newValue {
                                                inputPort = filtered
                                            }
                                            if let port = Int(filtered), port > 0 && port <= 65535 {
                                                networkManager.grpcPort = port
                                            }
                                        }
                                }
                            }
                            
                            HStack {
                                Circle()
                                    .fill(connectionStatusColor)
                                    .frame(width: 8, height: 8)
                                Text("\(networkManager.connectionStatus)")
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                            
                            if !networkManager.isServerRunning {
                                Button(action: {
                                    Task {
                                        await startServer()
                                    }
                                }) {
                                    HStack {
                                        Image(systemName: "play.fill")
                                        Text("START SERVER")
                                            .font(.caption.weight(.semibold))
                                    }
                                    .foregroundColor(.white)
                                    .padding(.horizontal, 16)
                                    .padding(.vertical, 8)
                                    .background(.green, in: RoundedRectangle(cornerRadius: 8))
                                }
                                .buttonStyle(.plain)
                            } else {
                                Button(action: {
                                    Task {
                                        await stopServer()
                                    }
                                }) {
                                    HStack {
                                        Image(systemName: "stop.fill")
                                        Text("STOP SERVER")
                                            .font(.caption.weight(.semibold))
                                    }
                                    .foregroundColor(.white)
                                    .padding(.horizontal, 16)
                                    .padding(.vertical, 8)
                                    .background(.red, in: RoundedRectangle(cornerRadius: 8))
                                }
                                .buttonStyle(.plain)
                            }
                        }
                    }
                    .frame(minWidth: 200)
                    .padding(20)
                    .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
                    .glassBackgroundEffect()
                }
            }
            .onAppear {
                // Register the custom system and component
                StatusDisplayComponent.registerComponent()
                StatusDisplaySystem.registerSystem()
            }
        }
        .task {
            print("🚀 [ImmersiveView] Starting initialization...")
            
            // Just initialize network info, don't start server automatically
            networkManager.updateNetworkInfo()
            
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
                networkManager.updateConnectionStatus("Stopping Server...")
                await grpcManager.stopServer()
                networkManager.updateConnectionStatus("Disconnected")
            }
        }
    }
    
    private var connectionStatusColor: Color {
        switch networkManager.connectionStatus {
        case let status where status.contains("Streaming"):
            return .green
        case let status where status.contains("Connected"):
            return .blue
        case let status where status.contains("Running"):
            return .green
        case let status where status.contains("Starting") || status.contains("Stopping"):
            return .orange
        case let status where status.contains("Stopped"):
            return .gray
        default:
            return .red
        }
    }
    
    // MARK: - Server Control Functions
    private func startServer() async {
        print("🚀 [startServer] Starting gRPC server on port \(networkManager.grpcPort)...")
        
        networkManager.updateConnectionStatus("Starting Server...")
        networkManager.isServerRunning = true
        
        // Update the gRPC manager with the selected port (you'll need to modify GRPCServerManager for this)
        await grpcManager.startServer(immersiveView: self, port: networkManager.grpcPort)
        
        networkManager.updateConnectionStatus("Server Running")
        print("✅ [startServer] gRPC server started successfully")
    }
    
    private func stopServer() async {
        print("🛑 [stopServer] Stopping gRPC server...")
        
        networkManager.updateConnectionStatus("Stopping Server...")
        
        await grpcManager.stopServer()
        
        networkManager.updateConnectionStatus("Server Stopped")
        networkManager.isServerRunning = false
        
        print("✅ [stopServer] gRPC server stopped successfully")
    }
    
    // MARK: - Head Following Status Display
    private func createStatusDisplay(content: RealityViewContent, attachments: RealityViewAttachments) {
        guard let statusAttachment = attachments.entity(for: "status") else {
            print("❌ Could not find status attachment")
            return
        }
        
        // Create an anchor that follows the head
        let headAnchor = AnchorEntity(.head)
        headAnchor.name = "statusHeadAnchor"
        
        // Create a container for the status display
        let statusContainer = Entity()
        statusContainer.name = "statusContainer"
        
        // Add the attachment to the container
        statusContainer.addChild(statusAttachment)
        
        // Position the status display in the upper right of the user's field of view
        // Offset it so it's not directly in front of their face
        statusContainer.setPosition([0.4, 0.3, -0.8], relativeTo: headAnchor)
        
        // Add the container to the head anchor
        headAnchor.addChild(statusContainer)
        
        // Add the head anchor to the scene
        content.add(headAnchor)
        
        // Add the status component for periodic updates
        statusContainer.components.set(StatusDisplayComponent(networkManager: networkManager))
        
        // Store reference for cleanup
        statusEntity = headAnchor
        
        print("✅ Created head-following status display")
    }
    
    // MARK: - gRPC Integration Methods
    func updateUsdzURL(_ url: String) {
        print("🔄 [updateUsdzURL] Called with URL: \(url)")
        
        // Update connection status to show client activity
        networkManager.updateConnectionStatus("Client Connected - USDZ Received")

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
        
        // Update connection status to show active pose streaming
        networkManager.updateConnectionStatus("Streaming Poses - \(poses.count) Bodies")

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
            
            // Remove previous entity from RealityView if it exists
            await MainActor.run {
                if let oldEntity = entity, let content = realityContent {
                    content.remove(oldEntity)
                    print("🧹 Removed previous entity from RealityView")
                }
            }
            
            entity = nil
            bodyEntities.removeAll()
            initialLocalTransforms.removeAll()

            // --- Load new model ---
            print("📦 Loading USDZ from \(url.absoluteString)")
            let loadedEntity = try await spatialGen.loadEntity(from: url)
            
            // Wrap or assign
            let newEntity: ModelEntity
            if let model = loadedEntity as? ModelEntity {
                newEntity = model
            } else {
                let wrapper = ModelEntity()
                wrapper.addChild(loadedEntity)
                newEntity = wrapper
            }
            
            // Add new entity to RealityView
            await MainActor.run {
                if let content = realityContent {
                    content.add(newEntity)
                    print("✅ Added new entity to RealityView: \(newEntity.name)")
                }
                entity = newEntity
            }
            
            indexBodyEntities(newEntity)
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
