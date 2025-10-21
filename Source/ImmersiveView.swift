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
    
    // Model attachment offset state (ZUP coordinates)
    @State private var attachToPosition: SIMD3<Float>? = nil
    @State private var attachToRotation: simd_quatf? = nil
    
    // Pose update trigger - stores final transformed matrices ready to apply
    @State private var finalTransforms: [String: simd_float4x4] = [:]
    @State private var poseUpdateTrigger: UUID = UUID()
    
    // Hand tracking state
    @State private var handTrackingData: MujocoAr_HandTrackingUpdate = MujocoAr_HandTrackingUpdate()
    @State private var arkitSession = ARKitSession()
    @State private var handTracking = HandTrackingProvider()
    @State private var worldTracking = WorldTrackingProvider()
    
    let spatialGen = SpatialGenHelper()
    // If true, apply transforms directly in world (ground) space using parent-first ordering.
    // If false, compute local transforms relative to the parent for order-independent application.
    private let applyInWorldSpace: Bool = true
    

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
                
            } update: { updateContent, updatedAttachments in
                // This is called whenever @State variables change
                // Use this for pose updates and model changes
                
                // Handle model updates
                if let newEntity = entity {
                    // Check if this entity is already in the scene
                    let existingEntities = updateContent.entities
                    let entityExists = existingEntities.contains { $0.id == newEntity.id }
                    
                    if !entityExists {
                        // Remove any previous model entities (keep status display)
                        for existingEntity in existingEntities {
                            if existingEntity.name != "statusHeadAnchor" {
                                updateContent.remove(existingEntity)
                            }
                        }
                        
                        // Add the new entity
                        updateContent.add(newEntity)
                        print("✅ [RealityView.update] Added new entity: \(newEntity.name)")
                    }
                }
                
                // Handle pose updates through the update mechanism - DON'T modify state here
                if !finalTransforms.isEmpty {
                    print("🔄 [RealityView.update] Applying \(finalTransforms.count) final transforms")
                    // Apply transforms without modifying state
                    applyFinalTransforms(finalTransforms)
                    print("✅ [RealityView.update] Applied transforms successfully")
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
            
            // Start hand tracking
            await startHandTracking()
            
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
        .onChange(of: poseUpdateTrigger) { _, _ in
            // Clear transforms after they've been applied
            // This happens outside the update block to avoid state modification warnings
            Task {
                try? await Task.sleep(nanoseconds: 1_000_000) // 1ms delay to ensure update block ran
                finalTransforms = [:]
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
        statusContainer.setPosition([0.0, 0.1, -0.8], relativeTo: headAnchor)
        
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
    func updateUsdzURL(_ url: String, attachToPosition: SIMD3<Float>? = nil, attachToRotation: simd_quatf? = nil) {
        print("🔄 [updateUsdzURL] Called with URL: \(url)")
        
        // Store the attach_to offset
        self.attachToPosition = attachToPosition
        self.attachToRotation = attachToRotation
        
        if let position = attachToPosition, let rotation = attachToRotation {
            print("📍 [updateUsdzURL] Attach to position: \(position), rotation: \(rotation)")
        } else {
            print("📍 [updateUsdzURL] No attach_to offset specified, using origin")
        }
        
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
    
    func updatePosesWithTransforms(_ transforms: [String: simd_float4x4]) {
        print("🔄 [updatePosesWithTransforms] Called with \(transforms.count) final transforms")
        
        // Update connection status to show active pose streaming
        networkManager.updateConnectionStatus("Streaming Poses - \(transforms.count) Bodies")

        // Always update - let SwiftUI handle the update scheduling
        finalTransforms = transforms
        poseUpdateTrigger = UUID()
    }
    
    private func applyFinalTransforms(_ transforms: [String: simd_float4x4]) {
        // Apply transforms in a hierarchy-safe way to avoid order-dependent "wave" artifacts.
        // 1) Compute application order by depth (parents first)
        // 2) Convert desired world transforms to local transforms relative to the current/new parent world
        // 3) Apply local transforms relative to parent so updates are atomic per frame

        // Cache for depths to avoid recomputation
        var depthCache: [String: Int] = [:]

        func depth(for name: String) -> Int {
            if let d = depthCache[name] { return d }
            guard let entity = bodyEntities[name] else { depthCache[name] = 0; return 0 }
            var d = 0
            var p = entity.parent
            while let parent = p {
                d += 1
                p = parent.parent
            }
            depthCache[name] = d
            return d
        }

        // Sort keys by increasing depth so parents update before children
        let sortedNames = transforms.keys.sorted { lhs, rhs in depth(for: lhs) < depth(for: rhs) }

        // Helper to get current world matrix of an entity (fallback when parent's new world not provided)
        func currentWorld(of entity: Entity) -> simd_float4x4 {
            entity.transformMatrix(relativeTo: nil)
        }

        // Keep a map of newly applied world transforms so children can reference parent's new world
        var appliedWorld: [String: simd_float4x4] = [:]

        for name in sortedNames {
            guard let entity = bodyEntities[name], let desiredWorld = transforms[name] else { continue }

            if applyInWorldSpace {
                // Apply directly in world (ground) space. Because we update parents first, the
                // computed local transform RealityKit derives will yield the exact desired world.
                entity.setTransformMatrix(desiredWorld, relativeTo: nil)
                appliedWorld[name] = desiredWorld
            } else {
                // Determine parent world to compute local matrix
                let parentEntity = entity.parent
                let parentWorld: simd_float4x4 = {
                    if let pe = parentEntity, !pe.name.isEmpty {
                        if let newParentWorld = transforms[pe.name] ?? appliedWorld[pe.name] {
                            return newParentWorld
                        } else {
                            return currentWorld(of: pe)
                        }
                    } else {
                        return matrix_identity_float4x4
                    }
                }()

                // Convert world -> local relative to parent
                let localMatrix = simd_mul(parentWorld.inverse, desiredWorld)
                entity.setTransformMatrix(localMatrix, relativeTo: parentEntity)
                appliedWorld[name] = desiredWorld
            }
        }
    }
    
    // Helper method to compute final transforms (called from gRPC)
    func computeFinalTransforms(_ poses: [String: MujocoAr_BodyPose]) -> [String: simd_float4x4] {
        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0)) // Z-up → Y-up
        var finalTransforms: [String: simd_float4x4] = [:]

        for (bodyName, pose) in poses {
            // Only compute transforms for bodies that exist
            guard initialLocalTransforms[bodyName] != nil else {
                print("⚠️ Body '\(bodyName)' not found in initialLocalTransforms, skipping")
                continue
            }

            let mjPos = SIMD3<Float>(pose.position.x, pose.position.y, pose.position.z)
            let mjRot = simd_quatf(ix: pose.rotation.x, iy: pose.rotation.y, iz: pose.rotation.z, r: pose.rotation.w)

            // Build MuJoCo world-space transform (still in ZUP)
            var mjWorldTransform = matrix_identity_float4x4
            mjWorldTransform = simd_mul(matrix_float4x4(mjRot), mjWorldTransform)
            mjWorldTransform.columns.3 = SIMD4<Float>(mjPos, 1.0)
            
            // Apply attach_to offset BEFORE axis correction (both in ZUP coordinates)
            if let attachPos = attachToPosition, let attachRot = attachToRotation {
                // Create attach_to transform matrix (in ZUP)
                var attachTransform = matrix_identity_float4x4
                attachTransform = simd_mul(matrix_float4x4(attachRot), attachTransform)
                attachTransform.columns.3 = SIMD4<Float>(attachPos, 1.0)
                
                // Apply attach_to offset: attach_to_transform * mujoco_transform (both in ZUP)
                mjWorldTransform = simd_mul(attachTransform, mjWorldTransform)
            }
            
            // Now convert the combined transform from ZUP to RealityKit YUP
            let rkPos = axisCorrection.act(SIMD3<Float>(mjWorldTransform.columns.3.x, mjWorldTransform.columns.3.y, mjWorldTransform.columns.3.z))
            let mjRotFromMatrix = simd_quatf(mjWorldTransform)
            let rkRot = axisCorrection * mjRotFromMatrix
            
            // Build final RealityKit world transform
            var rkWorldTransform = matrix_identity_float4x4
            rkWorldTransform = simd_mul(matrix_float4x4(rkRot), rkWorldTransform)
            rkWorldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)
            
            // Retrieve the initial local transform offset (geometry alignment)
            let localOffset = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4
            
            // define inverse of local Offset
            let invlocalOffset = localOffset.inverse

            // Combine: RealityKit_world * local_offset
            let finalTransform = simd_mul(rkWorldTransform, invlocalOffset)
            
            finalTransforms[bodyName] = finalTransform
        }
        
        return finalTransforms
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
            
            // Note: attachToPosition and attachToRotation are preserved from the send_model call
            // They are only reset when explicitly sending a new model without attach_to

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
    
    
    // MARK: - Hand Tracking Methods
    
    func getHandTrackingData() -> MujocoAr_HandTrackingUpdate {
        return handTrackingData
    }
    
    private func startHandTracking() async {
        print("🖐️ [startHandTracking] Initializing hand tracking...")
        
        do {
            if HandTrackingProvider.isSupported {
                print("🖐️ [startHandTracking] Hand tracking is supported")
                try await arkitSession.run([handTracking, worldTracking])
                print("🖐️ [startHandTracking] ARKit session started")
                
                // Start processing hand updates
                Task {
                    await processHandUpdates()
                }
            } else {
                print("⚠️ [startHandTracking] Hand tracking not supported on this device")
            }
        } catch {
            print("❌ [startHandTracking] Failed to start hand tracking: \(error)")
        }
    }
    
    private func processHandUpdates() async {
        print("🖐️ [processHandUpdates] Starting hand tracking update loop")
        
        for await update in handTracking.anchorUpdates {
            let handAnchor = update.anchor
            print("processHandUpdates is running.")
            
            guard handAnchor.isTracked else { continue }
            
            switch handAnchor.chirality {
            case .left:
                DispatchQueue.main.async {
                    self.updateLeftHandData(handAnchor: handAnchor)
                }
            case .right:
                DispatchQueue.main.async {
                    self.updateRightHandData(handAnchor: handAnchor)
                }
            @unknown default:
                break
            }
        }
    }
    
    private func updateLeftHandData(handAnchor: HandAnchor) {
        // Update wrist transform
        handTrackingData.leftHand.wristMatrix = convertToMatrix4x4(handAnchor.originFromAnchorTransform)
        print(handAnchor.originFromAnchorTransform)
        
        // Define the joint types in the specific order
        let jointTypes: [HandSkeleton.JointName] = [
            .wrist,
            .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
            .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
            .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
            .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
            .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
        ]
        
        // Clear and rebuild joint matrices
        handTrackingData.leftHand.skeleton.jointMatrices.removeAll()
        
        for (index, jointType) in jointTypes.enumerated() {
            guard let joint = handAnchor.handSkeleton?.joint(jointType), joint.isTracked else {
                // Add identity matrix for missing joints to maintain index consistency
                handTrackingData.leftHand.skeleton.jointMatrices.append(convertToMatrix4x4(matrix_identity_float4x4))
                continue
            }
            handTrackingData.leftHand.skeleton.jointMatrices.append(convertToMatrix4x4(joint.anchorFromJointTransform))
        }
        
        // Update timestamp
        handTrackingData.timestamp = Date().timeIntervalSince1970
        print("Updated left hand skeleton")
    }
    
    private func updateRightHandData(handAnchor: HandAnchor) {
        // Update wrist transform
        handTrackingData.rightHand.wristMatrix = convertToMatrix4x4(handAnchor.originFromAnchorTransform)
        print(handAnchor.originFromAnchorTransform)
        
        // Define the joint types in the specific order
        let jointTypes: [HandSkeleton.JointName] = [
            .wrist,
            .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
            .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
            .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
            .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
            .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
        ]
        
        // Clear and rebuild joint matrices
        handTrackingData.rightHand.skeleton.jointMatrices.removeAll()
        
        for (index, jointType) in jointTypes.enumerated() {
            guard let joint = handAnchor.handSkeleton?.joint(jointType), joint.isTracked else {
                // Add identity matrix for missing joints to maintain index consistency
                handTrackingData.rightHand.skeleton.jointMatrices.append(convertToMatrix4x4(matrix_identity_float4x4))
                continue
            }
            print(index)
            handTrackingData.rightHand.skeleton.jointMatrices.append(convertToMatrix4x4(joint.anchorFromJointTransform))
        }
        
        // Update timestamp
        handTrackingData.timestamp = Date().timeIntervalSince1970
        print("Updated right hand skeleton")
    }
    
    private func convertToMatrix4x4(_ matrix: simd_float4x4) -> MujocoAr_Matrix4x4 {
        var result = MujocoAr_Matrix4x4()
        // Correct matrix layout - columns should be accessed properly
        result.m00 = matrix.columns.0.x
        result.m01 = matrix.columns.1.x
        result.m02 = matrix.columns.2.x
        result.m03 = matrix.columns.3.x
        result.m10 = matrix.columns.0.y
        result.m11 = matrix.columns.1.y
        result.m12 = matrix.columns.2.y
        result.m13 = matrix.columns.3.y
        result.m20 = matrix.columns.0.z
        result.m21 = matrix.columns.1.z
        result.m22 = matrix.columns.2.z
        result.m23 = matrix.columns.3.z
        result.m30 = matrix.columns.0.w
        result.m31 = matrix.columns.1.w
        result.m32 = matrix.columns.2.w
        result.m33 = matrix.columns.3.w
        return result
    }
}

