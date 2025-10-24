import SwiftUI
import RealityKit
import Network
import ARKit
//import Foundation
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
    // Use the previous app model solely for hand tracking; keep existing gRPC/server logic
    @StateObject private var appModel = 🥽AppModel()
    @State private var entity: ModelEntity?
    @State private var bodyEntities: [String: ModelEntity] = [:]
    @State private var usdzURL: String? = nil
    @State private var initialLocalTransforms: [String: Transform] = [:]
    // Cached 1:1 mapping from Python body names -> Swift RealityKit entity names
    @State private var pythonToSwiftNameMap: [String: String] = [:]
    @State private var nameMappingInitialized: Bool = false
    // Cached 1:N mapping: Python body name -> multiple Swift entity target names to move together
    @State private var pythonToSwiftTargets: [String: [String]] = [:]
    @State private var entityPathByObjectID: [ObjectIdentifier: String] = [:]
    @State private var statusEntity: Entity?
    @State private var statusContainerEntity: Entity?
    @State private var realityContent: RealityViewContent?
    @State private var inputPort: String = "50051"
    @State private var isMinimized: Bool = false
    @Namespace private var minimizeNS
    
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
                    ZStack(alignment: .topTrailing) {
                        if isMinimized {
                            // Minimized: a single circular button in the top-right
                            Button(action: {
                                withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                                    isMinimized = false
                                }
                            }) {
                                ZStack {
                                    Circle()
                                        .fill(.regularMaterial)
                                        .frame(width: 44, height: 44)
                                        .shadow(radius: 4)
                                    Image(systemName: "network")
                                        .foregroundColor(.blue)
                                }
                            }
                            .buttonStyle(.plain)
                            .matchedGeometryEffect(id: "statusMiniDot", in: minimizeNS)
                            .padding(6)
                        } else {
                            VStack(alignment: .leading, spacing: 12) {
                                HStack {
                                    Image(systemName: "network")
                                        .foregroundColor(.blue)
                                    Text("MuJoCo ARViewer")
                                        .font(.headline)
                                        .foregroundColor(.primary)
                                    Spacer()
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
                            .overlay(alignment: .topTrailing) {
                                HStack(spacing: 8) {

                                // Minimize control inside the window pane
                                Button(action: {
                                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                                        isMinimized = true
                                    }
                                }) {
                                    ZStack {
                                        Circle()
                                            .fill(.thinMaterial)
                                            .frame(width: 28, height: 28)
                                            .shadow(radius: 2)
                                        Image(systemName: "minus")
                                            .font(.system(size: 12, weight: .bold))
                                            .foregroundColor(.primary)
                                    }
                                }
                                .buttonStyle(.plain)
                                .matchedGeometryEffect(id: "statusMiniDot", in: minimizeNS)
                            
                            
                                    // Close (exit) control
                                    Button(action: {
                                        // Immediately terminate the app
                                        exit(0)
                                    }) {
                                        ZStack {
                                            Circle()
                                                .fill(.thinMaterial)
                                                .frame(width: 28, height: 28)
                                                .shadow(radius: 2)
                                            Image(systemName: "xmark")
                                                .font(.system(size: 12, weight: .bold))
                                                .foregroundColor(.red)
                                        }
                                    }
                                    .buttonStyle(.plain)
                                    .accessibilityLabel("Close App")
                                }

                            }
                            .frame(width: 250)
                            .padding(20)
                            .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
                            .glassBackgroundEffect()
                            .transition(.asymmetric(insertion: .opacity.combined(with: .scale),
                                                    removal: .opacity.combined(with: .move(edge: .top))))
                        }
                    }
                    .animation(.spring(response: 0.45, dampingFraction: 0.85), value: isMinimized)
                }
            }
            .onAppear {
                // Register the custom system and component
                StatusDisplayComponent.registerComponent()
                StatusDisplaySystem.registerSystem()
            }
            .onChange(of: isMinimized) { _, _ in
                // Smoothly move the status container when minimizing/restoring
                updateStatusContainerPosition(animated: true)
            }
        }
        .task {
            print("🚀 [ImmersiveView] Starting initialization...")
            
            // Just initialize network info, don't start server automatically
            networkManager.updateNetworkInfo()
            
            // Start hand tracking using the old, proven AppModel pipeline
            // run() internally starts ARKit session and processes hand updates
            
            // Legacy TCP server (can be removed once gRPC is working)
            // server.startListening()
        }
        .task {
            appModel.run()
        }
        .task {
            // Device/head tracking at ~90 Hz (non-blocking child task)
            await appModel.processDeviceAnchorUpdates()
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
        
        // Position the status display in the user's field of view
        // Default (expanded) location; we'll animate to top-right when minimized
        statusContainer.setPosition([0.0, 0.1, -0.8], relativeTo: headAnchor)
            
        // Add the container to the head anchor
        headAnchor.addChild(statusContainer)
        
        // Add the head anchor to the scene
        content.add(headAnchor)
        
        // Add the status component for periodic updates
        statusContainer.components.set(StatusDisplayComponent(networkManager: networkManager))
        
        // Store reference for cleanup
        statusEntity = headAnchor
        statusContainerEntity = statusContainer
            
        // Ensure initial position matches current minimized state without animation
        updateStatusContainerPosition(animated: false)
        
        print("✅ Created head-following status display")
    }

    // Smoothly update the status container position based on minimized state
    @MainActor
    private func updateStatusContainerPosition(animated: Bool) {
        guard let head = statusEntity, let container = statusContainerEntity else { return }
        let expandedPos = SIMD3<Float>(0.0, 0.1, -0.8)
        let minimizedPos = SIMD3<Float>(0.3, 0.3, -0.8)
        let target = isMinimized ? minimizedPos : expandedPos
        let transform = Transform(translation: target)
        if animated {
            // Animate the movement so users can see where it goes
            container.move(to: transform, relativeTo: head, duration: 0.4)
        } else {
            container.setPosition(target, relativeTo: head)
        }
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
                // Use full world transform to place the entity precisely, then restore local scale
                let prevLocalScale = entity.scale
                entity.setTransformMatrix(desiredWorld, relativeTo: nil)
                if let parent = entity.parent {
                    entity.setScale(prevLocalScale, relativeTo: parent)
                } else {
                    entity.setScale(prevLocalScale, relativeTo: nil)
                }
                appliedWorld[name] = desiredWorld
            } else {
                // Determine parent world to compute local matrix
                let parentEntity = entity.parent
                let parentWorld: simd_float4x4 = {
                    if let pe = parentEntity {
                        let pKey = entityPathByObjectID[ObjectIdentifier(pe)]
                        if let pk = pKey, let newParentWorld = transforms[pk] ?? appliedWorld[pk] {
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

                // Apply full local transform for precision, then restore local scale
                let prevLocalScale = entity.scale
                entity.setTransformMatrix(localMatrix, relativeTo: parentEntity)
                if let parent = parentEntity {
                    entity.setScale(prevLocalScale, relativeTo: parent)
                } else {
                    entity.setScale(prevLocalScale, relativeTo: nil)
                }

                appliedWorld[name] = desiredWorld
            }
        }
    }
    
    // Helper method to compute final transforms (called from gRPC)
    func computeFinalTransforms(_ poses: [String: MujocoAr_BodyPose]) -> [String: simd_float4x4] {
        // If entities aren't indexed yet, skip until ready to prevent bad mappings
        guard !bodyEntities.isEmpty, !initialLocalTransforms.isEmpty else {
            print("⏳ Entities not fully indexed yet; deferring pose application")
            return [:]
        }

        // Ensure name mapping is initialized once per model (only when entities exist)
        if !nameMappingInitialized {
            let pythonNames = Array(poses.keys)
            initializeNameMapping(pythonNames: pythonNames)
            // Only mark initialized if at least one mapping points to an existing entity
            let validMatches = pythonNames.reduce(0) { acc, py in
                if let swift = pythonToSwiftNameMap[py], bodyEntities[swift] != nil { return acc + 1 }
                return acc
            }
            if validMatches > 0 { nameMappingInitialized = true }
        }

        // Build expanded mapping to multiple Swift targets per python body when available
        func targets(for pyName: String) -> [String] {
            if let cached = pythonToSwiftTargets[pyName], !cached.isEmpty {
                return cached
            }
            // Derive from 1:1 map or find best
            var baseName: String? = pythonToSwiftNameMap[pyName]
            if baseName == nil || baseName!.isEmpty || bodyEntities[baseName!] == nil {
                if let best = findBestSwiftMatch(for: pyName) {
                    pythonToSwiftNameMap[pyName] = best
                    baseName = best
                }
            }
            guard let base = baseName, let baseEntity = bodyEntities[base] else {
                print("⚠️ [mapping] No base match for Python body '\(pyName)' — skipping multi-target mapping")
                return []
            }
            // Compute co-move targets: leaf ModelEntities directly under base, or one-hop via a same-named node
            var result: [String] = []
            for child in baseEntity.children {
                if let model = child as? ModelEntity {
                    if model.children.isEmpty, let key = entityPathByObjectID[ObjectIdentifier(model)] {
                        result.append(key)
                    }
                } else {
                    // Common pattern: importer creates a Node with the same name as base; look one level deeper
                    if child.name == baseEntity.name {
                        for grand in child.children {
                            if let leaf = grand as? ModelEntity, leaf.children.isEmpty,
                               let key = entityPathByObjectID[ObjectIdentifier(leaf)] {
                                result.append(key)
                            }
                        }
                    }
                }
            }
            pythonToSwiftTargets[pyName] = result
            let targetsList = result.joined(separator: ", ")
            print("🎯 [mapping] Python '\(pyName)' → Swift leafs with parent='\(base)' [\(targetsList)]")
            return result
        }

        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0)) // Z-up → Y-up
        var finalTransforms: [String: simd_float4x4] = [:]

        for (pyName, pose) in poses {
            let targetNames = targets(for: pyName)
            if targetNames.isEmpty {
                // Fall back to attempting pyName directly
                let single = pythonToSwiftNameMap[pyName] ?? pyName
                let names = [single]
                for bodyName in names {
                    guard initialLocalTransforms[bodyName] != nil else {
                        print("⚠️ Body '\(bodyName)' not found in initialLocalTransforms, skipping")
                        continue
                    }
                    // Compute final transform for this target using same pose
                    let computed = computeFinalTransformForTarget(bodyName: bodyName, pyName: pyName, pose: pose,
                                                                  axisCorrection: axisCorrection)
                    finalTransforms[bodyName] = computed
                }
                continue
            }
            for bodyName in targetNames {
                // Only compute transforms for targets that exist
                guard initialLocalTransforms[bodyName] != nil else {
                    print("⚠️ Body '\(bodyName)' not found in initialLocalTransforms, skipping")
                    continue
                }
                let computed = computeFinalTransformForTarget(bodyName: bodyName, pyName: pyName, pose: pose,
                                                              axisCorrection: axisCorrection)
                finalTransforms[bodyName] = computed
            }

            // per-target transforms already handled above
        }
        
        return finalTransforms
    }

    // Compute the final transform for a specific target name, using that target's local offset
    private func computeFinalTransformForTarget(bodyName: String,
                                                pyName: String,
                                                pose: MujocoAr_BodyPose,
                                                axisCorrection: simd_quatf) -> simd_float4x4 {
        let mjPos = SIMD3<Float>(pose.position.x, pose.position.y, pose.position.z)
        let mjRot = simd_quatf(ix: pose.rotation.x, iy: pose.rotation.y, iz: pose.rotation.z, r: pose.rotation.w)

        // Build MuJoCo world-space transform (ZUP)
        var mjWorldTransform = matrix_identity_float4x4
        mjWorldTransform = simd_mul(matrix_float4x4(mjRot), mjWorldTransform)
        mjWorldTransform.columns.3 = SIMD4<Float>(mjPos, 1.0)

        // Keep a pristine PS(X) before attach/axis for debug
        let psMatrix = mjWorldTransform

        // Apply attach_to offset BEFORE axis correction (ZUP)
        if let attachPos = attachToPosition, let attachRot = attachToRotation {
            var attachTransform = matrix_identity_float4x4
            attachTransform = simd_mul(matrix_float4x4(attachRot), attachTransform)
            attachTransform.columns.3 = SIMD4<Float>(attachPos, 1.0)
            mjWorldTransform = simd_mul(attachTransform, mjWorldTransform)
        }

        // Convert ZUP -> YUP
        let rkPos = axisCorrection.act(SIMD3<Float>(mjWorldTransform.columns.3.x, mjWorldTransform.columns.3.y, mjWorldTransform.columns.3.z))
        let mjRotFromMatrix = simd_quatf(mjWorldTransform)
        let rkRot = axisCorrection * mjRotFromMatrix

        // Build final RealityKit world transform
        var rkWorldTransform = matrix_identity_float4x4
        rkWorldTransform = simd_mul(matrix_float4x4(rkRot), rkWorldTransform)
        rkWorldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)

        // Option (1): Apply extra transformation on top of Y-side parent (X) transform, then Y's local
        // final = pose(X) @ local(X on Y-side) @ local(Y)
        // Where:
        //  - pose(X) is rkWorldTransform (axis-corrected + attach_to applied)
        //  - local(X on Y-side) is the imported local transform of Y's parent (e.g., 'torso_link')
        //  - local(Y) is the imported local transform of the Y leaf itself
        let parentEntity = bodyEntities[bodyName]?.parent
        let parentKey = parentEntity.flatMap { entityPathByObjectID[ObjectIdentifier($0)] }
        let parentLocal = (parentKey.flatMap { initialLocalTransforms[$0]?.matrix }) ?? matrix_identity_float4x4
        let yLocal = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4

        // Combine per option (1)
        let finalTransform = rkWorldTransform * yLocal

        return finalTransform
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
            pythonToSwiftNameMap.removeAll()
            nameMappingInitialized = false
            pythonToSwiftTargets.removeAll()
            entityPathByObjectID.removeAll()
            
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

    // MARK: - Name Mapping (Python -> Swift)
    private func initializeNameMapping(pythonNames: [String]) {
        // Build a 1:1 mapping based on substring containment with minimal length difference
        // Use sanitized, lowercase comparisons; store actual Swift names in the map
        let swiftNames = Array(bodyEntities.keys)
        let sanitizedSwiftPairs: [(original: String, sanitized: String)] = swiftNames.map { ($0, sanitizeName($0)) }
        var usedSwift: Set<String> = []

        func bestMatch(for pyName: String) -> String? {
            return findBestSwiftMatch(for: pyName, sanitizedSwiftPairs: sanitizedSwiftPairs, excluding: usedSwift)
        }

        var newMap: [String: String] = [:]
        for py in pythonNames {
            if let match = bestMatch(for: py) {
                newMap[py] = match
                usedSwift.insert(match)
            } else {
                // No containment match found; leave unmapped for fallback/direct use
                newMap[py] = py
            }
        }

        pythonToSwiftNameMap = newMap

        // Log summary
        print("🔤 Built name mapping (Python → Swift):")
        for (k, v) in newMap.sorted(by: { $0.key < $1.key }) {
            print("   \(k)  →  \(v)")
        }
    }

    private func sanitizeName(_ s: String) -> String {
        let lowered = s.lowercased()
        let filtered = lowered.unicodeScalars.filter { CharacterSet.alphanumerics.contains($0) }
        return String(String.UnicodeScalarView(filtered))
    }

    // Reusable best-match finder (contains + minimal delta). When sanitized pairs provided, uses them; else builds from bodyEntities.
    private func findBestSwiftMatch(for pyName: String,
                                    sanitizedSwiftPairs: [(original: String, sanitized: String)]? = nil,
                                    excluding used: Set<String> = []) -> String? {
        let pairs: [(original: String, sanitized: String)]
        if let provided = sanitizedSwiftPairs {
            pairs = provided
        } else {
            let swiftNames = Array(bodyEntities.keys)
            pairs = swiftNames.map { ($0, sanitizeName($0)) }
        }

        let pySan = sanitizeName(pyName)
        var best: (swiftOriginal: String, delta: Int)? = nil
        for (orig, san) in pairs where !used.contains(orig) {
            if san.contains(pySan) {
                let delta = max(0, san.count - pySan.count)
                if let current = best {
                    if delta < current.delta || (delta == current.delta && orig.count < current.swiftOriginal.count) {
                        best = (orig, delta)
                    }
                } else {
                    best = (orig, delta)
                }
            }
        }
        return best?.swiftOriginal
    }
    
    // Build a human-readable scene hierarchy string for saving/logging
    private func sceneHierarchyString(from root: Entity) -> String {
        var lines: [String] = []
        func walk(_ entity: Entity, depth: Int) {
            let indent = String(repeating: "  ", count: depth)
            let name = entity.name.isEmpty ? "<unnamed>" : entity.name
            let comps = entity.components.count
            let childCount = entity.children.count
            lines.append("\(indent)- \(name) [components: \(comps), children: \(childCount)]")
            for child in entity.children {
                walk(child, depth: depth + 1)
            }
        }
        walk(root, depth: 0)
        return lines.joined(separator: "\n")
    }
    
    
    private func indexBodyEntities(_ rootEntity: Entity) {
        bodyEntities.removeAll()
        initialLocalTransforms.removeAll()
        entityPathByObjectID.removeAll()
        print("🔍 [indexBodyEntities] Starting recursive indexing...")

        func entityDepth(_ entity: Entity) -> Int {
            var d = 0
            var p = entity.parent
            while let parent = p {
                d += 1
                p = parent.parent
            }
            return d
        }

        func printHierarchy(from entity: Entity, depth: Int = 0) {
            let indent = String(repeating: "  ", count: depth)
            let typeLabel = (entity as? ModelEntity) != nil ? "Model" : "Node"
            let parentName = entity.parent?.name ?? "(root)"
            print("📂 \(indent)- [\(typeLabel)] '\(entity.name)' parent='\(parentName)' children=\(entity.children.count)")
            for child in entity.children {
                printHierarchy(from: child, depth: depth + 1)
            }
        }

        func indexRec(_ entity: Entity, parentPath: String) {
            // Skip unnamed entities for keying, but still recurse
            if entity.name.isEmpty {
                for child in entity.children {
                    indexRec(child, parentPath: parentPath)
                }
                return
            }

            if let modelEntity = entity as? ModelEntity {
                let pathKey = parentPath.isEmpty ? modelEntity.name : "\(parentPath)/\(modelEntity.name)"
                bodyEntities[pathKey] = modelEntity
                initialLocalTransforms[pathKey] = modelEntity.transform
                entityPathByObjectID[ObjectIdentifier(modelEntity)] = pathKey

                let parentName = modelEntity.parent?.name ?? "(root)"
                let depth = entityDepth(modelEntity)
                print("📎 [hier] \(String(repeating: "  ", count: depth))- [Model] '\(modelEntity.name)' parent='\(parentName)' children=\(modelEntity.children.count)")
                print("✅ Added ModelEntity: '\(modelEntity.name)' \(modelEntity.scale)")

                for child in modelEntity.children {
                    indexRec(child, parentPath: pathKey)
                }
            } else {
                // Insert a wrapper to ensure we can drive transforms for non-Model entities
                let wrapper = ModelEntity()
                wrapper.name = entity.name
                wrapper.transform = entity.transform
                let preservedScale = entity.scale

                // Capture the original children before reparenting
                let originalChildren = Array(entity.children)

                let originalParentName = entity.parent?.name ?? "(root)"
                if let parent = entity.parent {
                    parent.addChild(wrapper)
                    entity.removeFromParent()
                    wrapper.addChild(entity)
                    // wrapper.setScale(preservedScale, relativeTo: parent)
                } else {
                    wrapper.scale = preservedScale
                }

                let pathKey = parentPath.isEmpty ? wrapper.name : "\(parentPath)/\(wrapper.name)"
                bodyEntities[pathKey] = wrapper
                initialLocalTransforms[pathKey] = wrapper.transform
                entityPathByObjectID[ObjectIdentifier(wrapper)] = pathKey

                let depth = entityDepth(wrapper)
                print("📎 [hier] \(String(repeating: "  ", count: depth))- [Wrapper] '\(wrapper.name)' inserted between parent='\(originalParentName)' and child='\(entity.name)'")
                print("✅ Added wrapper: '\(wrapper.name)' (cached local offset) \(wrapper.scale)")

                // Recurse into the original node's children under the wrapper's path (avoid re-wrapping the original node)
                for child in originalChildren {
                    indexRec(child, parentPath: pathKey)
                }
            }
        }

        indexRec(rootEntity, parentPath: "")
        print("📝 Indexed \(bodyEntities.count) entities with cached local transforms")
        print("📑 Scene Hierarchy:")
        printHierarchy(from: rootEntity, depth: 0)

        // Build and save the scene hierarchy to a file
        let hierarchyText = sceneHierarchyString(from: rootEntity)
        let formatter = ISO8601DateFormatter()
        var timestamp = formatter.string(from: Date())
        // Make filename-safe
        timestamp = timestamp.replacingOccurrences(of: ":", with: "-")
        let filename = "SceneHierarchy-\(timestamp).txt"
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent(filename)
        do {
            try hierarchyText.write(to: fileURL, atomically: true, encoding: .utf8)
            print("💾 Scene hierarchy saved to: \(fileURL.path)")
        } catch {
            print("❌ Failed to save scene hierarchy: \(error)")
        }
    }
    
    
    // MARK: - Hand Tracking Methods
    
    func getHandTrackingData() -> MujocoAr_HandTrackingUpdate {
        // Build the protobuf from the DataManager-backed AppModel state
        var update = MujocoAr_HandTrackingUpdate()
        let data = DataManager.shared.latestHandTrackingData

        // Timestamp
        update.timestamp = Date().timeIntervalSince1970

        // Wrist matrices
        update.leftHand.wristMatrix = convertToMatrix4x4(from: data.leftWrist)
        update.rightHand.wristMatrix = convertToMatrix4x4(from: data.rightWrist)

        // Skeleton joints (fixed size 25 per AppModel.Skeleton)
        update.leftHand.skeleton.jointMatrices = data.leftSkeleton.joints.map { convertToMatrix4x4(from : $0) }
        update.rightHand.skeleton.jointMatrices = data.rightSkeleton.joints.map { convertToMatrix4x4(from : $0) }

        return update
    }
    
}

func convertToMatrix4x4(from jointMatrix: simd_float4x4) -> MujocoAr_Matrix4x4 {
    var matrix = MujocoAr_Matrix4x4()
    matrix.m00 = Float(jointMatrix.columns.0.x)
    matrix.m01 = Float(jointMatrix.columns.1.x)
    matrix.m02 = Float(jointMatrix.columns.2.x)
    matrix.m03 = Float(jointMatrix.columns.3.x)
    matrix.m10 = Float(jointMatrix.columns.0.y)
    matrix.m11 = Float(jointMatrix.columns.1.y)
    matrix.m12 = Float(jointMatrix.columns.2.y)
    matrix.m13 = Float(jointMatrix.columns.3.y)
    matrix.m20 = Float(jointMatrix.columns.0.z)
    matrix.m21 = Float(jointMatrix.columns.1.z)
    matrix.m22 = Float(jointMatrix.columns.2.z)
    matrix.m23 = Float(jointMatrix.columns.3.z)
    matrix.m30 = Float(jointMatrix.columns.0.w)
    matrix.m31 = Float(jointMatrix.columns.1.w)
    matrix.m32 = Float(jointMatrix.columns.2.w)
    matrix.m33 = Float(jointMatrix.columns.3.w)
    return matrix
}
