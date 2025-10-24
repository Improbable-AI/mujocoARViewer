import SwiftUI
import ARKit
import RealityKit
import simd
import QuartzCore

struct Skeleton {
    var joints: [simd_float4x4]

    init() {
        // Initialize the joints array with 24 identity matrices
        self.joints = Array(repeating: simd_float4x4(1), count: 25)
    }
}

struct HandTrackingData {
    var leftWrist: simd_float4x4 = simd_float4x4(1)
    var rightWrist: simd_float4x4 = simd_float4x4(1)
    var leftSkeleton: Skeleton = Skeleton()
    var rightSkeleton: Skeleton = Skeleton()
    var Head: simd_float4x4 = simd_float4x4(1)
}

class DataManager {
    static let shared = DataManager()
    
    var latestHandTrackingData: HandTrackingData = HandTrackingData()
    
    private init() {}
}


@MainActor
class 🥽AppModel: ObservableObject {
    @Published private(set) var authorizationStatus: ARKitSession.AuthorizationStatus?
    
    private let session = ARKitSession()
    private let handTracking = HandTrackingProvider()
    private let worldTracking = WorldTrackingProvider()
    private let sceneReconstruction = SceneReconstructionProvider()

}

extension 🥽AppModel {
    
    func run() {
#if targetEnvironment(simulator)
        print("Not support handTracking in simulator.")
#else
        
        Task {
            @MainActor in
            do {
                try await self.session.run([self.handTracking, self.worldTracking, self.sceneReconstruction])
                await self.processHandUpdates();
            } catch {
                print(error)
            }
        }
#endif
    }

    // gRPC server is managed elsewhere (ImmersiveView); no-op here
}

extension 🥽AppModel {
    
    @MainActor
    func run_device_tracking(function: () async -> Void, withFrequency hz: UInt64) async {
        while true {
            if Task.isCancelled {
                return
            }
            
            // Sleep for 1 s / hz before calling the function.
            let nanoSecondsToSleep: UInt64 = NSEC_PER_SEC / hz
            do {
                try await Task.sleep(nanoseconds: nanoSecondsToSleep)
            } catch {
                // Sleep fails when the Task is cancelled. Exit the loop.
                return
            }
            
            await function()
        }
    }

    @MainActor
    func processDeviceAnchorUpdates() async {
        await run_device_tracking(function: self.queryAndProcessLatestDeviceAnchor, withFrequency: 90)
    }
    
    func processReconstructionUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            print("reconstruction update")
            let meshAnchor = update.anchor
            let mesh_description = meshAnchor.geometry.description
            print(mesh_description)
        }
    }

    
    @MainActor
    private func queryAndProcessLatestDeviceAnchor() async {
        // Device anchors are only available when the provider is running.\
        guard worldTracking.state == .running else { return }
        
        let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime())
        print(" *** device tracking running ")
//        print(deviceAnchor?.originFromAnchorTransform)
        guard let deviceAnchor, deviceAnchor.isTracked else { return }
        DataManager.shared.latestHandTrackingData.Head = deviceAnchor.originFromAnchorTransform
            }

    private func processHandUpdates() async {
        for await update in self.handTracking.anchorUpdates {
            let handAnchor = update.anchor
            print("processHandUpates is running.")
            switch handAnchor.chirality {
            case .left:
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                    print(handAnchor.originFromAnchorTransform)
                    

                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
                        .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                        .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                        .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                        .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                        .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                    ]
                    
                    for (index, jointType) in jointTypes.enumerated() {
                        let joint = handAnchor.handSkeleton?.joint(jointType)
                        guard let joint = handAnchor.handSkeleton?.joint(jointType) else {
                            continue
                        }
                        DataManager.shared.latestHandTrackingData.leftSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                    
                    print("Updated left hand skeleton")
                    // Repeat for right hand and other fingers as needed
                }

            case .right:
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                    print(handAnchor.originFromAnchorTransform)
                    
                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
                        .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                        .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                        .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                        .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                        .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                    ]
 
                    for (index, jointType) in jointTypes.enumerated() {
                        let joint = handAnchor.handSkeleton?.joint(jointType)
                        guard let joint = handAnchor.handSkeleton?.joint(jointType) else {
                            continue
                        }
                        DataManager.shared.latestHandTrackingData.rightSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                    
                    print("Updated right hand skeleton")
                }
            }
            
        }
        
        
    }
}

