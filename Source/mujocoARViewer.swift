import SwiftUI

@main
struct mujocoARViewerApp: App {
    var body: some Scene {
        ImmersiveSpace {
            ImmersiveView()
        }.immersionStyle(selection: .constant(.mixed), in: .mixed)
    }
}
