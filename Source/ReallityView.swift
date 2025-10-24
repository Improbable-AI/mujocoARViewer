//
//  ReallityView.swift
//  mujocoARViewer
//
//  Created by younghyopark on 10/23/25.
//

import SwiftUI
import RealityKit
import ARKit

struct 🌐RealityView: View {
    var model: 🥽AppModel
    var body: some View {
        RealityView { content, attachments in
            let resultLabelEntity = attachments.entity(for: Self.attachmentID)!
        } attachments: {
            Attachment(id: Self.attachmentID) {
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
        )
        .task { self.model.run() }
        .task { await self.model.processDeviceAnchorUpdates() }
        .task(priority: .low) { await self.model.processReconstructionUpdates() }
    }
    static let attachmentID: String = "resultLabel"
}
