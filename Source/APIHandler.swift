import Foundation
import RealityKit
import Observation

class SpatialGenHelper {

    @MainActor
    private var storedEntities: [URL: Entity] = [:]
    private let session: URLSession = .shared

    @MainActor
    func loadEntity(from url: URL) async throws -> Entity {
        // Check if entity is already stored in memory
        if let existingEntity = storedEntities[url] {
            return existingEntity as! Entity
        }

        let entity: Entity

        if url.isFileURL {
            guard FileManager.default.fileExists(atPath: url.path) else {
                throw NSError(domain: "ARHelperErrorDomain", code: 1002, userInfo: [NSLocalizedDescriptionKey: "No file exists at the provided URL."])
            }
            entity = try await Entity(contentsOf: url)
        } else {
            // For non-file URLs, we expect the USDZ data to be delivered via gRPC
            // This method now only handles file URLs since gRPC will provide local files
            throw NSError(domain: "ARHelperErrorDomain", code: 1005, userInfo: [NSLocalizedDescriptionKey: "Non-file URLs should be handled via gRPC data transfer."])
        }

        // Store the entity in memory
        storedEntities[url] = entity

        return entity
    }
    
    @MainActor
    func loadEntity(fromLocalPath path: String) async throws -> Entity {
        let url = URL(fileURLWithPath: path)
        return try await loadEntity(from: url)
    }

}
