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
            let localURL = try await download(from: url)
            entity = try await Entity(contentsOf: localURL)
        }

        // Store the entity in memory
        storedEntities[url] = entity

        return entity
    }

    private func download(from url: URL) async throws -> URL {
        let (data, response) = try await session.data(from: url)
        
        guard let httpResponse = response as? HTTPURLResponse, httpResponse.statusCode == 200 else {
            throw NSError(domain: "ARHelperErrorDomain", code: 1003, userInfo: [NSLocalizedDescriptionKey: "Failed to download file from the provided URL."])
        }
        
        let localURL = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString).appendingPathExtension("usdz")
        print(localURL)

        do {
            try data.write(to: localURL)
        } catch {
            throw NSError(domain: "ARHelperErrorDomain", code: 1004, userInfo: [NSLocalizedDescriptionKey: "Failed to write the downloaded data to a local file."])
        }

        return localURL
    }

}
