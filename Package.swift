// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "MuJoCoARViewer",
    platforms: [
        .visionOS(.v1),
        .iOS(.v17),
        .macOS(.v15)
    ],
    dependencies: [
        .package(url: "https://github.com/grpc/grpc-swift-2.git", from: "2.0.0"),
        .package(url: "https://github.com/grpc/grpc-swift-protobuf.git", from: "2.0.0"),
    ],
    targets: [
        .target(
            name: "MuJoCoARViewer",
            dependencies: [
                .product(name: "GRPCCore", package: "grpc-swift-2"),
                .product(name: "GRPCNIOTransportHTTP2", package: "grpc-swift-nio-transport"),
                .product(name: "GRPCProtobuf", package: "grpc-swift-protobuf"),
            ]
        ),
        .testTarget(
            name: "MuJoCoARViewerTests",
            dependencies: ["MuJoCoARViewer"]
        ),
    ]
)
