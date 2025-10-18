#!/bin/bash

# Generate Swift gRPC stubs
# Note: You need to install protoc-gen-swift and protoc-gen-grpc-swift first
# Install via: brew install swift-protobuf grpc-swift

PROTO_DIR="proto"
OUTPUT_DIR="ImmersiveMoveAndRotate/Generated"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Generate Swift protobuf files
protoc \
    --proto_path="$PROTO_DIR" \
    --swift_out="$OUTPUT_DIR" \
    --grpc-swift_out="$OUTPUT_DIR" \
    "$PROTO_DIR/mujoco_ar.proto"

echo "Swift gRPC stubs generated successfully in $OUTPUT_DIR!"
echo "Don't forget to add these files to your Xcode project."