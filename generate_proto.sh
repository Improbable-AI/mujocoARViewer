#!/bin/bash

# Generate Python gRPC stubs
python -m grpc_tools.protoc \
    --proto_path=proto \
    --python_out=. \
    --grpc_python_out=. \
    proto/mujoco_ar.proto

echo "Python gRPC stubs generated successfully!"