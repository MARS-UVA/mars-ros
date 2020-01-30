# gRPC server

Run this node:

```bash
source ./devel/setup.sh
rosrun rpc-server grpc-server
```

## Compile Protobuf to Python Classes

```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. jetsonrpc.proto
```