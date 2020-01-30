# Segmentation Node

Before running this node, you need to make sure you have camera topics available. You can either run `bash ./camera.sh` to launch a physical camera or use rosbag to replay.

Run the segmentation node (publish to /seg/raw)

```bash
source ./devel/setup.sh
rosrun segmentation segmentation_node
```

Run the segmentation visualization (publish to /seg/visual)

```bash
source ./devel/setup.sh
rosrun segmentation vis_seg.py
```

## Compile Protobuf to Python Classes

```bash
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. jetsonrpc.proto
```