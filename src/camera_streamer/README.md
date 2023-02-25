1. Start video compression nodes for all cameras
    - `image_compression` and not `video_compression` because only want frame-by-frame
2. Subscribe to compressed video stream
3. Publish raw bytes over gRPC
4. (on UI) Read from gRPC and convert load the raw bytes into an image on the screen
