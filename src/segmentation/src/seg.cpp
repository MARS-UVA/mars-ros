#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#include <segNet.h>
#include <iostream>
#include "cudaMappedMemory.h"
#include "cudaRGB.h"
using namespace std;

segNet* net;

uint8_t* imgMask;
uchar4* cudaImgRGB;
float* cudaImgRGBA;

const int width = 640;
const int height = 480;

void initializeNet() {
    net = segNet::Create(segNet::FCN_RESNET18_CITYSCAPES_1024x512, 1);

    if (!net) {
        printf("segnet-camera:   failed to initialize imageNet\n");
    }
    net->SetOverlayAlpha(255);

    bool r = cudaAllocMapped((void**)&imgMask, width * height);
    r = r && cudaAllocMapped((void**)&cudaImgRGB, width * height * sizeof(uchar4));
    r = r && cudaAllocMapped((void**)&cudaImgRGBA, width * height * sizeof(float4));

    if (!r) {
        printf("segnet-camera:  failed to allocate CUDA memory for mask image\n");
        return;
    }
    printf("Memory allocated!\n");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    assert(msg->width == width);
    assert(msg->height == height);
    assert(msg->encoding == "rgb8");
    
    memcpy(cudaImgRGB, msg->data.data(), width * height * sizeof(uchar3));
    if (CUDA_FAILED(cudaRGB8ToRGBA32((uchar3*)cudaImgRGB, (float4*)cudaImgRGBA, width, height))) {
        ROS_ERROR("failed to convert %ux%u image with CUDA", width, height);
        return;
    }
    if (!net->Process(cudaImgRGBA, width, height)) {
        ROS_ERROR("fail to process image");
        return;
    }
    if (!net->Mask(imgMask, width, height)) {
        ROS_ERROR("fail to mask image");
        return;
    }
}

int main(int argc, char** argv) {
    initializeNet();

    ros::init(argc, argv, "image_listener");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
    ros::spin();

    printf("shutting down...\n");
    SAFE_DELETE(net);
}