#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <segNet.h>
#include "cudaMappedMemory.h"
#include "cudaRGB.h"

segNet* net;

uint8_t* cudaImgMask;
uchar4* cudaImgRGB;
float* cudaImgRGBA;

int width = 640;
int height = 480;

image_transport::Publisher pub;

void initializeNet() {
    net = segNet::Create(segNet::FCN_RESNET18_CITYSCAPES_1024x512, 1);
    if (!net) {
        ROS_ERROR("Failed to initialize segNet!");
        return;
    }
    net->SetOverlayAlpha(255);
    ROS_INFO("Network initialzed!");
}

void allocateImgMemory() {
    if (cudaImgMask != NULL) cudaFreeHost(cudaImgMask);
    if (cudaImgRGB != NULL) cudaFreeHost(cudaImgRGB);
    if (cudaImgRGBA != NULL) cudaFreeHost(cudaImgRGBA);

    bool r = cudaAllocMapped((void**)&cudaImgMask, width * height);
    r = r && cudaAllocMapped((void**)&cudaImgRGB, width * height * sizeof(uchar4));
    r = r && cudaAllocMapped((void**)&cudaImgRGBA, width * height * sizeof(float4));

    if (!r) {
        ROS_ERROR("Failed to allocate CUDA memory!");
        return;
    }
    ROS_INFO("Memory allocated!");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // don't do anything if there're no subscribers
    if (pub.getNumSubscribers() == 0) return;

    assert(msg->encoding == "rgb8");
    if (msg->width != width || msg->height != height) { // if msg has different 
        ROS_INFO("Reallocating cuda memory due to change in image size (from %dx%d to %dx%d)", width, height, msg->width, msg->height);
        width = msg->width;
        height = msg->height;
        allocateImgMemory();
    }

    memcpy(cudaImgRGB, msg->data.data(), width * height * sizeof(uchar3));
    if (CUDA_FAILED(cudaRGB8ToRGBA32((uchar3*)cudaImgRGB, (float4*)cudaImgRGBA, width, height))) {
        ROS_ERROR("failed to convert %ux%u image with CUDA", width, height);
        return;
    }
    if (!net->Process(cudaImgRGBA, width, height)) {
        ROS_ERROR("fail to process image");
        return;
    }
    if (!net->Mask(cudaImgMask, width, height)) {
        ROS_ERROR("fail to mask image");
        return;
    }

    sensor_msgs::Image pubMsg;
    pubMsg.header = msg->header; // same time stamp

    pubMsg.height = height;
    pubMsg.width = width;
    pubMsg.encoding = sensor_msgs::image_encodings::MONO8;
    pubMsg.is_bigendian = false;

    pubMsg.step = width * 1;  // single channel
    pubMsg.data.resize(width * height); // allocate msg storage
    // copy the mask image into the msg
    memcpy(pubMsg.data.data(), cudaImgMask, width * height);
    pub.publish(pubMsg);
}

int main(int argc, char** argv) {
    initializeNet();
    allocateImgMemory();
    ros::init(argc, argv, "image_listener");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("seg/raw", 1);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);

    ros::spin();
    ROS_INFO("shutting down...");
    SAFE_DELETE(net);
}