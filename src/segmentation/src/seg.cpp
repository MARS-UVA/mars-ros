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
uchar3* cudaImgRGB;
float* cudaImgRGBA;

const int width = 640;
const int height = 480;

void initialzeNet() {
    segNet* net = segNet::Create(segNet::FCN_RESNET18_CITYSCAPES_1024x512, 1);

    if (!net) {
        printf("segnet-camera:   failed to initialize imageNet\n");
    }
    net->SetOverlayAlpha(255);

    bool r = cudaAllocMapped((void**)&imgMask, width * height);
    r = r && cudaAllocMapped((void**)&cudaImgRGB, width * height * sizeof(uchar3));
    r = r && cudaAllocMapped((void**)&cudaImgRGBA, width * height * sizeof(float4));

    if (!r) {
        printf("segnet-camera:  failed to allocate CUDA memory for mask image\n");
        return;
    }
    printf("Memory allocated!\n");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // const uint32_t width = msg->width;
    // const uint32_t height = msg->height;

    // get the desired alpha blend filtering mode
    const segNet::FilterMode filterMode = segNet::FilterModeFromStr("point");

    /*
	 * processing loop
	 */
    float confidence = 0.0f;

    cout << msg->encoding << " " << msg->step << endl;
    cout << msg->width << " " << msg->height << endl;
    // memcpy(cudaImgRGB, , width * height * sizeof(uchar3));
    if (CUDA_FAILED(cudaRGB8ToRGBA32((uchar3*)(msg->data.data()), (float4*)cudaImgRGBA, width, height))) {
        ROS_ERROR("failed to convert %ux%u image with CUDA", width, height);
        return;
    }
    for (int i = 0; i < 10; i++) {
        printf("%f ", cudaImgRGBA[i]);
    }
    if (!net->Process(cudaImgRGBA, width, height)) {
        printf("segnet-console:  failed to process segmentation\n");
        return;
    }

    // generate mask
    if (!net->Mask(imgMask, width, height)) {
        printf("segnet-console:  failed to process segmentation mask.\n");
        return;
    }

    // while (!signal_recieved) {
    //     // capture RGBA image
    //     float* imgRGBA = NULL;

    //     if (!camera->CaptureRGBA(&imgRGBA, 1000, true))
    //         printf("segnet-camera:  failed to convert from NV12 to RGBA\n");

    //     // process the segmentation network

    //     // generate overlay
    //     if (!net->Overlay(imgOverlay, width, height, filterMode)) {
    //         printf("segnet-console:  failed to process segmentation overlay.\n");
    //         continue;
    //     }

    //     // generate mask
    //     if (!net->Mask(imgMask, width / 2, height / 2, filterMode)) {
    //         printf("segnet-console:  failed to process segmentation mask.\n");
    //         continue;
    //     }

    //     // wait for the GPU to finish
    //     CUDA(cudaDeviceSynchronize());

    //     // print out timing info
    //     net->PrintProfilerTimes();
    // }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    initialzeNet();

    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");

    printf("segnet-camera:  shutting down...\n");
    SAFE_DELETE(net);
}