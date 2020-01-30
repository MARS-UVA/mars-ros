#include <chrono>
#include <iostream>
#include <queue>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>
#include "jetsonrpc.grpc.pb.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <csignal>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerWriter;
using grpc::Status;

using namespace jetsonrpc;
using namespace std;

ros::NodeHandlePtr nh;
sensor_msgs::CompressedImagePtr imgPtr;

void processImage(const sensor_msgs::CompressedImagePtr& image) {
    imgPtr = image;
}

class GreeterServiceImpl final : public JetsonRPC::Service {
    Status SendMotorCmd(ServerContext* context, ServerReader<MotorCmd>* reader, Void* _) override {
        // --------------
        // some ros service calls...
        //
        // --------------
        MotorCmd cmd;
        while (reader->Read(&cmd)) {
        }
        return Status::OK;
    }
    Status StreamIMU(ServerContext* context, const Void* _, ServerWriter<IMUData>* writer) override {
        IMUFlag = true;
        IMUData msg;
        while (IMUFlag) {
            msg.set_m1(1.0);  // just a dummy value
            if (!writer->Write(msg))
                break;

            this_thread::sleep_for(chrono::milliseconds(100));
        }
        return Status::OK;
    }
    Status EndIMU(ServerContext* context, const Void* _, Void* __) override {
        IMUFlag = false;
        return Status::OK;
    }
    Status StreamImage(ServerContext* context, const Void* _, ServerWriter<Image>* writer) override {
        ImageFlag = true;
        Image msg;

        auto sub = nh->subscribe("/camera/color/image_raw/compressed", 1, processImage);
        while (ImageFlag) {
            ros::spinOnce();  // note: spinOnce is called within the same thread
            if (imgPtr == NULL)
                continue;

            msg.set_encoding(imgPtr->format);
            msg.set_data({reinterpret_cast<const char*>(imgPtr->data.data()), imgPtr->data.size()});
            if (!writer->Write(msg))  // break if client closes the connection
                break;

            imgPtr = NULL;
        }
        return Status::OK;
    }
    Status EndImage(ServerContext* context, const Void* _, Void* __) override {
        ImageFlag = false;
        return Status::OK;
    }

   private:
    atomic<bool> IMUFlag;
    atomic<bool> ImageFlag;
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    GreeterServiceImpl service;

    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    signal(SIGINT, [](int signum) {
        cout << "Keyboard Interrupt" << endl;
        exit(signum);
    });
    server->Wait();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    RunServer();
    return 0;
}
