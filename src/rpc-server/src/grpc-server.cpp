#include <chrono>
#include <iostream>
#include <queue>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>
#include "jetsonrpc.grpc.pb.h"

#include <cv_bridge/cv_bridge.h>
#include <hero_board/MotorVal.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <csignal>
#include <sensor_msgs/Imu.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerWriter;
using grpc::Status;

using namespace jetsonrpc;
using namespace std;

ros::NodeHandlePtr nh;

// convenient pointer holder and updator
template <typename T>
class PtrHolder {
    public:
    T ptr;
    inline T& operator=(T newPtr) {
        ptr = newPtr;
    }
    inline void update(T newPtr) {
        ptr = newPtr;
    }
    inline bool operator==(T other) {
        return ptr == other;
    }
    inline T operator->() {
        return ptr;
    }
};

class GreeterServiceImpl final : public JetsonRPC::Service {
    /**
     * publish motor commands send from the client, disable autonomy
     */
    Status SendMotorCmd(ServerContext* context, ServerReader<MotorCmd>* reader, Void* _) override {
        // --------------
        // some ros service calls...
        //
        // --------------
        MotorCmd cmd;
        hero_board::MotorVal msg; // definitions see hero-serial/Program.cs
        auto pub = nh->advertise<decltype(msg)>("/motor_pub", 1);
        while (reader->Read(&cmd)) {
            // decode motor values
            uint32_t raw = cmd.values();
            msg.motorval[7] = (raw & 0b11) * 100;  // 0, 100, or 200
            raw >>= 2;
            msg.motorval[6] = (raw & 0b111111) << 2;
            raw >>= 6;
            msg.motorval[5] = (raw & 0b111111) << 2;
            raw >>= 6;
            msg.motorval[4] = (raw & 0b111111) << 2;
            raw >>= 6;
            msg.motorval[3] = msg.motorval[2] = (raw & 0b111111) << 2;
            raw >>= 6;
            msg.motorval[1] = msg.motorval[0] = raw << 2;

            for (int i = 0; i < 8; i++) {
                cout << (int)msg.motorval[i] << " ";
            }
            cout << endl;
            // publish message
            pub.publish(msg);
        }
        ROS_INFO("Client closes motor command stream");
        return Status::OK;
    }
    Status StreamIMU(ServerContext* context, const Void* _, ServerWriter<IMUData>* writer) override {
        IMUFlag = true;

        IMUData msg;
        msg.mutable_values()->Resize(6, 0); // allocate memory for the underlying buffer

        PtrHolder<sensor_msgs::ImuConstPtr> imuPtr;
        auto sub = nh->subscribe("/camera/imu", 5, &decltype(imuPtr)::update, &imuPtr);
        while (IMUFlag) {
            ros::spinOnce();
            if (imuPtr == NULL) 
                continue;

            msg.set_values(0, imuPtr->angular_velocity.x);
            msg.set_values(1, imuPtr->angular_velocity.y);
            msg.set_values(2, imuPtr->angular_velocity.z);

            msg.set_values(3, imuPtr->linear_acceleration.x);
            msg.set_values(4, imuPtr->linear_acceleration.y);
            msg.set_values(5, imuPtr->linear_acceleration.z);

            imuPtr = NULL;
            if (!writer->Write(msg))
                break;
        }
        return Status::OK;
    }
    Status EndIMU(ServerContext* context, const Void* _, Void* __) override {
        IMUFlag = false;
        return Status::OK;
    }
    Status StreamImage(ServerContext* context, const Void* _, ServerWriter<Image>* writer) override {
        context->set_compression_level(GRPC_COMPRESS_LEVEL_HIGH);

        ImageFlag = true;
        Image msg;

        PtrHolder<sensor_msgs::CompressedImageConstPtr> imgPtr;
        auto sub = nh->subscribe("/camera/color/image_raw/compressed", 5, &decltype(imgPtr)::update, &imgPtr);
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
    Status StreamMotorCurrent(ServerContext* context, const Void* _, ServerWriter<MotorCurrent>* writer) {
        CurrentFlag = true;
        MotorCurrent current;

        PtrHolder<hero_board::MotorValConstPtr> currentPtr;
        auto sub = nh->subscribe("/topic", 5, &decltype(currentPtr)::update, &currentPtr);
        while (CurrentFlag) {
            ros::spinOnce();  // note: spinOnce is called within the same thread
            if (currentPtr == NULL)
                continue;

            // reinterpret 8 packed uint8 as uint64
            current.set_values(*reinterpret_cast<const uint64_t*>(currentPtr->motorval.data()));
            if (!writer->Write(current))  // break if client closes the connection
                break;

            currentPtr = NULL;
        }
        return Status::OK;
    }
    Status EndMotorCurrent(ServerContext* context, const Void* _, Void* __) override {
        CurrentFlag = false;
        return Status::OK;
    }

   private:
    atomic<bool> IMUFlag;
    atomic<bool> ImageFlag;
    atomic<bool> CurrentFlag;
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    GreeterServiceImpl service;

    ServerBuilder builder;

    // no compression by default
    // builder.SetDefaultCompressionAlgorithm(GRPC_COMPRESS_GZIP);

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
