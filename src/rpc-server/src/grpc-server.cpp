#include <chrono>
#include <iostream>
#include <queue>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>
#include "jetsonrpc.grpc.pb.h"

#include <cv_bridge/cv_bridge.h>

#include <hero_board/MotorVal.h>
#include <hero_board/SetState.h>
#include <hero_board/SetStateResponse.h>
#include <hero_board/GetState.h>
#include <hero_board/GetStateResponse.h>

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

bool SwitchControl(bool state) {
    hero_board::SetStateRequest req;
    req.state = state;

    hero_board::SetStateResponse res;
    bool status = ros::service::call("/set_state", req, res);
    ROS_INFO("%s", res.controlResponse.c_str());
    return status;
}

pair<bool, bool> GetControlState() {
    hero_board::GetStateRequest req;
    hero_board::GetStateResponse res;

    bool status = ros::service::call("/get_state", req, res);
    return {res.state, status};
}


class JetsonServiceImpl final : public JetsonRPC::Service {
    /**
     * publish motor commands send from the client, disable autonomy
     */
    Status SendMotorCmd(ServerContext* context, ServerReader<MotorCmd>* reader, Void* _) override {
        auto state = GetControlState();
        if (!state.second) {
            ROS_ERROR("Failed to get control state");
            return Status::OK;
        }

        auto stateStr = state.first == hero_board::GetStateResponse::MANUAL ? "manual" : "autonomy";
        ROS_INFO("Switching from %s to %s", stateStr, "manual");

        if (!SwitchControl(hero_board::SetStateRequest::MANUAL)) {
            ROS_ERROR("Failed to switch to manual control");
            return Status::OK; 
        }

        MotorCmd cmd;
        hero_board::MotorVal msg; // definitions see hero-serial/Program.cs
        // publisher is only initialized once
        static auto motor_pub = nh->advertise<hero_board::MotorVal>("/motor/output", 1);

        msg.motorval.resize(8);
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

            cout << "Client send motor command (decoded): ";
            for (int i = 0; i < 8; i++) {
                cout << (int)msg.motorval[i] << " ";
            }
            cout << endl;
            // publish message
            motor_pub.publish(msg);
        }

        ROS_INFO("Client closes motor command stream, switching to previous control state %s", stateStr);

        if (!SwitchControl(state.first)) {
            ROS_ERROR("Failed to switch to %s", stateStr);
            return Status::OK; 
        }
        return Status::OK;
    }

    Status SendTwist(ServerContext* context, ServerReader<Twist>* reader, Void* _) override {
        Twist twist;

        hero_board::MotorVal msg; // definitions see hero-serial/Program.cs
        msg.motorval.resize(12);
        // reinterpret 12 bytes as 3 floats
        auto raw_data = reinterpret_cast<float*>(msg.motorval.data());

        auto pub = nh->advertise<hero_board::MotorVal>("/motor/output", 1);
        while (reader->Read(&twist)) {
            raw_data[0] = twist.values(0);
            raw_data[1] = twist.values(1);
            raw_data[2] = twist.values(2);

            pub.publish(msg);
        }
        ROS_INFO("Client closes twist stream");
        return Status::OK;
    }

    Status StreamIMU(ServerContext* context, const Rate* _rate, ServerWriter<IMUData>* writer) override {
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            rpc_val.set_values(0, ros_msg_ptr->angular_velocity.x);
            rpc_val.set_values(1, ros_msg_ptr->angular_velocity.y);
            rpc_val.set_values(2, ros_msg_ptr->angular_velocity.z);

            rpc_val.set_values(3, ros_msg_ptr->linear_acceleration.x);
            rpc_val.set_values(4, ros_msg_ptr->linear_acceleration.y);
            rpc_val.set_values(5, ros_msg_ptr->linear_acceleration.z);
        };
        return StreamToClient<IMUData, sensor_msgs::ImuConstPtr, decltype(process)>(
            context, _rate, writer, process, "/camera/imu", "Client closes IMU stream"
        );
    }

    Status StreamImage(ServerContext* context, const Rate* _rate, ServerWriter<Image>* writer) override { 
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            rpc_val.set_data({reinterpret_cast<const char*>(ros_msg_ptr->data.data()), ros_msg_ptr->data.size()});
        };
        return StreamToClient<Image, sensor_msgs::CompressedImageConstPtr, decltype(process)>(
            context, _rate, writer, process, "/camera/color/image_raw/compressed", "Client closes image stream"
        );
    }

    Status StreamMotorCurrent(ServerContext* context, const Rate* _rate, ServerWriter<MotorCurrent>* writer) override {
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            // reinterpret 8 motor current bytes as uint64
            rpc_val.set_values(*reinterpret_cast<const uint64_t*>(ros_msg_ptr->motorval.data()));
        };
        return StreamToClient<MotorCurrent, hero_board::MotorValConstPtr, decltype(process)>(
            context, _rate, writer, process, "/motor/status", "Client closes motor current stream"
        );
    }

    Status StreamArmStatus(ServerContext* context, const Rate* _rate, ServerWriter<ArmStatus>* writer) override {
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            rpc_val.set_angle(ros_msg_ptr->angle);
            rpc_val.set_translation(ros_msg_ptr->translation);
        };
        return StreamToClient<ArmStatus, hero_board::MotorValConstPtr, decltype(process)>(
            context, _rate, writer, process, "/motor/status", "Client closes angle stream"
        );
    }
    
    // a template function for streaming data from jetson (server) to laptop (client)
    template <typename RPCMsg, typename ROSMsgPtr, typename Process, int queue_size = 1>
    Status StreamToClient(ServerContext* context, const Rate* _rate, ServerWriter<RPCMsg>* writer, 
    Process process, const char* topic, const char* message) {
        RPCMsg rpc_val;

        static ROSMsgPtr ros_msg_ptr;
        struct X { // use a nested struct because it can access variable at the parent scope
            static void update(ROSMsgPtr newPtr) { // method for updating the current ros message (pointer)
                ros_msg_ptr = newPtr;
            }
        };

        auto sub = nh->subscribe(topic, queue_size, X::update);
        ros::Rate rate(_rate->rate());
        while (true) {
            ros::spinOnce();  // note: spinOnce is called within the same thread
                              // so reading ros_msg_ptr does not require a lock
                              // it updates the ros_msg_ptr in the local scope if there are new messages
            if (ros_msg_ptr == NULL)
                continue;
            
            process(ros_msg_ptr, rpc_val);
            if (!writer->Write(rpc_val))  // break if client closes the connection
                break;

            ros_msg_ptr = NULL;
            rate.sleep();
        }
        ROS_INFO("%s", message);
        return Status::OK;
    }
   private:
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    JetsonServiceImpl service;

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
