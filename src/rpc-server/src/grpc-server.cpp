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

template <typename ROSMsgPtr>
void update_ptr(ROSMsgPtr* ptr, ROSMsgPtr newPtr) {
    *ptr = newPtr;
}

bool SwitchControlState(DriveStateEnum grpcEnum) {
    // must convert between enum used in gRPC and enum used in ROS services
    hero_board::SetStateRequest req;
    std::string stateString;
    if(grpcEnum == DriveStateEnum::DIRECT_DRIVE) {
        req.state = hero_board::SetStateRequest::DIRECT_DRIVE;
        stateString = "DIRECT_DRIVE";
    } else if(grpcEnum == DriveStateEnum::AUTONOMY) {
        req.state = hero_board::SetStateRequest::AUTONOMY;
        stateString = "AUTONOMY";
    } else { // includes DriveStateEnum::IDLE
        req.state = hero_board::SetStateRequest::IDLE;
        stateString = "IDLE";
    }

    ROS_INFO("Setting state to %s", stateString.c_str());
    hero_board::SetStateResponse res;
    bool success = ros::service::call("/set_state", req, res);
    // ROS_INFO("%s", res.controlResponse.c_str());
    return success;
}

pair<DriveStateEnum, bool> GetControlState() {
    hero_board::GetStateRequest req;
    hero_board::GetStateResponse res;

    bool success = ros::service::call("/get_state", req, res);
    if(!success) {
        ROS_ERROR("GetControlState failed to get control state!");
        return {DriveStateEnum::IDLE, success};
    }

    if(res.state == hero_board::GetStateResponse::DIRECT_DRIVE) {
        return {DriveStateEnum::DIRECT_DRIVE, success};
    } else if(res.state == hero_board::GetStateResponse::AUTONOMY) {
        return {DriveStateEnum::AUTONOMY, success};
    } else { // includes hero_board::GetStateResponse::IDLE
        return {DriveStateEnum::IDLE, success};
    }
}


class JetsonServiceImpl final : public JetsonRPC::Service {

     // publish motor commands send from the client (disabled autonomy)
    Status SendMotorCmd(ServerContext* context, ServerReader<MotorCmd>* reader, Void* _) override {
        auto stateAndSuccess = GetControlState();
        DriveStateEnum state = stateAndSuccess.first;
        bool success = stateAndSuccess.second;
        if (!success) {
            ROS_ERROR("SendMotorCMD failed to get initial drive state");
            return Status::CANCELLED;
        }
        if(state != DriveStateEnum::DIRECT_DRIVE) { 
            // The state needs to be DIRECT_DRIVE (because the hero node needs to be subscribed to the right topics)
            ROS_ERROR("SendMotorCMD not in drive state DIRECT_DRIVE! Switch to this state before using this function. Returning. ");
            return Status::CANCELLED;

            // We could make the switch here, but for now let's make the client switch
            // ROS_INFO("SendMotorCMD switching from old state to DIRECT_DRIVE...");
            // if (!SwitchControlState(DriveStateEnum::DIRECT_DRIVE)) {
            //     ROS_ERROR("Failed to switch to DIRECT_DRIVE control");
            //     return Status::CANCELLED; 
            // }
        }
       

        MotorCmd cmd;
        hero_board::MotorVal msg; // definitions see hero-serial/Program.cs (?)
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
            msg.motorval[3] = msg.motorval[1] = (raw & 0b111111) << 2;
            raw >>= 6;
            msg.motorval[2] = msg.motorval[0] = raw << 2;

            cout << "Client send motor command (decoded): ";
            for (int i = 0; i < 8; i++) {
                cout << (int)msg.motorval[i] << " ";
            }
            cout << endl;

            motor_pub.publish(msg);
        }
        // Done receiving direct motor controls
        return Status::OK;

        // If we want to switch back to the drive state before SendMotorCmd was called:
        // ROS_INFO("SendMotorCMD switching from DIRECT_DRIVE back to old state...");
        // if (!SwitchControlState(oldState)) {
        //     ROS_ERROR("SendMotorCMD failed to switch back to previous state!");
        //     return Status::CANCELLED; 
        // } else {
        //     return Status::OK;
        // }
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

    Status ChangeDriveState(ServerContext* context, const DriveState* _state, Void* _) override {
        if(!SwitchControlState(_state->dse())) {
            ROS_ERROR("ChangeDriveState failed!");
            return Status::CANCELLED;
        } else {
            return Status::OK;
        }
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
        return StreamToClient<IMUData, sensor_msgs::Imu, sensor_msgs::ImuConstPtr, decltype(process)>(
            context, _rate, writer, process, "/camera/imu", "Client closes IMU stream"
        );
        return Status::OK;
    }

    Status StreamImage(ServerContext* context, const Rate* _rate, ServerWriter<Image>* writer) override { 
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            rpc_val.set_data({reinterpret_cast<const char*>(ros_msg_ptr->data.data()), ros_msg_ptr->data.size()});
        };
        return StreamToClient<Image, sensor_msgs::CompressedImage, sensor_msgs::CompressedImageConstPtr, decltype(process)>(
            context, _rate, writer, process, "/camera/color/image_raw/compressed", "Client closes image stream"
        );
    }

    Status StreamMotorCurrent(ServerContext* context, const Rate* _rate, ServerWriter<MotorCurrent>* writer) override {
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            // reinterpret 8 motor current bytes as uint64
            rpc_val.set_values(*reinterpret_cast<const uint64_t*>(ros_msg_ptr->motorval.data()));
        };
        return StreamToClient<MotorCurrent, hero_board::MotorVal, hero_board::MotorValConstPtr, decltype(process)>(
            context, _rate, writer, process, "/motor/status", "Client closes motor current stream"
        );
    }

    Status StreamArmStatus(ServerContext* context, const Rate* _rate, ServerWriter<ArmStatus>* writer) override {
        auto process = [](const auto& ros_msg_ptr, auto& rpc_val) {
            rpc_val.set_angle(ros_msg_ptr->angle);
            rpc_val.set_translation(ros_msg_ptr->translation);
        };
        return StreamToClient<ArmStatus, hero_board::MotorVal, hero_board::MotorValConstPtr, decltype(process)>(
            context, _rate, writer, process, "/motor/status", "Client closes angle stream"
        );
    }
    
    // a template function for streaming data from jetson (server) to laptop (client)
    template <typename RPCMsg, typename ROSMsg, typename ROSMsgPtr, typename Process, int queue_size = 1>
    Status StreamToClient(ServerContext* context, const Rate* _rate, ServerWriter<RPCMsg>* writer, 
    Process process, const char* topic, const char* message) {
        RPCMsg rpc_val;

        ROSMsgPtr ros_msg_ptr;
        auto sub = nh->subscribe<ROSMsg>(topic, queue_size, boost::bind(&update_ptr<ROSMsgPtr>, &ros_msg_ptr, _1));
        ros::Rate rate(_rate->rate());
        while (true) {
            ros::spinOnce();  // note: spinOnce is called within the same thread
                              // so reading ros_msg_ptr does not require a lock
                              // it updates the ros_msg_ptr in the local scope if there are new messages
            rate.sleep();
            if (ros_msg_ptr == NULL)
                continue;
            
            process(ros_msg_ptr, rpc_val);
            if (!writer->Write(rpc_val))  // break if client closes the connection
                break;

            ros_msg_ptr = NULL;
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
    ros::init(argc, argv, "grpc-server");
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    RunServer();
    return 0;
}
