// Copyright 2024 Laboratoire des signaux et systèmes
//
// This program is free software: you can redistribute it and/or 
// modify it under the terms of the GNU General Public License as 
// published by the Free Software Foundation, either version 3 of 
// the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License 
// along with this program. If not, see <https://www.gnu.org/licenses/>. 
//
// Author: Aarsh Thakker <aarsh.thakker@centralesupelec.fr>

#include "rclcpp/rclcpp.hpp"
#include "natnet_ros2/srv/marker_poses.hpp"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

#include <NatNetCAPI.h>
#include <NatNetClient.h>

class GetMarkerPosesServer : public rclcpp::Node
{
public:
    GetMarkerPosesServer();
    ~GetMarkerPosesServer();
    void set_conn_params();

private:
    rclcpp::Service<natnet_ros2::srv::MarkerPoses>::SharedPtr service_;
    NatNetClient* natnet_client_{nullptr};
    sNatNetClientConnectParams g_connectParams;
    std::string serverIP;
    std::string clientIP;
    std::string serverType;
    std::string multicastAddress;
    int serverCommandPort;
    int serverDataPort;
    ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    bool frame_received_{false};
    std::vector<double> x_position_;
    std::vector<double> y_position_;
    std::vector<double> z_position_;

    void frame_callback(sFrameOfMocapData *data);
    static void static_frame_callback(sFrameOfMocapData *data, void *pUserData);
    bool update(const std::shared_ptr<natnet_ros2::srv::MarkerPoses::Request> req,
                std::shared_ptr<natnet_ros2::srv::MarkerPoses::Response> res);
};

void GetMarkerPosesServer::static_frame_callback(sFrameOfMocapData *data, void *pUserData)
{
    static_cast<GetMarkerPosesServer *>(pUserData)->frame_callback(data);
    //GetMarkerPosesServer* instance = static_cast<GetMarkerPosesServer*>(pUserData);
    //if (instance)
    //{
    //    instance->frame_callback(data);
    //}
}

GetMarkerPosesServer::GetMarkerPosesServer()
    : Node("marker_poses_server")
{
    declare_parameter<std::string>("global_frame", "world");
    declare_parameter<std::string>("serverIP", "192.168.0.100");
    declare_parameter<std::string>("clientIP", "192.168.0.101");
    declare_parameter<std::string>("serverType", "multicast");
    declare_parameter<std::string>("multicastAddress", "239.255.42.99");
    declare_parameter<int>("serverCommandPort", 1510);
    declare_parameter<int>("serverDataPort", 1511);
    service_ = this->create_service<natnet_ros2::srv::MarkerPoses>(
        "get_marker_position", std::bind(&GetMarkerPosesServer::update, this, std::placeholders::_1, std::placeholders::_2 ));

    natnet_client_ = new NatNetClient();

    
}

GetMarkerPosesServer::~GetMarkerPosesServer()
{
    if (natnet_client_)
    {
        natnet_client_->SetFrameReceivedCallback(nullptr, nullptr);
        natnet_client_->Disconnect();
        delete natnet_client_;
        natnet_client_ = nullptr;
    }
}

void GetMarkerPosesServer::frame_callback(sFrameOfMocapData *data)
{
    std::vector<double> x_position;
    std::vector<double> y_position;
    std::vector<double> z_position;

    for (int i = 0; i < data->nLabeledMarkers; i++)
    {
        bool bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        if (bUnlabeled)
        {
            x_position.push_back(data->LabeledMarkers[i].x);
            y_position.push_back(data->LabeledMarkers[i].y);
            z_position.push_back(data->LabeledMarkers[i].z);
        }
    }

    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        x_position_ = std::move(x_position);
        y_position_ = std::move(y_position);
        z_position_ = std::move(z_position);
        frame_received_ = true;
    }
    response_cv_.notify_one();
}

bool GetMarkerPosesServer::update(const std::shared_ptr<natnet_ros2::srv::MarkerPoses::Request> req,
                                std::shared_ptr<natnet_ros2::srv::MarkerPoses::Response> res)
{
    natnet_client_->Disconnect();
    set_conn_params();

    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        frame_received_ = false;
    }
    natnet_client_->SetFrameReceivedCallback(static_frame_callback, this);

    int iResult = natnet_client_->Connect(g_connectParams);
    if (iResult != ErrorCode_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Error connecting client. Exiting. Try again");
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Client initialized and ready.");
    }
    if (!rclcpp::ok())
    {
        natnet_client_->Disconnect();
        return false;
    }

    std::unique_lock<std::mutex> lock(response_mutex_);
    const bool received = response_cv_.wait_for(
        lock, std::chrono::seconds(2), [this]() { return frame_received_; });
    if (!received)
    {
        lock.unlock();
        natnet_client_->Disconnect();
        RCLCPP_WARN(this->get_logger(), "Timed out waiting for a fresh NatNet frame.");
        return false;
    }

    res->num_of_markers = static_cast<int64_t>(x_position_.size());
    res->x_position = x_position_;
    res->y_position = y_position_;
    res->z_position = z_position_;
    lock.unlock();
    natnet_client_->Disconnect();
    return true;
}

void GetMarkerPosesServer::set_conn_params()
{
    if (get_parameter("serverIP", serverIP))
    {
        RCLCPP_INFO(get_logger(),"Got server IP : %s", serverIP.c_str());
    }
    else
    {
        RCLCPP_WARN(get_logger(),"Failed to get server IP, using default 192.168.0.100");
        serverIP = "192.168.0.100";
    }

    if (get_parameter("clientIP", clientIP))
    {
        RCLCPP_INFO(get_logger(),"Got client IP : %s", clientIP.c_str());
    }
    else
    {
        RCLCPP_WARN(get_logger(),"Failed to get client IP, using default 192.168.0.101");
        clientIP = "192.168.0.101";
    }

    if (get_parameter("serverType", serverType))
    {
        RCLCPP_INFO(get_logger(),"Got server Type : %s", serverType.c_str());
    }
    else
    {
        RCLCPP_WARN(get_logger(),"Failed to get server type, using default multicast");
        serverType = "multicast";
    }

    if (serverType == "multicast")
    {
        if (get_parameter("multicastAddress", multicastAddress))
            RCLCPP_INFO(get_logger(),"Got server Type : %s", multicastAddress.c_str());
        else
        {
            RCLCPP_WARN(get_logger(),"Failed to get server IP, using default multicast address 239.255.42.99");
            multicastAddress = "239.255.42.99";
        }
    }

    if (get_parameter("serverCommandPort", serverCommandPort))
    {
        RCLCPP_INFO(get_logger(),"Got server Command Port : %i", serverCommandPort);
    }
    else
    {
        RCLCPP_WARN(get_logger(),"Failed to get server command port, using default 1510");
        serverCommandPort = 1510;
    }

    if (get_parameter("serverDataPort", serverDataPort))
    {
        RCLCPP_INFO(get_logger(),"Got server Command Port : %i", serverDataPort);
    }
    else
    {
        RCLCPP_WARN(get_logger(),"Failed to get server command port, using default 1511");
        serverDataPort = 1511;
    }
    if (serverType == "unicast")
        kDefaultConnectionType = ConnectionType_Unicast;

    // Setting up parameters for the natnet connection
    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverCommandPort = serverCommandPort;
    g_connectParams.serverDataPort = serverDataPort;
    g_connectParams.serverAddress = serverIP.c_str();
    g_connectParams.localAddress = clientIP.c_str();
    g_connectParams.multicastAddress = serverType=="multicast" ? multicastAddress.c_str() : NULL;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GetMarkerPosesServer>());
    rclcpp::shutdown();
    return 0;
}
