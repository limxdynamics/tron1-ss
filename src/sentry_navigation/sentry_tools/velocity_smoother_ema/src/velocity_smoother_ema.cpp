// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include <velocity_smoother_ema/velocity_smoother_ema.hpp>
#include <iostream>
#include <atomic>
#include <string>
#include <thread>
#include <chrono>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio.hpp> 
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using json = nlohmann::json;
using websocketpp::client;
using websocketpp::connection_hdl;

// Replace this ACCID value with your robot's actual serial number (SN)
static const std::string ACCID = "WF_TRON1A_237";

// WebSocket client instance
static client<websocketpp::config::asio> ws_client;

// Atomic flag for graceful exit
static std::atomic<bool> should_exit(false);

// Connection handle for sending messages
static connection_hdl current_hdl;

// Generate dynamic GUID
static std::string generate_guid() {
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();
    return boost::uuids::to_string(u);
}

// Send WebSocket request with title and data
static void send_request(const std::string& title, const json& data = json::object()) {
    json message;
    
    // Adding necessary fields to the message
    message["accid"] = ACCID;
    message["title"] = title;
    message["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
    message["guid"] = generate_guid();
    message["data"] = data;

    std::string message_str = message.dump();
    
    // Send the message through WebSocket
    ws_client.send(current_hdl, message_str, websocketpp::frame::opcode::text);
}

// Handle user commands
static void handle_commands() {
    while (!should_exit) {
        std::string command;
        std::cout << "Enter command ('stand', 'walk', 'twist', 'sit', 'stair', 'stop', 'imu') or 'exit' to quit:" << std::endl;
        std::getline(std::cin, command);  // Read user input

        if (command == "exit") {
            should_exit = true;  // Exit flag to stop the loop
            break;
        } else if (command == "stand") {
            send_request("request_stand_mode");  // Send stand mode request
        } else if (command == "walk") {
            send_request("request_walk_mode");  // Send walk mode request
        } else if (command == "twist") {
            float x, y, z;
            std::cout << "Enter x, y, z values:" << std::endl;
            std::cin >> x >> y >> z;  // Get twist values from user
            send_request("request_twist", {{"x", x}, {"y", y}, {"z", z}});
        } else if (command == "sit") {
            send_request("request_sitdown");  // Send sit down request
        } else if (command == "stair") {
            bool enable;
            std::cout << "Enable stair mode (true/false):" << std::endl;
            std::cin >> enable;  // Get stair mode enable flag from user
            send_request("request_stair_mode", {{"enable", enable}});
        } else if (command == "stop") {
            send_request("request_emgy_stop");  // Send emergency stop request
        } else if (command == "imu") {
            std::string enable;
            std::cout << "Enable IMU (true/false):" << std::endl;
            std::cin >> enable;  // Get IMU enable flag from user
            send_request("request_enable_imu", {{"enable", enable == "true" ? true : false}});
        }
    }
}

// WebSocket open callback
static void on_open(connection_hdl hdl) {
    std::cout << "Connected!" << std::endl;
    
    // Save connection handle for sending messages later
    current_hdl = hdl;

    // Start handling commands in a separate thread
    // std::thread(handle_commands).detach();
}

// WebSocket message callback
static void on_message(connection_hdl hdl, client<websocketpp::config::asio>::message_ptr msg) {
    //std::cout << "Received: " << msg->get_payload() << std::endl;  // Print received message
}

// WebSocket close callback
static void on_close(connection_hdl hdl) {
    std::cout << "Connection closed." << std::endl;
}

// Close WebSocket connection
static void close_connection(connection_hdl hdl) {
    ws_client.close(hdl, websocketpp::close::status::normal, "Normal closure");  // Close connection normally
}

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh):nh_(*nh)
{
    nh_.param<double>("/alpha_v", alpha_v, 0.4);
    nh_.param<double>("/alpha_w", alpha_w, 0.4);
    nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
    nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
    nh_.param<double>("/cmd_rate", cmd_rate, 30.0);

    velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 10, true);
    timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);

    previous_x_vel = 0.0;
    previous_y_vel = 0.0;
    previous_w_vel = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;
    w_vel = 0.0;
    smoothed_x_vel = 0.0;
    smoothed_y_vel = 0.0;
    smoothed_w_vel = 0.0;
}

VelocitySmootherEma::~VelocitySmootherEma()
{
    ros::shutdown();
}

void VelocitySmootherEma::twist_callback(const geometry_msgs::Twist::ConstPtr msg)
{
    // ROS_INFO("I RECEIVED A NEW MESSAGE");
    cmd_vel_msg_ = *msg;
    send_request("request_twist", {{"x", cmd_vel_msg_.linear.x}, {"y",0}, {"z", cmd_vel_msg_.angular.z}});
    std::cout<<"request_twist : x = "<<cmd_vel_msg_.linear.x<<" ; y = "<<   cmd_vel_msg_.linear.y<<" ; z = "<<cmd_vel_msg_.angular.z<<std::endl;  
}

void VelocitySmootherEma::update(const ros::TimerEvent&)
{
    x_vel = cmd_vel_msg_.linear.x;
    y_vel = cmd_vel_msg_.linear.y;
    w_vel = cmd_vel_msg_.angular.z;

    smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
    smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
    smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

    cmd_vel_msg_.linear.x = smoothed_x_vel;
    cmd_vel_msg_.linear.y = smoothed_y_vel;
    cmd_vel_msg_.angular.z = smoothed_w_vel;

    previous_x_vel = smoothed_x_vel;
    previous_y_vel = smoothed_y_vel;
    previous_w_vel = smoothed_w_vel;    

    velocity_pub_.publish(cmd_vel_msg_);
    // ROS_INFO("PUBLISHING TWIST MESSAGE!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");

    ros::NodeHandle nh;

    VelocitySmootherEma vse(&nh);
    ws_client.init_asio();  // Initialize ASIO for WebSocket client
    
    // Set WebSocket event handlers
    ws_client.set_open_handler(&on_open);  // Set open handler
    ws_client.set_message_handler(&on_message);  // Set message handler
    ws_client.set_close_handler(&on_close);  // Set close handler

    std::string server_uri = "ws://10.192.1.2:5000";  // WebSocket server URI

    websocketpp::lib::error_code ec;
    client<websocketpp::config::asio>::connection_ptr con = ws_client.get_connection(server_uri, ec);  // Get connection pointer

    if (ec) {
        std::cout << "Error: " << ec.message() << std::endl;
        return 1;  // Exit if connection error occurs
    }

    connection_hdl hdl = con->get_handle();  // Get connection handle
    ws_client.connect(con);  // Connect to server
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Run the WebSocket client loop
     // 启动异步的 ROS Spinner（4个线程）
    ros::AsyncSpinner spinner(4); // 线程数可根据需要调整
    spinner.start(); // 非阻塞

    std::cout << "ROS和WebSocket已启动，输入命令或Ctrl+C退出..." << std::endl;

    // 主线程继续运行（例如处理用户输入或WebSocket事件）
    while (ros::ok() && !should_exit) {
        ws_client.run(); // 处理WebSocket事件（非阻塞，需配合事件循环）
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 清理
    ws_client.stop();
    spinner.stop();
    return 0;
    

    return 0;
}
