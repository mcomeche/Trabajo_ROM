#include <chrono>
#include <memory>
#include <map>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ZoneNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ZoneNavigator() : Node("zone_navigator")
    {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        // Configuración de zonas (ajusta coordenadas según tu mapa)
        zones_ = {
            {"Norte", create_pose(0.2, 2.6, 0.0)},
            {"Sur", create_pose(-0.2, -2.6, 3.14159)},
            {"Este", create_pose(2.6, -0.2, 1.5708)},
            {"Oeste", create_pose(-2.6, -0.2, -1.5708)}
        };

        // Hilo separado para entrada de usuario
        user_input_thread_ = std::thread(&ZoneNavigator::process_user_input, this);
    }

    ~ZoneNavigator() {
        if (user_input_thread_.joinable()) {
            user_input_thread_.join();
        }
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> zones_;
    std::thread user_input_thread_;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_;

    void process_user_input() {
        while (rclcpp::ok()) {
            print_menu();
            
            std::string command;
            std::getline(std::cin, command);
            
            if (command.empty()) continue;
            
            if (command == "exit") {
                rclcpp::shutdown();
                break;
            }
            
            if (zones_.count(command)) {
                if (current_goal_) {
                    cancel_current_goal();
                }
                send_navigation_goal(command, zones_[command]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Zona desconocida: '%s'", command.c_str());
            }
        }
    }

    void print_menu() {
        RCLCPP_INFO(this->get_logger(), "\nZonas disponibles:");
        for (const auto& [name, _] : zones_) {
            RCLCPP_INFO(this->get_logger(), " - %s", name.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Ingrese nombre de zona o 'exit' para salir:");
    }

    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        double half_yaw = yaw * 0.5;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(half_yaw);
        pose.pose.orientation.w = cos(half_yaw);
        
        return pose;
    }

    void cancel_current_goal() {
        if (current_goal_) {
            client_ptr_->async_cancel_goal(current_goal_);
            current_goal_.reset();
        }
    }

    void send_navigation_goal(const std::string& zone_name, const geometry_msgs::msg::PoseStamped& goal_pose) {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Servidor de navegación no disponible");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this, zone_name](const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Objetivo rechazado para %s", zone_name.c_str());
                } else {
                    current_goal_ = goal_handle;
                    RCLCPP_INFO(this->get_logger(), "Navegando a %s...", zone_name.c_str());
                }
            };

        send_goal_options.result_callback = 
            [this, zone_name](const GoalHandleNavigateToPose::WrappedResult& result) {
                current_goal_.reset();
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Llegada exitosa a %s", zone_name.c_str());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Navegación abortada");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Navegación cancelada");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Error desconocido");
                }
            };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZoneNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
