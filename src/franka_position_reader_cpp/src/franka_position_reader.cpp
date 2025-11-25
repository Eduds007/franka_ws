#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>

class FrankaPositionReader : public rclcpp::Node
{
public:
    FrankaPositionReader() : Node("franka_position_reader_cpp")
    {
        // Subscribers para diferentes tipos de informação de posição
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FrankaPositionReader::joint_state_callback, this, std::placeholders::_1));

        robot_state_subscriber_ = this->create_subscription<franka_msgs::msg::FrankaRobotState>(
            "/franka_robot_state_broadcaster/robot_state", 10,
            std::bind(&FrankaPositionReader::robot_state_callback, this, std::placeholders::_1));

        // Timer para imprimir posição periodicamente
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FrankaPositionReader::print_position_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "🤖 Franka Position Reader C++ iniciado");
        RCLCPP_INFO(this->get_logger(), "📡 Aguardando dados do robô...");

        // Inicializar variáveis
        joint_positions_received_ = false;
        robot_state_received_ = false;
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() >= 7) {
            joint_positions_ = std::vector<double>(msg->position.begin(), msg->position.begin() + 7);
            joint_names_ = std::vector<std::string>(msg->name.begin(), msg->name.begin() + 7);
            joint_positions_received_ = true;
        }
    }

    void robot_state_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg)
    {
        // Pose do end-effector (matriz de transformação 4x4)
        if (msg->o_t_ee.size() == 16) {
            // Copiar matriz de transformação (column-major para row-major)
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    end_effector_pose_[i][j] = msg->o_t_ee[j * 4 + i]; // Column-major to row-major
                }
            }
            robot_mode_ = msg->robot_mode;
            robot_state_received_ = true;
        }
    }

    void print_position_timer_callback()
    {
        print_current_position();
    }

    void print_current_position()
    {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "📍 POSIÇÃO ATUAL DO ROBÔ FRANKA (C++)" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // Informações das juntas
        if (joint_positions_received_) {
            std::cout << "\n🔧 POSIÇÕES DAS JUNTAS:" << std::endl;
            for (size_t i = 0; i < joint_positions_.size() && i < joint_names_.size(); ++i) {
                double degrees = joint_positions_[i] * 180.0 / M_PI;
                std::cout << "  " << joint_names_[i] << ": " 
                         << std::fixed << std::setprecision(4) << joint_positions_[i] 
                         << " rad (" << std::setprecision(2) << degrees << "°)" << std::endl;
            }
        } else {
            std::cout << "\n❌ Dados das juntas não disponíveis" << std::endl;
        }

        // Informações do end-effector
        if (robot_state_received_) {
            std::cout << "\n🎯 POSE DO END-EFFECTOR:" << std::endl;
            
            // Posição (x, y, z) - últimas linhas da matriz de transformação
            double x = end_effector_pose_[0][3];
            double y = end_effector_pose_[1][3];
            double z = end_effector_pose_[2][3];
            
            std::cout << "  Posição (x, y, z): [" 
                     << std::fixed << std::setprecision(4) 
                     << x << ", " << y << ", " << z << "] m" << std::endl;

            // Matriz de rotação
            std::cout << "  Matriz de Rotação:" << std::endl;
            for (int i = 0; i < 3; ++i) {
                std::cout << "    [";
                for (int j = 0; j < 3; ++j) {
                    std::cout << std::setw(8) << std::setprecision(4) << end_effector_pose_[i][j];
                    if (j < 2) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }

            // Conversão aproximada para ângulos de Euler (ZYX)
            auto euler_angles = rotation_matrix_to_euler_zyx();
            std::cout << "  Ângulos de Euler (ZYX): [" 
                     << std::fixed << std::setprecision(2)
                     << euler_angles[0] << "°, " 
                     << euler_angles[1] << "°, " 
                     << euler_angles[2] << "°]" << std::endl;

        } else {
            std::cout << "\n❌ Dados do end-effector não disponíveis" << std::endl;
        }

        // Status do robô
        if (robot_state_received_) {
            std::string mode_name = get_robot_mode_name(robot_mode_);
            std::cout << "\n🤖 STATUS DO ROBÔ: " << mode_name << std::endl;
        }

        std::cout << std::string(60, '=') << std::endl;
    }

    std::vector<double> rotation_matrix_to_euler_zyx()
    {
        std::vector<double> euler(3);
        
        // Conversão de matriz de rotação para ângulos de Euler (ZYX)
        double sy = sqrt(end_effector_pose_[0][0] * end_effector_pose_[0][0] + 
                        end_effector_pose_[1][0] * end_effector_pose_[1][0]);

        bool singular = sy < 1e-6;

        if (!singular) {
            euler[0] = atan2(end_effector_pose_[2][1], end_effector_pose_[2][2]) * 180.0 / M_PI; // Roll (X)
            euler[1] = atan2(-end_effector_pose_[2][0], sy) * 180.0 / M_PI;                      // Pitch (Y)
            euler[2] = atan2(end_effector_pose_[1][0], end_effector_pose_[0][0]) * 180.0 / M_PI; // Yaw (Z)
        } else {
            euler[0] = atan2(-end_effector_pose_[1][2], end_effector_pose_[1][1]) * 180.0 / M_PI;
            euler[1] = atan2(-end_effector_pose_[2][0], sy) * 180.0 / M_PI;
            euler[2] = 0;
        }

        return euler;
    }

    std::string get_robot_mode_name(uint8_t mode)
    {
        switch (mode) {
            case 0: return "Other";
            case 1: return "Idle";
            case 2: return "Move";
            case 3: return "Guiding";
            case 4: return "Reflex";
            case 5: return "User Stopped";
            case 6: return "Automatic Error Recovery";
            default: return "Unknown (" + std::to_string(mode) + ")";
        }
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscriber_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Dados recebidos
    std::vector<double> joint_positions_;
    std::vector<std::string> joint_names_;
    double end_effector_pose_[4][4];
    uint8_t robot_mode_;
    
    // Flags de controle
    bool joint_positions_received_;
    bool robot_state_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::cout << "\n🤖 LEITOR DE POSIÇÃO FRANKA (C++)" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Pressione Ctrl+C para sair" << std::endl;
    std::cout << "O programa irá imprimir a posição a cada 2 segundos" << std::endl;

    auto node = std::make_shared<FrankaPositionReader>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Erro: %s", e.what());
    }

    std::cout << "\n👋 Encerrando leitor de posição..." << std::endl;
    rclcpp::shutdown();
    return 0;
}
