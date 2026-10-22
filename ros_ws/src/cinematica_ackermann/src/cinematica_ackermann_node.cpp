#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "cinematica_interfaces/msg/steering_angles.hpp"
#include "cinematica_interfaces/msg/rear_wheel_velocities.hpp"

using std::placeholders::_1;


// MINHAS CONVENÇÕES
// x+ para frente
// y+ para a esquerda
// rotação positiva: anti-horário, vista de cima
// velocidade linear medida no centro do eixo traseiro


class AckermannKinematics : public rclcpp::Node
{
private:


    //VARIAVEIS INICIAIS
    double linear_velocity;
    double angular_velocity;

    rclcpp::Time command_stamp;
    bool is_command_recepted;

    double steering_angle_left;
    double steering_angle_right;

    double angular_velocity_wheel_left;
    double angular_velocity_wheel_right;



    //VARIAVEIS(PONTEIROS) PARA OS PUBLISHERS E SUBSCRIBERS
    rclcpp::Publisher<cinematica_interfaces::msg::SteeringAngles>::SharedPtr steering_publisher;
    rclcpp::Publisher<cinematica_interfaces::msg::RearWheelVelocities>::SharedPtr wheel_publisher;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription;

    rclcpp::TimerBase::SharedPtr timer;



    //FUNCAO DE VERIFICACAO
    void verify_valid_values(double WHEEL_RADIUS, double WHEEL_BASE, double TRACK_WIDTH, double MAX_STEERING_ANGLE, double TIMEOUT, double PUBLISH_PER_SECOND)
    {
        if (WHEEL_RADIUS <= 0.0 || !std::isfinite(WHEEL_RADIUS))
        {
            RCLCPP_WARN(this->get_logger(), "Raio deve ser finito e maior que zero!");
            rclcpp::shutdown();
            return;
        }

        if (WHEEL_BASE <= 0.0 || !std::isfinite(WHEEL_BASE))
        {
            RCLCPP_WARN(this->get_logger(), "Distância entre os eixos deve ser finita e maior que zero!");
            rclcpp::shutdown();
            return;
        }

        if (TRACK_WIDTH <= 0.0 || !std::isfinite(TRACK_WIDTH))
        {
            RCLCPP_WARN(this->get_logger(), "Bitola deve ser finita e maior que zero!");
            rclcpp::shutdown();
            return;
        }

        if (
            !std::isfinite(MAX_STEERING_ANGLE)
            || MAX_STEERING_ANGLE <= 0.0
            || MAX_STEERING_ANGLE >= M_PI / 2.0
        )
        {
            RCLCPP_WARN(this->get_logger(), "Ângulo máximo deve ser finito, positivo e menor que pi/2!");
            rclcpp::shutdown();
            return;
        }

        if (TIMEOUT <= 0.0 || !std::isfinite(TIMEOUT))
        {
            RCLCPP_WARN(this->get_logger(), "Timeout deve ser finito e maior que zero!");
            rclcpp::shutdown();
            return;
        }

        if (PUBLISH_PER_SECOND <= 0.0 || !std::isfinite(PUBLISH_PER_SECOND))
        {
            RCLCPP_WARN(this->get_logger(), "Frequência de publicação deve ser finita e maior que zero!");
            rclcpp::shutdown();
            return;
        }
    }
    //FIM DA FUNCAO DE VERFICAÇÃO

    void command_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!std::isfinite(msg->twist.linear.x))
        {
            is_command_recepted = false;
            return;
        }

        if (!std::isfinite(msg->twist.angular.z))
        {
            is_command_recepted = false;
            return;
        }

        // Sem o horário de emissão, não há como verificar a validade.
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        {
            is_command_recepted = false;
            return;
        }

        if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u)
        {
            is_command_recepted = false;
            return;
        }

        linear_velocity = msg->twist.linear.x;
        angular_velocity = msg->twist.angular.z;

        // Usar o mesmo relógio evita comparar tempos de fontes diferentes.
        command_stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

        is_command_recepted = true;
    }


    void timer_callback()
    {
        double WHEEL_RADIUS = this->get_parameter("wheel_radius").as_double();
        double WHEEL_BASE = this->get_parameter("wheel_base").as_double();
        double TRACK_WIDTH = this->get_parameter("track_width").as_double();
        double MAX_STEERING_ANGLE = this->get_parameter("max_steering_angle").as_double();
        double TIMEOUT = this->get_parameter("timeout").as_double();
        double PUBLISH_PER_SECOND = this->get_parameter("publish_per_second").as_double();

        verify_valid_values(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH, MAX_STEERING_ANGLE, TIMEOUT, PUBLISH_PER_SECOND);

        if (!rclcpp::ok())
        {
            return;
        }


        // A parada é mantida quando n existe um comando valido.
        steering_angle_left = 0.0;
        steering_angle_right = 0.0;
        angular_velocity_wheel_left = 0.0;
        angular_velocity_wheel_right = 0.0;


        if (is_command_recepted)
        {
            double command_age = (this->now() - command_stamp).seconds();

            if (command_age < 0.0 || command_age >= TIMEOUT)
            {
                is_command_recepted = false;
            }
            else
            {
                // Ackermann não gira no lugar...
                if (std::abs(linear_velocity) >= 0.001)
                {
                    double steering_tangent = std::tan(MAX_STEERING_ANGLE);
                    double maximum_curvature = steering_tangent / (WHEEL_BASE + steering_tangent * TRACK_WIDTH / 2.0);
                    double curvature = angular_velocity / linear_velocity;

                    if (curvature > maximum_curvature)
                    {
                        curvature = maximum_curvature;
                    }

                    if (curvature < -maximum_curvature)
                    {
                        curvature = -maximum_curvature;
                    }

                    // A mesma curvatura mantém direção e velocidades compatíveis.
                    steering_angle_left = std::atan(WHEEL_BASE * curvature / (1.0 - curvature * TRACK_WIDTH / 2.0));
                    steering_angle_right = std::atan(WHEEL_BASE * curvature / (1.0 + curvature * TRACK_WIDTH / 2.0));

                    angular_velocity_wheel_left = linear_velocity * (1.0 - curvature * TRACK_WIDTH / 2.0) / WHEEL_RADIUS;
                    angular_velocity_wheel_right = linear_velocity * (1.0 + curvature * TRACK_WIDTH / 2.0) / WHEEL_RADIUS;
                }
            }
        }

        // Resultados inválidos não devem virar comandos para as rodas.
        if (
            !std::isfinite(steering_angle_left)
            || !std::isfinite(steering_angle_right)
            || !std::isfinite(angular_velocity_wheel_left)
            || !std::isfinite(angular_velocity_wheel_right)
        )
        {
            steering_angle_left = 0.0;
            steering_angle_right = 0.0;

            angular_velocity_wheel_left = 0.0;
            angular_velocity_wheel_right = 0.0;

            is_command_recepted = false;
        }


        //AQUI HAS OS PUBLISHERS

        auto steering_msg = cinematica_interfaces::msg::SteeringAngles();

        steering_msg.front_left_angle = steering_angle_left;
        steering_msg.front_right_angle = steering_angle_right;

        steering_publisher->publish(steering_msg);

        auto wheel_msg = cinematica_interfaces::msg::RearWheelVelocities();

        wheel_msg.rear_left_velocity = angular_velocity_wheel_left;
        wheel_msg.rear_right_velocity = angular_velocity_wheel_right;

        wheel_publisher->publish(wheel_msg);
    }


public:

    AckermannKinematics() : Node("cinematica_ackermann_node"), command_stamp(0, 0, this->get_clock()->get_clock_type())
    {
        // PARÂMETROS
        this->declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("wheel_base", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("track_width", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("max_steering_angle", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("timeout", rclcpp::ParameterType::PARAMETER_DOUBLE);
        this->declare_parameter("publish_per_second", rclcpp::ParameterType::PARAMETER_DOUBLE);

        double WHEEL_RADIUS = this->get_parameter("wheel_radius").as_double();
        double WHEEL_BASE = this->get_parameter("wheel_base").as_double();
        double TRACK_WIDTH = this->get_parameter("track_width").as_double();
        double MAX_STEERING_ANGLE = this->get_parameter("max_steering_angle").as_double();
        double TIMEOUT = this->get_parameter("timeout").as_double();
        double PUBLISH_PER_SECOND = this->get_parameter("publish_per_second").as_double();

        verify_valid_values(WHEEL_RADIUS, WHEEL_BASE, TRACK_WIDTH, MAX_STEERING_ANGLE, TIMEOUT, PUBLISH_PER_SECOND);

        if (!rclcpp::ok())
        {
            return;
        }

        // VARIÁVEIS
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        is_command_recepted = false;

        steering_angle_left = 0.0;
        steering_angle_right = 0.0;

        angular_velocity_wheel_left = 0.0;
        angular_velocity_wheel_right = 0.0;

        // PUBLISHERS
        steering_publisher = this->create_publisher<cinematica_interfaces::msg::SteeringAngles>("steering_angles", 10);
        wheel_publisher = this->create_publisher<cinematica_interfaces::msg::RearWheelVelocities>("rear_wheel_velocities", 10);

        // SUBSCRIBERS
        command_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 10, std::bind(&AckermannKinematics::command_callback, this, _1));

        // O timer mantém a publicação mesmo quando nenhum comando novo chega.
        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / PUBLISH_PER_SECOND));

        timer = this->create_wall_timer(period, std::bind(&AckermannKinematics::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nó da cinemática Ackermann iniciado!");
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AckermannKinematics>();

    if (rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}