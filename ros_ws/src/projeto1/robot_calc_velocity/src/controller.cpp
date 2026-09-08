#include <memory>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::vector<double> calculate_velocity(std::vector<double> target, std::vector<double> current)
{

    //Bloco para extrair valores de target
    //Variaveis que criei pra facilitar a compreensão do código.
    double x_target = target.at(0);
    double y_target = target.at(1);
    double yaw_target = target.at(2);

    //Bloco para extrair os valores de current
    //Veriaveis que criei pra facilitar a compreensão do código.
    double x_current = current.at(0);
    double y_current = current.at(1);
    double yaw_current = current.at(2);

    //Erros do mundo real
    double erro_x = x_target - x_current;
    double erro_y = y_target - y_current;
    double yaw = atan2(sin(yaw_target - yaw_current),cos(yaw_target - yaw_current));

    //Distância do Robô ao ponto desejado
    double rho = sqrt(pow(erro_x,2) + pow(erro_y,2));

    //Angulo de Vector que aponta para o ponto
    double theta = atan2(erro_y,erro_x);


    //================== Máquina de Estados ==================
    //CONSTANTES PARA DAR CERTO.
    const double TOLERANCIA_LINEAR = 0.05;
    const double TOLERANCIA_ANGULAR = 0.001;

    //velocidades de retono
    double velocity = 0;
    double omega = 0;
    //std::vector<double> return_velocities = {velocity,omega};

    //angulo entre tehta e yaw_current
    double erro_theta = atan2(sin(theta - yaw_current),cos(theta - yaw_current));

    //constantes de proporcionalidade
    const double PROPORCIONAL_LINEAR = 0.05;
    const double PROPORCIONAL_ANGULAR = 0.1;

    //velocidades máximas pra n queimar o motor
    const double MAX_LINEAR = 1.5;
    const double MAX_ANGULAR = 1;

    //velocidades minimas pra n fuder a sequencia
    const double MIN_LINEAR = 0.01;
    const double MIN_ANGULAR = 0.001;

    //1. - Girando em direção a (x_target,y_target)
    // Correção: abs() de int trunca double, troquei por std::fabs para comparar direito
    if(rho > TOLERANCIA_LINEAR && std::fabs(erro_theta) > TOLERANCIA_ANGULAR)
    {

        //Horário ou antihorário, eis a questão
        if(erro_theta >= 0){
            omega = PROPORCIONAL_ANGULAR * erro_theta;
            velocity = 0;
        }else{
            omega = (-1)*PROPORCIONAL_ANGULAR * erro_theta;
            velocity = 0;
        }

        //Cuidado pro motor não queimar
        if(omega > MAX_ANGULAR){
            omega = MAX_ANGULAR;
        }

        //Velocidade Mínima pra n tender ao infinito
        if(std::fabs(omega) < MIN_ANGULAR){
            omega = MIN_ANGULAR;
        }
    }

    //2. - Se movendo em direção a (x_target,y_target)
    else if (rho > TOLERANCIA_LINEAR && std::fabs(erro_theta) <= TOLERANCIA_ANGULAR)
    {
        velocity = PROPORCIONAL_LINEAR * rho;
        omega = 0;

        //Cuidado pro motor não queimar
        if(velocity > MAX_LINEAR){
            velocity = MAX_LINEAR;
        }

        //Velocidade Mínima pra n tender ao infinito
        // Correção: aqui era pra setar velocity, não omega
        if(std::fabs(velocity) < MIN_LINEAR){
            velocity = MIN_LINEAR;
        }
    }

    //3. Girando até chegar em yaw_target
    else if (rho <= TOLERANCIA_LINEAR and std::fabs(yaw) > TOLERANCIA_ANGULAR)
    {
        //horário ou antihorario, eis a questão
        if(yaw > 0){
            omega = PROPORCIONAL_ANGULAR * yaw;
            velocity = 0;
        }else{
            omega = PROPORCIONAL_ANGULAR * yaw;
            velocity = 0;
        }

        //cuidado pro motor n queimar
        if(omega > MAX_ANGULAR){
            omega = MAX_ANGULAR;
        }

        //velocidade mínima pra n enrolar
        if(std::fabs(omega) < MIN_ANGULAR)
        {
            omega = MIN_ANGULAR;
        }

    }

    //4 - Chegou em seu destino
    else if(rho <= TOLERANCIA_LINEAR && std::fabs(yaw) <= TOLERANCIA_ANGULAR)
    {
        velocity = 0;
        omega = 0;
    }

    std::vector<double> return_velocities = {velocity,omega};

    return return_velocities;

}

class ControllerNode : public rclcpp::Node
{
private:
    //Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    //Timer 10 Hz
    rclcpp::TimerBase::SharedPtr timer_;

    //estado interno do nó
    geometry_msgs::msg::Vector3 goal_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_goal_;   // true assim que a primeira /goal chegar
    bool has_pose_;   // true assim que a primeira /robot_position chegar

    // Extrai o yaw (rotação em torno de Z) a partir do quaternion de orientation.
    // Fórmula padrão de quaternion -> yaw, sem precisar de dependência extra (tf2).
    // Eu não quis implementar a dependência de tf2 por que não queria implicar mais coisas
    // quis deixar isso "cru" afim de entender mais como implementar um subscriber e publiusher apenas
    // evitar colocar novos pacotes para entender. Uma coisa que quero fazer é reemplementar esse projeto
    // usando essa depêndencia.
    static double quaternion_2_yaw(const geometry_msgs::msg::Quaternion & q)
    {

        //para ter uam noção de conversão eu usei esse vídeo:
        //https://www.youtube.com/watch?v=bKd2lPjl92c
        //meio que talvez ajude alguém futuramente, n sei, fica a recomendação.
        
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Callback do subscriber de /goal
    void goal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        goal_ = *msg;
        has_goal_ = true;
    }

    // Callback do subscriber de /robot_position
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        has_pose_ = true;
    }

    void control_loop()
    {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "way2go";

        if (!has_goal_ || !has_pose_) {
            // Requisito do projeto: sem objetivo (ou sem pose ainda) recebido, robô fica parado.
            // linear e angular já nascem zerados.
        } else {
            double yaw_current = quaternion_2_yaw(current_pose_.orientation);

            std::vector<double> target = {goal_.x, goal_.y, yaw_current};
            std::vector<double> current = {
                current_pose_.position.x,
                current_pose_.position.y,
                yaw_current
            };

            std::vector<double> vel = calculate_velocity(target, current);

            cmd.twist.linear.x = vel.at(0);
            cmd.twist.angular.z = vel.at(1);
        }

        cmd_vel_pub_->publish(cmd);
    }

public:
    ControllerNode() : Node("controller_node"), has_goal_(false), has_pose_(false)
    {
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "goal", 10, std::bind(&ControllerNode::goal_callback, this, _1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_position", 10, std::bind(&ControllerNode::pose_callback, this, _1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel", 10);

        // 10 Hz = a cada 100 ms, exigido pelo enunciado
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ControllerNode::control_loop, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}