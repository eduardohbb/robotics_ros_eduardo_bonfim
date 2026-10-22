# Projeto 2 — Cinemática em ROS 2

Neste projeto, implementei dois nós no ROS 2 Jazzy: um diferencial em Python, que calcula as velocidades das quatro rodas, e um Ackermann em C++, que calcula os ângulos dianteiros e as velocidades traseiras.

Os dois recebem comandos em `/cmd_vel`, usando `geometry_msgs/msg/TwistStamped`. A velocidade linear vem de `twist.linear.x` e a angular de `twist.angular.z`. O `header.stamp` é usado para verificar a validade: quando o comando vence, o timer publica velocidades zero.

O código ficou dividido em `cinematica_diferencial`, `cinematica_ackermann` e `cinematica_bringup`. Também separei as mensagens personalizadas em arquivos `.msg` no pacote adicional `cinematica_interfaces`. O bringup carrega o YAML e inicia somente o modelo escolhido.

A geometria fica em `config/parametros_diferencial.yaml` e `config/parametros_ackermann.yaml`, nos respectivos pacotes. O diferencial usa `wheel_radius` e `wheel_separation`; o Ackermann usa `wheel_radius`, `wheel_base`, `track_width` e `max_steering_angle`. Esses valores podem mudar com `ros2 param set`, sem recompilar. O `timeout` é de 0.5 s e `publish_per_second` é de 20 Hz; a frequência é definida ao iniciar.

As velocidades das rodas são publicadas em rad/s, e os ângulos em radianos. O Ackermann respeita o limite de direção e não gira no lugar.

## Como rodar

Com os pacotes em `~/ros_ws/src`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

Para iniciar o diferencial:

```bash
ros2 launch cinematica_bringup cinematica_bringup.launch.py cinematica:=diferencial
```

Para iniciar o Ackermann, encerre o launch anterior e execute:

```bash
ros2 launch cinematica_bringup cinematica_bringup.launch.py cinematica:=ackermann
```

Em novos terminais, carregue o ambiente ROS e `~/ros_ws/install/setup.bash`. Sem informar o modelo, o padrão é diferencial; uma opção inválida gera erro.

## Testes e entrega

Fiz a maioria dos testes com `ros2 topic echo`, acompanhando `/wheel_velocities`, `/steering_angles` e `/rear_wheel_velocities`. Enviei os comandos com `ros2 topic pub` e conferi reta, curvas, ré, giro diferencial, limite de direção, parada por timeout, mensagens antigas e mudanças nos parâmetros. Também conferi a seleção dos nós pelo launch.

Além dos pacotes completos, o PDF pede um documento manuscrito com a dedução matemática dos dois modelos e a identificação das grandezas que viraram parâmetros, usando os mesmos nomes do código.