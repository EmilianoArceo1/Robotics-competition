# Migración a ROS 2

## Decisión de arquitectura

Todo el proyecto se implementa en C++. La capa de movimiento se encuentra en
`ros2_ws/src/exploration_robot_control` y los algoritmos se encuentran en
`ros2_ws/src/exploration_robot_exploration`.

- Gazebo calculará la física durante la simulación.
- Una interfaz de `ros2_control` leerá y escribirá el hardware real.
- El nodo C++ `acceleration_controller_node` limitará e integrará órdenes de
  aceleración y publicará velocidades.

## Contrato del controlador

Entrada:

- Topic: `cmd_accel`
- Tipo: `geometry_msgs/msg/AccelStamped`
- Campos usados: `accel.linear.x` y `accel.angular.z`

Salida:

- Topic: `cmd_vel`
- Tipo: `geometry_msgs/msg/TwistStamped`
- Campos usados: `twist.linear.x` y `twist.angular.z`

Si no llega una orden nueva durante `command_timeout`, la salida se lleva a
cero. Los límites y la frecuencia se configuran en
`config/acceleration_controller.yaml`.

## Compilación prevista en Ubuntu 24.04 con ROS 2 Jazzy

```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Ejecución provisional del nodo:

```bash
ros2 run exploration_robot_control acceleration_controller_node \
  --ros-args \
  --params-file src/exploration_robot_control/config/acceleration_controller.yaml
```

## Próximos paquetes

1. `exploration_robot_description`: URDF/Xacro, geometría, masas y sensores.
2. `exploration_robot_gazebo`: mundo, spawn y `gz_ros2_control`.
3. `exploration_robot_bringup`: lanzamiento conjunto de Gazebo, control y RViz.
4. Nodos ROS 2 C++ para publicar mapa, fronteras, rutas y metas.

No se debe portar `RobotPhysics.step()` a ROS 2: la pose provendrá de odometría
y TF, calculados por Gazebo o por los sensores del robot real.
