# Tópicos ROS 2 – o que cada tópico está publicando

Este arquivo documenta os principais tópicos (ROS 2 topics) encontrados neste workspace (`franka_ws`), com foco nos pacotes **franka_ros2** e **tension_control**.

> Observação: alguns tópicos são **relativos ao namespace do nó** (ex.: `~/joint_states`). Em tempo de execução eles aparecem como `/<nome_do_no>/<topic>`.

---

## 1) Robot description / visualização

| Tópico | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/robot_description` | `std_msgs/msg/String` (param/"topic" usado pelo RViz/robot_state_publisher) | normalmente `robot_state_publisher` (via parâmetro) | Descrição URDF do robô (string gerada por xacro). Usada pelo RViz/RobotModel e por nós que precisam do modelo. |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` (e outros) | Transformações TF entre frames do robô (árvore cinemática). |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` e `static_transform_publisher` | Transformações estáticas (não mudam no tempo). |

---

## 2) Estados das juntas (arm + gripper)

| Tópico | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` (ros2_control) e/ou nós do gripper (dependendo da configuração) | Posições/velocidades/esforços das juntas do braço (e possivelmente do gripper). É o tópico padrão consumido por `robot_state_publisher` e por ferramentas de visualização. |
| `/<arm_id>_gripper/joint_states` (ex.: `/panda_gripper/joint_states`) | `sensor_msgs/msg/JointState` | `franka_gripper_node` (real) **ou** `fake_gripper_state_publisher.py` (simulado) | Estado das juntas do gripper (tipicamente `panda_finger_joint1` e `panda_finger_joint2`). No script fake, publica posição fixa `[0.035, 0.035]`. |

**Referências no código**
- `franka_gripper/scripts/fake_gripper_state_publisher.py`: publica `~/joint_states`.

---

## 3) Comando de trajetória (braço)

| Tópico / Interface | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` (Action) | Cliente: `tension_control/move_homing` e outros | **Ação** de controle do braço. O cliente envia uma trajetória (juntas + pontos). O controlador executa e retorna feedback/resultado. |
| `/joint_trajectory_controller/joint_trajectory` (quando usado) | `trajectory_msgs/msg/JointTrajectory` | nós de comando de trajetória (varia) | Trajetória em formato tópico (nem sempre habilitado; depende do controller). |

**Referências no código**
- `tension_control/src/move_homing.cpp`: envia goal para `/joint_trajectory_controller/follow_joint_trajectory`.

---

## 4) Estado do robô Franka (FrankaRobotStateBroadcaster)

O `franka_robot_state_broadcaster` publica um tópico interno (relativo) com o estado do robô.

| Tópico | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/<controller_name>/robot_state` (ex.: `/franka_robot_state_broadcaster/robot_state`) | `franka_msgs/msg/FrankaRobotState` | `franka_robot_state_broadcaster` | Mensagem "completa" do estado Franka (ex.: estados internos, poses/matrizes, forças estimadas, etc. conforme `franka_msgs`). |

**Referências no código**
- `franka_robot_state_broadcaster/src/franka_robot_state_broadcaster.cpp`: cria publisher em `"~/" + state_interface_name` onde `state_interface_name = "robot_state"`.

---

## 5) Serviços do Franka hardware params (FrankaParamServiceServer)

Esses são **services** (não tópicos). Ainda assim são essenciais para operação.

| Serviço | Tipo | Quem oferece | Para que serve |
|---|---|---|---|
| `/<node>/set_joint_stiffness` (relativo: `~/set_joint_stiffness`) | `franka_msgs/srv/SetJointStiffness` | `franka_hardware` (service_server) | Ajustar rigidez articular no robô. |
| `~/set_cartesian_stiffness` | `franka_msgs/srv/SetCartesianStiffness` | `franka_hardware` | Ajustar rigidez cartesiana. |
| `~/set_tcp_frame` | `franka_msgs/srv/SetTCPFrame` | `franka_hardware` | Ajustar frame TCP. |
| `~/set_stiffness_frame` | `franka_msgs/srv/SetStiffnessFrame` | `franka_hardware` | Ajustar frame de rigidez. |
| `~/set_force_torque_collision_behavior` | `franka_msgs/srv/SetForceTorqueCollisionBehavior` | `franka_hardware` | Ajustar comportamento de colisão (FT). |
| `~/set_full_collision_behavior` | `franka_msgs/srv/SetFullCollisionBehavior` | `franka_hardware` | Ajustar comportamento de colisão (full). |
| `~/set_load` | `franka_msgs/srv/SetLoad` | `franka_hardware` | Ajustar parâmetros de carga (payload). |

---

## 6) Gripper (ações e serviços)

| Interface | Tipo | Quem oferece | Para que serve |
|---|---|---|---|
| `/<arm_id>_gripper/homing` (relativo: `~/homing`) | `franka_msgs/action/Homing` | `franka_gripper_node` | Executa homing no gripper. |
| `/<arm_id>_gripper/move` (relativo: `~/move`) | `franka_msgs/action/Move` | `franka_gripper_node` | Move o gripper para uma abertura desejada. |
| `/<arm_id>_gripper/grasp` (relativo: `~/grasp`) | `franka_msgs/action/Grasp` | `franka_gripper_node` | Comando de grasp com parâmetros (força/epsilon/velocidade). |
| `/<arm_id>_gripper/stop` (relativo: `~/stop`) | `std_srvs/srv/Trigger` | `franka_gripper_node` | Para o gripper. |
| `/panda_gripper/gripper_action` | `control_msgs/action/GripperCommand` | `franka_gripper_node` | Ação padrão MoveIt/ROS para comando do gripper (posição + esforço). |

---

## 7) Tactile Cameras (pacote tactile_cameras)

### 7.1 Câmeras (publishing de imagens)

| Tópico | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/camera/camera1/image_raw` | `sensor_msgs/msg/Image` | `camera_publisher.py` (tactile_cameras) | Imagem da câmera USB 1. |
| `/camera/camera2/image_raw` | `sensor_msgs/msg/Image` | `camera_publisher.py` (tactile_cameras) | Imagem da câmera USB 2. |
| `/gelsight/left/image_raw` | `sensor_msgs/msg/Image` | `gelsight_camera_node` (C++, tactile_cameras) | Imagem da câmera GelSight esquerda. |
| `/gelsight/right/image_raw` | `sensor_msgs/msg/Image` | `gelsight_camera_node` (C++, tactile_cameras) | Imagem da câmera GelSight direita. |
| `/gelsight/left/camera_info` | `sensor_msgs/msg/CameraInfo` | `gelsight_camera_node` (C++, tactile_cameras) | Informações da câmera esquerda. |
| `/gelsight/right/camera_info` | `sensor_msgs/msg/CameraInfo` | `gelsight_camera_node` (C++, tactile_cameras) | Informações da câmera direita. |

**Referências no código**
- `tactile_cameras/src/camera_publisher.py`: publica imagens de câmeras USB.
- `tactile_cameras/src/gelsight_camera_node.cpp`: publica imagens e info das câmeras GelSight.

### 7.2 Detecção tátil (publicadores de estado/detecção)

| Tópico | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| `/gelsight/object_detected` | `std_msgs/msg/Bool` | `gelsight_object_detection.py` (tactile_cameras) | `true/false` indicando se há objeto detectado pelos sensores táteis. |
| `/gelsight/object_confidence` | `std_msgs/msg/Float32` | `gelsight_object_detection.py` (tactile_cameras) | Confiança (0.0–1.0) da detecção. |
| `/gelsight/detection_status` | `std_msgs/msg/String` | detectores GelSight (tactile_cameras) | Texto com status (ex.: aprendendo background, detectando, etc.). |
| `/gelsight/object_location` | `std_msgs/msg/Int32` | detectores GelSight (tactile_cameras) | Local da detecção: 0=nenhum, 1=cam1, 2=cam2, 3=ambos. |
| `/gelsight/deformation` | `geometry_msgs/msg/Vector3` | `gelsight_marker_detector.py` (tactile_cameras) | Vetor de deformação detectado pelos marcadores GelSight. |

**Referências no código**
- `tactile_cameras/src/gelsight_object_detection.py`: publicador principal de detecção.
- `tactile_cameras/src/gelsight_marker_detector.py`: detecção baseada em rastreamento de marcadores.
- `tactile_cameras/src/gelsight_detector_visual.py`: visualização e feedback visual.
- `tactile_cameras/src/gelsight_detector_debug.py`: versão debug com logging detalhado.

---

## 8) Tension Control (pacote tension_control)

### 8.1 Admittance controller

O arquivo `admittance_controller.cpp` implementa controle por admitância.

| Tópico (esperado pelo código) | Tipo | Quem publica | O que é publicado |
|---|---|---|---|
| *(entrada)* wrench/força | `geometry_msgs/msg/WrenchStamped` | tipicamente `franka_robot_state_broadcaster`/outro nó de FT (depende do sistema) | Forças/torques medidos/estimados para controle por admitância. |
| *(entrada)* trajetória alvo | `geometry_msgs/msg/PoseStamped` | nó de planejamento/primtive (depende) | Pose desejada para seguimento (quando `enable_trajectory_following=true`). |
| *(saída)* comando de velocidade | `geometry_msgs/msg/TwistStamped` | `admittance_controller` | Velocidade cartesiana desejada (quando em modo velocity). |
| *(saída)* comando de pose | `geometry_msgs/msg/PoseStamped` | `admittance_controller` | Pose desejada (quando em modo position). |
| *(saída)* status/debug | `std_msgs/msg/Float64MultiArray` | `admittance_controller` | Arrays com status e/ou debug (conteúdo depende da implementação final). |

---

## 9) Onde completar/ajustar esta documentação

Se você quiser que este arquivo liste **todos** os tópicos do sistema (incluindo MoveIt, RViz, controllers e plugins), posso gerar uma segunda versão baseada em uma lista/scan do runtime (por exemplo, saída do `ros2 topic list` e `ros2 topic info -v`).
