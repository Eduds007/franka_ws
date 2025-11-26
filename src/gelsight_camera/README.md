# GelSight Tactile Control System

Sistema completo de controle táctil para robôs usando sensores GelSight com controlador de admittance para controle de tensão de cabo.

## 🎯 Componentes do Sistema

### 1. **Camera Nodes** (`gelsight_camera_node`)
- Captura imagens das câmeras GelSight esquerda e direita
- Publica stream contínuo de imagens em `/gelsight/left/image_raw` e `/gelsight/right/image_raw`

### 2. **Tactile Analysis Node** (`tactile_sensor_node`)
- Analisa imagens usando técnicas do GelSight (depth mapping, contact detection)
- Estima forças aplicadas baseado na deformação do gel
- Publica forças estimadas em `/gelsight/combined_force_estimation`

### 3. **Admittance Controller** (`admittance_controller`)
- Implementa controle de admittance para tensão de cabo
- **Formula:** `M*ẍ + B*ẋ + K*(x - x_d) = F_ext - F_des`
- Gera comandos de velocidade/posição baseados na diferença de força

## 🚀 Como Usar

### Lançar Sistema Completo
```bash
# Sistema completo com câmeras reais
ros2 launch gelsight_camera cable_tension_control.launch.py

# Apenas análise táctil e controle (sem câmeras - para simulação)
ros2 launch gelsight_camera cable_tension_control.launch.py enable_cameras:=false

# Com força desejada personalizada
ros2 launch gelsight_camera cable_tension_control.launch.py desired_force:=8.0
```

### Ativar/Desativar Controlador
```bash
# Ativar controlador de admittance
ros2 service call /admittance_controller/activate std_srvs/srv/Trigger

# Desativar controlador
ros2 service call /admittance_controller/deactivate std_srvs/srv/Trigger
```

### Comandos de Trajetória (opcional)
```bash
# Enviar posição desejada para seguir trajetória
ros2 topic pub /admittance_controller/desired_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'base_link'
pose:
  position: {x: 0.0, y: 0.0, z: 0.5}
  orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}"
```

## 📊 Tópicos Principais

### Entradas
- `/gelsight/left/image_raw` - Imagem câmera esquerda
- `/gelsight/right/image_raw` - Imagem câmera direita
- `/admittance_controller/desired_pose` - Trajetória desejada (opcional)

### Saídas
- `/gelsight/combined_force_estimation` - Força estimada (WrenchStamped)
- `/admittance_controller/cmd_vel` - Comando velocidade (TwistStamped)
- `/admittance_controller/cmd_pose` - Comando posição (PoseStamped)
- `/admittance_controller/status` - Status do controlador

### Debug/Monitoramento
- `/gelsight/left/debug_image` - Visualização análise táctil esquerda
- `/gelsight/right/debug_image` - Visualização análise táctil direita
- `/admittance_controller/debug` - Debug dados controlador

## ⚙️ Parâmetros de Calibração

### Análise Táctil
```yaml
tactile_sensor_node:
  ros__parameters:
    force_scaling_factor: 150.0    # N/mm - calibração específica do gel
    gel_stiffness: 2000.0         # N/m - rigidez do gel GelSight
    contact_threshold: 25         # threshold detecção contato
```

### Controlador de Admittance
```yaml
admittance_controller:
  ros__parameters:
    desired_force_z: 5.0         # N - tensão desejada do cabo
    
    # Matriz M (inertia virtual)
    mass_z: 1.0                  # kg - responsividade (menor = mais rápido)
    
    # Matriz B (amortecimento virtual)  
    damping_z: 50.0             # Ns/m - estabilidade (maior = mais estável)
    
    # Matriz K (rigidez virtual)
    stiffness_z: 100.0          # N/m - rigidez (maior = mais rígido)
```

## 🔧 Integração com Franka

### 1. Conectar ao Controlador do Franka
```bash
# Remapear saída do admittance controller para entrada do Franka
ros2 run gelsight_camera admittance_controller --ros-args \
  --remap /admittance_controller/cmd_vel:=/cartesian/cmd_vel
```

### 2. Usar com Trajectory Following
O controlador pode ser combinado com seguimento de trajetória:
- **Trajetória nominal:** seguir curva desejada
- **Admittance:** ajustar força/tensão mantendo trajetória

### 3. Monitoramento
```bash
# Visualizar forças em tempo real
ros2 topic echo /gelsight/combined_force_estimation

# Status do controlador
ros2 topic echo /admittance_controller/status
```

## 🎛️ Ajuste de Parâmetros

### Para Controle Mais Suave:
- ↑ `mass_z` (maior inércia virtual)
- ↑ `damping_z` (maior amortecimento)

### Para Controle Mais Responsivo:
- ↓ `mass_z` (menor inércia virtual)
- ↑ `stiffness_z` (maior rigidez)

### Para Melhor Tracking de Força:
- Ajustar `force_scaling_factor` baseado em calibração
- Ajustar `contact_threshold` para detecção de contato

## 🔍 Troubleshooting

### Controlador Não Ativa
```bash
# Verificar se há dados de força
ros2 topic hz /gelsight/combined_force_estimation

# Verificar logs
ros2 service call /admittance_controller/activate std_srvs/srv/Trigger
```

### Oscilações no Controle
- ↑ `damping_z`
- ↓ `stiffness_z`
- Verificar `control_frequency` (deve ser ≥ 100 Hz)

### Resposta Muito Lenta
- ↓ `mass_z`
- ↑ `stiffness_z`
- Verificar latência dos dados de força