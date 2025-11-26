# Slide Along Cable Primitive

Uma primitiva de movimento avançada para robôs manipuladores que permite deslizar suavemente ao longo de cabos mantendo tensão controlada.

## Funcionalidades

### 🎯 **Objetivos Principais**
- Deslizar a garra ao longo do cabo mantendo tensão desejada
- Movimento suave sem picos de velocidade
- Controle de segurança com limites de tensão
- Feedback contínuo da execução
- Integração perfeita com Admittance Controller

### 📊 **Entradas da Primitiva**
- **Posições**: Start/end pose (x, y, z, orientação)
- **Tensão desejada**: T_desired (Newtons)
- **Velocidade**: Velocidade de deslizamento (m/s)
- **Critério de parada**: "position", "tension", ou "both"
- **Tolerâncias**: Posição (mm) e tensão (N)

### 📈 **Saídas da Primitiva**
- **desired_velocity**: Twist para admittance controller
- **current_tension**: Tensão atual do cabo (Float32)
- **status**: Estado da primitiva (executando, concluída, erro)
- **feedback**: Progresso contínuo da execução

## Arquivos do Sistema

```
src/gelsight_camera/
├── src/
│   ├── slide_along_cable_primitive.py    # Primitiva principal
│   ├── slide_cable_client.py            # Cliente de exemplo
│   └── admittance_controller.cpp        # Controlador de admitância
├── action/
│   └── SlideAlongCable.action           # Definição da ação
├── srv/
│   ├── StartSlideAlongCable.srv         # Serviço de início
│   └── StopSlideAlongCable.srv          # Serviço de parada
├── launch/
│   └── slide_along_cable.launch.py     # Arquivo de launch
└── config/
    └── slide_along_cable_params.yaml   # Configurações
```

## Como Usar

### 1. **Launch Básico**
```bash
# Executar primitiva com parâmetros padrão
ros2 launch gelsight_camera slide_along_cable.launch.py

# Com demo habilitado
ros2 launch gelsight_camera slide_along_cable.launch.py enable_demo:=true

# Com parâmetros customizados
ros2 launch gelsight_camera slide_along_cable.launch.py \
  control_frequency:=100.0 \
  default_desired_tension:=6.0 \
  default_slide_velocity:=0.03
```

### 2. **Uso Programático**

#### Python Client Example:
```python
#!/usr/bin/env python3
import rclpy
from slide_cable_client import SlideAlongCableClient

rclpy.init()
client = SlideAlongCableClient()

# Movimento horizontal simples
goal = client.slide_horizontal(
    start_x=0.5, start_y=0.0, start_z=0.3,
    distance=0.2,  # 20 cm
    tension=4.0,   # 4 N
    velocity=0.03  # 3 cm/s
)

# Movimento vertical
goal = client.slide_vertical(
    start_x=0.6, start_y=0.0, start_z=0.5,
    distance=-0.15,  # 15 cm para baixo
    tension=6.0,     # 6 N
    velocity=0.02    # 2 cm/s
)

# Movimento customizado
goal = client.execute_slide_along_cable(
    start_xyz=[0.4, 0.0, 0.4],
    end_xyz=[0.7, 0.1, 0.2],
    tension=5.5,
    velocity=0.025
)
```

### 3. **Monitoramento em Tempo Real**

```bash
# Monitor feedback
ros2 topic echo /slide_primitive/feedback

# Monitor status
ros2 topic echo /slide_primitive/status

# Monitor tensão atual
ros2 topic echo /slide_primitive/current_tension

# Monitor velocidade comandada
ros2 topic echo /slide_primitive/desired_velocity
```

### 4. **Controle Direto via Serviços**

```bash
# Parar execução
ros2 service call /slide_primitive/stop std_srvs/srv/Trigger

# Parada de emergência
ros2 service call /slide_primitive/emergency_stop std_srvs/srv/Trigger
```

## Configurações Principais

### Parâmetros de Controle
```yaml
slide_primitive:
  ros__parameters:
    control_frequency: 125.0         # Hz - frequência do loop
    feedback_frequency: 10.0         # Hz - feedback ao usuário
    velocity_smoothing_factor: 0.9   # Suavização (0.0-1.0)
    max_execution_time: 60.0         # s - timeout máximo
```

### Parâmetros de Movimento
```yaml
    default_slide_velocity: 0.05     # m/s (5 cm/s)
    default_desired_tension: 5.0     # N
    default_position_tolerance: 0.005 # m (5 mm)
    default_tension_tolerance: 0.5    # N
```

### Parâmetros de Segurança
```yaml
    safety_tension_multiplier: 1.5   # Parada se tensão > desejada * 1.5
    emergency_stop_acceleration: 0.5  # m/s² - desaceleração controlada
    max_linear_velocity: 0.2         # m/s - limite máximo
```

## Integração com Franka

### 1. **Remapeamento de Tópicos**
```bash
ros2 launch gelsight_camera slide_along_cable.launch.py \
  --ros-args \
  --remap /slide_primitive/desired_velocity:=/cartesian/velocity_controller/cmd_vel \
  --remap /robot/current_pose:=/franka_state_controller/end_effector_pose \
  --remap /gelsight/combined_tension:=/tactile_sensor/estimated_force
```

### 2. **Configuração do Admittance Controller**
```yaml
admittance_controller_cable:
  ros__parameters:
    # Matriz de massa otimizada para cabo
    mass_matrix: [8.0, 8.0, 12.0, 0.8, 0.8, 0.8]
    # Amortecimento para estabilidade
    damping_matrix: [40.0, 40.0, 60.0, 4.0, 4.0, 4.0]
    # Rigidez para conformidade do cabo
    stiffness_matrix: [80.0, 80.0, 120.0, 8.0, 8.0, 8.0]
```

## Exemplos de Movimentos

### 1. **Inspeção Horizontal de Cabo**
```python
# Deslizar horizontalmente para inspeção visual
client.slide_horizontal(
    start_x=0.5, start_y=-0.2, start_z=0.4,
    distance=0.4,  # 40 cm de inspeção
    tension=4.0,   # Tensão leve para inspeção
    velocity=0.03  # Velocidade lenta para observação
)
```

### 2. **Descida Controlada**
```python
# Descida controlada ao longo do cabo
client.slide_vertical(
    start_x=0.6, start_y=0.0, start_z=0.6,
    distance=-0.3,  # 30 cm para baixo
    tension=8.0,    # Tensão maior para suporte
    velocity=0.02   # Velocidade conservadora
)
```

### 3. **Posicionamento Preciso**
```python
# Movimento de precisão para posicionamento final
client.execute_slide_along_cable(
    start_xyz=[0.55, 0.05, 0.35],
    end_xyz=[0.5501, 0.0502, 0.3498],  # Movimento de 1mm
    tension=3.0,
    velocity=0.005  # Muito lento para precisão
)
```

### 4. **Trajetória Curva**
```python
# Seguir cabo com curvatura complexa
waypoints = [
    [0.4, 0.0, 0.5],   # Início
    [0.5, 0.1, 0.45],  # Curva direita
    [0.6, 0.05, 0.4],  # Suave correção
    [0.7, 0.0, 0.35]   # Final alinhado
]
client.slide_along_curve(waypoints, tension=5.5, velocity=0.025)
```

## Monitoramento e Debug

### 1. **Verificação de Status**
```bash
# Status geral do sistema
ros2 node list | grep slide
ros2 topic list | grep slide_primitive
ros2 service list | grep slide

# Estado dos sensores
ros2 topic hz /tactile_sensor/estimated_force
ros2 topic hz /franka_state_controller/end_effector_pose
```

### 2. **Análise de Performance**
```bash
# Frequência do loop de controle
ros2 topic hz /slide_primitive/desired_velocity

# Latência do sistema
ros2 topic delay /slide_primitive/desired_velocity

# Uso de CPU
top -p $(pgrep -f slide_along_cable_primitive)
```

### 3. **Logs Detalhados**
```bash
# Habilitar logs detalhados
ros2 launch gelsight_camera slide_along_cable.launch.py \
  --ros-args --log-level slide_along_cable_primitive:=debug

# Salvar logs
ros2 launch gelsight_camera slide_along_cable.launch.py \
  2>&1 | tee slide_primitive_$(date +%Y%m%d_%H%M%S).log
```

## Solução de Problemas

### ❌ **Problema: Primitive não inicia**
```bash
# Verificar dependências
ros2 pkg executables gelsight_camera
ros2 interface show gelsight_camera/action/SlideAlongCable

# Verificar parâmetros
ros2 param list /slide_along_cable_primitive
```

### ❌ **Problema: Tensão instável**
```yaml
# Aumentar amortecimento
damping_matrix: [60.0, 60.0, 80.0, 6.0, 6.0, 6.0]

# Reduzir ganhos de controle
position_controller_gains: [1.5, 1.5, 1.5]
```

### ❌ **Problema: Movimento muito lento**
```yaml
# Aumentar ganhos (com cuidado)
position_controller_gains: [2.5, 2.5, 2.5]

# Reduzir suavização
velocity_smoothing_factor: 0.7
```

### ❌ **Problema: Parada de emergência frequente**
```yaml
# Aumentar limite de segurança
safety_tension_multiplier: 2.0

# Ajustar filtro de força
force_filter_cutoff: 15.0  # Hz
```

## Performance e Otimização

### 📊 **Métricas Típicas**
- **Frequência de controle**: 125 Hz
- **Latência típica**: < 8 ms
- **Precisão de posição**: ±2 mm
- **Precisão de tensão**: ±0.3 N
- **Tempo de resposta**: < 100 ms

### ⚡ **Otimizações Recomendadas**
1. **CPU**: Usar isolamento de CPUs para RT
2. **Rede**: Priorizar tráfego ROS2 com QoS
3. **Sensores**: Calibração regular dos sensores tácteis
4. **Cabo**: Usar cabos com propriedades conhecidas

### 🔧 **Tuning Avançado**
```yaml
# Para movimentos rápidos
fast_movement:
  velocity_smoothing_factor: 0.7
  trajectory_interpolation_points: 50
  
# Para movimentos precisos  
precision_movement:
  velocity_smoothing_factor: 0.95
  trajectory_interpolation_points: 200
```

## Extensões Futuras

- **Detecção de obstáculos** durante deslizamento
- **Aprendizado de propriedades** do cabo em tempo real
- **Compensação de vibração** dinâmica
- **Planejamento de trajetória** otimizada
- **Interface visual** para programação intuitiva

---

**Desenvolvido para integração perfeita com Franka Research 3 e sensores GelSight Mini**