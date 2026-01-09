# Migração: Câmeras GelSight → Pacote tactile_cameras

## Resumo

Todas as funcionalidades relacionadas às câmeras táteis GelSight foram movidas do pacote `tension_control` para o novo pacote `tactile_cameras`.

## Novo Pacote: tactile_cameras

**Localização:** `/home/nuc_6g_life_3/franka_ws/src/tactile_cameras`

### Estrutura
```
tactile_cameras/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   ├── gelsight_camera_node.cpp
│   ├── camera_publisher.py
│   ├── camera_viewer.py
│   ├── gelsight_object_detection.py
│   ├── gelsight_detector_visual.py
│   ├── gelsight_marker_detector.py
│   └── gelsight_detector_debug.py
├── launch/
│   ├── camera_demo.launch.xml
│   ├── gelsight_camera.launch.py
│   ├── gelsight_debug.launch.xml
│   ├── gelsight_detection_test.launch.xml
│   └── gelsight_marker_detection_test.launch.xml
└── config/
    └── gelsight_params.yaml
```

## Arquivos Movidos

### De tension_control → tactile_cameras

**Código-fonte (src/):**
- `gelsight_camera_node.cpp`
- `camera_publisher.py`
- `camera_viewer.py`
- `gelsight_object_detection.py`
- `gelsight_detector_visual.py`
- `gelsight_marker_detector.py`
- `gelsight_detector_debug.py`

**Launch files:**
- `camera_demo.launch.xml`
- `gelsight_camera.launch.py`
- `gelsight_debug.launch.xml`
- `gelsight_detection_test.launch.xml`
- `gelsight_marker_detection_test.launch.xml`

**Configuração:**
- `gelsight_params.yaml`

## Mudanças no tension_control

### Removido do CMakeLists.txt:
- Dependência `cv_bridge`
- Dependência `OpenCV`
- Executável `gelsight_camera_node`
- Instalação de scripts Python relacionados a câmeras
- Referências a arquivos srv que não existiam

### Removido do package.xml:
- `<build_depend>cv_bridge</build_depend>`
- `<build_depend>opencv4</build_depend>`
- `<exec_depend>cv_bridge</exec_depend>`
- `<exec_depend>opencv4</exec_depend>`

## Como Usar o Novo Pacote

### 1. Build
```bash
cd /home/nuc_6g_life_3/franka_ws
colcon build --packages-select tactile_cameras --symlink-install
source install/setup.bash
```

### 2. Exemplos de Uso

**Visualizar câmeras:**
```bash
ros2 launch tactile_cameras camera_demo.launch.xml
```

**Câmeras GelSight (esquerda/direita):**
```bash
ros2 launch tactile_cameras gelsight_camera.launch.py
```

**Teste de detecção de objetos:**
```bash
ros2 launch tactile_cameras gelsight_detection_test.launch.xml
```

**Teste de detecção por marcadores:**
```bash
ros2 launch tactile_cameras gelsight_marker_detection_test.launch.xml
```

**Debug:**
```bash
ros2 launch tactile_cameras gelsight_debug.launch.xml
```

**Viewer standalone:**
```bash
ros2 run tactile_cameras camera_viewer --ros-args -p image_topic:=/camera/camera1/image_raw
```

## Atualizações Necessárias

Se você tem scripts ou launch files em outros lugares que usavam `tension_control` para câmeras, atualize as referências:

**Antes:**
```xml
<node pkg="tension_control" exec="camera_publisher.py" .../>
```

**Depois:**
```xml
<node pkg="tactile_cameras" exec="camera_publisher.py" .../>
```

## Tópicos Publicados

O novo pacote publica os mesmos tópicos:
- `/camera/camera1/image_raw` - sensor_msgs/Image
- `/camera/camera2/image_raw` - sensor_msgs/Image
- `/gelsight/left/image_raw` - sensor_msgs/Image
- `/gelsight/right/image_raw` - sensor_msgs/Image
- `/gelsight/object_detected` - std_msgs/Bool
- `/gelsight/deformation` - geometry_msgs/Vector3

## Status

✅ Pacote `tactile_cameras` criado e compilado com sucesso  
✅ Pacote `tension_control` limpo e recompilado sem dependências de câmera  
✅ Todos os launch files atualizados para usar `tactile_cameras`  
✅ README criado para o novo pacote
