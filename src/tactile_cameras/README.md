# Tactile Cameras Package

ROS 2 package for GelSight tactile camera acquisition and processing.

## Overview

Este pacote é responsável pela aquisição e processamento de imagens das câmeras táteis GelSight. Inclui nós para publicação de imagens, detecção de objetos, análise de marcadores e visualização.

## Nodes

### C++ Nodes

- **gelsight_camera_node**: Nó para aquisição de imagens das câmeras GelSight

### Python Nodes

- **camera_publisher.py**: Publica imagens de múltiplas câmeras USB
- **camera_viewer.py**: Visualiza streams de câmera em tempo real
- **gelsight_object_detection.py**: Detecta objetos usando análise de diferença de imagem
- **gelsight_detector_visual.py**: Detecção visual com feedback
- **gelsight_marker_detector.py**: Detecção baseada em rastreamento de marcadores
- **gelsight_detector_debug.py**: Versão debug com logging detalhado

## Launch Files

- **camera_demo.launch.xml**: Demo básico com publisher e viewer
- **gelsight_camera.launch.py**: Launch para câmeras GelSight esquerda/direita
- **gelsight_debug.launch.xml**: Launch com debug logging
- **gelsight_detection_test.launch.xml**: Teste de detecção de objetos
- **gelsight_marker_detection_test.launch.xml**: Teste de detecção baseada em marcadores

## Configuration

Os parâmetros das câmeras GelSight estão em `config/gelsight_params.yaml`.

## Topics Publicados

- `/camera/camera1/image_raw` - Imagem da câmera 1 (sensor_msgs/Image)
- `/camera/camera2/image_raw` - Imagem da câmera 2 (sensor_msgs/Image)
- `/gelsight/left/image_raw` - Imagem GelSight esquerda
- `/gelsight/right/image_raw` - Imagem GelSight direita
- `/gelsight/object_detected` - Status de detecção (std_msgs/Bool)
- `/gelsight/deformation` - Deformação detectada (geometry_msgs/Vector3)

## Uso

### Visualizar câmeras:
```bash
ros2 launch tactile_cameras camera_demo.launch.xml
```

### Câmeras GelSight:
```bash
ros2 launch tactile_cameras gelsight_camera.launch.py
```

### Teste de detecção:
```bash
ros2 launch tactile_cameras gelsight_detection_test.launch.xml
```

## Dependências

- rclcpp
- rclpy
- sensor_msgs
- cv_bridge
- OpenCV
- numpy
