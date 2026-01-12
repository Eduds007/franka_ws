#!/bin/bash
# Script para iniciar o MoveIt com o robô Franka real (172.16.0.3)

echo "=========================================="
echo "Iniciando MoveIt para Robô Franka Real"
echo "IP do robô: 172.16.0.3"
echo "=========================================="
echo ""

# Source do workspace
cd /home/nuc_6g_life_3/franka_ws
source install/setup.bash

echo "⚠️  ATENÇÃO:"
echo "   1. Verifique se o robô está ligado"
echo "   2. Verifique se o cabo de rede está conectado"
echo "   3. Libere os freios do robô pela interface web"
echo "   4. Aguarde o RViz abrir"
echo ""
read -p "Pressione ENTER para continuar..."

# Iniciar o MoveIt com o robô real
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.3 use_fake_hardware:=false
