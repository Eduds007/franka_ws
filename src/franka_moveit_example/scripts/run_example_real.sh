#!/bin/bash
# Script para executar o exemplo MoveIt com o robô real
# Execute este script em um SEGUNDO terminal, após o MoveIt ter iniciado

echo "=========================================="
echo "Executando Exemplo MoveIt - Robô Real"
echo "=========================================="
echo ""

# Source do workspace
cd /home/nuc_6g_life_3/franka_ws
source install/setup.bash

echo "⚠️  IMPORTANTE:"
echo "   1. Certifique-se de que o MoveIt já está rodando"
echo "   2. Verifique se o RViz está aberto"
echo "   3. Verifique se o robô está com os freios liberados"
echo "   4. Mantenha-se afastado do robô durante a execução"
echo ""
read -p "Pressione ENTER para iniciar os movimentos..."

# Executar o exemplo
ros2 run franka_moveit_example moveit_real_robot
