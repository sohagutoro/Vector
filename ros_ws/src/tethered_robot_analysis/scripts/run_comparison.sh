#!/bin/bash

echo "=========================================="
echo "Запуск сравнения кинематических схем"
echo "ROS 2 Jazzy + Gazebo 8.5"
echo "=========================================="

# Функция для запуска процесса в фоне с логированием
run_in_background() {
    local name=$1
    local command=$2
    local log_file="${name}.log"
    
    echo "Запуск: $name"
    echo "Команда: $command"
    echo "Лог: $log_file"
    
    # Запуск в фоне с перенаправлением вывода
    bash -c "$command" > "$log_file" 2>&1 &
    local pid=$!
    echo $pid > "${name}.pid"
    echo "PID: $pid"
    sleep 2
}

# Функция для остановки процесса
stop_process() {
    local name=$1
    local pid_file="${name}.pid"
    
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        echo "Остановка $name (PID: $pid)..."
        kill -9 $pid 2>/dev/null
        rm -f "$pid_file"
    fi
}

# Очистка предыдущих процессов
echo -e "\n[0/4] Очистка предыдущих процессов..."
stop_process "gazebo_mecanum"
stop_process "data_recorder_mecanum"
stop_process "trajectory_mecanum"
stop_process "gazebo_swerve"
stop_process "data_recorder_swerve"
stop_process "trajectory_swerve"

# 1. Запуск симуляции с платформой Меканума
echo -e "\nЗапуск симуляции с платформой Меканума..."
run_in_background "gazebo_mecanum" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 launch tethered_robot_analysis mecanum_hodograph.launch.py
"

sleep 5

echo -e "\nЗапись данных для платформы Меканума..."
run_in_background "data_recorder_mecanum" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 run tethered_robot_analysis data_recorder mecanum
"

sleep 3

echo -e "\nЗапуск движения по траектории для Меканума..."
run_in_background "trajectory_mecanum" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 run tethered_robot_analysis trajectory_publisher mecanum
"

echo "Ожидание завершения теста (30 секунд)..."
sleep 32

# Остановка процессов Меканума
echo -e "\nОстановка процессов Меканума..."
stop_process "trajectory_mecanum"
stop_process "data_recorder_mecanum"
stop_process "gazebo_mecanum"

sleep 3

# 2. Запуск симуляции с платформой 2SWD
echo -e "\nЗапуск симуляции с платформой 2SWD..."
run_in_background "gazebo_swerve" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 launch tethered_robot_analysis swerve_hodograph.launch.py
"

sleep 5

echo -e "\nЗапись данных для платформы 2SWD..."
run_in_background "data_recorder_swerve" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 run tethered_robot_analysis data_recorder swerve
"

sleep 3

echo -e "\nЗапуск движения по траектории для 2SWD..."
run_in_background "trajectory_swerve" "
    source /opt/ros/jazzy/setup.bash
    source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash
    ros2 run tethered_robot_analysis trajectory_publisher swerve
"

echo "Ожидание завершения теста (30 секунд)..."
sleep 32

# Остановка процессов 2SWD
echo -e "\nОстановка процессов 2SWD..."
stop_process "trajectory_swerve"
stop_process "data_recorder_swerve"
stop_process "gazebo_swerve"

# 3. Анализ результатов
echo -e "\nАнализ результатов и построение годографов..."
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/denis/Vector/ros_ws/install/setup.bash

# Поиск последних файлов данных
MECANUM_DATA=$(ls -t mecanum_processed_*.npz 2>/dev/null | head -1)
SWERVE_DATA=$(ls -t swerve_processed_*.npz 2>/dev/null | head -1)

if [ -z "$MECANUM_DATA" ] || [ -z "$SWERVE_DATA" ]; then
    echo "Файлы данных не найдены. Попытка найти raw данные..."
    MECANUM_DATA=$(ls -t mecanum_data_*.npz 2>/dev/null | head -1)
    SWERVE_DATA=$(ls -t swerve_data_*.npz 2>/dev/null | head -1)
fi

if [ -n "$MECANUM_DATA" ] && [ -n "$SWERVE_DATA" ]; then
    echo "Найдены файлы данных:"
    echo "Меканум: $MECANUM_DATA"
    echo "2SWD: $SWERVE_DATA"
    
    python3 /mnt/c/Users/denis/Vector/ros_ws/src/tethered_robot_analysis/scripts/plot_hodographs.py \
        --mecanum "$MECANUM_DATA" \
        --swerve "$SWERVE_DATA"
else
    echo "ОШИБКА: Не удалось найти файлы данных для анализа."
    echo "Проверьте логи процессов в файлах *.log"
fi

echo -e "\n=========================================="
echo "Сравнение завершено!"
echo "Проверьте файлы:"
echo "  - hodograph_comparison.png"
echo "  - metrics_comparison.png"
echo "  - comparison_results.csv"
echo "Логи процессов в файлах:"
echo "  - gazebo_mecanum.log"
echo "  - data_recorder_mecanum.log"
echo "  - trajectory_mecanum.log"
echo "  - gazebo_swerve.log"
echo "  - data_recorder_swerve.log"
echo "  - trajectory_swerve.log"
echo "=========================================="