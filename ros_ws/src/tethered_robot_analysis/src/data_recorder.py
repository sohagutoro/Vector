#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import numpy as np
from scipy import signal
import pandas as pd
from datetime import datetime
import signal as sig
import sys

class DataRecorder(Node):
    def __init__(self, robot_type='mecanum'):
        super().__init__('data_recorder')
        self.robot_type = robot_type
        
        # Подписка на одометрию
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{robot_type}_robot/odom',
            self.odom_callback,
            10
        )
        
        # Буферы для данных
        self.time_data = []
        self.position_data = {'x': [], 'y': [], 'z': []}
        self.velocity_data = {'x': [], 'y': [], 'z': []}
        self.acceleration_data = {'x': [], 'y': [], 'z': []}
        
        self.prev_time = None
        self.prev_velocity = None
        
        # Фильтр Баттерворта
        self.fs = 100
        self.cutoff = 10
        self.butter_b, self.butter_a = signal.butter(2, self.cutoff/(self.fs/2), 'low')
        
        # Файл для записи
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'{robot_type}_data_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'vx', 'vy', 'ax', 'ay'])
        
        self.get_logger().info(f"Data recorder started for {robot_type} robot")
        self.get_logger().info(f"Data will be saved to {self.csv_filename}")
        
        # Флаг завершения
        self.shutdown_requested = False
        
    def odom_callback(self, msg):
        if self.shutdown_requested:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Позиция
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Скорость
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        # Ускорение
        if self.prev_time is not None and self.prev_velocity is not None:
            dt = current_time - self.prev_time
            
            if dt > 0:
                ax_raw = (vx - self.prev_velocity['x']) / dt
                ay_raw = (vy - self.prev_velocity['y']) / dt
                
                # Простая фильтрация
                ax = signal.filtfilt(self.butter_b, self.butter_a, [ax_raw])[0]
                ay = signal.filtfilt(self.butter_b, self.butter_a, [ay_raw])[0]
                
                # Сохранение данных
                self.time_data.append(current_time)
                self.position_data['x'].append(x)
                self.position_data['y'].append(y)
                self.velocity_data['x'].append(vx)
                self.velocity_data['y'].append(vy)
                self.acceleration_data['x'].append(ax)
                self.acceleration_data['y'].append(ay)
                
                # Запись в CSV
                self.csv_writer.writerow([
                    f"{current_time:.6f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{vx:.6f}",
                    f"{vy:.6f}",
                    f"{ax:.6f}",
                    f"{ay:.6f}"
                ])
        
        self.prev_time = current_time
        self.prev_velocity = {'x': vx, 'y': vy}
        
    def save_data(self):
        """Сохранение данных в NPZ файл"""
        if len(self.time_data) == 0:
            self.get_logger().warning("Нет данных для сохранения")
            return None
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        npz_filename = f'{self.robot_type}_processed_{timestamp}.npz'
        
        np.savez(
            npz_filename,
            time=np.array(self.time_data),
            position_x=np.array(self.position_data['x']),
            position_y=np.array(self.position_data['y']),
            velocity_x=np.array(self.velocity_data['x']),
            velocity_y=np.array(self.velocity_data['y']),
            acceleration_x=np.array(self.acceleration_data['x']),
            acceleration_y=np.array(self.acceleration_data['y'])
        )
        
        self.csv_file.close()
        self.get_logger().info(f"Данные сохранены в {npz_filename} и {self.csv_filename}")
        
        return npz_filename
        
    def cleanup(self):
        """Очистка ресурсов"""
        self.shutdown_requested = True
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()

def signal_handler(sig, frame, node):
    print("\nПолучен сигнал завершения, сохранение данных...")
    node.cleanup()
    node.save_data()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    import sys
    robot_type = 'mecanum'
    if len(sys.argv) > 1:
        robot_type = sys.argv[1]
    
    node = DataRecorder(robot_type)
    
    # Регистрация обработчика сигналов
    import signal
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, node))
    signal.signal(signal.SIGTERM, lambda sig, frame: signal_handler(sig, frame, node))
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nПрервано пользователем, сохранение данных...")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        node.cleanup()
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()