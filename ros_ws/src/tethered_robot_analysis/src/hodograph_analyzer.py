#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, stats
import pandas as pd

class HodographAnalyzer(Node):
    def __init__(self):
        super().__init__('hodograph_analyzer')
        
    def load_data(self, filename):
        """Загрузка данных из файла"""
        data = np.load(filename)
        return {
            'time': data['time'],
            'vx': data['velocity_x'],
            'vy': data['velocity_y'],
            'ax': data['acceleration_x'],
            'ay': data['acceleration_y']
        }
    
    def calculate_metrics(self, data):
        """Расчет метрик из диссертации (Таблица 2)"""
        # Идеальная траектория (окружность)
        t = data['time']
        radius = 2.0
        omega = 0.5
        
        vx_ideal = radius * omega * np.cos(omega * t)
        vy_ideal = -radius * omega * np.sin(omega * t)
        ax_ideal = -radius * omega**2 * np.sin(omega * t)
        ay_ideal = -radius * omega**2 * np.cos(omega * t)
        
        # RMSE скорости
        v_error = np.sqrt((data['vx'] - vx_ideal)**2 + 
                          (data['vy'] - vy_ideal)**2)
        rmse_velocity = np.sqrt(np.mean(v_error**2))
        
        # Максимальное отклонение скорости
        max_velocity_deviation = np.max(v_error)
        
        # Среднеквадратичное ускорение
        rms_acceleration = np.sqrt(np.mean(data['ax']**2 + data['ay']**2))
        
        # Джерк (производная ускорения)
        dt = np.mean(np.diff(t))
        jerk_x = np.gradient(data['ax'], dt)
        jerk_y = np.gradient(data['ay'], dt)
        jerk_magnitude = np.sqrt(jerk_x**2 + jerk_y**2)
        mean_jerk = np.mean(jerk_magnitude)
        
        metrics = {
            'RMSE_velocity': rmse_velocity,
            'max_velocity_deviation': max_velocity_deviation,
            'RMS_acceleration': rms_acceleration,
            'mean_jerk': mean_jerk,
            'velocity_std': np.std(v_error),
            'acceleration_std': np.std(np.sqrt(data['ax']**2 + data['ay']**2))
        }
        
        return metrics
    
    def plot_hodographs(self, data_mecanum, data_swerve, save_path='.'):
        """Построение годографов как в диссертации (Рисунок 1)"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Годограф скорости: Меканум
        ax = axes[0, 0]
        ax.plot(data_mecanum['vx'], data_mecanum['vy'], 'b-', alpha=0.7, linewidth=1)
        ax.set_xlabel('$v_x$ [m/s]', fontsize=12)
        ax.set_ylabel('$v_y$ [m/s]', fontsize=12)
        ax.set_title('Velocity Hodograph: Mecanum', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Годограф скорости: 2SWD
        ax = axes[0, 1]
        ax.plot(data_swerve['vx'], data_swerve['vy'], 'r-', alpha=0.7, linewidth=1)
        ax.set_xlabel('$v_x$ [m/s]', fontsize=12)
        ax.set_title('Velocity Hodograph: 2SWD', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Годограф ускорения: Меканум
        ax = axes[1, 0]
        scatter = ax.scatter(data_mecanum['ax'], data_mecanum['ay'], 
                           c=data_mecanum['time'], cmap='viridis', 
                           s=10, alpha=0.6)
        ax.set_xlabel('$a_x$ [m/s²]', fontsize=12)
        ax.set_ylabel('$a_y$ [m/s²]', fontsize=12)
        ax.set_title('Acceleration Hodograph: Mecanum', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        plt.colorbar(scatter, ax=ax, label='Time [s]')
        
        # Годограф ускорения: 2SWD
        ax = axes[1, 1]
        scatter = ax.scatter(data_swerve['ax'], data_swerve['ay'], 
                           c=data_swerve['time'], cmap='viridis', 
                           s=10, alpha=0.6)
        ax.set_xlabel('$a_x$ [m/s²]', fontsize=12)
        ax.set_title('Acceleration Hodograph: 2SWD', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        plt.colorbar(scatter, ax=ax, label='Time [s]')
        
        plt.tight_layout()
        plt.savefig(f'{save_path}/hodograph_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Дополнительный график: сравнение метрик
        metrics_mecanum = self.calculate_metrics(data_mecanum)
        metrics_swerve = self.calculate_metrics(data_swerve)
        
        fig2, ax2 = plt.subplots(figsize=(10, 6))
        metrics_names = ['RMSE_velocity', 'max_velocity_deviation', 
                        'RMS_acceleration', 'mean_jerk']
        labels = ['RMSE скорости', 'Макс. отклонение скорости', 
                 'СКЗ ускорения', 'Средний джерк']
        
        x = np.arange(len(metrics_names))
        width = 0.35
        
        ax2.bar(x - width/2, [metrics_mecanum[m] for m in metrics_names], 
               width, label='Меканум', color='blue', alpha=0.7)
        ax2.bar(x + width/2, [metrics_swerve[m] for m in metrics_names], 
               width, label='2SWD', color='red', alpha=0.7)
        
        ax2.set_xlabel('Метрика', fontsize=12)
        ax2.set_ylabel('Значение', fontsize=12)
        ax2.set_title('Сравнение метрик качества движения', fontsize=14, fontweight='bold')
        ax2.set_xticks(x)
        ax2.set_xticklabels(labels, rotation=45, ha='right')
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(f'{save_path}/metrics_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Создание таблицы результатов
        results_table = pd.DataFrame({
            'Метрика': labels,
            'Меканум': [metrics_mecanum[m] for m in metrics_names],
            '2SWD': [metrics_swerve[m] for m in metrics_names],
            'Улучшение': [f"{metrics_mecanum[m]/metrics_swerve[m]:.1f}x" 
                         for m in metrics_names]
        })
        
        print("\n" + "="*60)
        print("СРАВНЕНИЕ МЕТРИК (Таблица 2 из диссертации)")
        print("="*60)
        print(results_table.to_string(index=False))
        print("="*60)
        
        return results_table

def main():
    analyzer = HodographAnalyzer()
    
    # Загрузка данных (предполагается, что данные уже записаны)
    try:
        data_mecanum = analyzer.load_data('mecanum_processed.npz')
        data_swerve = analyzer.load_data('swerve_processed.npz')
        
        # Построение графиков и расчет метрик
        results = analyzer.plot_hodographs(data_mecanum, data_swerve)
        
        # Сохранение результатов
        results.to_csv('comparison_results.csv', index=False)
        print("\nРезультаты сохранены в comparison_results.csv")
        
    except FileNotFoundError as e:
        print(f"Ошибка: {e}")
        print("Сначала запустите симуляции и запишите данные")

if __name__ == '__main__':
    main()