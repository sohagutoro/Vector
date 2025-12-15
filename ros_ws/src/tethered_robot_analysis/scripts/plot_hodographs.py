#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, stats
import pandas as pd
import glob

class HodographAnalyzer:
    def __init__(self):
        pass
    
    def load_data(self, filename):
        """Загрузка данных из файла"""
        try:
            data = np.load(filename)
            return {
                'time': data['time'],
                'vx': data['velocity_x'],
                'vy': data['velocity_y'],
                'ax': data['acceleration_x'],
                'ay': data['acceleration_y']
            }
        except Exception as e:
            print(f"Ошибка загрузки файла {filename}: {e}")
            # Попробуем загрузить из CSV
            try:
                df = pd.read_csv(filename.replace('.npz', '.csv'))
                return {
                    'time': df['time'].values,
                    'vx': df['vx'].values,
                    'vy': df['vy'].values,
                    'ax': df['ax'].values,
                    'ay': df['ay'].values
                }
            except:
                raise
    
    def calculate_metrics(self, data):
        """Расчет метрик из диссертации (Таблица 2)"""
        if len(data['time']) < 10:
            raise ValueError("Недостаточно данных для анализа")
        
        # Идеальная траектория (окружность)
        t = data['time']
        radius = 2.0
        omega = 0.5
        
        vx_ideal = radius * omega * np.cos(omega * t)
        vy_ideal = -radius * omega * np.sin(omega * t)
        
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
        if dt <= 0:
            dt = 0.01
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
                         if metrics_swerve[m] != 0 else "∞"
                         for m in metrics_names]
        })
        
        print("\n" + "="*60)
        print("СРАВНЕНИЕ МЕТРИК (Таблица 2 из диссертации)")
        print("="*60)
        print(results_table.to_string(index=False))
        print("="*60)
        
        return results_table

def main():
    parser = argparse.ArgumentParser(description='Анализ годографов для сравнения кинематических схем')
    parser.add_argument('--mecanum', type=str, help='Файл данных для платформы Меканум')
    parser.add_argument('--swerve', type=str, help='Файл данных для платформы 2SWD')
    parser.add_argument('--auto', action='store_true', help='Автоматический поиск последних файлов данных')
    
    args = parser.parse_args()
    
    analyzer = HodographAnalyzer()
    
    # Автоматический поиск файлов, если не указаны явно
    if args.auto:
        mecanum_files = glob.glob('mecanum_*.npz') + glob.glob('mecanum_*.csv')
        swerve_files = glob.glob('swerve_*.npz') + glob.glob('swerve_*.csv')
        
        if mecanum_files:
            args.mecanum = sorted(mecanum_files, key=lambda x: os.path.getmtime(x))[-1]
        if swerve_files:
            args.swerve = sorted(swerve_files, key=lambda x: os.path.getmtime(x))[-1]
    
    if not args.mecanum or not args.swerve:
        print("Ошибка: Необходимо указать файлы данных для обеих платформ")
        print("Использование: python plot_hodographs.py --mecanum <файл> --swerve <файл>")
        print("Или: python plot_hodographs.py --auto")
        return
    
    print(f"Загрузка данных Меканум из: {args.mecanum}")
    print(f"Загрузка данных 2SWD из: {args.swerve}")
    
    try:
        data_mecanum = analyzer.load_data(args.mecanum)
        data_swerve = analyzer.load_data(args.swerve)
        
        # Построение графиков и расчет метрик
        results = analyzer.plot_hodographs(data_mecanum, data_swerve)
        
        # Сохранение результатов
        results.to_csv('comparison_results.csv', index=False)
        print("\nРезультаты сохранены в comparison_results.csv")
        
    except Exception as e:
        print(f"Ошибка при анализе данных: {e}")
        print("Проверьте, что файлы содержат корректные данные")

if __name__ == '__main__':
    import os
    main()