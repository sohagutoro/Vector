# rrt_star_demo.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import time

class Node:
    """Узел дерева RRT*"""
    def __init__(self, config, parent=None):
        self.config = np.array(config)  # конфигурация [x, y, ...]
        self.parent = parent
        self.cost = 0.0  # стоимость пути от начала
        self.children = []
    
    def __repr__(self):
        return f"Node({self.config}, cost={self.cost:.2f})"

class RRTStar:
    """Реализация алгоритма RRT*"""
    
    def __init__(self, start, goal, bounds, obstacle_list, 
                 max_iter=1000, step_size=0.5, goal_sample_rate=0.1):
        self.start = Node(start)
        self.goal = Node(goal)
        self.bounds = bounds  # границы пространства [min, max] для каждой размерности
        self.obstacle_list = obstacle_list
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        
        self.node_list = [self.start]
        self.path = None
        self.iteration_count = 0
        
    def plan(self, visualize=False):
        """Основной метод планирования"""
        if visualize:
            self.setup_plot()
        
        for i in range(self.max_iter):
            self.iteration_count = i
            
            # Случайная выборка с вероятностью выбора целевой точки
            if np.random.random() < self.goal_sample_rate:
                rnd_point = self.goal.config
            else:
                rnd_point = self.sample()
            
            # Поиск ближайшего узла
            nearest_node = self.get_nearest_node(rnd_point)
            
            # Расширение в направлении случайной точки
            new_node = self.steer(nearest_node, rnd_point)
            
            # Проверка коллизий
            if self.check_collision(new_node, nearest_node):
                # Поиск соседей для переподключения
                neighbor_indices = self.find_near_nodes(new_node)
                self.choose_parent(new_node, neighbor_indices)
                self.node_list.append(new_node)
                self.rewire(new_node, neighbor_indices)
                
                if visualize and i % 50 == 0:  # Обновляем визуализацию каждые 50 итераций
                    self.update_plot(new_node, f"Итерация {i}")
                
                # Проверка достижения цели
                if self.is_goal_reached(new_node):
                    self.path = self.generate_final_path(new_node)
                    print(f"Путь найден на итерации {i}")
                    if visualize:
                        self.final_plot()
                    return self.path
        
        print("Достигнуто максимальное количество итераций")
        if visualize:
            self.final_plot()
        return None
    
    def sample(self):
        """Генерация случайной точки в пространстве конфигураций"""
        sample_point = []
        for bound in self.bounds:
            sample_point.append(np.random.uniform(bound[0], bound[1]))
        return np.array(sample_point)
    
    def get_nearest_node(self, point):
        """Поиск ближайшего узла к заданной точке"""
        distances = [np.linalg.norm(node.config - point) 
                    for node in self.node_list]
        min_index = np.argmin(distances)
        return self.node_list[min_index]
    
    def steer(self, from_node, to_point):
        """Движение от узла к целевой точке с ограничением шага"""
        direction = to_point - from_node.config
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            new_config = to_point
        else:
            new_config = from_node.config + (direction / distance) * self.step_size
        
        new_node = Node(new_config, from_node)
        new_node.cost = from_node.cost + np.linalg.norm(new_config - from_node.config)
        return new_node
    
    def check_collision(self, node, parent_node):
        """Проверка коллизий на отрезке между узлами"""
        # Проверяем всю линию между parent и node на столкновения
        steps = 10
        for i in range(steps + 1):
            # Интерполируем точку между parent и node
            t = i / steps
            test_point = parent_node.config * (1 - t) + node.config * t
            
            # Проверяем столкновение с каждым препятствием
            for obstacle in self.obstacle_list:
                obstacle_pos = np.array(obstacle[:2])
                obstacle_radius = obstacle[2]
                
                if np.linalg.norm(test_point - obstacle_pos) <= obstacle_radius:
                    return False
        return True
    
    def find_near_nodes(self, new_node):
        """Поиск соседних узлов для возможного переподключения"""
        n = len(self.node_list) + 1
        r = min(50 * np.sqrt((np.log(n) / n)), 3.0)  # радиус поиска соседей с ограничением
        distances = [np.linalg.norm(node.config - new_node.config) 
                    for node in self.node_list]
        near_indices = [i for i, d in enumerate(distances) if d <= r and d > 0]
        return near_indices
    
    def choose_parent(self, new_node, neighbor_indices):
        """Выбор оптимального родительского узла"""
        if not neighbor_indices:
            return
        
        costs = []
        for i in neighbor_indices:
            neighbor_node = self.node_list[i]
            # Временная проверка коллизий
            if self.check_collision(new_node, neighbor_node):
                cost = neighbor_node.cost + np.linalg.norm(new_node.config - neighbor_node.config)
                costs.append(cost)
            else:
                costs.append(float('inf'))
        
        if costs:  # Проверяем, что costs не пустой
            min_cost = min(costs)
            min_index = neighbor_indices[np.argmin(costs)]
            
            if min_cost < float('inf'):
                new_node.parent = self.node_list[min_index]
                new_node.cost = min_cost
    
    def rewire(self, new_node, neighbor_indices):
        """Переподключение соседних узлов для оптимизации пути"""
        for i in neighbor_indices:
            neighbor_node = self.node_list[i]
            # Пропускаем если это родитель нового узла
            if neighbor_node == new_node.parent:
                continue
                
            cost = new_node.cost + np.linalg.norm(neighbor_node.config - new_node.config)
            
            if cost < neighbor_node.cost and self.check_collision(neighbor_node, new_node):
                neighbor_node.parent = new_node
                neighbor_node.cost = cost
    
    def is_goal_reached(self, node):
        """Проверка достижения целевой точки"""
        distance_to_goal = np.linalg.norm(node.config - self.goal.config)
        return distance_to_goal <= self.step_size * 2  # Увеличиваем зону достижения цели
    
    def generate_final_path(self, final_node):
        """Генерация финального пути"""
        path = []
        current_node = final_node
        while current_node is not None:
            path.append(current_node.config)
            current_node = current_node.parent
        return path[::-1]  # разворот пути от начала к концу
    
    def setup_plot(self):
        """Настройка графика для визуализации"""
        plt.ion()  # Включение интерактивного режима
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(self.bounds[0][0], self.bounds[0][1])
        self.ax.set_ylim(self.bounds[1][0], self.bounds[1][1])
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('RRT* Path Planning')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        
        # Отрисовка препятствий
        for obstacle in self.obstacle_list:
            circle = plt.Circle(obstacle[:2], obstacle[2], color='red', alpha=0.7, label='Препятствия')
            self.ax.add_patch(circle)
        
        # Отрисовка начальной и конечной точек
        self.ax.plot(self.start.config[0], self.start.config[1], 'go', markersize=10, label='Старт')
        self.ax.plot(self.goal.config[0], self.goal.config[1], 'ro', markersize=10, label='Цель')
        
        self.ax.legend()
        plt.show()
    
    def update_plot(self, new_node, title):
        """Обновление графика"""
        if hasattr(self, 'ax'):
            # Отрисовка нового узла и связи
            self.ax.plot([new_node.config[0], new_node.parent.config[0]], 
                        [new_node.config[1], new_node.parent.config[1]], 
                        'b-', alpha=0.6, linewidth=1)
            self.ax.plot(new_node.config[0], new_node.config[1], 'bo', markersize=2, alpha=0.6)
            
            self.ax.set_title(f'RRT* Path Planning - {title}')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)
    
    def final_plot(self):
        """Финальная отрисовка с найденным путем"""
        if self.path:
            path_array = np.array(self.path)
            self.ax.plot(path_array[:, 0], path_array[:, 1], 'y-', linewidth=3, label='Найденный путь')
            self.ax.legend()
        
        self.ax.set_title(f'RRT* Path Planning - Завершено (итераций: {self.iteration_count})')
        plt.ioff()  # Выключение интерактивного режима
        plt.show()

def run_demo():
    """Запуск демонстрации работы алгоритма"""
    print("Запуск демонстрации RRT*")
    print("=" * 50)
    
    # Определение параметров
    start = [0, 0]
    goal = [8, 9]
    bounds = [[0, 10], [0, 10]]  # границы пространства
    obstacles = [
        [2, 2, 1.0],   # [x, y, радиус]
        [5, 5, 1.5],
        [7, 2, 0.8],
        [3, 7, 1.2],
        [8, 6, 1.0]
    ]
    
    print(f"Старт: {start}")
    print(f"Цель: {goal}")
    print(f"Препятствия: {len(obstacles)}")
    print(f"Границы: {bounds}")
    
    # Создание и запуск планировщика
    start_time = time.time()
    
    rrt_star = RRTStar(
        start=start, 
        goal=goal, 
        bounds=bounds, 
        obstacle_list=obstacles,
        max_iter=1000,
        step_size=0.3,
        goal_sample_rate=0.2
    )
    
    print("\nПоиск пути...")
    path = rrt_star.plan(visualize=True)
    
    end_time = time.time()
    
    print(f"\nРезультаты:")
    print(f"Время выполнения: {end_time - start_time:.2f} сек")
    print(f"Количество итераций: {rrt_star.iteration_count}")
    print(f"Количество узлов в дереве: {len(rrt_star.node_list)}")
    
    if path is not None:
        path_length = sum(np.linalg.norm(path[i] - path[i-1]) for i in range(1, len(path)))
        print(f"Длина пути: {path_length:.2f}")
        print(f"Количество точек в пути: {len(path)}")
        print("Путь успешно найден!")
    else:
        print("Путь не найден")
    
    return rrt_star, path

def test_different_scenarios():
    """Тестирование разных сценариев"""
    scenarios = [
        {
            "name": "Простой сценарий",
            "start": [1, 1],
            "goal": [9, 9],
            "obstacles": [[5, 5, 2.0]]
        },
        {
            "name": "Лабиринт",
            "start": [1, 1],
            "goal": [9, 9],
            "obstacles": [
                [3, 0, 0.5], [3, 2, 0.5], [3, 4, 0.5], [3, 6, 0.5], [3, 8, 0.5],
                [6, 1, 0.5], [6, 3, 0.5], [6, 5, 0.5], [6, 7, 0.5], [6, 9, 0.5]
            ]
        }
    ]
    
    for scenario in scenarios:
        print(f"\nТестирование: {scenario['name']}")
        print("-" * 30)
        
        rrt_star = RRTStar(
            start=scenario["start"],
            goal=scenario["goal"], 
            bounds=[[0, 10], [0, 10]],
            obstacle_list=scenario["obstacles"],
            max_iter=500
        )
        
        path = rrt_star.plan(visualize=False)
        
        if path:
            print(f"Путь найден за {rrt_star.iteration_count} итераций")
        else:
            print(f"Путь не найден")

if __name__ == "__main__":
    # Основная демонстрация
    rrt_star, path = run_demo()
    
    # Дополнительные тесты
    # test_different_scenarios()
    
    input("\nНажмите Enter для выхода...")