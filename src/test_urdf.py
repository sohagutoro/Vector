# test_urdf.py
import subprocess
import os

def test_urdf_visualization():
    """Тестирование визуализации URDF модели"""
    print("Тестирование URDF модели")
    print("=" * 40)
    
    # Создаем временный URDF файл
    urdf_content = '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
  
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.2 0" rpy="0 1.57 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>'''
    
    with open('test_robot.urdf', 'w', encoding='utf-8') as f:
        f.write(urdf_content)
    
    print("URDF файл создан: test_robot.urdf")
    
    # Проверяем URDF на корректность
    try:
        # Эта команда проверяет URDF на синтаксические ошибки
        result = subprocess.run(['check_urdf', 'test_robot.urdf'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("URDF файл корректен")
            print("Вывод проверки:")
            print(result.stdout)
        else:
            print("Ошибка в URDF файле:")
            print(result.stderr)
            
    except FileNotFoundError:
        print("Утилита check_urdf не найдена. Установите пакет liburdfdom-tools")
        print("Для Ubuntu/Debian: sudo apt-get install liburdfdom-tools")
        print("Файл URDF создан, но проверка не выполнена")
    
    # Предлагаем команды для визуализации
    print("\nКоманды для визуализации:")
    print("1. check_urdf test_robot.urdf")
    print("2. urdf_to_graphiz test_robot.urdf")
    print("3. roslaunch urdf_tutorial display.launch model:=test_robot.urdf")
    
    # Очистка
    # os.remove('test_robot.urdf')

if __name__ == "__main__":
    test_urdf_visualization()