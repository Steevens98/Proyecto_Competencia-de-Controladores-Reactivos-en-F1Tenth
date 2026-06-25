# Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth

# F1Tenth ROS2: Controlador Reactivo y Cronómetro de Vueltas

## 🎯 Objetivo del proyecto

Desarrollar un controlador reactivo para el simulador F1Tenth que permita al vehículo recorrer una pista de manera autónoma utilizando el algoritmo **Follow the Gap**, junto con un sistema de cronometraje que registre los tiempos de 10 vueltas completas.

## Parte 1:📦 Creación del paquete del controlador

Creamos el paquete `f1tenth_controller` con soporte para Python y las dependencias necesarias:

```bash
cd ~/ros2_ws/src
ros2 pkg create f1tenth_controller --build-type ament_python --dependencies rclpy sensor_msgs ackermann_msgs numpy
```

Estructura esperada del paquete:

```
f1tenth_controller/
├── f1tenth_controller/
│   └── __init__.py
├── package.xml
├── setup.cfg
├── setup.py
```

Parte 2: 📥 Nodo Controlador — `follow_the_gap.py`

Creamos el archivo del nodo controlador:

```bash
cd ~/ros2_ws/src/f1tenth_controller/f1tenth_controller
touch follow_the_gap.py
chmod +x follow_the_gap.py
```

### 📘 ¿Qué hace este nodo?
Este nodo implementa el algoritmo Follow the Gap para controlar el vehículo F1Tenth de manera reactiva:
* Se suscribe al tópico `/scan` para recibir datos del LIDAR.
* Procesa los datos para detectar obstáculos y paredes.
* Identifica los "gaps" (espacios libres) más amplios en el campo de visión del vehículo.
* Selecciona el mejor gap basado en: Ancho del gap, Distancia al obstáculo más cercano, Proximidad al centro de la trayectoria
* Calcula el ángulo de dirección y la velocidad adecuados.
* Publica comandos de control en el tópico /drive usando el mensaje AckermannDriveStamped.
* Implementa filtros de suavizado para evitar movimientos bruscos.

### 📄 Código: 

```python
#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        # Suscripciones
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publicador
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        # PARÁMETROS DEL CONTROLADOR
        self.max_speed = 5.8
        self.min_speed = 0.3
        self.max_steering = 0.4189              # 24 grados
        
        self.max_laser_range = 4.5
        self.safety_distance = 0.4
        self.brake_distance = 0.5
        
        self.bubble_radius = 30
        self.min_gap_distance = 0.3
        self.gap_separation = 15
        
        # Filtros de suavizado
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.smoothing_factor = 0.6
        self.steering_history = []
        self.history_size = 4
        
        self.no_gaps_count = 0
        self.last_log_time = 0
        
        self.get_logger().info('=== F1TENTH - CONTROLADOR ACTIVO ===')
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # 1. LIMITAR RANGO DEL LÁSER
        ranges[np.isinf(ranges)] = self.max_laser_range
        ranges[np.isnan(ranges)] = 0.0
        ranges = np.clip(ranges, 0.0, self.max_laser_range)
        ranges = np.convolve(ranges, np.ones(7)/7, mode='same')
        
        center = len(ranges) // 2
        
        # 2. ENFOCAR EN FRENTE (±90°)
        front_width = 160
        start_idx = center - front_width
        end_idx = center + front_width
        
        lidar_ranges = np.zeros_like(ranges)
        lidar_ranges[start_idx:end_idx] = ranges[start_idx:end_idx]
        
        # 3. DETECTAR PAREDES
        wall_indices = []
        for i in range(start_idx + 2, end_idx - 2):
            if lidar_ranges[i] < 0.8 and lidar_ranges[i] > 0.1:
                wall_indices.append(i)
            elif (lidar_ranges[i] > 0.3 and 
                  abs(lidar_ranges[i] - lidar_ranges[i-1]) > 0.8):
                wall_indices.append(i)
        
        for idx in wall_indices:
            for j in range(-7, 8):
                if start_idx <= idx + j < end_idx:
                    lidar_ranges[idx + j] = min(lidar_ranges[idx + j], 0.3)
        
        # 4. DISTANCIA FRONTAL
        front_window = 15
        front_distances = ranges[center - front_window:center + front_window]
        valid_front = front_distances[front_distances > 0.1]
        front_distance = np.min(valid_front) if len(valid_front) > 0 else 2.0
        
        # 5. DETECTAR OBSTÁCULO MÁS CERCANO (BURBUJA)
        front_ranges = lidar_ranges[start_idx:end_idx]
        valid_indices = np.where(front_ranges > 0.15)[0]
        
        if len(valid_indices) > 0:
            valid_ranges = front_ranges[valid_indices]
            closest_local_idx = valid_indices[np.argmin(valid_ranges)]
            closest_idx = closest_local_idx + start_idx
            
            bubble_start = max(start_idx, closest_idx - self.bubble_radius)
            bubble_end = min(end_idx, closest_idx + self.bubble_radius)
            lidar_ranges[bubble_start:bubble_end] = 0.0
        
        # 6. ENCONTRAR GAPS
        gaps = []
        current_start = None
        
        for i in range(start_idx, end_idx):
            if lidar_ranges[i] > self.min_gap_distance:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    gap_width = i - current_start
                    if gap_width > 7:
                        gap_center = current_start + gap_width // 2
                        gap_distance = np.mean(lidar_ranges[current_start:i])
                        angle = msg.angle_min + gap_center * msg.angle_increment
                        
                        wall_penalty = 0
                        for j in range(-3, 4):
                            if current_start + j >= start_idx and i + j < end_idx:
                                if lidar_ranges[current_start + j] < 0.5:
                                    wall_penalty += 0.2
                                if lidar_ranges[i + j] < 0.5:
                                    wall_penalty += 0.2
                        
                        center_bias = 1.0 - abs(angle) / self.max_steering
                        center_bias = max(0.3, center_bias)
                        
                        if gap_width > 200:
                            wall_penalty += 3.0
                        
                        gaps.append({
                            'start': current_start,
                            'end': i,
                            'width': gap_width,
                            'distance': gap_distance,
                            'angle': angle,
                            'center_bias': center_bias,
                            'score': (gap_width * gap_distance * center_bias) - wall_penalty
                        })
                    current_start = None
        
        if current_start is not None:
            gap_width = end_idx - current_start
            if gap_width > 7:
                gap_center = current_start + gap_width // 2
                gap_distance = np.mean(lidar_ranges[current_start:end_idx])
                angle = msg.angle_min + gap_center * msg.angle_increment
                center_bias = 1.0 - abs(angle) / self.max_steering
                center_bias = max(0.3, center_bias)
                
                if gap_width > 200:
                    wall_penalty = 2.5
                else:
                    wall_penalty = 0
                
                gaps.append({
                    'start': current_start,
                    'end': end_idx,
                    'width': gap_width,
                    'distance': gap_distance,
                    'angle': angle,
                    'center_bias': center_bias,
                    'score': gap_width * gap_distance * center_bias - wall_penalty
                })
        
        # 7. SELECCIONAR MEJOR GAP
        steering = 0.0
        speed = self.min_speed
        
        if not gaps:
            steering = self.prev_steering * 0.85
            speed = max(2.0, self.prev_speed * 0.9)
            self.no_gaps_count += 1
        else:
            self.no_gaps_count = 0
            gaps_sorted = sorted(gaps, key=lambda g: g['score'], reverse=True)
            best_gap = gaps_sorted[0]
            
            if best_gap['width'] > 200 and len(gaps_sorted) > 1:
                for gap in gaps_sorted[1:]:
                    if gap['center_bias'] > best_gap['center_bias']:
                        best_gap = gap
                        break
            
            target_angle = best_gap['angle']
            raw_steering = np.clip(target_angle, -self.max_steering, self.max_steering)
            
            self.steering_history.append(raw_steering)
            if len(self.steering_history) > self.history_size:
                self.steering_history.pop(0)
            
            if len(self.steering_history) >= 2:
                steering = np.mean(self.steering_history)
            else:
                steering = raw_steering
            
            steering = self.smoothing_factor * steering + (1 - self.smoothing_factor) * self.prev_steering
            
            if abs(steering) < 0.01:
                steering = 0.0
            
            self.prev_steering = steering
            
            # 8. VELOCIDAD ADAPTATIVA
            abs_steering = abs(steering)
            gap_width = best_gap['width']
            
            if front_distance < 0.6:
                speed = 0.7
            elif len(wall_indices) > 100:
                speed = min(2.6, front_distance * 1.2)
            else:
                if front_distance < 0.5:
                    speed = 0.4
                elif front_distance < 0.7:
                    speed = 0.9
                elif front_distance < 1.0:
                    speed = 1.8
                elif abs_steering > 0.5:
                    speed = 1.8
                elif abs_steering > 0.4:
                    speed = 2.8
                elif abs_steering > 0.14:
                    speed = 4.5
                elif abs_steering > 0.07:
                    speed = 5.8
                else:
                    speed = min(self.max_speed, 5.8)
            
            if gap_width < 12:
                speed = min(speed, 2.0)
            elif gap_width < 22:
                speed = min(speed, 3.5)
            elif gap_width < 38:
                speed = min(speed, 5.0)
            
            speed = min(speed, front_distance * 2.3)
            speed = max(self.min_speed, speed)
            speed = 0.5 * self.prev_speed + 0.5 * speed
            self.prev_speed = speed
        
        # 9. PUBLICAR COMANDO
        cmd = AckermannDriveStamped()
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steering)
        
        try:
            self.drive_pub.publish(cmd)
        except:
            pass
    
    def stop_robot(self):
        try:
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.drive_pub.publish(cmd)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando nodo...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 📄 Código explicado:

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
```
* Se importan los módulos necesarios:

  * `numpy`: para procesamiento eficiente de arrays del LIDAR.
  * `LaserScan`: para recibir datos del sensor.
  * `AckermannDriveStamped`: para publicar comandos de dirección y velocidad.

 ```python
class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
```
* Se define un nodo llamado `follow_the_gap`.

```python
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publicador
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
```

* El nodo:

  * Se suscribe al tópico `/scan` para recibir datos LIDAR.
  * Publica en `/drive` los comandos de control.

```python
        self.max_speed = 5.8
        self.min_speed = 0.3
        self.max_steering = 0.4189              # 24 grados
```

* Define los límites de velocidad y dirección del vehículo.

```python
        self.bubble_radius = 30
        self.min_gap_distance = 0.3              
```
* Parámetros del algoritmo Follow the Gap:

  * `bubble_radius`: radio alrededor del obstáculo que se marca como no transitable.
  * `min_gap_distance`: distancia mínima para considerar un gap válido.

```python
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)            
```
* Función que se ejecuta cada vez que llegan datos del LIDAR. Convierte los datos a un array de numpy.

```python
        ranges[np.isinf(ranges)] = self.max_laser_range
        ranges[np.isnan(ranges)] = 0.0
        ranges = np.clip(ranges, 0.0, self.max_laser_range)          
```
* Limpia los datos del LIDAR: reemplaza infinitos y NaN con valores válidos

```python
        ranges = np.convolve(ranges, np.ones(7)/7, mode='same')         
```
* Aplica un filtro de suavizado para reducir el ruido en las mediciones.

```python
        front_width = 160
        start_idx = center - front_width
        end_idx = center + front_width  
```
* Enfoca el análisis en un campo de visión frontal de ±90°.

```python
        # DETECTAR PAREDES
        wall_indices = []
        for i in range(start_idx + 2, end_idx - 2):
            if lidar_ranges[i] < 0.8 and lidar_ranges[i] > 0.1:
                wall_indices.append(i)
```
* Identifica puntos que pertenecen a paredes cercanas.

```python
        # CREAR BURBUJA ALREDEDOR DEL OBSTÁCULO
        bubble_start = max(start_idx, closest_idx - self.bubble_radius)
        bubble_end = min(end_idx, closest_idx + self.bubble_radius)
        lidar_ranges[bubble_start:bubble_end] = 0.0
```
* Marca una "burbuja" de seguridad alrededor del obstáculo más cercano.

```python
        # ENCONTRAR GAPS
        for i in range(start_idx, end_idx):
            if lidar_ranges[i] > self.min_gap_distance:
                if current_start is None:
                    current_start = i
```
* Identifica espacios libres (gaps) donde la distancia es mayor que el mínimo.

```python
        # SELECCIONAR EL MEJOR GAP
        gaps_sorted = sorted(gaps, key=lambda g: g['score'], reverse=True)
        best_gap = gaps_sorted[0]
```
* Ordena los gaps por puntuación y selecciona el mejor.

```python
        # CALCULAR DIRECCIÓN Y VELOCIDAD
        steering = self.smoothing_factor * steering + (1 - self.smoothing_factor) * self.prev_steering
```
* Aplica suavizado a la dirección para evitar movimientos bruscos.

```python
        # PUBLICAR COMANDO
        cmd = AckermannDriveStamped()
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steering)
        self.drive_pub.publish(cmd)
```
* Publica el comando de control con la velocidad y dirección calculadas.

Parte 3: 📥 Nodo Cronómetro — `lap_timer.py`

Creamos el archivo del nodo controlador:

```bash
cd ~/ros2_ws/src/lap_timer/lap_timer
touch lap_timer.py
chmod +x lap_timer.py
```

### 📘 ¿Qué hace este nodo?
Este nodo registra y cronometra las vueltas completadas por el vehículo:
* Se suscribe al tópico `/ego_racecar/odom` para recibir la odometría del vehículo.
* Detecta cuando el vehículo cruza la línea de meta (x=0, con un margen en y).
* Calcula el tiempo de cada vuelta.
* Registra los tiempos de las 10 primeras vueltas.
* Al finalizar, muestra un resumen con: Tiempo de cada vuelta, Mejor vuelta, Peor vuelta y Tiempo promedio.
* Guarda los resultados en un archivo `lap_times.txt`.

### 📄 Código: 

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        self.lap_count = 0
        self.lap_times = []
        self.last_cross_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.last_detection_time = 0.0
        
        # Variables para detectar inicio del movimiento
        self.car_started = False
        self.start_time = None
        self.first_movement_detected = False
        
        self.get_logger().info(
            'Lap Timer iniciado - esperando que el carro arranque'
        )
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calcular velocidad para detectar movimiento
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx*vx + vy*vy)
        
        current_time = (
            self.get_clock().now().nanoseconds / 1e9
        )
        
        # Detectar cuando el carro comienza a moverse
        if not self.first_movement_detected and speed > 0.1:
            self.first_movement_detected = True
            self.start_time = self.get_clock().now()
            self.car_started = True
            self.get_logger().info(
                f'¡Carro en movimiento! Cronómetro iniciado'
            )
            self.get_logger().info(
                'Esperando primera vuelta...'
            )
        
        # Solo procesar cruces si el carro ya ha arrancado
        if not self.car_started:
            self.prev_x = x
            self.prev_y = y
            return
        
        # Detectar cruce de línea de meta (x=0)
        crossed = (
            self.prev_x < 0.0 and
            x >= 0.0 and
            abs(y) < 1.0
        )
        
        if crossed and (
            current_time -
            self.last_detection_time > 5.0
        ):
            self.last_detection_time = current_time
            now = self.get_clock().now()
            
            # Si es la primera vez que cruzamos (Vuelta 1)
            if self.last_cross_time is None:
                lap_time = (
                    now -
                    self.start_time
                ).nanoseconds / 1e9
                
                self.lap_count = 1
                self.lap_times.append(lap_time)
                self.last_cross_time = now
                
                self.get_logger().info(
                    f'Vuelta 1: {lap_time:.2f} s'
                )
            else:
                # Cruces siguientes (Vuelta 2, 3, 4, ...)
                lap_time = (
                    now -
                    self.last_cross_time
                ).nanoseconds / 1e9
                
                self.lap_count += 1
                self.lap_times.append(lap_time)
                self.last_cross_time = now
                
                self.get_logger().info(
                    f'Vuelta {self.lap_count}: '
                    f'{lap_time:.2f} s'
                )
                
                # Verificar si ya completamos 10 vueltas
                if self.lap_count == 10:
                    best_time = min(self.lap_times)
                    worst_time = max(self.lap_times)
                    avg_time = sum(self.lap_times) / len(self.lap_times)
                    
                    self.get_logger().info(
                        '========================================'
                    )
                    self.get_logger().info(
                        'RESUMEN DE 10 VUELTAS:'
                    )
                    self.get_logger().info(
                        '========================================'
                    )
                    
                    # Mostrar todas las vueltas
                    for i, t in enumerate(self.lap_times):
                        self.get_logger().info(
                            f'Vuelta {i+1}: {t:.2f} s'
                        )
                    
                    self.get_logger().info(
                        '========================================'
                    )
                    self.get_logger().info(
                        f'MEJOR VUELTA: {best_time:.2f} s  🏆'
                    )
                    self.get_logger().info(
                        f'PEOR VUELTA: {worst_time:.2f} s'
                    )
                    self.get_logger().info(
                        f'TIEMPO PROMEDIO: {avg_time:.2f} s'
                    )
                    self.get_logger().info(
                        '========================================'
                    )
                    
                    # Guardar en archivo
                    with open(
                        'lap_times.txt',
                        'w'
                    ) as f:
                        f.write('RESUMEN DE 10 VUELTAS\n')
                        f.write('=' * 40 + '\n')
                        for i, t in enumerate(self.lap_times):
                            f.write(f'Vuelta {i+1}: {t:.2f} s\n')
                        f.write('=' * 40 + '\n')
                        f.write(f'MEJOR VUELTA: {best_time:.2f} s  🏆\n')
                        f.write(f'PEOR VUELTA: {worst_time:.2f} s\n')
                        f.write(f'TIEMPO PROMEDIO: {avg_time:.2f} s\n')
                        f.write('=' * 40 + '\n')
                    
                    self.get_logger().info(
                        'Resultados guardados en lap_times.txt'
                    )
        
        self.prev_x = x
        self.prev_y = y

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 📄 Código explicado:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
```
* Se importan los módulos necesarios:

  * `Odometry`: para recibir la posición y velocidad del vehículo.
  * `math`: para cálculos de velocidad.

```python
class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')
```
* Se define un nodo llamado lap_timer.

```python
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
```
* El nodo se suscribe al tópico `/ego_racecar/odom` para recibir la odometría.

```python
        self.lap_count = 0
        self.lap_times = []
        self.last_cross_time = None
```
* Inicializa las variables para el conteo de vueltas y almacenamiento de tiempos.

```python
        self.car_started = False
        self.start_time = None
        self.first_movement_detected = False
```
* Variables para detectar cuándo el vehículo comienza a moverse.

```python
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
```
* Extrae la posición actual del vehículo del mensaje de odometría.

```python
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx*vx + vy*vy)
```
* Calcula la velocidad del vehículo para detectar movimiento.

```python
        if not self.first_movement_detected and speed > 0.1:
            self.first_movement_detected = True
            self.start_time = self.get_clock().now()
```
* Detecta el primer movimiento del vehículo y registra el tiempo de inicio.

```python
        crossed = (
            self.prev_x < 0.0 and
            x >= 0.0 and
            abs(y) < 1.0
        )
```
* Detecta el cruce de la línea de meta (x=0) con un margen en y de ±1.0 metros.

```python
        if crossed and (
            current_time -
            self.last_detection_time > 5.0
        ):
```
* Verifica que el cruce sea válido y no una falsa detección mínimo 5 segundos entre detecciones.

```python
        if self.last_cross_time is None:
            lap_time = (
                now -
                self.start_time
            ).nanoseconds / 1e9
```
* Para la primera vuelta, calcula el tiempo desde que arrancó el vehículo.

```python
        else:
            lap_time = (
                now -
                self.last_cross_time
            ).nanoseconds / 1e9
```
* Para las vueltas siguientes, calcula el tiempo desde el último cruce.

```python
        if self.lap_count == 10:
            best_time = min(self.lap_times)
            worst_time = max(self.lap_times)
            avg_time = sum(self.lap_times) / len(self.lap_times)
```
* Al completar 10 vueltas, calcula estadísticas: mejor, peor y promedio.

```python
        with open('lap_times.txt', 'w') as f:
            f.write('RESUMEN DE 10 VUELTAS\n')
            ...
```
* Guarda los resultados en un archivo de texto para referencia futura.


