# Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth

## Parte 1:📦 Instalación y Ejecución

### Paso 1: Clonar el repositorio

El repositorio ya está estructurado como un **workspace de ROS 2**, por lo que no se necesita crear carpetas adicionales.

```bash
cd $HOME
git clone https://github.com/Steevens98/Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth.git
```

Estructura esperada del paquete:

```
Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth/   
├── src/                                                      
│   └── f1tenth_controller/                                   
│       ├── f1tenth_controller/
│       │   ├── __init__.py
│       │   ├── follow_the_gap.py
│       │   └── lap_timer.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
├── videos/                                                   
├── README.md
└── .gitignore           
```

### Paso 2: Compilar el paquete

```bash
cd Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth/
colcon build
```
### Paso 3: Ejecutar el simulador y los nodos

⚠️ Nota : Tener instalado el simulador, sino instalarlo : https://github.com/widegonz/F1Tenth-Repository

En un terminal Ejecutar el Simulador 
```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Lanzar los nodos en terminales separadas:

Nodo `follow_the_gap.py`
```bash
cd Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth/
source install/setup.bash
ros2 run f1tenth_controller follow_the_gap
```

Nodo `lap_timer.py`
```bash
cd Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth/
source install/setup.bash
ros2 run f1tenth_controller lap_timer
```

Parte 2: 📥 Expliacion del Nodo Controlador — `follow_the_gap.py`

### 📘 ¿Qué hace este nodo?
Este nodo implementa el algoritmo Follow the Gap para controlar el vehículo F1Tenth de manera reactiva:
* Se suscribe al tópico `/scan` para recibir datos del LIDAR.
* Procesa los datos para detectar obstáculos y paredes.
* Identifica los "gaps" (espacios libres) más amplios en el campo de visión del vehículo.
* Selecciona el mejor gap basado en: Ancho del gap, Distancia al obstáculo más cercano, Proximidad al centro de la trayectoria
* Calcula el ángulo de dirección y la velocidad adecuados.
* Publica comandos de control en el tópico /drive usando el mensaje AckermannDriveStamped.
* Implementa filtros de suavizado para evitar movimientos bruscos.

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

Parte 3: 📥 Expliación del Nodo Cronómetro — `lap_timer.py`

### 📘 ¿Qué hace este nodo?
Este nodo registra y cronometra las vueltas completadas por el vehículo:
* Se suscribe al tópico `/ego_racecar/odom` para recibir la odometría del vehículo.
* Detecta cuando el vehículo cruza la línea de meta (x=0, con un margen en y).
* Calcula el tiempo de cada vuelta.
* Registra los tiempos de las 10 primeras vueltas.
* Al finalizar, muestra un resumen con: Tiempo de cada vuelta, Mejor vuelta, Peor vuelta y Tiempo promedio.
* Guarda los resultados en un archivo `lap_times.txt`.

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


## 🎬 Parte 4: Video de evidencia del programa
[Ver video de ejecución](./videos/Ejecucion_Proyecto.mp4)
