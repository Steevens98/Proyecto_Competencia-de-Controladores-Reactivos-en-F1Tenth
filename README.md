# Proyecto_Competencia-de-Controladores-Reactivos-en-F1Tenth

# F1Tenth ROS2: Controlador Reactivo y Cronómetro de Vueltas

## 🎯 Objetivo del proyecto

Desarrollar un controlador reactivo para el simulador F1Tenth que permita al vehículo recorrer una pista de manera autónoma utilizando el algoritmo **Follow the Gap**, junto con un sistema de cronometraje que registre los tiempos de 10 vueltas completas.

## 📦 Creación del paquete del controlador

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
