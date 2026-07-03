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