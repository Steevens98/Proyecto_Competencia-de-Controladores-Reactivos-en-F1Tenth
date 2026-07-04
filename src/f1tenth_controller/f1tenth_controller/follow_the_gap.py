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