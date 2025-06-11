#!/usr/bin/env python3
"""
Detail : Self-driving robot using ROS2 and LaserScan data for obstacle avoidance.This runs on the
portenta x8 board. Angular velocity is controlled by a PWM signal to the motor pins.
The robot will navigate autonomously by adjusting its speed and direction based on the
distance to obstacles detected by the laser scanner in the Gazebo simulation.
gazebo.launch.py
Date: May 25, 2024          BY: Aaron Gumba         Initial Version
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from car import control
from car.pwm import PWM




class SelfDrive(Node): 
    def __init__(self,motor_pin_A, motor_pin_B):
        super().__init__("SelfDrive_Node")
        self.motor_pin_A = motor_pin_A
        self.motor_pin_B = motor_pin_B
        self.pwm_A = PWM(chip=0, channel=9, period_ns=100000)  # 100 kHz
        self.pwm_A.enable()
        
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.8, self.control_timer_callback)
        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        
        self.max_speed = 0.5
        self.minimum_speed = 0.2
        self.max_angular = 1.60
        self.speed = 0.0
        self.angular = 0.0
        
        self.right_distance = float('inf')
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.front_left_distance = float('inf')
        self.front_right_distance = float('inf')

        self.turning_direction = None  # "left", "right", or None

    def scan_callback(self, msg):
        ranges = [r if not math.isinf(r) and not math.isnan(r) else float('inf') for r in msg.ranges]

        if len(ranges) < 5:
            self.get_logger().warn("Not enough scan data (need at least 5 values).")
            return

        self.right_distance = ranges[0] 
        self.front_right_distance = ranges[1] 
        self.front_distance = ranges[2] 
        self.front_left_distance = ranges[3] 
        self.left_distance = ranges[4] 

       # self.get_logger().info(
       #     " R: %.2f | FR: %.2f | F: %.2f | FL: %.2f | L: %.2f" %
       #     (self.right_distance, self.front_right_distance, self.front_distance,
       #      self.front_left_distance, self.left_distance)
        #)

    def control_timer_callback(self):
        msg = Twist()
        self.get_logger().warn("STart")
        # If committed to a turn, keep turning until front is clear
        if self.turning_direction is not None:
            if self.front_distance > 2.5:
                # Clear turn state if front is clear
                self.turning_direction = None
                self.angular = 0.0
                self.speed = self.max_speed
            else:
                # Continue turning in the chosen direction
                self.speed = self.minimum_speed
                if self.turning_direction == "left":
                    self.angular = min (self.max_angular, ((self.angular) + 0.5) *2.0)
                    self.get_logger().info("left turn: %.2f" % self.angular)
                elif self.turning_direction == "right":
                    self.angular = -1* (min (self.max_angular, ((self.angular) + 0.5) *2.0 ))
                    self.get_logger().info("right turn: %.2f" % self.angular)

            msg.linear.x = self.speed
            msg.angular.z = self.angular
            
            motor_speed = int(abs(msg.angular.z) * 255 / 1.60)
            motor_speed = max(0, min(motor_speed, 255))  # clamp to [0, 255]
            self.publisher_.publish(msg)
            """
            angular + left turn 
            angular - right turn
            """
            if msg.angular.z > 0:
                    pwmA_=  255 
                    pwmB_= -1* motor_speed
                    self.pwm_A.set_brightness(pwmA_)
                    self.get_logger().info(f"Turning LEft: pwmA_={pwmA_}, pwmB_={pwmB_}")
            elif msg.angular.z < 0:
                    pwmA_= -1* motor_speed
                    pwmB_= 255 
                    self.pwm_A.set_brightness(pwmA_)
                    self.get_logger().info(f"Turning RIGHT: pwmA_={pwmA_}, pwmB_={pwmB_}")
            else:
                pwmA_= 255
                pwmB_= 255
                self.pwm_A.set_brightness(pwmA_)
                self.get_logger().info(f"Going STRAIGHT: pwmA_={pwmA_}, pwmB_={pwmB_}")



            self.get_logger().info("angular: %.2f | speed: %.2f" % (msg.angular.z, msg.linear.x))
            return

        # Normal behavior (not committed to a direction)
        if self.front_distance <= 0.5:
            self.speed = -self.minimum_speed
            self.angular = 0.0

        elif self.front_distance <= 2.5 and self.front_distance > 1.0:
            self.speed = max(self.minimum_speed, self.speed - 0.4)

            if (self.left_distance > self.right_distance) :
                self.turning_direction = "left"
                
             
                self.get_logger().info("Left | angular: %.2f" % self.angular)
        
            elif (self.right_distance > self.left_distance) :
                self.turning_direction = "right"
            
                self.get_logger().info("Right | angular: %.2f" % self.angular)
            else:
                if not hasattr(self, 'turning_direction') or self.turning_direction is None:
                    self.turning_direction = "left"
                    self.get_logger().info("Left and right distances are equal or both infinite; continuing previous turn: %s" % self.turning_direction)


        elif self.front_distance <= 1.0:
            self.speed = 0.1

            if (self.left_distance > self.right_distance) :
                self.turning_direction = "left"
                
                
            elif (self.right_distance > self.left_distance) :
                self.turning_direction = "right"
            else:
                if not hasattr(self, 'turning_direction') or self.turning_direction is None:
                    self.turning_direction = "left"
                    self.get_logger().info("Left and right distances are equal or both infinite; continuing previous turn: %s" % self.turning_direction)


        else:
            if math.isinf(self.front_distance) or self.front_distance >= 10.0:
                if self.front_left_distance <= 3.0 and self.front_left_distance > 1.0 and self.front_right_distance <= 3.0 and self.front_right_distance > 1.0:
                    self.speed = max(self.minimum_speed, self.speed - 0.4)
                    if self.front_left_distance > self.front_right_distance:
                        self.turning_direction = "left"
              
                        self.get_logger().info("FLEFT | angular: %.2f" % self.angular)

                    else:
                        self.turning_direction = "right"
                        self.get_logger().info("FRIGHT | angular: %.2f" % self.angular)
                if self.front_left_distance <= 1.0 and self.front_right_distance <= 1.0:
                    self.speed = -0.5
                    if self.front_left_distance > self.front_right_distance:
                        self.turning_direction = "left"
                        self.angular = 0.5
              
                        self.get_logger().info("FLEFT LOW| angular: %.2f" % self.angular)

                    else:
                        self.turning_direction = "right"
                        self.angular = -0.5
                        self.get_logger().info("FRIGHT LOW | angular: %.2f" % self.angular)
                 
                
            else:
                self.angular = self.angular
                self.speed = self.max_speed

        msg.linear.x = self.speed
        msg.angular.z = self.angular
        motor_speed = int(abs(msg.angular.z) * 255 / 1.60)
        motor_speed = max(0, min(motor_speed, 255))  # clamp to [0, 255]
        
        self.publisher_.publish(msg)
        self.get_logger().info("angular: %.2f | pwma: %.2f" % (msg.angular.z, motor_speed))
        
        # Motor GPIO control based on linear.x
        if msg.linear.x > 0:
            self.motor_pin_A.set_value(1)
            self.motor_pin_B.set_value(0)
       
        elif msg.linear.x < 0:
            self.motor_pin_A.set_value(0)
            self.motor_pin_B.set_value(1)
        
        else:
            self.motor_pin_A.set_value(0)
            self.motor_pin_B.set_value(0)
            
        if msg.angular.z > 0:
            pwmA_=  255 
            pwmB_= 255 + motor_speed    
            self.pwm_A.set_brightness(pwmA_)
            self.get_logger().info(f"Turning Left: pwmA_={pwmA_}, pwmB_={pwmB_}")
        elif msg.angular.z < 0:
            pwmA_= 255 - motor_speed
            pwmB_= 255 
            self.pwm_A.set_brightness(pwmA_)
            self.get_logger().info(f"Turning RIGHT: pwmA_={pwmA_}, pwmB_={pwmB_}")
        else:
            pwmA_ = 255
            pwmB_ = 255
            self.pwm_A.set_brightness(pwmA_)
            self.get_logger().info(f"Going STRAIGHT: pwmA_={pwmA_}, pwmB_={pwmB_}")


def main(args=None):
    rclpy.init(args=args)
    GPIO2 = 162
    GPIO3 = 163


    motor_pin_A = control.GPIOController(GPIO3)
    motor_pin_A.export()
    motor_pin_A.set_direction("out")

    motor_pin_B = control.GPIOController(GPIO2)
    motor_pin_B.export()
    motor_pin_B.set_direction("out")

    node = SelfDrive(motor_pin_A, motor_pin_B)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
