import math

import pygame

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

from .functions_pygame import bounds, game_elements


class PyGameNode(Node):
    def __init__(self):
        super().__init__("gui")
        self.get_logger().info("PyGame Node Initalised")
        self.keypress_publisher = self.create_publisher(AckermannDriveStamped, "/driving_command", 10)

    def publish_command(self, control_message: AckermannDriveStamped):
        self.keypress_publisher.publish(control_message)


def py_game_main(py_game_node: PyGameNode, steering_intensity: float = 0.1, throttle_intensity: float = 0.1):
    pygame.init()
    pygame.display.set_caption("Car Controller")
    win = pygame.display.set_mode((400, 320))
    clock = pygame.time.Clock()
    throttle = 0.0
    brake = 0.0
    steering = 0.0
    Running = True
    while Running:
        clock.tick(50)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                Running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            throttle += throttle_intensity
        else:
            if throttle > 0:
                throttle -= throttle_intensity

        if keys[pygame.K_s]:
            throttle -= throttle_intensity
        else:
            if throttle < 0:
                throttle += throttle_intensity

        if keys[pygame.K_SPACE]:
            brake += throttle_intensity
        else:
            if brake > 0:
                brake -= throttle_intensity

        if keys[pygame.K_a]:
            steering -= steering_intensity
        else:
            if steering < 0:
                steering += steering_intensity

        if keys[pygame.K_d]:
            steering += steering_intensity
        else:
            if steering > 0:
                steering -= steering_intensity

        steering, throttle, brake = bounds(steering, throttle, brake)  # returns floats to publish
        game_elements(win, steering, throttle, brake)

        msg = AckermannDriveStamped()
        msg.drive.acceleration = throttle
        msg.drive.steering_angle = steering * math.pi
        msg.drive.jerk = brake

        py_game_node.publish_command(msg)

    pygame.quit()


def main():
    rclpy.init()
    node = PyGameNode()
    py_game_main(node)  # higher values for steering and throttle will make the car more responsive to keypresses
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
