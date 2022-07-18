import pygame

import rclpy
from rclpy.node import Node

from fs_msgs.msg import ControlCommand

from .functions_pygame import (
    bounds,
    brake_steering_deccelerate,
    deccelerate,
    game_elements,
    left_deccelerate,
    right_deccelerate,
    throttling_steering_deccelerate,
)


class PyGameNode(Node):
    def __init__(self):
        super().__init__("gui")
        self.get_logger().info("PyGame Node Initalised")
        self.keypress_publisher = self.create_publisher(
            ControlCommand, "/control_command", 10
        )  # sending steering values to sim, control command are the messages being sent to the topic

    def publish_command(self, control_message: ControlCommand):
        self.keypress_publisher.publish(control_message)


def py_game_main(py_game_node: PyGameNode, steering_intensity=1, throttle_intensity=1):
    pygame.display.set_caption("Car Controller")
    win = pygame.display.set_mode((400, 300))
    clock = pygame.time.Clock()
    throttle = 0
    brake = 0
    steering = 0
    Running = True
    while Running:
        clock.tick(50)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                Running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] and keys[pygame.K_a]:
            throttle += 0.2 / throttle_intensity
            steering -= 0.1 / steering_intensity
        if keys[pygame.K_w] and keys[pygame.K_d]:
            throttle += 0.2 / throttle_intensity
            steering += 0.1 / steering_intensity
        if keys[pygame.K_s] and keys[pygame.K_a]:
            brake += 0.25
            steering -= 0.1 / steering_intensity
        if keys[pygame.K_s] and keys[pygame.K_d]:
            brake += 0.25
            steering += 0.1 / steering_intensity
        elif keys[pygame.K_w]:
            throttle += 0.2 / throttle_intensity
            steering, throttle, brake = throttling_steering_deccelerate(steering, throttle, brake)
        elif keys[pygame.K_s]:
            brake += 0.25
            steering, throttle, brake = brake_steering_deccelerate(steering, throttle, brake)
        elif keys[pygame.K_a]:
            steering -= 0.1 / steering_intensity
            steering, throttle, brake = left_deccelerate(steering, throttle, brake)
        elif keys[pygame.K_d]:
            steering += 0.1 / steering_intensity
            steering, throttle, brake = right_deccelerate(steering, throttle, brake)

        else:
            steering, throttle, brake = deccelerate(steering, throttle, brake)

        steering, throttle, brake = bounds(steering, throttle, brake)  # returns floats to publish
        game_elements(win, steering, throttle, brake)
        msg = ControlCommand()
        msg.throttle = throttle
        msg.steering = steering
        msg.brake = brake
        py_game_node.publish_command(msg)
    pygame.quit()


def main():
    rclpy.init()
    node = PyGameNode()
    py_game_main(node, 2, 2)  # higher values for steering and throttle will make the car less responsive to keypresses
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
