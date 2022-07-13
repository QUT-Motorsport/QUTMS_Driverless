import pygame
import sys
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand

#os.environ["SDL_VIDEODRIVER"] = "dummy"

class PyGameNode(Node):
    def __init__(self):
        super().__init__("gui")
        #self.get_logger().info("PyGame Node Initalised")
        self.keypress_publisher = self.create_publisher(ControlCommand, "/control_command", 10) #sending steering values to sim, control command are the messages being sent to the topic
    
    def publish_command(self, control_message):
        self.keypress_publisher.publish(control_message)


def bounds(steering, throttle, brake):
    if steering > 0.5:
        steering = 0.5
    if steering < -0.5:
        steering = -0.5
    if throttle > 1:
        throttle = 1
    if throttle < 0:
        throttle = 0
    if brake > 1:
        brake = 1
    if brake < 0:
        brake = 0
    
    return round(float(steering),2), round(float(throttle),2), round(float(brake),2)    


def py_game_main(py_game_node: PyGameNode):
    pygame.init()
    pygame.display.set_mode((320, 240))

    clock = pygame.time.Clock()
    throttle = 0
    brake = 0
    steering = 0
    zero = 0
    Running = True
    while Running:
        clock.tick(50) # 50Hz
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # if user clicks close
                Running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] and keys[pygame.K_a]:
            throttle += 0.2
            steering -= 0.1
        if keys[pygame.K_w] and keys[pygame.K_d]:
            throttle += 0.2
            steering += 0.1
        if keys[pygame.K_s] and keys[pygame.K_a]:
            brake += 0.25
            steering -= 0.1
        if keys[pygame.K_s] and keys[pygame.K_d]:
            brake += 0.25
            steering += 0.1
        elif keys[pygame.K_w]:
            throttle += 0.2
        elif keys[pygame.K_s]:
            brake += 0.25
        elif keys[pygame.K_a]:
            steering -= 0.1
        elif keys[pygame.K_d]:
            steering += 0.1
        
        else:
            if throttle > 0:
                throttle = 0 
            if brake > 0:
                brake = 0
            if steering > 0:
                steering -= 0.025
                if steering < 0:
                    steering = int(zero)
            elif steering < 0:
                steering += 0.025
                if steering > 0:
                    steering = int(zero)
        
        steering, throttle, brake = bounds(steering, throttle, brake)  # returns floats to publish
        msg = ControlCommand()
        msg.throttle = throttle
        msg.steering = steering
        msg.brake = brake
        py_game_node.publish_command(msg)
        print(f'steering: {steering} | throttle: {throttle} | brake: {brake}')
    pygame.quit()


def main():
    rclpy.init()
    node = PyGameNode()
    py_game_main(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    
