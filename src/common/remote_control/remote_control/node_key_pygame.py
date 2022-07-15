import pygame
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand

pygame.init()
win = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Car Controller")
clock = pygame.time.Clock()
zero = 0
bar_ratio = [i/10 for i in range(1,11)]
steering_ratio_right = [i/20 for i in range(1,11)]
steering_ratio_left = [-i for i in steering_ratio_right]


class PyGameNode(Node):
    def __init__(self):
        super().__init__("gui")
        self.get_logger().info("PyGame Node Initalised")
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

def deccelerate(steering,throttle,brake):
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
    return steering, throttle, brake

def right_deccelerate(steering,throttle,brake):
    if throttle > 0:
        throttle = 0 
    if brake > 0:
        brake = 0
    if steering < 0:
        steering += 0.025
        if steering > 0:
            steering = int(zero)
    return steering, throttle, brake

def left_deccelerate(steering,throttle,brake):
    if throttle > 0:
        throttle = 0 
    if brake > 0:
        brake = 0
    if steering > 0:
        steering -= 0.025
        if steering < 0:
            steering = int(zero)
    return steering, throttle, brake

def throttling_steering_deccelerate(steering,throttle,brake):
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
    return steering, throttle, brake

def brake_steering_deccelerate(steering,throttle,brake):
    if throttle > 0:
        throttle = 0
    if steering > 0:
        steering -= 0.025
        if steering < 0:
            steering = int(zero)
    elif steering < 0:
        steering += 0.025
        if steering > 0:
            steering = int(zero)
    return steering, throttle, brake

def game_elements(steering,throttle,brake):
    # window dimensions ((400, 300))
    win.fill((0,0,0))
    steering_width = 15
    steering_height = 40
    steering_bar_y_loc = 20
    steering_bar_x_loc = 200
    width = 40
    height = 15
    base_bar_y_loc = 260
    throttle_x_loc = 120
    brake_x_loc = 240
    # display bars representing steering, throttle and brake inputs
    font = pygame.font.SysFont("comicsansms", 20)
    text_throttle = font.render("Throttle: " + str(throttle), True, (255,255,255))
    if throttle > bar_ratio[0]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc,width,height))
    if throttle > bar_ratio[1]:
       pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(1*height),width,height))
    if throttle > bar_ratio[2]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(2*height),width,height))
    if throttle > bar_ratio[3]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(3*height),width,height))
    if throttle > bar_ratio[4]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(4*height),width,height))
    if throttle > bar_ratio[5]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(5*height),width,height))
    if throttle > bar_ratio[6]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(6*height),width,height))
    if throttle > bar_ratio[7]:
        pygame.draw.rect(win, (0,255,0), (throttle_x_loc,base_bar_y_loc-(7*height),width,height))
    if throttle > bar_ratio[8]:
        pygame.draw.rect(win, (255,0,0), (throttle_x_loc,base_bar_y_loc-(8*height),width,height))
    if throttle == bar_ratio[9]:
        pygame.draw.rect(win, (255,0,0), (throttle_x_loc,base_bar_y_loc-(9*height),width,height))
    win.blit(text_throttle, (throttle_x_loc-20,280))

    text_brake = font.render("Brake: " + str(brake), True, (255,255,255))
    if brake > bar_ratio[0]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc,width,height))
    if brake > bar_ratio[1]:
       pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(1*height),width,height))
    if brake > bar_ratio[2]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(2*height),width,height))
    if brake > bar_ratio[3]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(3*height),width,height))
    if brake > bar_ratio[4]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(4*height),width,height))
    if brake > bar_ratio[5]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(5*height),width,height))
    if brake > bar_ratio[6]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(6*height),width,height))
    if brake > bar_ratio[7]:
        pygame.draw.rect(win, (255,0,0), (brake_x_loc,base_bar_y_loc-(7*height),width,height))
    if brake > bar_ratio[8]:
        pygame.draw.rect(win, (255,255,0), (brake_x_loc,base_bar_y_loc-(8*height),width,height))
    if brake == bar_ratio[9]:
        pygame.draw.rect(win, (255,255,0), (brake_x_loc,base_bar_y_loc-(9*height),width,height))
    win.blit(text_brake,(brake_x_loc-10,280))

    if steering < steering_ratio_left[0]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc,steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[1]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[2]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*2),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[3]:   
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*3),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[4]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*4),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[5]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*5),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[6]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*6),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[7]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*7),steering_bar_y_loc,steering_width,steering_height))
    if steering < steering_ratio_left[8]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc-(steering_width*8),steering_bar_y_loc,steering_width,steering_height))
    if steering == steering_ratio_left[9]:
        pygame.draw.rect(win, (255,255,255), (steering_bar_x_loc-(steering_width*9),steering_bar_y_loc,steering_width,steering_height))

   
    if steering > steering_ratio_right[0]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc,steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[1]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[2]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*2),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[3]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*3),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[4]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*4),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[5]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*5),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[6]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*6),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[7]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*7),steering_bar_y_loc,steering_width,steering_height))
    if steering > steering_ratio_right[8]:
        pygame.draw.rect(win, (255,0,255), (steering_bar_x_loc+(steering_width*8),steering_bar_y_loc,steering_width,steering_height))
    if steering == steering_ratio_right[9]:
        pygame.draw.rect(win, (255,255,255), (steering_bar_x_loc+(steering_width*9),steering_bar_y_loc,steering_width,steering_height))
    
    text_steering = font.render("Steering: " + str(steering), True, (255,255,255))
    win.blit(text_steering,(160,steering_bar_y_loc+steering_height+10))

    pygame.display.flip()


def py_game_main(py_game_node: PyGameNode,steering_intensity=1,throttle_intensity=1):
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
            throttle += 0.2/throttle_intensity
            steering -= 0.1/steering_intensity
        if keys[pygame.K_w] and keys[pygame.K_d]:
            throttle += 0.2/throttle_intensity
            steering += 0.1/steering_intensity
        if keys[pygame.K_s] and keys[pygame.K_a]:
            brake += 0.25
            steering -= 0.1/steering_intensity
        if keys[pygame.K_s] and keys[pygame.K_d]:
            brake += 0.25
            steering += 0.1/steering_intensity
        elif keys[pygame.K_w]:
            throttle += 0.2/throttle_intensity
            steering,throttle,brake = throttling_steering_deccelerate(steering,throttle,brake)
        elif keys[pygame.K_s]:
            brake += 0.25
            steering,throttle,brake = brake_steering_deccelerate(steering,throttle,brake)
        elif keys[pygame.K_a]:
            steering -= 0.1/steering_intensity
            steering,throttle,brake = left_deccelerate(steering,throttle,brake)
        elif keys[pygame.K_d]:
            steering += 0.1/steering_intensity
            steering,throttle,brake = right_deccelerate(steering,throttle,brake)
            
        else:
            steering,throttle,brake = deccelerate(steering,throttle,brake)
        
        steering, throttle, brake = bounds(steering, throttle, brake)  # returns floats to publish
        game_elements(steering,throttle,brake)
        msg = ControlCommand()
        msg.throttle = throttle
        msg.steering = steering
        msg.brake = brake
        py_game_node.publish_command(msg)
    pygame.quit()


def main():
    rclpy.init()
    node = PyGameNode()
    py_game_main(node,2,2) # higher values for steering and throttle will make the car less responsive to keypresses
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
