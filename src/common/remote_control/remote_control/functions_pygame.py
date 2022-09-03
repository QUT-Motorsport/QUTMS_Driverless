import pygame

bar_ratio = [i / 10 for i in range(1, 11)]
steering_ratio_right = [i / 20 for i in range(1, 11)]
steering_ratio_left = [-i for i in steering_ratio_right]


def bounds(steering: float, throttle: float, brake: float):
    if steering > 0.5:
        steering = 0.5
    if steering < -0.5:
        steering = -0.5
    if throttle > 1:
        throttle = 1
    if throttle < -1:
        throttle = -1
    if brake > 1:
        brake = 1
    if brake < 0:
        brake = 0
    return round(float(steering), 2), round(float(throttle), 2), round(float(brake), 2)


def game_elements(win, steering: float, throttle: float, brake: float):
    # window dimensions ((400, 300))
    win.fill((0, 0, 0))
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
    font = pygame.font.SysFont("calibri", 20)

    text_throttle = font.render("Throttle: " + str(throttle), True, (255, 255, 255))
    for i in range(len(bar_ratio)):
        if throttle >= bar_ratio[i]:
            pygame.draw.rect(win, (0, 255, 0), (throttle_x_loc, base_bar_y_loc - (i * height), width, height))
    win.blit(text_throttle, (throttle_x_loc - 40, 280))

    text_brake = font.render("Brake: " + str(brake), True, (255, 255, 255))
    for i in range(len(bar_ratio)):
        if brake >= bar_ratio[i]:
            pygame.draw.rect(win, (255, 0, 0), (brake_x_loc, base_bar_y_loc - (i * height), width, height))
    win.blit(text_brake, (brake_x_loc - 10, 280))

    for i in range(len(steering_ratio_right)):
        if steering >= steering_ratio_right[i]:
            pygame.draw.rect(
                win,
                (255, 0, 255),
                (steering_bar_x_loc - (i * steering_width), steering_bar_y_loc, steering_width, steering_height),
            )

    for i in range(len(steering_ratio_left)):
        if steering <= steering_ratio_left[i]:
            pygame.draw.rect(
                win,
                (255, 0, 255),
                (steering_bar_x_loc + (i * steering_width), steering_bar_y_loc, steering_width, steering_height),
            )

    text_steering = font.render("Steering: " + str(steering), True, (255, 255, 255))
    win.blit(text_steering, (160, steering_bar_y_loc + steering_height + 10))

    pygame.display.flip()
