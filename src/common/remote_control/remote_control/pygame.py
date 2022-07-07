import pygame
import time
#os.environ["SDL_VIDEODRIVER"] = "dummy"
s = pygame.display.set_mode((320, 240))

var1 = 0
var2 = 0
Running = True
# in a while loop print key being held down every 0.05 seconds
while Running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_k:
                print("quit")
                pygame.quit()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        var1 += 1
        print(var1)
    if keys[pygame.K_s]:
        var1 -= 1
        print(var1)
    if keys[pygame.K_a]:
        var2 -= 1
        print(var2)
    if keys[pygame.K_d]:
        var2 += 1
        print(var2)

    time.sleep(0.05)
    
