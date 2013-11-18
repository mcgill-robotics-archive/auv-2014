import pygame

pygame.init()
pygame.joystick.init()
numberOfDeviceConnected = pygame.joystick.get_count()
print numberOfDeviceConnected
controller = pygame.joystick.Joystick(0)
print controller.get_name()