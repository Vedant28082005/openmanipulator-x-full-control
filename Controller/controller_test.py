import pygame
import sys

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ No controller detected")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"✅ Controller detected: {joystick.get_name()}")
print("Move sticks / press buttons to test...\n")

try:
    while True:
        pygame.event.pump()

        # Buttons
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"Button {i} pressed")

        # Axes (sticks & triggers)
        for i in range(joystick.get_numaxes()):
            axis = joystick.get_axis(i)
            if abs(axis) > 0.2:  # Deadzone
                print(f"Axis {i}: {axis:.2f}")

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()