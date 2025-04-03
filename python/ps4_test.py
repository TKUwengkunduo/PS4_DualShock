import pygame
import time

# Initialize Pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Please connect a PS4 controller and restart the program.")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller connected: {joystick.get_name()}")

# Button and Axis Mappings
buttons = {
    0: "Cross (X)", 1: "Circle", 2: "Square", 3: "Triangle",
    4: "Share", 5: "PS Button", 6: "Options",
    7: "L3", 8: "R3", 9: "L1", 10: "R1",
    11: "D-Pad Up", 12: "D-Pad Down", 13: "D-Pad Left", 14: "D-Pad Right",
    15: "Touchpad Click"
}

axes = {
    0: "Right Stick X", 1: "Right Stick Y",
    2: "Left Stick X", 3: "Left Stick Y",
    4: "L2 Trigger", 5: "R2 Trigger"
}

print("Start pressing buttons or moving sticks to test input.")
print("Press CTRL+C to exit.")

try:
    while True:
        for event in pygame.event.get():
            # Handle button presses
            if event.type == pygame.JOYBUTTONDOWN:
                name = buttons.get(event.button, "Unknown Button")
                print(f"[Button {event.button}] {name} pressed")

            # Handle axis movement
            elif event.type == pygame.JOYAXISMOTION:
                value = event.value
                if abs(value) > 0.2:  # Ignore small movements
                    name = axes.get(event.axis, "Unknown Axis")
                    print(f"[Axis {event.axis}] {name} moved -> {value:.2f}")

        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nExiting controller test.")
finally:
    pygame.quit()
