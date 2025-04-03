import pygame
import time

# === User Configuration ===
MAX_SPEED = float(input("Enter maximum speed (m/s): "))       # e.g. 5.0
MAX_ANGULAR = float(input("Enter maximum angular speed (deg/s): "))  # e.g. 180.0

# === Initialize Pygame & Joystick ===
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected. Please connect a PS4 controller.")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using Controller: {joystick.get_name()}")
print("Move the stick to control speed and angular velocity (CTRL+C to quit).")

# === Main Loop ===
try:
    while True:
        pygame.event.pump()  # Update joystick state

        # Read axis values (Left Stick Y and X by default)
        move_input = -joystick.get_axis(1)  # Left Stick Y (forward/backward)
        turn_input = joystick.get_axis(0)   # Left Stick X (left/right)

        # Deadzone filtering
        if abs(move_input) < 0.1:
            linear_velocity = 0.0
        else:
            linear_velocity = move_input * MAX_SPEED
        if abs(turn_input) < 0.1:
            angular_velocity = 0.0
        else:
            angular_velocity = -turn_input * MAX_ANGULAR


        # Output
        print(f"Speed: {linear_velocity:.2f} m/s | Angular: {angular_velocity:.2f} deg/s")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    pygame.quit()
