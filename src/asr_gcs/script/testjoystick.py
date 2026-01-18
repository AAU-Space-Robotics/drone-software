import pygame

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Ensure a joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    exit()

# Get the first joystick (TX16S)
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to: {joystick.get_name()}")

# Print joystick info
num_axes = joystick.get_numaxes()
num_buttons = joystick.get_numbuttons()
num_hats = joystick.get_numhats()

print(f"Axes: {num_axes}, Buttons: {num_buttons}, Hats: {num_hats}")

# Event loop to detect inputs
running = True  # Define the loop control variable

while running:
    pygame.event.pump()  # Process events
    
    # Read axes (useful for sliders or rotary switches)
    axes_values = [joystick.get_axis(i) for i in range(num_axes)]
    
    # Read buttons (for physical push buttons)
    buttons_values = [joystick.get_button(i) for i in range(num_buttons)]
    
    # Read hat switches (for 4-way directional switches)
    hats_values = [joystick.get_hat(i) for i in range(num_hats)]
    
    print(f"Axes: {axes_values}")
    print(f"Buttons: {buttons_values}")
    print(f"Hats: {hats_values}")
    
    pygame.time.wait(500)  # Delay to avoid spamming output

    # Add a way to exit the loop (Press ESC key)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

pygame.quit()
