import serial
from pynput import keyboard

# Setup serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600) # Adjust port as per your connection

def on_press(key):
    try:
        if key.char == 'w': # If 'W' is pressed
            ser.write(b'W') # Send 'W' command to Arduino (increase throttle)
        elif key.char == 's': # If 'S' is pressed
            ser.write(b'S') # Send 'S' command to Arduino (stop throttle)
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener on 'Esc' key press
        return False

# Listen for keyboard inputs
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()