# Follows a yellow line and sends commands to Arduino motor controller.
import cv2
import numpy as np
import math
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import glob
import math
import serial, time, sys, termios, tty, threading
from picamera2 import Picamera2
from libcamera import controls, Transform
from collections import deque


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.2, write_timeout=0.2)
time.sleep(2)

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1280, 720)},
    transform=Transform(rotation=180)      # 👈 rotate image
)

picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
time.sleep(1.0)

def read_arduino():
    try:
        found_button = None
        while ser.in_waiting > 0:
            line = ser.readline().decode(
                errors="ignore"
            ).strip()

            if line:
                found_button = line
        return found_button
    except Exception as e:
        print("Serial read error:", e)


def send_motor_commands_time(l, r, time_to_run_s):
    start = time.time()
    while time.time() - start < time_to_run_s:
        send_motor_commands(l, r)

def send_motor_commands(l, r):
    cmd = f"{int(l)},{int(r)}\n"
    try:
        ser.write(cmd.encode())
    except serial.SerialTimeoutException:
        print("Serial write timeout")
    except serial.SerialException as e:
        print("Serial error:", e)

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Manually check for the Ctrl+C character
        if ch == '\x03':
            raise KeyboardInterrupt
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def get_offset(image, scan_frac=0.90, min_pixels=20):

    # --- Threshold for yellow/green wall ---
    lower = np.array([20, 40, 117])
    upper = np.array([35, 255, 255])
    hsv   = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask  = cv2.inRange(hsv, lower, upper)

    h, w = mask.shape
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    scan_y = int(h * scan_frac)
    scan_y = max(0, min(scan_y, h - 1))   # clamp to valid range

    row = mask_clean[scan_y, :]
    xs = np.where(row > 0)[0]
    if len(xs) < min_pixels:
        return None   # no reliable offset

    x_wall = float(np.mean(xs))

    x_ref = 300 # 15
    offset = x_wall - x_ref   # negative -> wall left of ref, positive -> right
    print(f"x_wall: {x_wall:.1f}, x_ref: {x_ref} offset: {offset:.1f}")

    return offset


if __name__ == "__main__":
    base = 140
    i = 0
    last_saw_wall_s = time.time()
    print("starting main loop")

    
    while True:
        arduino_output = read_arduino()
        # Go to turbo mode if button pressed on Arduino
        # if "1" == arduino_output or "2" == arduino_output:
        #     print("Hit bottom!")
        #     send_motor_commands_time(255, 255, 2.0)
        #     continue
        if "1" == arduino_output or "2" == arduino_output:
            print("Hit front wall!")
            send_motor_commands_time(-100, -100, 0.5)
            send_motor_commands_time(-100, 100, 0.5)
            continue

        i+=1
        img = picam2.capture_array()
        image_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        x_offset = get_offset(image_bgr, scan_frac=0.50)
        
        if x_offset is None and (time.time() - last_saw_wall_s) > 3.0:
            start_spin = time.time()
            while True:
                print("Spinning to find wall...")
                send_motor_commands(-100, 100)
                img = picam2.capture_array()
                image_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                x_offset = get_offset(image_bgr, scan_frac=0.1)
                if x_offset is not None:
                    print("Wall found!")
                    last_saw_wall_s = time.time()
                    # Go forward a bit
                    send_motor_commands_time(100, 100, 0.5)
                    break
                # Dangerous but can help if stuck.
                if time.time() - start_spin > 10.0:
                    print("Could not find wall, going crazy")
                    send_motor_commands_time(255, 255, 1.0)
                    send_motor_commands_time(-100, -100, 1.0)
                    send_motor_commands_time(-100, 100, 0.5)
                    break
            continue

        if x_offset is None:
            #print("No line detected, turning left")
            send_motor_commands(-100, 100)
            continue
        last_saw_wall_s = time.time()
        if x_offset > 300:
            print("Large offset, turning right--------------------")
            start = time.time()
            send_motor_commands_time(100, -100, 0.4)
            continue
        
        kp = 3.5
        l, r = base, base
        change = max(min(kp*x_offset, 50), -50)
        l += change
        r -= change
        send_motor_commands(l, r)