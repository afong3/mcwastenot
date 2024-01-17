import odrive
from odrive.enums import *
import time
import threading
import sys

# Global flag to control the printing thread
keep_printing = True

# Function to be run in a separate thread that prints the motor position
def print_motor_position(axis):
    global keep_printing
    while keep_printing:
        print(f"Motor position: {axis.encoder.pos_estimate}")
        time.sleep(0.5)  # Delay between prints, adjust as necessary

# Function to start and stop the position printing
def monitor_motor_position(axis):
    global keep_printing
    keep_printing = True
    
    # Start the print thread
    print_thread = threading.Thread(target=print_motor_position, args=(axis,))
    print_thread.start()

    # Wait for the user to type 'stop' in the terminal
    while True:
        if input().lower() == 'stop':
            keep_printing = False
            break
    
    # Wait for the print thread to finish before exiting the function
    print_thread.join()

# Function to calibrate a motor
def configure_and_calibrate_motor(axis, pole_pairs, current_lim, calibration_current, cpr):
    axis.motor.config.pole_pairs = pole_pairs
    axis.motor.config.current_lim = current_lim
    axis.motor.config.calibration_current = calibration_current
    axis.encoder.config.cpr = cpr
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    print("Starting calibration...")
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    
    # Wait for calibration to complete
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    # Check for errors
    if axis.error != AXIS_ERROR_NONE:
        print(f"Axis error: {axis.error}")
        return False

    if axis.motor.error != MOTOR_ERROR_NONE:
        print(f"Motor error: {axis.motor.error}")
        return False

    if axis.encoder.error != ENCODER_ERROR_NONE:
        print(f"Encoder error: {axis.encoder.error}")
        return False

    # Check if calibration was successful
    if not axis.motor.is_calibrated:
        print("Motor calibration failed.")
        return False

    if axis.encoder.config.use_index and not axis.encoder.index_found:
        print("Encoder index not found.")
        return False

    print("Calibration successful.")
    return True

def move_motor_to_position(axis, position):
    print(f"Moving motor to position: {position}")
    axis.controller.input_pos = position

# Connect to ODrive
odrv0 = odrive.find_any()

# Configure each motor
#configure_and_calibrate_motor(odrv0.axis0, 14, 8.27 / 480, 20, 10, 8192)
#configure_and_calibrate_motor(odrv0.axis0, 14, 20, 10, 8192)#
#configure_motor(odrv0.axis1, 7, 8.27 / 480, 20, 10, 8192)

pole_pairs = 14
current_limit = 20
calibration_current = 10
cpr = 8192

success = configure_and_calibrate_motor(odrv0.axis0, pole_pairs, current_limit, calibration_current, cpr)
if success:
    print("Motor on axis0 is calibrated and ready to use.")
else:
    print("Motor on axis0 failed to calibrate.")

# Now that the motors are calibrated, move axis0 to position 10000
move_motor_to_position(odrv0.axis0, 10000)

# Move axis1 to position 5000
move_motor_to_position(odrv0.axis1, 5000)
monitor_motor_position(odrv0.axis0)

# Calibrate each motor
#calibrate_motor(odrv0.axis0)
#calibrate_motor(odrv0.axis1)

# Your main logic here
# ...
