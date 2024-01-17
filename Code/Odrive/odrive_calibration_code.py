import odrive
import time
from odrive.enums import *

# Function to iteratively calibrate motor and test for CPR
def iterative_cpr_calibration(axis, pole_pairs, current_lim, calibration_current, cpr_guesses):
    # Configure motor parameters that don't change during CPR test
    axis.motor.config.pole_pairs = pole_pairs
    axis.motor.config.current_lim = current_lim
    axis.motor.config.calibration_current = calibration_current
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    # Iteratively test the CPR guesses
    for cpr in cpr_guesses:
        print(f"Testing with CPR: {cpr}")
        
        # Set the current CPR guess
        axis.encoder.config.cpr = cpr

        # Attempt calibration
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        # Check for errors specific to CPR
        if axis.encoder.error != ENCODER_ERROR_NONE:  # Check if there's any encoder error
            print(f"Encoder error with CPR {cpr}: {axis.encoder.error}")
            odrv0.clear_errors()  # Clear all errors on the ODrive
            # Handle the error or try the next CPR value
            continue

        # Check for general errors
        if axis.error != AXIS_ERROR_NONE or axis.motor.error != MOTOR_ERROR_NONE or axis.encoder.error != ENCODER_ERROR_NONE:
            print(f"General error occurred at CPR {cpr}:")
            print(f"Axis error: {axis.error}")
            print(f"Motor error: {axis.motor.error}")
            print(f"Encoder error: {axis.encoder.error}")
            odrv0.clear_errors()
            continue
        
        # Check if calibration was successful
        if not axis.motor.is_calibrated:
            print(f"Motor failed to calibrate at CPR {cpr}.")
            continue
        
        # If the calibration was successful, return the working CPR
        print(f"Motor calibrated successfully with CPR {cpr}.")
        return cpr

    # If no CPR value was successful
    print("Failed to calibrate motor with any of the CPR guesses.")
    return None

odrv0 = odrive.find_any()
pole_pairs = 7  
current_lim = 10 
calibration_current = 5
cpr_guesses = [2048, 4096, 8192, 16384]

successful_cpr = iterative_cpr_calibration(odrv0.axis0, pole_pairs, current_lim, calibration_current, cpr_guesses)
if successful_cpr is not None:
    print(f"Calibrated successfully with CPR: {successful_cpr}")
else:
    print("Unable to find correct CPR value.")
