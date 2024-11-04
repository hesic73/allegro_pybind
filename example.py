import allegro_pybind
import numpy as np
import time

kp = np.array([
    700, 800, 900, 500,
    700, 800, 900, 500,
    700, 800, 900, 500,
    700, 800, 900, 500
])

kd = np.array([
    28, 50, 55, 40,
    28, 50, 55, 40,
    28, 50, 55, 40,
    28, 50, 55, 40
])

if allegro_pybind.start():

    allegro_pybind.set_motion_type(allegro_pybind.eMotionType.JOINT_PD)

    # Set PD gains
    allegro_pybind.set_pd_gains(kp, kd)

    # Wait for 1 second
    time.sleep(1)

    # Set joint positions
    positions = np.zeros(16)  # Create a NumPy array with desired positions

    allegro_pybind.set_joint_positions(positions)

    time.sleep(1)

    # Get joint positions as a NumPy array
    current_positions = allegro_pybind.get_joint_positions()
    print("Joint positions:", current_positions)

    time.sleep(1)

    # Stop the device
    allegro_pybind.stop()
else:
    print("Failed to start Allegro Hand.")
