import allegro_pybind as ap
import numpy as np
import time

# Define PD gains
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

# Create an instance of AllegroInterface
allegro_hand = ap.AllegroInterface()

# Start the Allegro Hand
if allegro_hand.start():

    # Wait for 1 second
    time.sleep(1)

    # Set the motion type
    allegro_hand.set_motion_type(ap.eMotionType.JOINT_PD)

    initial_hand_joint_positions = allegro_hand.get_joint_positions()
    allegro_hand.set_joint_positions(initial_hand_joint_positions)

    # Set PD gains
    allegro_hand.set_pd_gains(kp, kd)

    # Set joint positions
    positions = np.zeros(16)  # Create a NumPy array with desired positions
    positions[12] = 0.4
    allegro_hand.set_joint_positions(positions)

    # Wait for 1 second
    time.sleep(1)

    # Get joint positions as a NumPy array
    current_positions = allegro_hand.get_joint_positions()
    print("Joint positions:", current_positions)

    # Get target joint positions
    target_joint_positions = allegro_hand.get_target_joint_positions()
    print("Target joint positions:", target_joint_positions)

    # Wait for 1 second
    time.sleep(1)

    # Stop the Allegro Hand
    allegro_hand.stop()
else:
    print("Failed to start Allegro Hand.")
