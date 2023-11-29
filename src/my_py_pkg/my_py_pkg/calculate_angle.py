import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def quaternion_to_rotation_matrix(quaternion):
    """
    Convert a quaternion into a rotation matrix.
    """
    q0, q1, q2, q3 = quaternion
    R = np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q3*q0, 2*q1*q3 + 2*q2*q0],
        [2*q1*q2 + 2*q3*q0, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q1*q0],
        [2*q1*q3 - 2*q2*q0, 2*q2*q3 + 2*q1*q0, 1 - 2*q1**2 - 2*q2**2]
    ])
    return R

def calculate_elbow_angle(arm_quat, arm_pos, forearm_quat, forearm_pos):
    """
    Calculate the elbow angle from arm and forearm quaternion and position.
    """
    # Convert quaternions to rotation matrices
    arm_rot_matrix = quaternion_to_rotation_matrix(arm_quat)
    forearm_rot_matrix = quaternion_to_rotation_matrix(forearm_quat)

    # Define unit vectors for arm and forearm in their local coordinate systems
    arm_direction = np.array([1, 0, 0])  # Assuming the sensor's X-axis points along the arm
    forearm_direction = np.array([1, 0, 0])  # Assuming the sensor's X-axis points along the forearm

    # Transform these vectors to the global coordinate system
    arm_vector = np.dot(arm_rot_matrix, arm_direction)
    forearm_vector = np.dot(forearm_rot_matrix, forearm_direction)

    # Calculate the angle between the vectors
    dot_product = np.dot(arm_vector, forearm_vector)
    angle = np.arccos(dot_product / (np.linalg.norm(arm_vector) * np.linalg.norm(forearm_vector)))

    # Convert angle to degrees
    angle_degrees = np.degrees(angle)

    return angle_degrees, arm_vector, forearm_vector, arm_pos, forearm_pos

def plot_arm_and_forearm(arm_pos, arm_vector, forearm_pos, forearm_vector, angle):
    """
    Create a 3D plot to visualize the arm and forearm position and orientation.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the arm and forearm vectors
    ax.quiver(*arm_pos, *arm_vector, color='blue', length=0.2, label='Arm Direction')
    ax.quiver(*forearm_pos, *forearm_vector, color='red', length=0.2, label='Forearm Direction')

    # Plotting the positions of the sensors
    ax.scatter(*arm_pos, color='blue', s=50, label='Arm Position')
    ax.scatter(*forearm_pos, color='red', s=50, label='Forearm Position')

    # Setting labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title(f'Arm and Forearm Orientation (Elbow Angle: {angle:.2f} degrees)')

    # Setting limits for better visualization
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Adding a legend
    ax.legend()

    plt.show()



# Using the provided data
# arm_quat = [0.88207424, 0.34301504, 0.06129225, 0.31710738]
# arm_pos = [0.1452632, 0.26766157, -0.5298454]
# forearm_quat = [0.71359897, 0.63619936, 0.2074767, 0.2073844]
# forearm_pos = [0.36693183, 0.40157384, -0.5065659]

# arm_quat = [0.876584, 0.295584, 0.0655076, 0.374123]
# arm_pos = [0.127087, 0.294738, -0.527362]
# forearm_quat = [0.761293, -0.0236154, 0.647292, 0.0302769]
# forearm_pos = [0.182314, 0.329937, -0.779002]

arm_quat = [0.920419, -0.0256769, -0.0707999, 0.383646]
arm_pos = [0.114518, 0.304706, -0.505085]
forearm_quat = [0.896905, 0.0451076, -0.0408769, -0.438046]
forearm_pos = [0.29413, 0.118875, -0.476491]

angle, arm_vector, forearm_vector, arm_pos, forearm_pos = calculate_elbow_angle(arm_quat, arm_pos, forearm_quat, forearm_pos)

# Plot the arm and forearm orientation
plot_arm_and_forearm(arm_pos, arm_vector, forearm_pos, forearm_vector, angle)



