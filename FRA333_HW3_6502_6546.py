# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. กันตินันท์_6502
2. ภนลภัส_6546
'''

# import library
from HW3_utils import FKHW3
import numpy as np

#=============================================<คำตอบข้อ 1>======================================================#
def endEffectorJacobianHW3(q:list[float])->list[float]:
    """
    Calculate the Jacobian matrix for the end effector based on the joint configuration q.
    
    The Jacobian matrix relates the joint velocities to the end effector's linear and angular velocities. 
    In robotic kinematics, the Jacobian plays a crucial role in tasks such as inverse kinematics and force transmission.
    
    Args:
    q: A list of 3 joint angles representing the configuration (in radians).

    Returns:
    A 6x3 Jacobian matrix where the first 3 rows represent the linear velocity components and 
    the last 3 rows represent the angular velocity components of the end effector with respect to the joint velocities.
    """
    
    # Perform forward kinematics to get rotation matrices (R) and position vectors (P)
    # R represents the orientation of each joint, and P gives the position of each joint.
    # R_e, p_e are the rotation matrix and position of the end effector.
    R, P, R_e, p_e = FKHW3(q)

    # Initialize the Jacobian matrices for linear (J_v) and angular (J_w) components.
    # J_v is for the linear velocity and J_w is for the angular velocity.
    J_v = np.zeros((3, 3))
    J_w = np.zeros((3, 3))

    # Loop over the first three joints to compute the Jacobian components for each joint.
    for i in range(3):
        # The rotation axis for joint i is given by the z-axis after applying the rotation matrix for that joint.
        r = np.dot(R[:, :, i], np.array([[0], [0], [1]])).flatten()

        # Calculate the vector from joint i to the end effector.
        d = p_e - P[:, i]

        # The linear velocity component (J_v) is the cross product of the rotation axis and the vector d.
        J_v[:, i] = np.cross(r, d)

        # The angular velocity component (J_w) is simply the rotation axis.
        J_w[:, i] = r

    # Concatenate the linear and angular Jacobians to form the full Jacobian (J).
    J = np.vstack((J_v, J_w))

    return J.tolist()

#==============================================================================================================#

#=============================================<คำตอบข้อ 2>======================================================#
def checkSingularityHW3(q:list[float])->bool:
    """
    Check if the given joint configuration results in a singularity.
    
    A singularity occurs when the robot loses a degree of freedom, meaning that the Jacobian loses rank and 
    the robot cannot move in certain directions. This is often indicated by the determinant of the Jacobian being close to zero.

    Args:
    q: A list of 3 joint angles representing the configuration (in radians).

    Returns:
    True if the configuration is singular, False otherwise.
    """
    
    epsilon = 0.001  # A small threshold to detect singularity (to handle numerical precision issues).

    # Compute the full Jacobian matrix.
    J = endEffectorJacobianHW3(q)

    # Extract the upper-left 3x3 part of the Jacobian (which relates to the linear velocity of the end effector).
    J_reduce = np.array(J[:3][:3])

    # Compute the determinant of the reduced Jacobian.
    # If the determinant is near zero, the robot is in a singular configuration.
    det_value = np.linalg.det(J_reduce)

    # Return True if the determinant is smaller than the threshold, indicating singularity.
    return abs(det_value) < epsilon

#==============================================================================================================#

#=============================================<คำตอบข้อ 3>======================================================#
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    """
    Compute the joint efforts (torques) required to achieve a given wrench (force/torque) at the end effector.
    
    This is useful in force control tasks in robotics where a certain force/torque is applied to the end effector, 
    and we need to calculate the corresponding torques at the joints.

    Args:
    q: A list of 3 joint angles representing the configuration (in radians).
    w: A list representing the wrench (force/torque) applied at the end effector. The list should contain 
       6 elements: the first 3 for the moments (torques) and the last 3 for the forces.

    Returns:
    A list of 3 joint torques/efforts corresponding to the given wrench.
    """
    
    # Compute the full Jacobian matrix.
    J = endEffectorJacobianHW3(q)

    # Extract the upper-left 3x3 part of the Jacobian that relates the joint velocities to the end effector's linear velocity.
    J_reduce = np.array(J[:3][:3])

    # Convert the wrench into a 6x1 array and extract only the force components (the last 3 elements).
    w_array = np.array(w).reshape(6, 1)
    force_vector = w_array[3:, :]

    # Compute the joint efforts (tau) using the transpose of the reduced Jacobian multiplied by the force vector.
    tau = np.dot(J_reduce.T, force_vector)

    return tau.flatten().tolist()
#==============================================================================================================#
