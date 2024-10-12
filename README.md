# FRA333_HW3_6502_6546

The assignment FRA333_HW3_6502_6546 is a program development task to calculate the kinematics of a robotic arm. The requirements provided by the instructor are as follows:

- Students can present the solution-checking method in any way, as long as it is understandable.
- Clearly explain the steps and the concept of solution checking, with appropriate code comments.
- Only submissions from the "main" branch will be graded. If it is marked late in the classroom, it will be considered late.

**Robot Arm Description**: The robotic arm used in this assignment has 3 Degrees of Freedom (DoF), which can control rotational and linear movements. This assists in calculating and analyzing values such as the Jacobian, singularity conditions, and joint torques. The robotic arm consists of three joints, each capable of rotating around a specific axis. This allows the end effector to reach different positions in space. Calculations for the Jacobian matrix and singularity analysis are crucial for determining how efficiently and accurately the robot can reach a target position, as well as ensuring that it does not enter configurations where movement becomes restricted or unpredictable.

# File Structure

The folder FRA333_HW3_6502_6546 is organized into different files for easy understanding and systematic testing, as follows:

1. **FRA333_HW3_6502_6546.py**: The main file that contains functions for calculating various kinematic values of the robotic arm, such as the Jacobian, checking singularity, and computing joint efforts. This file is essential for defining the calculations that govern the robotic arm's movement, ensuring the mathematical integrity of each operation, and providing detailed explanations of the internal processes.
2. **testScript.py**: A script to test all the implemented functions. Running this file will test the Jacobian calculation, check for singularity, and compute the efforts. This script serves as an interactive way to verify the correctness of each implemented function, giving users the opportunity to input values and observe the corresponding results in real time.

# Function Details

The program is divided into three main functions, each with a distinct purpose and workflow as described below:

## endEffectorJacobianHW3

This function is used to calculate the Jacobian of the robotic arm given a specific joint configuration.

- **Input**: `q[3]` - an array of joint angles (in radians) representing the configuration of the three joints.
  - `q[0]` (`q1`): The angle of the first joint, which controls the base rotation of the arm.
  - `q[1]` (`q2`): The angle of the second joint, which controls the elevation or lowering of the arm.
  - `q[2]` (`q3`): The angle of the third joint, which controls the extension or contraction of the arm.
- **Output**: `J^{6x3}` - the Jacobian matrix for the end effector.

To calculate the Jacobian of the end effector, the following equation is used:

```math
J_e(q)_{6\times3} = 
    \begin{bmatrix}
        Jv_e(q)_{3\times3} \\
        Jw_e(q)_{3\times3}
    \end{bmatrix}
```

Where:

- **Jacobian Linear Component (`Jv_e`)**: Calculated from the cross product of the joint rotation axis and the vector from the joint to the end effector. This represents the linear velocity of the end effector as a result of changes in each joint angle.
- **Jacobian Angular Component (`Jw_e`)**: The rotation axis of each joint. This component is crucial for understanding how the end effector's orientation changes with respect to joint movements.

Since the robotic arm has only 3 DoF, the Jacobian can be reduced to only the linear velocity component:

```math
J_{reduce}(q)_{3\times3} = Jv(q)_{3\times3}
```

This reduced Jacobian is used in calculations involving linear positioning, such as determining the end effector's ability to reach a specific target in space without considering orientation. By focusing only on linear velocity, we ensure that the limited degrees of freedom are fully utilized for positional accuracy, which is often a priority in simple robotic tasks.

## checkSingularityHW3

This function is used to check whether the given joint configuration results in a singularity.

- **Input**: `q[3]` - an array of joint angles representing the configuration (in radians).
  - `q[0]` (`q1`): The angle of the first joint.
  - `q[1]` (`q2`): The angle of the second joint.
  - `q[2]` (`q3`): The angle of the third joint.
- **Output**: `flag` - `True` if the configuration is near a singularity, `False` otherwise.

To check for singularity, the manipulability (`m`) of the robotic arm is calculated by taking the determinant of the reduced Jacobian (`J_{reduce}`) and comparing it to a small threshold value (`ε`). A singularity occurs when the determinant approaches zero, indicating a loss of one or more degrees of freedom in the robot's movement, making certain directions inaccessible or leading to unpredictable movements.

```math
\varepsilon = 0.001
```

```math
m = \det(J_{reduce})
```

The robot is near a singularity if:

```math
m < \varepsilon
```

When a configuration is near a singularity, the robot may lose the ability to move in certain directions or experience uncontrolled movements, which can be dangerous or lead to errors during operation. Singularities often imply that even a small change in joint angles might result in unpredictable or exaggerated end effector movements, which makes precise control difficult.

## computeEffortHW3

This function calculates the effort or torque required for each joint to achieve a given wrench (force/torque) at the end effector.

- **Input**: `q[3]` - an array of joint angles representing the configuration (in radians).
  - `q[0]` (`q1`): The angle of the first joint.
  - `q[1]` (`q2`): The angle of the second joint.
  - `q[2]` (`q3`): The angle of the third joint.
- **Input**: `w[6]` - an array representing the wrench (force and torque) applied at the end effector.
  - `w[0]` (`Mx`): Moment or torque applied about the x-axis.
  - `w[1]` (`My`): Moment or torque applied about the y-axis.
  - `w[2]` (`Mz`): Moment or torque applied about the z-axis.
  - `w[3]` (`Fx`): Force applied along the x-axis.
  - `w[4]` (`Fy`): Force applied along the y-axis.
  - `w[5]` (`Fz`): Force applied along the z-axis.
- **Output**: `τ^{3x1}` - the torques required for each of the three joints.

The wrench is represented as:

```math
W_{6\times1} =
    \begin{bmatrix}
        moment(n^e)_{3\times1} \\
        force(f^e)_{3\times1}
    \end{bmatrix}
```

Since the robotic arm has 3 DoF, it can only control linear forces, which reduces the wrench to:

```math
W_{3\times1} = force(f^e)_{3\times1}
```

The effort for each joint is then calculated using the following equation:

```math
\tau_{3x1} = J_{reduce}^T(q)W
```

This equation represents the inverse dynamics of the robotic arm, where the transpose of the Jacobian is used to map the desired forces at the end effector back to the torques needed at each joint. This is important in scenarios where the robotic arm is interacting with its environment, such as picking up objects or applying specific forces during assembly tasks. Understanding these torques helps in designing control algorithms that can ensure the arm moves safely and efficiently without overloading any joint.

# How to Use

1. **Clone the Main Branch**

   To start using the code, first clone the repository from the GitHub main branch by running the following command in your terminal:

   ```
   git clone https://github.com/mudmini009/FRA333_HW3_6502_6546.git
   ```

2. **Run the Test Script**

   Once the repository is cloned, navigate to the folder and run the test script to verify the functions and their outputs. You can execute the test script by running:

   ```
   python testScript.py
   ```

   When running the script, you will need to input `q` (joint configuration) and `w` (wrench). The results will be displayed in the terminal, showing the computed Jacobian, singularity status, and joint efforts.

   - **Output Explanation**: The output will display the results of the different function calculations, including the Jacobian matrix, singularity status, and joint efforts, presented as matrices and values. Each result is intended to help you understand how the joint configuration affects the arm's capabilities and what torques are needed to achieve specific movements.

   - For example, if you input `q = [0.5, -0.5, 1.0]` and `w = [0, 0, 0, 5, 3, -2]`, the script will calculate and display the Jacobian, determine whether the configuration is a singularity, and compute the required joint efforts. Here, `q` represents the angles of the three joints of the robot arm, where each element (`q1`, `q2`, `q3`) corresponds to a specific joint rotation in radians. These angles define the position of the robotic arm and help calculate how each joint affects the end effector's movement. Similarly, `w` represents the wrench applied at the end effector, consisting of six values: the first three (`Mx`, `My`, `Mz`) are moments (torques) acting along the x, y, and z axes respectively, and the last three (`Fx`, `Fy`, `Fz`) are forces acting along these axes.

   The script will compute the Jacobian matrix, which shows how joint velocities translate into end effector linear and angular velocities. This information directly impacts how the robotic arm can be controlled—if the Jacobian has a low determinant, it implies reduced manipulability, meaning the arm might struggle to move effectively in certain directions. If the configuration is found to be singular, it means the robotic arm loses some movement capability, meaning it cannot move in specific directions, which could lead to unintended or dangerous scenarios.

   The computed joint efforts (`τ`) tell you how much torque each joint must exert to achieve the desired force at the end effector. This information is crucial when programming the robot to handle various tasks, such as picking up objects or applying controlled force, as it helps determine the required joint actions under specific loading conditions. For instance, if the wrench input (`w`) represents a high force along the z-axis (`Fz`), the joint torques computed will indicate how much each joint needs to work to counteract or produce that force, ensuring the robotic arm can carry out its intended operation safely and accurately.


Kantinan Laosuwan (ID: 65340500002)  
Pollapaat Suttimala (ID: 65340500046)
