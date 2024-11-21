# RBE3001 Final Project

## Robot Control

### **DH Parameters**

We started by deriving the Denavit–Hartenberg parameters and frames for our robotic arm. This allowed us to calculate the location of the end effector with the inputs of the joint angles.

![DH_table.png](RBE3001%20Final%20Project%2062d94d828d8142c18bbaaf0d2d68fa3a/DH_table.png)

### **Inverse Kinematics**

We started by deriving the Denavit–Hartenberg parameters and frames for our robotic arm. This allowed us to calculate the location of the end effector with the inputs of the joint angles.

![IK1.png](RBE3001%20Final%20Project%2062d94d828d8142c18bbaaf0d2d68fa3a/IK1.png)

![IK2.png](RBE3001%20Final%20Project%2062d94d828d8142c18bbaaf0d2d68fa3a/IK2.png)

### **Trajectory Planning**

With the derivation of the inverse kinematics and DH parameters we implimented 3 different interpolated motion functions. The first was a basic interpolated motion, interpolate_jp(), which moved the robot in a linear motion, interpolated over a set time in milliseconds. We also implimented cubic trajectories, to give constant velocity at the start and end points, and quintic trajectories, to give both constant velocity and acceleration at the start and end points. This overall allowed us to get smooth motion of the robot arm, though for our final project we primarily used basic interpolation.

![Trajectory.png](RBE3001%20Final%20Project%2062d94d828d8142c18bbaaf0d2d68fa3a/Trajectory.png)

### Computer Vision Implementation

Finally for our final project we took our privous work with controlling the robotic arm and implimented computer vision to organize balls and pick up colored objects. The in-depth details can be found in the report linked below.

![Image Processing Flowchart.png](RBE3001%20Final%20Project%2062d94d828d8142c18bbaaf0d2d68fa3a/Image_Processing_Flowchart.png)