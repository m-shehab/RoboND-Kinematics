# Project: Robot Arm - Pick & Place 
Before starting kinematics analysis, we build the project as explained in RoboND-Kinematics-Project readme file 
---
**Then our work main steps are:**
1. Derivation of DH prameters. 
2. Create Transformation Matrices.
3. Correction Between URDF and DH parameters. 
4. Inverse Kinematics Calculation. 
5. Handling Multiple Solutions of theta5 
5. Run IK_Server.py. 
6. Task Completion 

[//]: # (Image References) 
[image1]: ./images/Figure_1.jpg
[image2]: ./images/Figure_2.jpg
[image3]: ./images/Figure_3.jpg
[image4]: ./images/Figure_4.jpg
[image5]: ./images/Figure_5.jpg
[image6]: ./images/Figure_6.jpg
[image7]: ./images/Figure_7.jpg
[image8]: ./images/Figure_8.jpg
[image9]: ./images/Figure_9.jpg
[image10]: ./images/Figure_10.jpg

## We explain each of these steps below 
### 1. DH Parmaters
We derive DH parameters from arm geometry to produce DH table and create the described transformation matrix.

![alt_text][image1]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

### 2. Transformation Matrices 
We need 7 transformation matrices, as shown below, for successive link relations. a function called `Transformation_matrix` is created, taking alpha, a, d, and q as arguments and return the corresponding 4*4 transformation matrix. **Note:** Forward kinematics is performed out of the loop of EE points, to achieve performance optimization. 

![alt_text][image2]

The whole transformation matrix of the end effector relative to the base could be driven from the above transformations. However, if the End Effector pose is given, then we can deduce the whole transformation matrix for certain pose directly. 

![alt_text][image3]

### 3. Correction between URDF and DH 
As mentioned in the lesson there is a difference between urdf parameters and DH parameters in EE frame orientation. Thus the roll, pitch and yaw angles given in the request will define a rotation matrix according to urdf parameters. We need to correct this rotation matrix to match EE frame w.r.t DH paramters. **Our Correction Algorithm** is different from the one in the lesson. All we need, from the relation between EE frame in urdf and dh, is column swapping in the EE rotation matrix w.r.t urdf parameters to turn it to DH parameters. a function called `rpy2rot_DH` return the modified rotation matrix w.r.t DH parameters.
**Note**.
1. Zee w.r.t urdf is equivalent to Xee w.r.t DH.
2. Xee w.r.t urdf is equivalent to Zee w.r.t DH.
3. Yee w.r.t urdf is equivalent to -Yee w.r.t. DH. 

### 4. Inverse kinematics Calculation 
1. calculate WC=[WCx,WCy,WCz] from EE position and R0_6 (according to DH parameters).
2. Calculate the first three joint variables (j1, j2, and j3) from arm geometry.
    * theta1 = atan2(WCy,WCx)
    * theta2 = pi/2 - angle_a - atan2(WCz-0.75,sqrt(WCx**2+WCy**2)-0.35)      
    * theta3 = pi/2 - angle_b - atan2(0.054,1.5) 
3. Calculate wrist orientation R3_6 = inv(R0_3)*R0_6.
4. Get euler angles from wrist orientation to calculate j4, j5, and j6.
    * theta4 = atan2(R3_6[2,2],-R3_6[0,2])
    * theta5 = atan2(m.sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
    * theta6 = atan2(-R3_6[1,1],R3_6[1,0])
  
![alt_text][image4]

![alt_text][image5]

### 5. Handling Multiple Solution 
Actually for the same orientation of the spherical wrist (i.e. R3_6) we have multiple values of theta5,
and consequently theta4 and theta6 must be handled in a correct way to give a correct solution as shown in the figure. 

![alt_text][image6]

### 6. Run the project 
1. run the command `./safe_spawner.sh` to launch simulations. 
2. run the command `rosrun kuka_arm IK_server.py` to call our service calculating inverse kinematics.
3. Through RViz interact with KuKa arm in gazebo, motion planning is handled for us. 
4. Our Inverse Kinematics solution is responsible for tracking the defined trajectory.

![alt text][image7]

![alt text][image8]

### 7. Task Completion 
**Note:** My algorithm worked well for all the test cases (ten successive processes), but unfortunatly the cylinders dropped off above each other
making a tower of cylinders. The arm hit that tower in some iteration and one of the cylinders have been thrown away from the basket. 
So, I tried another experiment (i.e. I have 11 successive processes with 10 cylinders in the basket as shown in the 2nd figure)     

![alt_text][image9]

![alt_text][image10]
