## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/illustrator.png
[image2]: ./misc_images/coord.png
[image3]: ./misc_images/screenshot.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The DH parameters are defined and illustrated in Figure 1. Each cylinder represents a joint. Only non-zero distances and angles are shown. To clarify, $\theta_1$ rotates into the page, $\theta_4$ and $\theta_6$ are on the $x_5$-$z_5$ plane and $\theta_5$ is on the $x_4$-$z_z$ plane. Pink dashed lines represent potential rotations of the adjacent axis. Based on the kr210.urdf.xacro file, the DH parameter table was derived as below.

Links | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

$d_1=0.33+0.42=0.75$

$\alpha_1=-\pi/2$, the sign is determined by right hand rule.

$a_1=0.35$, the distance between $z_1$ and $z_2$ in the direction of $x_1$.

$\theta_2=-\pi/2+\theta_2'$, where $\theta_2'$ is the twist angle away from $x_1$. There is an offset of $x_2$ in the $-\pi/2$ direction.

$a_2=1.25$, the distance between $z_2$ and $z_3$ in the $x_2$ direction. Because the $x_2$ direction is parallel with the z direction of the URDF system, we use the z-increment of joint 3 here.

$\alpha_3=-\pi/2$, the angle between $z_3$ and $z_4$ about $x_3$.

$a_3=-0.054$, the distance between $z_3$ and $z_4$ in the direction of $x_3$. In the URDF system, it corresponds to the z-increment of joint 4.

$d_4=0.96+0.54=1.5$. Because $x_4$ is located at joint 5 (wrist center), the distance between $x_3$ and $x_4$ is the increment of joint 4 plus the increment of joint 5 in the $z_4$ direction.

$\alpha_4=\pi/2$, the angle between $z_4$ and $z_5$ in the direction of $x_4$.

$\alpha_5=-\pi/2$, the angle between $z_5$ and $z_6$ in the direction of $x_5$.

$d_5=d_6=0$, because the origins are located at joint 5 for joint 4,5,6.

$d_{EE}=0.193+0.11=0.303$, the end-effector's distance from $x_6$ is the distance between joint 5 and joint 6 plus the length of the gripper.

![alt text][image1]

Figure 1. Illustration of DH parameters.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The homogeneous transform between two links is
\[
T_i^{i-1} = \begin{bmatrix} c\theta_i & -s\theta_i & 0 & a_{i-1} \\ s\theta_ic\alpha_{i-1} & c\theta_ic\alpha_{i-1} & -s\alpha_{i-1} & -s\alpha_{i-1}d_i \\ s\theta_is\alpha_{i-1} & c\theta_is\alpha_{i-1} & c\alpha_{i-1} & c\alpha_{i-1}d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
\]

The individual transformation matrices were calculated by plugging in the DH parameters. The generalized homogeneous transform between base_link and gripper_link is hence
\[
T_{EE}^0=T_1^0T_2^1T_3^2T_4^3T_5^4T_6^5T_{EE}^6
\]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The position of the wrist center was obtained using
\[
^0r_{WC/0}=^0r_{EE/0}-d\cdot R_{EE}^0\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}=\begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix}-d\cdot R_{EE}^0\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
\]
in which $p_x$, $p_y$ and $p_z$ was the gripper position from the request. $d$ is the distance between the wrist center and the end-effector ($d=0.193+0.11=0.303$). $R_{EE}^0$ is the orientation of the EE. The orientation matrix was calculated from the roll, pitch and yaw from the request
$R_{EE}^0=R_zR_yR_x$. The matrix was then corrected to the DH system from the URDF system:
\[
R_{corr}=R_z(\pi)R_y(-\pi/2)
\]
as we do an intrinsic rotation of the gripper coordinate first about the z-axis for $\pi$ and then about the y-axis for $-\pi/2$ to the URDF reference coordinate. Up to here, we calculated the inverse position kinematics of the wrist center, from which we can derive all the twist angles of the joints.

$\theta_1$ is the angle rotated away from the x-z plane of the URDF system (inwards the page). If we look at the wrist center from above (z), it corresponds to the orientation in the x-y plan (Fig.2i).

$\theta_2$ (in fact it is the $\theta_2'$ as described above) is the angle away from $x_1$ in the clockwise direction and is equal to the angle between link 2-3 and z-axis (Fig. 2ii). Thus $\theta_2=\pi/2-\angle A-\phi$. To calculate $\angle A$, we use $cos(\angle A)=\frac{AB^2+CA^2-BC^2}{2AB\cdot CA}$. $AB$ is the length of link 2-3 and is equal to 1.25. $BC$ is the distance between joint 3 and joint 5 (Fig. 2iii). To calculate $BC$, we use$BC^2=BD^2+CD^2-2BD\cdot CDcos(\gamma+\pi/2)=BD^2+CD^2+2BD\cdot CDsin(\gamma)=BE^2+DE^2+CD^2+2BE\cdot CD$.
$\phi$ is the angle between z-axis and the 2-WC link projection on the x-y plane: $tan(\phi)=z_{WC}/\sqrt{x_{WC}^2+y_{WC}^2}$.

$\theta_3$ is the rotational angle of $x_3$. From Fig. 2iv, it is easy to see that $\theta_3=\pi/2-\delta-\angle B$.

The angles $\theta_4$, $\theta_5$ and $\theta_6$ can be calculated from the wrist orientation. Because the base_link to gripper_link orientation can be represented by the roll, pitch and yaw of the EE, using the DH transforms, we have
\[
R_{EE}^0=R_3^0R_{EE}^3=R_{rpy}
\]
Thus $R_{EE}^3=R_3^{0-1}R_{rpy}$, where $R_3^0=R_1^0R_2^1R_3^2$, which can be calculated by plugging the $\theta_1$, $\theta_2$ and $\theta_3$ in the rotation portion of the transform matrix. The left-hand-side of the equation is
\[
R_{EE}^3=R_4^3R_5^4R_{EE}^5=\begin{bmatrix} c\theta_4c\theta_5c\theta_6-s\theta_4s\theta_6 & -c\theta_4c\theta_5s\theta_6-s\theta_4c\theta_6 & -c\theta_4s\theta_5 \\ s\theta_5c\theta_6 & -s\theta_5s\theta_6 & c\theta_5 \\ -s\theta_4c\theta_5c\theta_6-c\theta_4s\theta_6 & -s\theta_4c\theta_5s\theta_6-c\theta_4c\theta_6 & s\theta_4s\theta_5 \end{bmatrix}
\]

From this, it is easy to calculate $\theta_4$, $\theta_5$ and $\theta_6$.

![alt text][image2]

Figure 2. Illustration of the calculation of twist angles. Red numbers/letters are joints. The coordinate is the URDF reference system.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The code implemented the forward kinematics and the inverse kinematics to calculate the $\theta$s. The DH parameter table was constructed based on Fig. 1. It successfully derived the inverse kinematics of the robot arm to pick and drop the objects. The robot can complete the task almost 100% of the time. Figure 3 shows the screen shots of the robot arm picking up an object. However, the time it takes to move is relatively long. This might be due to inefficient calculation of the inverse kinematics. To improve the performance, constraints of the joint angles should be included.
The code 'IK_server.py' is included in the submission.

![alt text][image3]

Figure 3. Screen shots of a successful picking-up.
