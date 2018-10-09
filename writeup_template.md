## Project: Kinematics Pick & Place
### This Readme contains desciption of works that I have done to complete KUKA210 Pick and Drop Project
---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png


---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf. 

This is it :)

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Running the forward_kinematics demo and turning the joint angles in joint_state_pulisher to examine the links postion, **especially gripper link aka end effector and link_5 aka wist center**, and kuka arm pose.

Also, I add this position to be the extra testing case for IK_debug.py
```
test_cases = {4:[[[1.99, 1.1324, 1.0011],
        [0.739359, 0.110256, -0.0915909, 0.657877]],
        [1.6961, 1.122, 1.0854],
        [0.58, 0.36, 0.2, 1.19, -0.56, -5.56]]}
```
![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here is the DB Table filled with joint information from [kr210.urdf.xacro](./kuku_arm/urdf/kr210.urdf.xacro)

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | - 0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

For example, a1 is _0.35_, which represents the joint distance between joint_1 and joint_2,measuring in x1.

```
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Here is how I find out theta1, theta2 and theta3:
**calculate WC position:**
```
Rrpy = simplify(Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll) * R_corr)
nx = Rrpy[0, 2]
ny = Rrpy[1, 2]
nz = Rrpy[2, 2]
wx = px - 0.303 * nx
wy = py - 0.303 * ny
wz = pz - 0.303 * nz
```
**Calculate orientation theta1 to 3:**
```
d4 = 0.96+0.54
d1 = 0.75
a3 = 0.054
a2 = 1.25
a1 = 0.35
#line projected from straight distance b/w O2 & WC to plane x y 
diagonal = sqrt(wx**2 + wy**2) - a1 
length_a = sqrt(a3**2 + d4**2)
#here we need to subtract the length of a1 and d1 to obtain the actual length starting from O2 
length_b = sqrt(diagonal**2 + (wz-d1)**2)
length_c = a2
# Cosine Laws formula:
angle_a = acos((length_b**2+length_c**2-length_a**2)/(2*length_b*length_c))
angle_b = acos((length_a**2+length_c**2-length_b**2)/(2*length_a*length_c))
angle_c = acos((length_a**2+length_b**2-length_c**2)/(2*length_a*length_b))

theta1 = atan2(wy, wx)
theta2 = pi/2 - angle_a - atan2((wz-d1), diagonal)
theta3 = pi/2 - angle_b - atan2(a3, d4)
```
**Lastly, use Euler angle composition to find out R3_6 to calculate theta4 to 6**
```
R3_6 = simplify(R0_3.inv('LU') * Rrpy)
```
![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In the **forward kinematics** part, I get the final transform from base_link to end effector by combine each joint transform using DH Table. 
This is great that it DH table reduces the number of variables needed to be substited because by chosing the right origin in each joint, many paramters are 0
```
# Create Modified DH parameters
s ={alpha0: 0,       a0: 0,      d1: 0.33+0.42,  q1: q1,
    alpha1: -90*dtr, a1: 0.35,   d2: 0,          q2: q2-90*dtr,
    alpha2: 0,       a2: 1.25,   d3: 0,          q3: q3,
    alpha3: -90*dtr, a3: -0.054, d4: 0.96+0.54,  q4: q4,
    alpha4: 90*dtr,  a4: 0,      d5: 0,          q5: q5,
    alpha5: -90*dtr, a5: 0,      d6: 0,          q6: q6,
    alpha6: 0,       a6: 0,      d7: 0.193+0.11, q7: 0}
# Define Modified DH Transformation matrix
def transMat(q, d, a, alpha):
    transform = Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]])
    return transform.subs(s)
T0_1 = transMat(q1, d1, a0, alpha0)
T0_2 = simplify(T0_1 * transMat(q2, d2, a1, alpha1))
T0_3 = simplify(T0_2 * transMat(q3, d3, a2, alpha2))
T0_4 = simplify(T0_3 * transMat(q4, d4, a3, alpha3))
T0_5 = simplify(T0_4 * transMat(q5, d5, a4, alpha4))
T0_6 = simplify(T0_5 * transMat(q6, d6, a5, alpha5))
T0_EE = simplify(T0_6 * transMat(q7, d7, a6, alpha6))
```   
In the **inverse kinematics** part, i got the geometry wrong on the first trial, where I didn't substract some of the arm's length.

I need to pay attention to more paramters and consider more scenarios when doing this part. e.g. 
* need to subtract *a1* here `diagonal = sqrt(wx**2 + wy**2) - a1`
* need to subtract *d1* here `length_b = sqrt(diagonal**2 + (wz-d1)**2)`

Finally getting theta4, theta5 and theta6. In this part, we can reuse the tranform `T0_3` to get the rotation `R0_3`. After that, *Euler angles* and *atan2* lets me solve the problem. 
```
R0_3 = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})[0:3, 0:3]
R0_3 = R0_3.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
R3_6 = simplify(R0_3.inv('LU') * Rrpy)
#print (R4_6)
r31 = R3_6[2, 0]
r32 = R3_6[2, 1]
r33 = R3_6[2, 2]
r11 = R3_6[0, 0]
r21 = R3_6[1, 0]
theta4 = atan2(r32, r33) #R3_4 #roll #Rx
theta5 = atan2(-r31, sqrt(r11**2+r21**2)) #R4_5 #pitch #Ry
theta6 = atan2(r21, r11) #R5_6 #yaw #Rz
```

And just for fun, another example image:
![alt text][image3]


