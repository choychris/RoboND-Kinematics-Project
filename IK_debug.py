from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''
# WC position = link_5 position
test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[1.99, 1.1324, 1.0011],
                  [0.739359, 0.110256, -0.0915909, 0.657877]],
                  [1.6961, 1.122, 1.0854],
                  [0.58, 0.36, 0.2, 1.19, -0.56, -5.56]],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    ### Your IK code here
    # Create individual transformation matrices
    def Rot_z(z):
        r = Matrix([[cos(z), -sin(z), 0, 0],
                    [sin(z),  cos(z), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        return r
    def Rot_y(y):
        r = Matrix([[cos(y), 0,  sin(y), 0],
                    [0, 1, 0, 0],
                    [-sin(y), 0, cos(y), 0],
                    [0, 0, 0, 1]])
        return r
    def Rot_x(x):
        r = Matrix([[1, 0, 0, 0],
                    [0, cos(x), -sin(x), 0],
                    [0, sin(x),  cos(x), 0],
                    [0, 0, 0, 1]])
        return r
    dtr = pi/180
    R_y = Rot_y(-90*dtr)
    R_z = Rot_z(180*dtr)
    R_corr = simplify(R_z * R_y)
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
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
    # print (T0_1)
    T1_2 = transMat(q2, d2, a1, alpha1)
    # print (T1_2)
    T2_3 = transMat(q3, d3, a2, alpha2)
    # print (T2_3)
    T3_4 = transMat(q4, d4, a3, alpha3)
    # print (T3_4)
    T4_5 = transMat(q5, d5, a4, alpha4)
    # print (T4_5)
    T5_6 = transMat(q6, d6, a5, alpha5)
    # print (T5_6)
    T6_EE = transMat(q7, d7, a6, alpha6)
    # print (T6_EE)

    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_EE = simplify(T0_6 * T6_EE)
    # print (T0_EE)
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    # we need to inverse R_crr here since Rgazebo_EE = DH0_EE * R_corr
    # Rrpy(orientation from gazebo) = T0_EE * R_corr
    # T0_EE = Rrpy * R_corr.inv()
    # print simplify(transMat(q4, d4, a3, alpha3) * transMat(q5, d5, a4, alpha4) * transMat(q6, d6, a5, alpha5))
    R0_6 = simplify(Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll) * R_corr.inv("LU"))
    nx = R0_6[0, 2]
    ny = R0_6[1, 2]
    nz = R0_6[2, 2]
    wx = px - 0.303 * nx
    wy = py - 0.303 * ny
    wz = pz - 0.303 * nz

    # Calculate joint angles using Geometry method
    theta1 = atan2(wy, wx)
    d4 = 0.96+0.54
    d1 = 0.75
    a3 = 0.054
    a2 = 1.25
    a1 = 0.35
    diagonal = sqrt(wx**2 + wy**2) - a1 #line projected from straight distance b/w O2 & WC to plane x y 
    length_a = sqrt(a3**2 + d4**2)
    # here we need to subtract the length of a1 and d1 to obtain the actual length starting from O2 
    length_b = sqrt(diagonal**2 + (wz-d1)**2) # this is where I did wrong
    length_c = a2
    # Cosine Laws formula:
    # length_a**2 = length_b**2 + length_c**2 - 2*length_b*length_c*cos(angle_a)
    # inverse to find angle_a
    angle_a = acos((length_b**2+length_c**2-length_a**2)/(2*length_b*length_c))
    angle_b = acos((length_a**2+length_c**2-length_b**2)/(2*length_a*length_c))
    angle_c = acos((length_a**2+length_b**2-length_c**2)/(2*length_a*length_b))
    # print (angle_a, angle_b, angle_c)
    # print (angle_a+angle_b+angle_c)
    theta2 = pi/2 - angle_a - atan2((wz-d1), diagonal)
    theta3 = pi/2 - angle_b - atan2(a3, d4)

    # Calculate R4_6 
    # R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    # R0_3 = R0_3.col_del(3).row_del(3)
    # R0_3 = R0_3.col_insert(3, Matrix([0, 0, 0]))
    # R0_3 = R0_3.row_insert(3, Matrix([[0, 0, 0, 1]]))
     
    R0_3 = T0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})[0:3, 0:3]
    R0_3 = R0_3.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
    R3_6 = simplify(R0_3.inv('LU') * R0_6)
    # r31 = R3_6[2, 0]
    # r32 = R3_6[2, 1]
    # r33 = R3_6[2, 2]
    # r11 = R3_6[0, 0]
    # r21 = R3_6[1, 0]
    # theta4 = atan2(r32, r33) #R3_4 #roll #Rx
    # theta5 = atan2(-r31, sqrt(r11**2+r21**2)) #R4_5 #pitch #Ry
    # theta6 = atan2(r21, r11) #R5_6 #yaw #Rz
    theta5 = atan2(sqrt(R3_6[0,2]**2+R3_6[2,2]**2), R3_6[1,2])
    if sin(theta5) < 0:
        theta4 = atan2(-R3_6[2,2], R3_6[0,2])
        theta6 = atan2(R3_6[1,1], -R3_6[1,0])
    else:
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    ###

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    ### Your FK code here
    

    # Create individual transformation matrices
    T_final = simplify(T0_EE * R_corr)
    base_link = Matrix([[0], [0], [0], [1]])
    EE = simplify(T_final * base_link).evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    WC = simplify((T0_5) * base_link).evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: 0, q5: 0})
    # print (WC)
    # print (wx, wy, wz)
    # Extract rotation matrices from the transformation matrices
    # R_final = T_final.row_del(3).col_del(3)
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    wc_fromDH_table = [WC[0,0], WC[1,0], WC[2,0]]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [EE[0,0],EE[1,0],EE[2,0]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))
    if not(sum(wc_fromDH_table)==3):
        print (wc_fromDH_table)
        wc_x_eh = abs(wc_fromDH_table[0]-test_case[1][0])
        wc_y_eh = abs(wc_fromDH_table[1]-test_case[1][1])
        wc_z_eh = abs(wc_fromDH_table[2]-test_case[1][2])
        wc_offseth = sqrt(wc_x_eh**2 + wc_y_eh**2 + wc_z_eh**2)
        print ("\nWrist error from DH for x position is: %04.8f" % wc_x_eh)
        print ("Wrist error from DH for y position is: %04.8f" % wc_y_eh)
        print ("Wrist error from DH for z position is: %04.8f" % wc_z_eh)
        print ("Overall wrist from DH offset is: %04.8f units" % wc_offseth)

    # Find WC error
    if not(sum(your_wc)==3):
        print (your_wc)
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
