from numpy import cos, sin
import numpy as np

#############################################################################
# Forward Kinematics
# Tell the location of the manipulator's end-effector given the values of the joints.
#############################################################################


def main():
    # Stores the length of each link in cm
    lengths = [25.0, 2.0, 10.0, 5.0, 2.0]

    # **********************************************
    # Declare each length individually as a1, a2, etc
    # make lengths = [a1, a2, etc] after that.
    # change a, b, c, d, e, to a1, a2, a3, a4, a4
    # **********************************************
    # Angles might be good as it is cause each value must be changed to rad
    # **********************************************

    # Stores the desired angles in degrees
    angles = [0, 0, 0, 0]

    # Convert each angle into radians
    r = (angles[0]/180.0) * np.pi
    s = (angles[1]/180.0) * np.pi
    t = (angles[2]/180.0) * np.pi
    u = (angles[3]/180.0) * np.pi

    # Lengths made shorter
    a = lengths[0]; b = lengths[1]; c = lengths[2]; d = lengths[3]; e = lengths[4]

    # Rotation matrix frame 4 on 0 (end-effector on base frame)
    r0_4 = [
        [cos(u)*(-cos(r)*sin(s)*cos(t) - cos(r)*cos(s)*sin(t)),                     sin(u)*(-(-cos(r)*sin(s)*cos(t) - cos(r)*cos(s)*sin(t))),                   cos(r)*cos(s)*cos(t) - cos(r)*sin(s)*sin(t)],
        [cos(u)*(sin(r)*sin(s)*(-cos(t)) - sin(r)*cos(s)*sin(t)) - cos(r)*sin(u),   -sin(u)*(sin(r)*sin(s)*(-cos(t)) - sin(r)*cos(s)*sin(t)) - cos(r)*cos(u),   sin(r)*cos(s)*cos(t) - sin(r)*sin(s)*sin(t)],
        [cos(u)*(cos(s)*cos(t) - sin(s)*sin(t)),                                    sin(u)*(-(cos(s)*cos(t) - sin(s)*sin(t))),                                  sin(s)*cos(t) + cos(s)*sin(t)]
    ]

    # Print resulting matrix
    print("\n===== r0_4 =====")
    print(np.matrix(r0_4))
    print("\n================")

    # Homogenous matrix frame 4 on 0 (end-effector on base frame)
    h0_4 = [
        [cos(u)*(-cos(r)*sin(s)*cos(t) - cos(r)*cos(s)*sin(t)),                     sin(u)*(-(-cos(r)*sin(s)*cos(t) - cos(r)*cos(s)*sin(t))),                   cos(r)*cos(s)*cos(t) - cos(r)*sin(s)*sin(t),        b * cos(r) + c * cos(r)*cos(s) + (d + e)*(cos(r)*cos(s)*cos(t) - cos(r)*sin(s)*sin(t))],
        [cos(u)*(sin(r)*sin(s)*(-cos(t)) - sin(r)*cos(s)*sin(t)) - cos(r)*sin(u),   -sin(u)*(sin(r)*sin(s)*(-cos(t)) - sin(r)*cos(s)*sin(t)) - cos(r)*cos(u),   sin(r)*cos(s)*cos(t) - sin(r)*sin(s)*sin(t),        b * sin(r) + c * sin(r)*cos(s) + (d + e)*(sin(r)*cos(s)*cos(t) - sin(r)*sin(s)*sin(t))],
        [cos(u)*(cos(s)*cos(t) - sin(s)*sin(t)),                                    sin(u)*(-(cos(s)*cos(t) - sin(s)*sin(t))),                                  sin(s)*cos(t) + cos(s)*sin(t),                      a + c * sin(s) + (d + e)*(sin(s)*cos(t) + cos(s)*sin(t))],
        [0,                                                                         0,                                                                          0,                                                  1]
    ]
    
    # Print resulting matrix
    print("\n===== h0_4 =====")
    print(np.matrix(h0_4))
    print("\n================")

    # Displacement vector frame 4 on 0 (end-effector on base frame)
    d0_4 = [h0_4[0][3], h0_4[1][3], h0_4[2][3]]

    # Print the displacement vector
    print("\n===== d0_4 =====")
    print(np.matrix(d0_4))
    print("\n================")
    

# angles is an array with the values in degrees of each joint
# lengths is an array with the values in cm of each link
def denavitHartenberg(angles, lengths):
    """
    D-H parameter table for the manipulator

        THETA    ||    ALPHA    ||    R    ||    D

    1   theta1          90          a2          a1
    2   theta2          0           a3          0
    3   theta3 + 90     90          0           0
    4   theta4          0           0           a4 + a5
    """
    
    # Convert 90 degrees into radians
    rad90 = (90.0 / 180.0) * np.pi

    # Parameter table for the manipulator
    pTable = [
        [angles[0],             rad90,      lengths[1],     lengths[0]              ],
        [angles[1],             0,          lengths[2],     0                       ],
        [angles[2] + rad90,     rad90,      0,              0                       ],
        [angles[3],             0,          0,              lengths[3] + lengths[4] ]
    ]

    matrices = {}

    # determines the row accessed in pTable
    i = 0

    # get the homogeneous transformation matrices
    while i < 4:
        matrices[i] = [
            [cos(pTable[i][0]),     -sin(pTable[i][0]) * cos(pTable[i][1]),     sin(pTable[i][0]) * sin(pTable[i][1]),      pTable[i][2] * cos(pTable[i][0])],
            [sin(pTable[i][0]),     cos(pTable[i][0]) * cos(pTable[i][1]),      -cos(pTable[i][0]) * sin(pTable[i][1]),     pTable[i][2] * sin(pTable[i][0])], 
            [0,                     sin(pTable[i][1]),                          cos(pTable[i][1]),                          pTable[i][3]],
            [0,                     0,                                          0,                                          1]
        ]
        i += 1

    # Homogenous matrix frames 0 - 2
    h0_2 = np.dot(matrices[0], matrices[1])

    # Homogenous matrix frames 2 - 4
    h2_4 = np.dot(matrices[2], matrices[3])

    # Homogenous matrix frames 0 - 4
    h0_4 = np.dot(h0_2, h2_4)

    # Print resulting matrix
    print("\n===== h0_4 =====")
    print(np.matrix(h0_4))
    print("\n================")


if __name__ == "__main__":
    main()
