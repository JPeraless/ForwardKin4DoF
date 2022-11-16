from numpy import cos, sin
import numpy as np
from math import sqrt, acos, atan

#############################################################################
# Inverse Kinematics
# Get to specify where we want the end-effector to be 
# and calculate the angles needed to get to that position.
#############################################################################

'''
For 6 DoF manipulators, as they normally consist of a spherical wrist attached to a
standard 3 DoF manipulator, it is assumed that the first 3 joints (standard manipulator)
dictate the position of the end-effector, whereas the last 3 joints (spherical wrist)
dictate its rotation.

In this case, however, a 4 DoF manipulator is being used, and it is not viable to
assume that the first 3 joints are only responsible for position while the last one
entirely determines the end-effector's rotation.

Thus, one cannot expect to make use of the fundamental IK assumptions.

'''

# IK from frame 0 to 4, not from frame 0 to 3 as suggested for 6 DoF manipulators
def main():
    
    # Set link lengths
    a1 = 0; a2 = 0; a3 = 0; a4 = 0; a5 = 0

    # Desired end-effector coordinates
    x = 0
    y = 0

    # Height from frame 0 to 2
    z0_2 = 0  # not sure how to get this

    # y displacement from frame 1 to 2
    r2 = z0_2 - a1  # can z0_2 be used? Might need to use Z as a whole and subtract from it

    # x displacement from frame 1 to 2
    r1 = sqrt((a3 * a3) + (r2 * r2))  # only valid if z0_2 is

    # Straight line between frame 1 and 4
    k = sqrt((x * x) + (y * y)) - a2

    # phi1 = 180 - theta1
    phi1 = acos(((a3 * a3) + (a4 + a5) - (k * k)) / 2 * a3 * (a4 + a5))

    # Joint variable equations
    theta1 = atan(x / y)
    theta2 = atan(r2 / r1)  # asin(r2 / a3) should also work
    theta3 = 180 - phi1

    # Use to test correctness of k (needs phi1 to be known)
    kTest = sqrt((a3 * a3) + (a4 + a5) - 2 * a3 * (a4 + a5) * np.cos(phi1))

def finalRotation(theta1, theta2, theta3):
    r = theta1; s = theta2; t = theta3

    # Desired end-effector rotation on base frame
    r0_4 = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ]

    # Rotation matrix frame 3 on 0
    r0_3 = [
        [-cos(r)*sin(s)*cos(t) - cos(r)*cos(s)*sin(t),    0,           cos(r)*cos(s)*cos(t) - cos(r)*sin(s)*sin(t)],
        [sin(r)*sin(s)*(-cos(t)) - sin(r)*cos(s)*sin(t),  -cos(r),     sin(r)*cos(s)*cos(t) - sin(r)*sin(s)*sin(t)],
        [cos(s)*cos(t) - sin(s)*sin(t),                 0,           sin(s)*cos(t) + cos(s)*sin(t)]
    ]

    invR0_3 = np.linalg.inv(r0_3)

    r3_4 = np.dot(invR0_3, r0_4)

    print("\n===== r3_4 =====")
    print(np.matrix(r3_4))
    print("\n================")

    # Test rotation matrix for frame 4 on 3
    testR3_4 = [
        [cos(u),    -sin(u),    0],
        [sin(u),    cos(u),     0],
        [0,         0,          1]
    ]


if __name__ == "__main__":
    main()
 