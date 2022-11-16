import numpy as np
from math import sqrt, acos, atan

#############################################################################
# Inverse Kinematics
# Get to specify where we want the end-effector to be 
# and calculate the angles needed to get to that position.
#############################################################################


# Does IK from base frame to end-effector (0 - 4), not from frame 0 to 3 as suggested.
def main():
    
    # Set link lengths
    a1 = 0; a2 = 0; a3 = 0; a4 = 0; a5 = 0

    # Desired end-effector coordinates
    x = 0
    y = 0

    # Height from frame 0 to 2
    z0_2 = 0  # not sure how to get this

    # y displacement from frame 1 to 2
    r2 = z0_2 - a1  # can z0_2 be used?

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


if __name__ == "__main__":
    main()
