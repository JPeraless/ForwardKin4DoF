#############################################################################
# Inverse Kinematics (Analytical solution by geometric approach)
# Given the target position and orientation of the end-effector, 
# calculate the joint parameters needed to get to that position.
#############################################################################

from numpy import cos, sin
import numpy as np
from math import sqrt, acos, atan


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

def finalRotation():
    pass


def main():
    pass


if __name__ == "__main__":
    main()