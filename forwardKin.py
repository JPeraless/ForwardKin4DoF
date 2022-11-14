import numpy as np

#############################################################################
# Forward Kinematics
# Tell the location of the manipulator's end-effector given the values of the joints.
#############################################################################


def main():
    # Stores the length of each link in cm
    lengths = [0, 0, 0, 0, 0]

    # Stores the desired angles in degrees
    angles = [0, 0, 0, 0]

    # Convert each angle into radians
    angles[0] = (angles[0]/180.0) * np.pi
    angles[0] = (angles[1]/180.0) * np.pi
    angles[2] = (angles[2]/180.0) * np.pi
    angles[3] = (angles[3]/180.0) * np.pi

    # Rotation matrix frame 1 on 0
    r0_1 = [
        [np.cos(angles[0]),     0,              0                   ],
        [np.sin(angles[0]),     0,              -np.cos(angles[0])  ],
        [0,                     1,              0                   ]]

    # Rotation matrix frame 2 on 1
    r1_2 = [
        [np.cos(angles[1]),     -np.sin(angles[1]),     0           ],
        [np.sin(angles[1]),     np.cos(angles[1]),      0           ],
        [0,                     0,                      1           ]]

    # Rotation matrix frame 3 on 2
    r2_3 = [
        [-np.sin(angles[2]),    0,              np.cos(angles[2])   ],
        [np.cos(angles[2]),     0,              np.sin(angles[2])   ],
        [0,                     1,              0                   ]]

    # Rotation matrix frame 4 on 3
    r3_4 = [
        [np.cos(angles[3]),     -np.sin(angles[3]),     0           ],
        [np.sin(angles[3]),     np.cos(angles[3]),      0           ],
        [0,                     0,                      1           ]]
    
    # Rotation matrix frame 2 on 0
    r0_2 = np.dot(r0_1, r1_2)

    # Rotation matrix frame 4 on 2
    r2_4 = np.dot(r2_3, r3_4)

    # Final rotation matrix (end-effector on base frame)
    r0_4 = np.dot(r0_2, r2_4)

    # Print resulting matrix
    print("\n === r0_4 ===")
    print(np.matrix(r0_4))

    d0_1 = [    
        [lengths[0] * np.cos(angles[0])],
        [lengths[0] * np.sin(angles[0])],
        [lengths[0]]
    ]

    d1_2 = [
        [lengths[1] * np.cos(angles[1])],
        [lengths[1] * np.sin(angles[1])],
        [0]
    ]

    d2_3 = [
        [0],
        [0],
        [0]
    ]

    d3_4 = [
        [0],
        [0],
        [lengths[3] + lengths[4]]
    ]    


# rotations contains the rotation matrices of the rover
# vectors contains the displacement vectors of the rover
def homogeneousMatrices(rotations, vectors):

    # Homogenous matrix frames 0 - 1
    h0_1 = np.concatenate((rotations[0], vectors[0]), 1)  # First left, second right
    h0_1 = np.concatenate((h0_1, [[0, 0, 0, 1]]), 0)  # First top, second bottom

    # Homogenous matrix frames 1 - 2
    h1_2 = np.concatenate((rotations[1], vectors[1]), 1)
    h1_2 = np.concatenate((h1_2, [[0, 0, 0, 1]]), 0)

    # Homogenous matrix frames 2 - 3
    h2_3 = np.concatenate((rotations[2], vectors[2]), 1)
    h2_3 = np.concatenate((h2_3, [[0, 0, 0, 1]]), 0)

    # Homogenous matrix frames 3 - 4
    h3_4 = np.concatenate((rotations[3], vectors[3]), 1)
    h3_4 = np.concatenate((h3_4, [[0, 0, 0, 1]]), 0)

    # Homogenous matrix frames 4 - 5
    h4_5 = np.concatenate((rotations[4], vectors[4]), 1)
    h4_5 = np.concatenate((h4_5, [[0, 0, 0, 1]]), 0)

    # Homogenous matrix frames 0 - 2
    h0_2 = np.dot(h0_1, h1_2)

    # Homogenous matrix frames 2 - 4
    h2_4 = np.dot(h2_3, h3_4)

    # Homogenous matrix frames 0 - 4 (end-effector on base frame)
    h0_4 = np.dot(h0_2, h2_4)

    # Print resulting matrix
    print("\n === h0_4 ===")
    print(np.matrix(h0_4))

# ===========================================================================

# DIFFERENT APPROACHES TO GET HOMOGENOUS TRANSFORMATION MATRICES (SOLVE FK)

# ===========================================================================

# angles is an array with the values in degrees of each joint
# lengths is an array with the values in cm of each link
def denavitHartenberg(angles, lengths):
    """
    Sketch of the D-H parameter table

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
            [np.cos(pTable[i][0]), -np.sin(pTable[i][0]) * np.cos(pTable[i][1]), np.sin(pTable[i][0]) * np.sin(pTable[i][1]), pTable[i][2] * np.cos(pTable[i][0])],
            [np.sin(pTable[i][0]), np.cos(pTable[i][0]) * np.cos(pTable[i][1]), -np.cos(pTable[i][0]) * np.sin(pTable[i][1]), pTable[i][2] * np.sin(pTable[i][0])], 
            [0, np.sin(pTable[i][1]), np.cos(pTable[i][1]), pTable[i][3]]
            [0, 0, 0, 1]
        ]
        i += 1

    # Homogenous matrix frames 0 - 2
    h0_2 = np.dot(matrices[0], matrices[1])

    # Homogenous matrix frames 2 - 4
    h2_4 = np.dot(matrices[2], matrices[3])

    # Homogenous matrix frames 0 - 4
    h0_4 = np.dot(h0_2, h2_4)

    # Print resulting matrix
    print("\n === h0_5 ===")
    print(np.matrix(h0_4))


if __name__ == "__main__":
    main()
