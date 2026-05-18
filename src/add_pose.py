
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)
    dx = 2.0 * np.cos(np.deg2rad(45)) # x-movement
    dy = 2.0 * np.sin(np.deg2rad(45)) # y-movement
    dtheta = np.deg2rad(90) # degrees to rad

    # Between X(3) and X(4): Move forward 2m and rotates 45 degrees (anti-clockwise)
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), gtsam.Pose2(dx, dy, dtheta), ODOMETRY_NOISE))

    # Insert initial guesses for poses (Pose2: x, y, theta)
    initial_estimate.insert(X(4), gtsam.Pose2(5.5, 1.5, np.deg2rad(90)))
    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    
    return graph, initial_estimate