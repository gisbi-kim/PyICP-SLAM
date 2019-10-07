import os 
import csv
import copy
import time
import math

import numpy as np
import matplotlib.pyplot as plt

import minisam


def getConstDigitsNumber(val, num_digits):
    return "{:.{}f}".format(val, num_digits)

def getUnixTime():
    return int(time.time())

def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R

def yawdeg2so3(yaw_deg):
    yaw_rad = np.deg2rad(yaw_deg)
    return eulerAnglesToRotationMatrix([0, 0, yaw_rad])

def yawdeg2se3(yaw_deg):
    se3 = np.eye(4)
    se3[:3, :3] = yawdeg2so3(yaw_deg)
    return se3 


def getGraphNodePose(graph, idx):
    pose = graph.at(minisam.key('x', idx))
    pose_trans = pose.translation()
    pose_rot = pose.so3().matrix()
    return pose_trans, pose_rot

def saveOptimizedGraphPose(curr_node_idx, graph_optimized, filename):
    for opt_idx in range(curr_node_idx):
        pose_trans, pose_rot = getGraphNodePose(graph_optimized, opt_idx)
        pose_trans = np.reshape(pose_trans, (-1, 3)).squeeze()
        pose_rot = np.reshape(pose_rot, (-1, 9)).squeeze()
        optimized_pose_ith = np.array([ pose_rot[0], pose_rot[1], pose_rot[2], pose_trans[0], 
                                        pose_rot[3], pose_rot[4], pose_rot[5], pose_trans[1], 
                                        pose_rot[6], pose_rot[7], pose_rot[8], pose_trans[2],
                                        0.0, 0.0, 0.0, 0.1 ])
        if(opt_idx == 0):
            optimized_pose_list = optimized_pose_ith
        else:
            optimized_pose_list = np.vstack((optimized_pose_list, optimized_pose_ith))

    np.savetxt(filename, optimized_pose_list, delimiter=",")


class PoseGraphResultSaver:
    def __init__(self, init_pose, save_gap, num_frames, seq_idx, save_dir):
        self.pose_list = np.reshape(init_pose, (-1, 16))
        self.save_gap = save_gap
        self.num_frames = num_frames

        self.seq_idx = seq_idx
        self.save_dir = save_dir

    def saveUnoptimizedPoseGraphResult(self, cur_pose, cur_node_idx):
        # save 
        self.pose_list = np.vstack((self.pose_list, np.reshape(cur_pose, (-1, 16))))

        # write
        if(cur_node_idx % self.save_gap == 0 or cur_node_idx == self.num_frames):        
            # save odometry-only poses
            filename = "pose" + self.seq_idx + "unoptimized_" + str(getUnixTime()) + ".csv"
            filename = os.path.join(self.save_dir, filename)
            np.savetxt(filename, self.pose_list, delimiter=",")

    def saveOptimizedPoseGraphResult(self, cur_node_idx, graph_optimized):
        filename = "pose" + self.seq_idx + "optimized_" + str(getUnixTime()) + ".csv"
        filename = os.path.join(self.save_dir, filename)
        saveOptimizedGraphPose(cur_node_idx, graph_optimized, filename)    
    
        optimized_pose_list = np.loadtxt(open(filename, "rb"), delimiter=",", skiprows=1)
        self.pose_list = optimized_pose_list # update with optimized pose 

    def vizCurrentTrajectory(self, fig_idx):
        x = self.pose_list[:,3]
        y = self.pose_list[:,7]
        z = self.pose_list[:,11]

        fig = plt.figure(fig_idx)
        plt.clf()
        plt.plot(-y, x, color='blue') # kitti camera coord for clarity
        plt.axis('equal')
        plt.xlabel('x', labelpad=10)
        plt.ylabel('y', labelpad=10)
        plt.draw()
        plt.pause(0.01) #is necessary for the plot to update for some reason

