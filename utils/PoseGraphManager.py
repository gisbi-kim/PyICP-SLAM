import numpy as np
np.set_printoptions(precision=4)

# import minisam
import gtsam
from utils.UtilsMisc import *
    
class PoseGraphManager:
    def __init__(self):

        self.prior_cov = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        self.const_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])
        self.odom_cov = gtsam.noiseModel.Diagonal.Sigmas(self.const_cov)
        self.loop_cov = gtsam.noiseModel.Diagonal.Sigmas(self.const_cov)

        self.graph_factors = gtsam.NonlinearFactorGraph()
        self.graph_initials = gtsam.Values()

        self.opt_param = gtsam.LevenbergMarquardtParams()
        self.opt = gtsam.LevenbergMarquardtOptimizer(self.graph_factors, self.graph_initials, self.opt_param)

        self.curr_se3 = None
        self.curr_node_idx = None
        self.prev_node_idx = None

        self.graph_optimized = None

    def addPriorFactor(self):
        self.curr_node_idx = 0
        self.prev_node_idx = 0

        self.curr_se3 = np.eye(4)

        self.graph_initials.insert(gtsam.symbol('x', self.curr_node_idx), gtsam.Pose3(self.curr_se3))
        self.graph_factors.add(gtsam.PriorFactorPose3(
                                                gtsam.symbol('x', self.curr_node_idx), 
                                                gtsam.Pose3(self.curr_se3), 
                                                self.prior_cov))

    def addOdometryFactor(self, odom_transform):

        self.graph_initials.insert(gtsam.symbol('x', self.curr_node_idx), gtsam.Pose3(self.curr_se3))
        self.graph_factors.add(gtsam.BetweenFactorPose3(
                                                gtsam.symbol('x', self.prev_node_idx), 
                                                gtsam.symbol('x', self.curr_node_idx), 
                                                gtsam.Pose3(odom_transform), 
                                                self.odom_cov))

    def addLoopFactor(self, loop_transform, loop_idx):

        self.graph_factors.add(gtsam.BetweenFactorPose3(
                                        gtsam.symbol('x', loop_idx), 
                                        gtsam.symbol('x', self.curr_node_idx), 
                                        gtsam.Pose3(loop_transform), 
                                        self.odom_cov))

    def optimizePoseGraph(self):

        self.opt = gtsam.LevenbergMarquardtOptimizer(self.graph_factors, self.graph_initials, self.opt_param)
        self.graph_optimized = self.opt.optimize()

        # status = self.opt.optimize(self.graph_factors, self.graph_initials, self.graph_optimized)
        # if status != minisam.NonlinearOptimizationStatus.SUCCESS:
            # print("optimization error: ", status)

        # correct current pose 
        pose_trans, pose_rot = getGraphNodePose(self.graph_optimized, self.curr_node_idx)
        self.curr_se3[:3, :3] = pose_rot
        self.curr_se3[:3, 3] = pose_trans
        
        