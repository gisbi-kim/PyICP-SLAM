#! /usr/bin/env python3
import kiss_icp.tools.cmd
import numpy as np

np.set_printoptions(precision=4)
import open3d as o3d
import argparse
from utils.ScanContextManager import *
from utils.PoseGraphManager import *
from utils.UtilsMisc import *
import copy
import utils.kiss_icp
from pathlib import Path


def run():
    # params
    parser = argparse.ArgumentParser(description='PyICP SLAM arguments')

    parser.add_argument('--data', type=Path, default='...', help='The data directory used by the specified dataloader')
    parser.add_argument('--config', type=Path, default='./config/config_kiss_icp.yaml', help='Path to the '
                                                                                             'configuration file')
    parser.add_argument('--visualize', action='store_true', help='Open an online visualization')
    parser.add_argument('--n_scans', type=int, default=-1, help='Number of scans to process')

    parser.add_argument('--num_icp_points', type=int, default=5000)  # 5000 is enough for real time
    parser.add_argument('--num_rings', type=int, default=20)  # same as the original paper
    parser.add_argument('--num_sectors', type=int, default=60)  # same as the original paper
    parser.add_argument('--num_candidates', type=int, default=30)  # must be int
    parser.add_argument('--try_gap_loop_detection', type=int, default=10)  # same as the original paper
    parser.add_argument('--loop_threshold', type=float,
                        default=0.11)  # 0.11 is usually safe (for avoiding false loop closure)
    parser.add_argument('--save_gap', type=int, default=300)

    args = parser.parse_args()

    # dataset
    dataloader, data = kiss_icp.tools.cmd.guess_dataloader(args.data, default_dataloader="generic")
    dataset = kiss_icp.datasets.dataset_factory(dataloader=dataloader, data_dir=data)
    config = kiss_icp.config.load_config(args.config, deskew=False, max_range=None)

    # KISS-ICP initialization
    KICP = kiss_icp.kiss_icp.KissICP(config=config)

    first = 0
    last = len(dataset) if args.n_scans == -1 else min(len(dataset), args.n_scans)
    times = []

    compensator = kiss_icp.deskew.get_motion_compensator(config)
    preprocessor = kiss_icp.preprocess.get_preprocessor(config)

    visualizer = kiss_icp.tools.visualizer.RegistrationVisualizer() if args.visualize else kiss_icp.tools.visualizer.StubVisualizer()
    visualizer.global_view = True

    # Pose Graph Manager (for back-end optimization) initialization
    PGM = PoseGraphManager()
    PGM.addPriorFactor()

    # Scan Context Manager (for loop detection) initialization
    SCM = ScanContextManager(shape=[args.num_rings, args.num_sectors],
                             num_candidates=args.num_candidates,
                             threshold=args.loop_threshold)

    absolute_pose = np.eye(4, 4)

    for idx in get_progress_bar(first, last):
        try:
            raw_frame, time_stamp = dataset[idx]
        except ValueError:
            frame = dataset[idx]
            timestamps = np.zeros(frame.shape[0])

        start_time = time.perf_counter_ns()

        # LiDAR point pre-processing & Downsampling
        frame = compensator.deskew_scan(frame, KICP.poses, timestamps)
        frame = preprocessor(frame)

        ds_frame = kiss_icp.voxelization.voxel_down_sample(frame, config.mapping.voxel_size * 0.5)
        source = kiss_icp.voxelization.voxel_down_sample(ds_frame, config.mapping.voxel_size * 1.5)

        # Motion model & Adaptive threshold
        sigma = KICP.get_adaptive_threshold()
        if len(KICP.poses) < 2:
            prediction = np.eye(4)
        else:
            prediction = invert_SE3(KICP.poses[-2]) @ KICP.poses[-1]
        last_pose = KICP.poses[-1] if KICP.poses else np.eye(4)
        initial_guess = last_pose @ prediction

        # Upload point cloud data to pose graph optimization and scan context
        PGM.curr_node_idx = idx
        SCM.addNode(node_idx=PGM.curr_node_idx, ptcloud=ds_frame)
        if PGM.curr_node_idx == 0:
            PGM.prev_node_idx = PGM.curr_node_idx
            prev_scan_pts = copy.deepcopy(frame)
            icp_initial = np.eye(4)
            continue

        # ICP registration
        new_pose = kiss_icp.registration.register_frame(
            points=source,
            voxel_map=KICP.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        # Get relative pose
        relative_pose = np.matmul(np.linalg.inv(last_pose), new_pose)

        # Update the adaptive threshold
        KICP.adaptive_threshold.update_model_deviation(np.linalg.inv(initial_guess) @ new_pose)
        KICP.local_map.update(ds_frame, new_pose)
        KICP.poses.append(new_pose)

        # Pose graph optimization
        # update the current (moved) pose
        PGM.curr_se3 = np.matmul(PGM.curr_se3, relative_pose)

        # add the odometry factor to the graph
        PGM.addOdometryFactor(relative_pose)

        # renewal the prev information
        PGM.prev_node_idx = PGM.curr_node_idx
        prev_scan_pts = copy.deepcopy(frame)

        # loop detection and optimize the graph
        if PGM.curr_node_idx > 1 and PGM.curr_node_idx % args.try_gap_loop_detection == 0:
            # 1/ loop detection
            loop_idx, loop_dist, yaw_diff_deg = SCM.detectLoop()
            if loop_idx == None:  # NOT FOUND
                pass
            else:
                print("Loop event detected: ", PGM.curr_node_idx, loop_idx, loop_dist)
                # 2-1/ add the loop factor
                loop_scan_down_pts = SCM.getPtcloud(loop_idx)

                loop_source = o3d.geometry.PointCloud()
                loop_source.points = o3d.utility.Vector3dVector(ds_frame)

                loop_target = o3d.geometry.PointCloud()
                loop_target.points = o3d.utility.Vector3dVector(loop_scan_down_pts)

                loop_transform = o3d.pipelines.registration.registration_icp(
                    source=loop_source,
                    target=loop_target,
                    max_correspondence_distance=10,
                    init=yawdeg2se3(yaw_diff_deg),
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
                )
                if loop_transform.transformation is None:
                    pass

                PGM.addLoopFactor(loop_transform.transformation, loop_idx)

                # 2-2/ graph optimization
                PGM.optimizePoseGraph()

                # 2-3/ update poses
                apply_optimized_poses(PGM.graph_optimized, PGM.curr_node_idx, KICP.poses)

        # Processing time counter + visualization
        times.append(time.perf_counter_ns() - start_time)
        visualizer.update(frame, source, KICP.local_map, KICP.poses[-1])


if __name__ == "__main__":
    run()
