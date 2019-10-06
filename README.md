# PyICP SLAM
Full-python LiDAR SLAM.

### Purpose
- Full-python LiDAR SLAM
    - Easy to exchange or connect with any Python-based components (e.g., DL front-ends such as [Deep Odometry](https://www.youtube.com/watch?v=Y2s08dv-Mq0))
        - Here, ICP, which is a very basic option for LiDAR, and [Scan Context (IROS 18)](https://github.com/irapkaist/scancontext) are used for odometry and loop detection, respectively. 
- Hands-on LiDAR SLAM 
    - Easy to understand (could be used for educational purpose)
- The practical use case of [miniSAM](https://github.com/dongjing3309/minisam)
    - The miniSAM is easy to use at Python

### What is SLAM?
- In this repository, SLAM (Simultaneous localization and mapping) is considered as 
    - SLAM = Odometry + Loop closure

### Overview of the PyICP SLAM
- The pipeline of the PyICP SLAM is composed of three parts
    1. Odometry: [Point-to-point ICP (iterative closest point)](https://github.com/ClayFlannigan/icp/blob/master/icp.py)
    2.  Loop detection: [Scan Context (IROS 18)](https://github.com/irapkaist/scancontext)
    3. Back-end (graph optimizer): [miniSAM](https://github.com/dongjing3309/minisam)


### Features 
- Thanks to the Scan Context, reverse loops can be successfully closed.
    - E.g., see KITTI 08, KITTI 14 at Results section below. 

- Time costs 
    - (No accelerated, naive here) ICP gets 7-10 Hz when sampled (with 8000 points)
    - (No accelerated, naive here, too) Scan Context gets 1-2 Hz (when 10 ringkey candidates).
    - miniSAM is enough fast. 


### How to use 
Just run 

```sh
$ python3 main_icp_slam.py
```

The details of parameters are eaily found in the argparser in that .py file.


### Results (KITTI dataset) 
Those results are produced under the same parameter conditions:
- ICP used random downsampling, 7000 points.
- Scan Context's parameters:
    - Ring: 20, Sector: 60
    - The number of ringkey candidates: 30
    - Loop threshold: 0.11

gifs (TBA)

Some of the results are good, and some of them are not enough. 
Those results are for the study to understand when is the algorithm works or not. 



### Findings (lessons)
- TBA


### Author 
```sh
  Giseop Kim (paulgkim@kaist.ac.kr)
```




