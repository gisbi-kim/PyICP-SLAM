import os 
import random
import numpy as np

def random_sampling(orig_points, num_points):
    assert orig_points.shape[0] > num_points

    points_down_idx = random.sample(range(orig_points.shape[0]), num_points)
    down_points = orig_points[points_down_idx, :]

    return down_points

def readScan(bin_path, dataset='KITTI'):
    if(dataset == 'KITTI'):
        return readKittiScan(bin_path)


def readKittiScan(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    ptcloud_xyz = scan[:, :-1]
    return ptcloud_xyz
    

class KittiScanDirManager:
    def __init__(self, scan_dir):
        self.scan_dir = scan_dir
        
        self.scan_names = os.listdir(scan_dir)
        self.scan_names.sort()    
        
        self.scan_fullpaths = [os.path.join(self.scan_dir, name) for name in self.scan_names]
  
        self.num_scans = len(self.scan_names)

    def __repr__(self):
        return ' ' + str(self.num_scans) + ' scans in the sequence (' + self.scan_dir + '/)'

    def getScanNames(self):
        return self.scan_names
    def getScanFullPaths(self):
        return self.scan_fullpaths
    def printScanFullPaths(self):
        return print("\n".join(self.scan_fullpaths))

