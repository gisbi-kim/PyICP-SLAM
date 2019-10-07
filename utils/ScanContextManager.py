import numpy as np
np.set_printoptions(precision=4)

import time
from scipy import spatial

def xy2theta(x, y):
    if (x >= 0 and y >= 0): 
        theta = 180/np.pi * np.arctan(y/x);
    if (x < 0 and y >= 0): 
        theta = 180 - ((180/np.pi) * np.arctan(y/(-x)));
    if (x < 0 and y < 0): 
        theta = 180 + ((180/np.pi) * np.arctan(y/x));
    if ( x >= 0 and y < 0):
        theta = 360 - ((180/np.pi) * np.arctan((-y)/x));

    return theta


def pt2rs(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    # z = point[2]
    
    if(x == 0.0):
        x = 0.001
    if(y == 0.0):
        y = 0.001
    
    theta = xy2theta(x, y)
    faraway = np.sqrt(x*x + y*y)
    
    idx_ring = np.divmod(faraway, gap_ring)[0]       
    idx_sector = np.divmod(theta, gap_sector)[0]

    if(idx_ring >= num_ring):
        idx_ring = num_ring-1 # python starts with 0 and ends with N-1
    
    return int(idx_ring), int(idx_sector)


def ptcloud2sc(ptcloud, sc_shape, max_length):
    num_ring = sc_shape[0]
    num_sector = sc_shape[1]

    gap_ring = max_length/num_ring
    gap_sector = 360/num_sector
    
    enough_large = 500
    sc_storage = np.zeros([enough_large, num_ring, num_sector])
    sc_counter = np.zeros([num_ring, num_sector])
    
    num_points = ptcloud.shape[0]
    for pt_idx in range(num_points):
        point = ptcloud[pt_idx, :]
        point_height = point[2] + 2.0 # for setting ground is roughly zero 
        
        idx_ring, idx_sector = pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
        
        if sc_counter[idx_ring, idx_sector] >= enough_large:
            continue
        sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
        sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1

    sc = np.amax(sc_storage, axis=0)
        
    return sc


def sc2rk(sc):
    return np.mean(sc, axis=1)

def distance_sc(sc1, sc2):
    num_sectors = sc1.shape[1]

    # repeate to move 1 columns
    _one_step = 1 # const
    sim_for_each_cols = np.zeros(num_sectors)
    for i in range(num_sectors):
        # Shift
        sc1 = np.roll(sc1, _one_step, axis=1) #  columne shift

        #compare
        sum_of_cossim = 0
        num_col_engaged = 0
        for j in range(num_sectors):
            col_j_1 = sc1[:, j]
            col_j_2 = sc2[:, j]
            if (~np.any(col_j_1) or ~np.any(col_j_2)): 
                # to avoid being divided by zero when calculating cosine similarity
                # - but this part is quite slow in python, you can omit it.
                continue 

            cossim = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
            sum_of_cossim = sum_of_cossim + cossim

            num_col_engaged = num_col_engaged + 1

        # save 
        sim_for_each_cols[i] = sum_of_cossim / num_col_engaged

    yaw_diff = np.argmax(sim_for_each_cols) + 1 # because python starts with 0 
    sim = np.max(sim_for_each_cols)
    dist = 1 - sim

    return dist, yaw_diff

    
class ScanContextManager:
    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): # defualt configs are same as the original paper 
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold

        self.max_length = 80 # recommended but other (e.g., 100m) is also ok.

        self.ENOUGH_LARGE = 15000 # capable of up to ENOUGH_LARGE number of nodes 
        self.ptclouds = [None] * self.ENOUGH_LARGE
        self.scancontexts = [None] * self.ENOUGH_LARGE
        self.ringkeys = [None] * self.ENOUGH_LARGE

        self.curr_node_idx = 0
       

    def addNode(self, node_idx, ptcloud):
        sc = ptcloud2sc(ptcloud, self.shape, self.max_length)
        rk = sc2rk(sc)

        self.curr_node_idx = node_idx
        self.ptclouds[node_idx] = ptcloud
        self.scancontexts[node_idx] = sc
        self.ringkeys[node_idx] = rk
        

    def getPtcloud(self, node_idx):
        return self.ptclouds[node_idx]


    def detectLoop(self):        
        exclude_recent_nodes = 30
        valid_recent_node_idx = self.curr_node_idx - exclude_recent_nodes

        if(valid_recent_node_idx < 1):
            return None, None, None
        else:
            # step 1
            ringkey_history = np.array(self.ringkeys[:valid_recent_node_idx])
            ringkey_tree = spatial.KDTree(ringkey_history)

            ringkey_query = self.ringkeys[self.curr_node_idx]
            _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

            # step 2
            query_sc = self.scancontexts[self.curr_node_idx]
            
            nn_dist = 1.0 # initialize with the largest value of distance
            nn_idx = None
            nn_yawdiff = None
            for ith in range(self.num_candidates):
                candidate_idx = nncandidates_idx[ith]
                candidate_sc = self.scancontexts[candidate_idx]
                dist, yaw_diff = distance_sc(candidate_sc, query_sc)
                if(dist < nn_dist):
                    nn_dist = dist
                    nn_yawdiff = yaw_diff
                    nn_idx = candidate_idx

            if(nn_dist < self.threshold):
                nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
                return nn_idx, nn_dist, nn_yawdiff_deg # loop detected!
            else:
                return None, None, None
