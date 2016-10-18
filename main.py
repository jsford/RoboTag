import sys
import numpy as np
import time
from world import World

DISPLAY_SEARCH = False

def insert_dwell(path, dwell_loc, dwell_amt):
    for loc in range(0, len(path)):
        location = path[loc]
        if(location == dwell_loc):
            for d in range(0, dwell_amt):
                min_intercept_path.insert(loc, dwell_loc)
    return path
    

if __name__ == "__main__":

    start = time.time()
    if len(sys.argv) != 2:
        print 'usage: python chase.py prob.txt'
        exit(1)

    # Construct the world
    world = World(sys.argv[1], verbose=DISPLAY_SEARCH)

    unique_locations = set([])
    for i in range(0, len(world.path)):
        if world.manhattan_dist(world.path[i], world.start) > i \
           or world.path[i] in unique_locations: 
            continue
        unique_locations.add((i, world.path[i]))

    dist, prev = world.djikstra_solve()

    min_intercept_path = None
    min_intercept_cost = np.inf
    min_intercept_dwell_cost = None
    min_intercept_dwell_loc = None
    min_intercept_time = np.inf
    
    unique_locations = list(unique_locations)

    for R2D2_idx in range(0, len(unique_locations)):
        (R2D2_time, R2D2_loc) = unique_locations[R2D2_idx]

        path, cost, dwell_cost, dwell_loc = world.reconstruct_path(prev, R2D2_loc)
        if( R2D2_time < len(path) ): continue

        cost += (R2D2_time-len(path))*dwell_cost

        if(cost < min_intercept_cost or (cost == min_intercept_cost and R2D2_time < min_intercept_time) ):
            min_intercept_cost = cost
            min_intercept_time = R2D2_time
            min_intercept_dwell_cost = dwell_cost
            min_intercept_dwell_loc = dwell_loc
            min_intercept_path = path


    if(min_intercept_time != len(min_intercept_path)):
        #print "DWELL at " + str(min_intercept_dwell_loc) + " for " + str(min_intercept_time-len(min_intercept_path)) + " timesteps at a cost of " + str(min_intercept_dwell_cost) + " per timestep."
        world.insert_dwell(path, min_intercept_dwell_loc, min_intercept_time-len(min_intercept_path))


    print min_intercept_cost
    for l in min_intercept_path:
        print l
    print "Time: ", time.time()-start











