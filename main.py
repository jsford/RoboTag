import sys
from astar import *
from djikstra import *
from world import World
import time

DISPLAY_SEARCH = False


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print 'usage: python chase.py prob.txt'
        exit(1)

    # Construct the world
    world = World(sys.argv[1], verbose=DISPLAY_SEARCH)

    for i in range(0, len(world.path), 250):
        start = time.time()
        astar_path, cost = astar_solve(world, tuple(world.path[i]), tuple(world.start), weight=1, verbose=DISPLAY_SEARCH)
        end = time.time()
        break
    
    print "ELAPSED TIME: ", (end-start)
    print "Unweighted Path Cost: ", cost
    for l in astar_path:
        world.img[l] = [255, 0, 255, 255]

    for loc in astar_path:
        print loc
       
    if(DISPLAY_SEARCH):
        while(1):
            world.update_display()
    
    

    
      
