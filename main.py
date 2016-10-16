import sys
from astar import *
from djikstra import *
from world import World

DISPLAY_SEARCH = True


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print 'usage: python chase.py prob.txt'
        exit(1)

    # Construct the world
    world = World(sys.argv[1], verbose=DISPLAY_SEARCH)

    for i in range(0, len(world.path), 250):
        astar_path, cost = astar_solve(world, tuple(world.start), tuple(world.path[i]), weight=100, verbose=DISPLAY_SEARCH)
    
        print "Path Cost: ", cost
        for loc in astar_path:
            print loc
        for l in astar_path:
            world.img[l] = [255, 0, 255, 255]

    while(1):
        world.update_display()
    
    

    
      
