import sys
import numpy as np
import time
from world import World


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print 'usage: python chase.py prob.txt'
        exit(1)

    # Construct the world
    world = World(sys.argv[1])
    world.djikstra_intercept()
    #world.intercept_solve()


    











