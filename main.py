import sys
from world import World

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print 'usage: python chase.py prob.txt'
        exit(1)

    # Construct the world
    world = World(sys.argv[1], verbose=True)

    #(path, cost) = world.intercept()
    
        
    
    

    
      
