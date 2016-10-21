import sys
import numpy as np
import heapq as hq
import time


class World:
    def __init__(self, fname):
        
        f = open(fname, 'r')

        # Get N
        f.readline().strip()
        self.N = int(f.readline().strip(','))

        # Get R
        f.readline().strip()
        self.start = f.readline().strip().split(',')
        self.start = tuple([int(c) for c in self.start])

        # Get T
        f.readline().strip()
        self.path = []
        while(1):
            next_pos = f.readline().strip()
            if next_pos == 'B': break
            next_pos = next_pos.split(',')
            next_pos = tuple([int(c) for c in next_pos])
            self.path.append(next_pos)

        # Get B
        self.costmap = np.empty(shape=(self.N, self.N), dtype = np.int64)
        for x in range(0, self.N):
            line = f.readline().strip().split(',')
            line = [int(w) for w in line]
            self.costmap[x,:] = np.array(line)

    def xy_in_bounds(self, point):
        (x, y) = point
        return 0 <= x < self.N and 0 <= y < self.N

    def txy_in_bounds(self, point):
        (t, x, y) = point
        return t >= (x+y) and 0 <= x < self.N and 0 <= y < self.N

    def manhattan_dist(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

    def reconstruct_path(self, prev, current):
        cost = 0 #self.costmap[current[1], current[2]] 
        total_path = [current]

        while True:
            current = prev[current]
            total_path.append(current)
            cost += self.costmap[current[1], current[2]]
            if prev[current] == None: break

        return total_path, cost 

    def djikstra_solve(self):
        closed_set = set([])
        open_set = []
        hq.heappush(open_set, (0, self.start)) 

        prev   = {} 
        prev[self.start] = None
        gscore = {} 
        gscore[self.start] = 0

        while open_set:
            # Pick the frontier node with smallest fscore.
            curr_time, curr = hq.heappop(open_set)

            # If curr has been explored previously, skip it.
            if curr in closed_set:
                continue

            # Add this node to the closed set.
            closed_set.add(curr)
            # Expand this node.
            for neighbor in filter(self.xy_in_bounds, [(curr[0]+1, curr[1]  ), (curr[0]-1, curr[1]  ), (curr[0]  , curr[1]-1), (curr[0]  , curr[1]+1)]):

                # Skip if we've already expanded this neighbor
                if neighbor in closed_set: continue 
                # Find the cost of the cheapest path to neighbor via curr
                new_gscore = gscore[curr] + self.costmap[neighbor[0], neighbor[1]]
                # If neighbor is new or we found a cheaper path
                if neighbor not in gscore or new_gscore < gscore[neighbor]:
                    # Add neighbor to the frontier
                    hq.heappush(open_set, (new_gscore, neighbor))
                    # Keep track of its predecessor
                    prev[neighbor] = curr
                    # Update its gscore
                    gscore[neighbor] = new_gscore

        return prev, gscore

    def intercept_solve(self, w=1):
        start = time.time()
        prev, dists = self.djikstra_solve()
        
        djikstra_heuristic = {}
        for d in dists:
            djikstra_heuristic[d] = dists[d] - self.costmap[d]

        path, cost = self.astar_solve(self.path, self.start, weight=w, djikstra_dists=dists, h=djikstra_heuristic)

        end = time.time()
        print cost
        #swizzle_path = []
        for p in path:
            print p
            (t, x, y) = p
        #    swizzle_path.append( (x, y, t) )
        #print swizzle_path
        print end-start

    def astar_solve(self, start, end, weight=1, h=None, djikstra_dists=None):
        start = list(start)

        open_set = []
        closed_set = set([])
        prev   = {} 
        gscore = {} 

        for s in range(0, len(start)):
            if(s >= self.manhattan_dist(start[s], end)):
                state = (s, start[s][0], start[s][1])
                hq.heappush(open_set, (djikstra_dists[start[s]], state))
                prev[state] = None
                gscore[state] = 0

        expanded_states = 0
        while open_set:
            # Pick the frontier node with smallest fscore.
            curr_time, curr = hq.heappop(open_set)

            # If curr has been explored previously, skip it.
            #if curr in closed_set:
            #    continue


            # If we found the goal, quit.
            if curr == (0, end[0], end[1]):
                print "EXPANDED STATES: ", expanded_states
                return self.reconstruct_path(prev, curr)


            # Add this node to the closed set.
            #closed_set.add(curr)

            # Expand this node.
            expanded_states += 1
            (t, x, y) = curr
            for neighbor in filter(self.txy_in_bounds, [(t-1, x  , y  ),   \
                                                        (t-1, x+1, y  ),   \
                                                        (t-1, x-1, y  ),   \
                                                        (t-1, x  , y-1),   \
                                                        (t-1, x  , y+1)]):
                # Skip if we've already expanded this neighbor
                if neighbor in closed_set: continue

                (n_t, n_x, n_y) = neighbor

                # Find the cost of the cheapest path to neighbor via curr
                new_gscore = gscore[curr] + self.costmap[n_x, n_y]
                # If neighbor is new or we found a cheaper path
                if neighbor not in gscore or new_gscore < gscore[neighbor]:
                    # fscore is g+h
                    fscore = new_gscore + weight*h[n_x, n_y]
                    # Add neighbor to the frontier
                    hq.heappush(open_set, (fscore, neighbor))
                    # Keep track of its predecessor
                    prev[neighbor] = curr
                    # Update its gscore
                    gscore[neighbor] = new_gscore

        print 'FAILED!'
        return None


def usage():
    print "To run the optimal planner, use:"
    print "python world.py -a prob1.txt"
    print "To run the fast but suboptimal planner, use:"
    print "python world.py -b prob1.txt"


if __name__ == '__main__':
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        exit(1)

    if len(sys.argv) == 2:
        world = World(sys.argv[1])
        world.intercept_solve()

    elif len(sys.argv) == 3:
        world = World(sys.argv[2])

        if (sys.argv[1] == "-a"):
            world.intercept_solve()

        elif (sys.argv[1] == "-b"):
            world.intercept_solve(100)
         
    
