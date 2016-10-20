import numpy as np
import heapq as hq
import time


class Path:
    def __init__(self, path=None, cost=np.inf, dwell_cost=None, dwell_loc=None):
        self.path = path
        self.cost   = cost
        self.dwell_cost = dwell_cost
        self.dwell_loc = dwell_loc
    def length(self):
        return len(self.path)
    def end(self):
        return self.path[-1]
    def start(self):
        return self.path[0]

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
        return t >= 0 and 0 <= x < self.N and 0 <= y < self.N

    def manhattan_dist(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

    def djikstra_intercept(self):
        time_start = time.time()

        dist, prev = self.djikstra_solve()

        min_path = Path()
        min_intercept_time = np.inf
        
        for R2D2_time in range(0, len(self.path)):

            if self.manhattan_dist(self.path[R2D2_time], self.start) > R2D2_time+1:
                continue

            current_path = self.reconstruct_path(prev, self.path[R2D2_time])

            if( R2D2_time < current_path.length() ):
                continue

            current_path.cost += (1+R2D2_time-current_path.length())*current_path.dwell_cost

            if(current_path.cost < min_path.cost or (current_path.cost == min_path.cost and R2D2_time < min_intercept_time) ):
                min_path = current_path
                min_intercept_time = R2D2_time


        if(min_intercept_time > min_path.length()-1):
            #print "DWELL at " + str(min_path.dwell_loc) + \
            #      " for " + str(min_intercept_time-min_path.length()+1) + \
            #      " timesteps at a cost of " + str(min_path.dwell_cost) + " per timestep."
            self.insert_dwell(min_path, min_intercept_time-min_path.length()+1)


        print min_path.cost
        for l in min_path.path:
            print l
        #print "Time: ", time.time()-time_start

    def insert_dwell(self, path, dwell_amt):
        for loc in range(0, path.length()):
            location = path.path[loc]
            if(location == path.dwell_loc):
                for d in range(0, dwell_amt):
                    path.path.insert(loc, path.dwell_loc)
                break
        return path

    def reconstruct_path(self, prev, current):
        cost = 0 
        total_path = [current]

        while True:
            current = prev[current]
            total_path.append(current)
            if prev[current] == None: break
            cost += self.costmap[current[1], current[2]]

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
            curr = hq.heappop(open_set)[-1]

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

    def intercept_solve(self):
        start = time.time()
        prev, dist = self.djikstra_solve()
        path, cost = self.astar_solve(self.path, self.start, weight=1, h=dist)
        end = time.time()
        print cost
        for p in path:
            print p
        print end-start

    def get_successors(self, curr):
        return filter(self.txy_in_bounds,                                   \
                                      [(curr[0]-1, curr[1]+1, curr[2]  ),   \
                                       (curr[0]-1, curr[1]-1, curr[2]  ),   \
                                       (curr[0]-1, curr[1]  , curr[2]-1),   \
                                       (curr[0]-1, curr[1]  , curr[2]+1),   \
                                       (curr[0]-1, curr[1]  , curr[2]  )])


    def astar_solve(self, start, end, weight=1, h=None):
        start = list(start)

        open_set = []
        closed_set = set([])
        prev   = {} 
        gscore = {} 

        for s in range(0, len(start)):
            if(s >= self.manhattan_dist(start[s], end)):
                state = (s, start[s][0], start[s][1])
                hq.heappush(open_set, (0, state))
                prev[state] = None
                gscore[state] = 0

        while open_set:
            # Pick the frontier node with smallest fscore.
            curr = hq.heappop(open_set)[-1]

            # If curr has been explored previously, skip it.
            if curr in closed_set:
                continue

            # If we found the goal, quit.
            if curr == (0, end[0], end[1]):
                return self.reconstruct_path(prev, curr)

            # Add this node to the closed set.
            closed_set.add(curr)

            # Expand this node.
            for neighbor in self.get_successors(curr):
                (n_t, n_x, n_y) = neighbor
                # Skip if we've already expanded this neighbor
                if neighbor in closed_set: continue 
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

