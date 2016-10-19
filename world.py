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

    def in_bounds(self, point):
        (x, y) = point
        return 0 <= x < self.N and 0 <= y < self.N

    def manhattan_dist(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

    def djikstra_solve(self):
        Q = []
        dist = np.full((self.N, self.N), np.inf)
        dist[self.start] = 0

        for i in range(0, self.N):
            for j in range(0, self.N):
                if (i,j) == self.start:
                    hq.heappush(Q, (0, self.start))
                else:
                    hq.heappush(Q, (np.inf, (i,j)))

        prev = {}
        prev[self.start] = None
       
        count = 0 
        while len(Q) > 0:
            d, u = hq.heappop(Q)

            for v in filter(self.in_bounds, [(u[0]+1, u[1]), (u[0]-1, u[1]), 
                                             (u[0], u[1]+1), (u[0], u[1]-1)]):
                alt = d + self.costmap[v] 
                if alt < dist[v]:
                    hq.heappush(Q, (alt, v))
                    dist[v] = alt
                    prev[v] = u

        return dist, prev

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
        cost = self.costmap[current]
        total_path = [current]
        dwell_cost = np.inf
        dwell_loc  = None

        while True:
            if self.costmap[current] < dwell_cost:
                dwell_cost = self.costmap[current] 
                dwell_loc  = current
            current = prev[current]
            if current == None: break
            cost += self.costmap[current]
            total_path.append(current)
        total_path.reverse()

        ret_path = Path(total_path, cost, dwell_cost, dwell_loc)
        return ret_path 

    def astar_solve(self, start, end, weight=1, h=None):

        closed_set = set([])
        open_set = []
        hq.heappush(open_set, (0, start)) # Don't need to calculate heuristic bc any value will be the min.

        prev   = {} 
        prev[start] = None
        gscore = {} 
        gscore[start] = 0

        count = 0

        if h == None:
            h = np.empty((self.N, self.N), dtype=np.int)
            for i in range(0, self.N):
                for j in range(0, self.N):
                    h[i][j] = self.manhattan_dist((i,j), self.start)

        while open_set:
            # Pick the frontier node with smallest fscore.
            curr = hq.heappop(open_set)[-1]

            # If curr has been explored previously, skip it.
            if curr in closed_set:
                continue

            # If we found the goal, quit.
            if curr == end:
                return self.reconstruct_path(prev, curr), gscore

            # Add this node to the closed set.
            closed_set.add(curr)
            # Expand this node.
            for neighbor in filter(self.in_bounds, [(curr[0]+1, curr[1]  ), (curr[0]-1, curr[1]  ), (curr[0]  , curr[1]-1), (curr[0]  , curr[1]+1)]):

                # Skip if we've already expanded this neighbor
                if neighbor in closed_set: continue 
                # Find the cost of the cheapest path to neighbor via curr
                new_gscore = gscore[curr] + self.costmap[neighbor[0], neighbor[1]]
                # If neighbor is new or we found a cheaper path
                if neighbor not in gscore or new_gscore < gscore[neighbor]:
                    # fscore is g+h
                    fscore = new_gscore + weight*h[neighbor]
                    # Add neighbor to the frontier
                    hq.heappush(open_set, (fscore, neighbor))
                    # Keep track of its predecessor
                    prev[neighbor] = curr
                    # Update its gscore
                    gscore[neighbor] = new_gscore

        print 'FAILED!'
        return None

    def intercept_solve(self):
        R2D2_path_set = set([])
        R2D2_path = {} 
        for i in range(0, len(self.path)):
            if self.manhattan_dist(self.path[i], self.start) > i or \
                self.path[i] in R2D2_path_set:
                continue
            R2D2_path_set.add((i, self.path[i]))
            R2D2_path[i] = self.path[i]

        
        h_init = np.empty((self.N, self.N), dtype=np.int)
        for i in range(0, self.N):
            for j in range(0, self.N):
                h_init[i][j] = self.manhattan_dist((i,j), self.start)

        for l in R2D2_path:
            s = time.time()
            retval = self.astar_solve( R2D2_path[l], self.start, weight=1, h=h_init)
            e = time.time()
            print "ASTAR: " + str(R2D2_path[l]) + " -> " + \
                  str(self.start) + " in " + str(e-s) + " sec."
            path, cost, dwell_cost, dwell_loc = retval[0]
            gscore = retval[1]
            
            s = time.time()
            for g in gscore:
                h_init[g] = gscore[self.start] - gscore[g] 
            e = time.time()
            print "Update: ", (e-s)

