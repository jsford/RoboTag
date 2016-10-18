import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import heapq as hq

class World:
    def __init__(self, fname, verbose=False):
        
        f = open(fname, 'r')
        self.verbose = verbose

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

        # Initialize the display.
        self._init_display()

    def _init_display(self):
        max = np.max(self.costmap)
        min = np.min(self.costmap)
        self.img = np.log10(self.costmap.reshape((self.N, self.N)))
        max = np.max(self.img)
        min = np.min(self.img)
        self.img -= min
        if max > min:
            self.img /= float(max-min)
        self.img = 1-self.img
        self.img = plt.cm.hot(self.img, bytes=True)

        self.img[self.start[0]][self.start[1]] = [0,0,255,255]
    
    def update_display(self):
        if(self.verbose):
            img = self.img
            img = cv.resize(self.img, (1000,1000), interpolation=cv.INTER_NEAREST)
            img = cv.cvtColor(img, cv.COLOR_BGRA2RGBA)
            cv.imshow("RoboChase", img)        
            key = cv.waitKey(1)
        
            if key==27 or cv.getWindowProperty('RoboChase', 0) < -1:
                cv.destroyAllWindows()
                exit(0)

    def in_bounds(self, point):
        (x, y) = point
        return 0 <= x < self.N and 0 <= y < self.N

    def manhattan_dist(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

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
        return total_path, cost, dwell_cost, dwell_loc

    def astar_solve(self, start, end, weight=1):

        closed_set = set([])
        open_set = []
        hq.heappush(open_set, (0, start)) # Don't need to calculate heuristic bc any value will be the min.

        prev   = {} 
        prev[start] = None
        gscore = {} 
        gscore[start] = 0

        count = 0

        h = np.empty((self.N, self.N), dtype=np.int)
        for i in range(0, self.N):
            for j in range(0, self.N):
                h[i][j] = manhattan_dist((i,j), end)

        while open_set:
            # Pick the frontier node with smallest fscore.
            curr = hq.heappop(open_set)[-1]

            # If curr has been explored previously, skip it.
            if curr in closed_set:
                continue

            # Bullshit for debug
            if self.verbose and (count % 1000 == 0 or curr == end):
                for o in open_set:
                    self.img[o[-1]] = [0, 0, 255, 255]
                for c in closed_set:
                    self.img[c] = [0, 255, 0, 255]
                self.update_display()
            count += 1

            # If we found the goal, quit.
            if curr == end:
                return reconstruct_path(prev, curr, self.costmap)

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

    def insert_dwell(self, path, dwell_loc, dwell_amt):
        for loc in range(0, len(path)):
            location = path[loc]
            if(location == dwell_loc):
                for d in range(0, dwell_amt):
                    min_intercept_path.insert(loc, dwell_loc)
        return path
