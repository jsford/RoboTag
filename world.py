import numpy as np
import numpy.ma as ma
import matplotlib.pyplot as plt
import cv2 as cv
import time
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
        self.start = [int(c) for c in self.start]

        # Get T
        f.readline().strip()
        self.path = []
        while(1):
            next_pos = f.readline().strip()
            if next_pos == 'B': break
            next_pos = next_pos.split(',')
            next_pos = [int(c) for c in next_pos]
            self.path.append(next_pos)


        # Get B
        self.weights = np.empty(shape=(self.N, self.N), dtype = np.int64)
        for x in range(0, self.N):
            line = f.readline().strip().split(',')
            line = [int(w) for w in line]
            self.weights[x,:] = np.array(line)

        for i in range(0, self.N):
            for j in range(0, self.N):
                if self.weights[i,j] < 200:
                    self.weights[i,j] = 1
            

        self._init_display()

        # SET_END
        astar_path = self.astar_solve(tuple(self.start), tuple(self.path[0]))
        #astar_path = self.astar_solve((500,100), (400,300))
        print "Path Length: ", len(astar_path)
        for l in astar_path:
            self.img[l] = [255, 0, 255, 255]

        while(1):
            self._update_display()

    def _init_display(self):
        max = np.max(self.weights)
        min = np.min(self.weights)
        self.img = np.log10(self.weights.reshape((self.N, self.N)))
        max = np.max(self.img)
        min = np.min(self.img)
        self.img -= min
        if max > min:
            self.img /= float(max-min)
        self.img = 1-self.img
        self.img = plt.cm.hot(self.img, bytes=True)

        self.img[self.start[0]][self.start[1]] = [0,0,255,255]
    
    def _update_display(self):
        if(self.verbose):
            img = self.img
            img = cv.resize(self.img, (1000,1000), interpolation=cv.INTER_NEAREST)
            img = cv.cvtColor(img, cv.COLOR_BGRA2RGBA)
            cv.imshow("RoboChase", img)        
            key = cv.waitKey(1)
        
            if key==27 or cv.getWindowProperty('RoboChase', 0) < -1:
                cv.destroyAllWindows()
                exit(0)

    def djikstra_solve(self):
        Q = set(range(0, self.N**2))
        dist = np.full(self.N**2, np.inf)
        mask = np.full((self.N**2), 0)
        dist = ma.masked_array(dist, mask)
        prev = np.full(self.N**2, -1)
        prev[0] = 0
        
        dist[self.start[0]*self.N + self.start[1]] = 0 
       
        count = 0 
        while( len(Q) > 0 ):
            u = np.argmin(dist)

            self.img[int(u/self.N), u%self.N] = [0,255,0,255]
            
            if( count == 1000 ): 
                self._update_display()
                count = 0
            count += 1
            
            for v in [u-1, u+1, u-self.N, u+self.N]:
                if( not (v in Q) or v < 0 or v >= self.N**2
                    or (v==u-1 and u%self.N==0) 
                    or (v==u+1 and u%self.N==self.N-1) ):
                    continue
                alt = dist[u] + self.weights[v] 
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u

            Q.remove(u)
            dist.mask[u] = 1

        print dist.reshape((self.N, self.N))
        print prev.reshape((self.N, self.N))
        print 

    def manhattan_dist(self, start, end):
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

    def euclidean_dist(self, start, end):
        return np.sqrt((start[0]-end[0])**2 + (start[1]-end[1])**2)

    def reconstruct_path(self, prev, current):
        current = current
        total_path = [current]
        while True:
            current = prev[current]
            if current == None: break
            total_path.append(current)
            print current
        return total_path

    def get_successors(self, node):
        neighbors = []
        for neighbor in [(node[0]+1, node[1]  ), (node[0]-1, node[1]  ), 
                         (node[0]  , node[1]-1), (node[0]  , node[1]+1)]:

            if(neighbor[0] >= self.N or neighbor[1] >= self.N or 
                neighbor[0] < 0 or neighbor[1] < 0):
                continue
            neighbors.append(neighbor)
        return neighbors

    def astar_solve(self, start, end):

        closed_set = set([])
        open_set = []
        hq.heappush(open_set, (0, start)) # Don't need to calculate heuristic bc any value will be the min.

        prev   = {} 
        prev[start] = None
        gscore = {} 
        gscore[start] = 0
        
        count = 0
        while open_set:
            # Pick the frontier node with smallest fscore.
            curr = hq.heappop(open_set)[-1]

            # If curr has been explored previously, skip it.
            if curr in closed_set:
                continue

            # Bullshit for debug
            if count % 1000 == 0 or curr == end:
                for o in open_set:
                    self.img[o[-1]] = [0, 0, 255, 255]
                for c in closed_set:
                    self.img[c] = [0, 255, 0, 255]
                self._update_display()
            count += 1

            # If we found the goal, quit.
            if curr == end:
                print "DONE! " + str(count) + " states"
                return self.reconstruct_path(prev, curr)

            # Add this node to the closed set.
            closed_set.add(curr)
            # Expand this node.
            for neighbor in self.get_successors(curr):
                if neighbor in closed_set: continue 
                # Find the cost of the cheapest path to neighbor via curr
                new_gscore = gscore[curr] + self.weights[neighbor[0], neighbor[1]]
                # If neighbor is new or we found a cheaper path
                if neighbor not in open_set or new_gscore < gscore[neighbor]:
                    # fscore is g+h
                    fscore = new_gscore + self.manhattan_dist(neighbor, end)
                    # Add neighbor to the frontier
                    hq.heappush(open_set, (fscore, neighbor))
                    # Keep track of its predecessor
                    prev[neighbor] = curr
                    # Update its gscore
                    gscore[neighbor] = new_gscore

        print 'FAILED!'
        return
