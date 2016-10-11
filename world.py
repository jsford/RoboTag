import numpy as np
import numpy.ma as ma
import matplotlib.pyplot as plt
import cv2 as cv
import time
from priority_queue import *

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
        self.weights = np.empty(shape=(self.N**2), dtype = np.int64)
        for x in range(0, self.N):
            line = f.readline().strip().split(',')
            line = [int(w) for w in line]
            self.weights[x*self.N:(x+1)*self.N] = np.array(line)

        self._init_display()

        #self.djikstra_solve()
        # SET_END
        for goal in range(0, len(self.path), 100):
            astar_path = self.astar_solve(self.start, self.path[goal])
            for l in astar_path:
                self.img[l] = [255, 255, 255, 255]

        while(1):
            self._update_display()

    def _init_display(self):
        max = np.max(self.weights)
        min = np.min(self.weights)
        self.img = np.log10(self.weights.reshape((self.N, self.N)))
        max = np.max(self.img)
        min = np.min(self.img)
        self.img = 1 - (self.img-min) / float(max-min)
        self.img = plt.cm.hot(self.img, bytes=True)

        self.img[self.start[0]][self.start[1]] = [0,0,255,255]
    
    def _update_display(self):
        if(self.verbose):
            img = self.img
            #img = cv.resize(self.img, (800,800), interpolation=cv.INTER_NEAREST)
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

    def reconstruct_path(self, prev, current):
        current = tuple(current)
        total_path = [current]
        print current
        while current != (-1, -1):
            current = tuple(prev[current])
            total_path.append(current)
        return total_path

    def astar_solve(self, start, end):
        start = tuple(start)
        end   = tuple(end)

        print "ASTAR: "+str(start)+" -> "+str(end)

        closed_set = [] 
        open_set = PriorityQueue()
        open_set.push(self.manhattan_dist(start, end), start)

        prev   = np.full((self.N, self.N, 2), (-1,-1))
        gscore = np.full((self.N, self.N), np.inf)
        gscore[start[0],start[1]] = 0
        
        count = 0
        while open_set.notEmpty():
            if count >= 100:
                for o in open_set.array:
                    self.img[o.value] = [0, 0, 255, 255]
                for c in closed_set:
                    self.img[c] = [0, 255, 0, 255]
                self._update_display()
                count = 0
            count += 1

            curr = open_set.pop() 
            if curr == end:
                print "DONE!"
                return self.reconstruct_path(prev, curr)

            closed_set.append(curr)
            for neighbor in [(curr[0]+1, curr[1]  ), (curr[0]-1, curr[1]  ),
                             (curr[0]  , curr[1]-1), (curr[0]  , curr[1]+1)]:
                if(neighbor[0] >= self.N or neighbor[1] >= self.N or 
                    neighbor[0] < 0 or neighbor[1] < 0):
                    continue
                   
                if neighbor in closed_set:
                    continue

                tentative_gscore = gscore[curr] + self.weights[neighbor[0]*self.N + neighbor[1]]
                if tentative_gscore >= gscore[neighbor]:
                    continue
                elif neighbor not in open_set:
                    open_set.push(tentative_gscore + 500*self.manhattan_dist(neighbor, end), neighbor)

                prev[neighbor]   = list(curr)
                gscore[neighbor] = tentative_gscore

        print 'FAILED!'
        return
