import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv

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

    


