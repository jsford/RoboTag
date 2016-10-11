import os, sys
import numpy as np
import Tkinter
import Image, ImageTk
import matplotlib.pyplot as plt

class World:
    def __init__(self, fname):

        f = open(fname, 'r')

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
        self.weights = np.empty(shape=(self.N,self.N), dtype = np.int64)
        for x in range(0, self.N):
            line = f.readline().strip().split(',')
            line = [int(w) for w in line]
            self.weights[x] = np.array(line)

        
        img = np.log10(self.weights)
        max = np.max(img)
        min = np.min(img)
        img = 1 - (img-min) / float(max-min)
        img = plt.cm.hot(img, bytes=True)

        img[self.start[0]][self.start[1]] = [0,0,255,255]

        self.root = Tkinter.Tk()
        self.root.geometry('%dx%d+0+0' % (800, 800))
        self.old_label_image = None

        for coord in self.path:
            img[coord[0]][coord[1]] = [0,255,0,255]
            self.image = Image.fromarray(img).resize((800,800))
            self.update_display()
            break

    def update_display(self):
        tkpi = ImageTk.PhotoImage(self.image)
        label_image = Tkinter.Label(self.root, image=tkpi)
        label_image.place(x=0,y=0,width=800,height=800)
        self.root.title('Jordan')
        if self.old_label_image is not None:
            self.old_label_image.destroy()
        self.old_label_image = label_image
        self.root.after(300, self.root.quit)
        self.root.mainloop()



