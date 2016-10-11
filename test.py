from priority_queue import *

a = PQ_Entry(0, (0,1))
b = PQ_Entry(2, (2,2))
c = PQ_Entry(1, (1,0))

pq = PriorityQueue()
pq.push(a)
pq.push(b)
pq.push(c)

while pq.notEmpty():
    print pq.pop()
