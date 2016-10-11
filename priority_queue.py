import heapq as hq

class PQ_Entry:
    def __init__(self, priority, value):
        self.priority = priority
        self.value = value
    def __cmp__(self, entry):
        return self.priority > entry.priority
    def __eq__(self, entry):
        return self.value == entry.value
    def __repr__(self):
        return "(" + str(self.priority) + " , " + str(self.value) + ")"

class PriorityQueue:
    def __init__(self):
        self.array = []
    def __contains__(self, val):
        return PQ_Entry(0, val) in self.array
    def __repr__(self):
        return_str = ''
        for e in self.array:
            return_str += str(e) + '\n'
        return return_str

    def push(self, priority, value):
        hq.heappush(self.array, PQ_Entry(priority, value))

    def pop(self):
        entry = hq.heappop(self.array)
        return entry.value

    def isEmpty(self):
        return len(self.array) <= 0
    def notEmpty(self):
        return len(self.array) > 0
    




if __name__ == "__main__":
    a = PQ_Entry(0, (0,1))
    b = PQ_Entry(2, (2,2))
    c = PQ_Entry(1, (1,0))

    pq = PriorityQueue()
    pq.push(a)
    pq.push(b)
    pq.push(c)

    while pq.notEmpty():
        print pq.pop()
