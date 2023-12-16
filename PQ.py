class PriorityQueue:
    def __init__(self, sz=1):
        self.size = 0
        self.capacity = sz
        self.heap = [None] * sz
        self.map = {}

    def is_empty(self):
        return self.size == 0

    def clear(self):
        self.heap = [None] * self.capacity
        self.size = 0
        self.map.clear()

    def poll(self):
        return self.remove_at(0)

    def contains(self, node):
        return node in self.map

    def add(self, node):
        if node is None:
            raise ValueError("No null elements please :)")
        if self.size < self.capacity:
            self.heap[self.size] = node
        else:
            self.heap.append(node)
            self.capacity += 1
        self.map_add(node, self.size)
        self.swim(self.size)
        self.size += 1

    def less(self, i, j):
        return self.heap[i].distance <= self.heap[j].distance

    def sink(self, i):
        while True:
            left = 2 * i + 1
            right = 2 * i + 2
            smallest = left
            if right < self.size and self.less(right, left):
                smallest = right
            if left >= self.size or self.less(i, smallest):
                break
            self.swap(smallest, i)
            i = smallest

    def swim(self, i):
        parent = (i - 1) // 2
        while i > 0 and self.less(i, parent):
            self.swap(parent, i)
            i = parent
            parent = (parent - 1) // 2

    def remove(self, node):
        if node is None:
            return False
        index = self.map_get(node)
        if index is not None:
            self.remove_at(index)
        return index is not None

    def remove_at(self, i):
        if self.is_empty():
            return None
        self.size -= 1
        data = self.heap[i]
        self.swap(i, self.size)
        self.heap[self.size] = None
        self.map_remove(data, self.size)
        if i == self.size:
            return data
        elem = self.heap[i]
        self.sink(i)
        if self.heap[i] == elem:
            self.swim(i)
        return data

    def swap(self, i, j):
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]
        self.map_swap(self.heap[i], self.heap[j], i, j)

    def is_min_heap(self, i):
        if i >= self.size:
            return True
        left = 2 * i + 1
        right = 2 * i + 2
        if left < self.size and not self.less(i, left):
            return False
        if right < self.size and not self.less(i, right):
            return False
        return self.is_min_heap(left) and self.is_min_heap(right)

    def map_add(self, elem, index):
        if elem not in self.map:
            self.map[elem] = set()
        self.map[elem].add(index)

    def map_remove(self, elem, index):
        if elem in self.map:
            self.map[elem].discard(index)
            if len(self.map[elem]) == 0:
                del self.map[elem]

    def map_get(self, elem):
        if elem in self.map:
            return max(self.map[elem])
        return None

    def map_swap(self, node1, node2, node1_index, node2_index):
        if node1 in self.map:
            self.map[node1].discard(node1_index)
            self.map[node1].add(node2_index)
        if node2 in self.map:
            self.map[node2].discard(node2_index)
            self.map[node2].add(node1_index)