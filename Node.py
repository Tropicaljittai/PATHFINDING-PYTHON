class Node:
    def __init__(self, col, row):
        self.parent = None
        self.col = col
        self.row = row
        self.distance = 0
        self.rootDistance = 0
        self.manhattanDistance = 0
        self.start = False
        self.goal = False
        self.solid = False
        self.open = False
        self.visited = False

    def __lt__(self, other):
        # In Java, this was compareTo method, which returns 0.
        # Assuming the comparison here is based on distance
        return self.distance < other.distance

    def set_as_start(self):
        self.start = True
        self.goal = False
        self.visited = False
        self.solid = False

    def set_as_discovered(self):
        self.visited = True

    def set_as_goal(self):
        self.goal = True
        self.visited = False
        self.start = False
        self.solid = False

    def set_as_solid(self):
        self.goal = False
        self.visited = False
        self.start = False
        self.solid = True

    def set_as_path(self):
        pass

    def deselect(self):
        self.solid = False
        self.start = False
        self.goal = False
        self.visited = False