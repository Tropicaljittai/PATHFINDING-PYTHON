import time

import pygame, sys
from pygame import mixer
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

import os
from Node import *
from PQ import PriorityQueue
from collections import deque

class Pathfinder:
    def __init__(self, matrix, algorithm = 1):

        # setup
        self.matrix = matrix
        self.grid = [[Node(col, row) for col in range(len(matrix[0]))] for row in range(len(matrix))]
        self.select_surf = pygame.image.load('selection.png').convert_alpha()
        self.algorithm = algorithm
        
        # pathfinding
        self.path = []

        # Roomba
        self.roomba = pygame.sprite.GroupSingle(Roomba(self.empty_path))

    def empty_path(self):
        self.path = []

    def get_algorithm_name(self):
        algorithm_names = {
            1: "Dijkstra",
            2: "A*",
            3: "BFS",
            4: "Best-First Search",
            5: "DFS",
            6: "Iterative Deeping A*",
            7: "JPS"
        }
        return algorithm_names.get(self.algorithm, "Unknown Algorithm")

    def draw_active_cell(self):
        mouse_pos = pygame.mouse.get_pos()
        row = mouse_pos[1] // 32
        col = mouse_pos[0] // 32
        current_cell_value = self.matrix[row][col]
        if current_cell_value == 1:
            rect = pygame.Rect((col * 32, row * 32), (32, 32))
            screen.blit(self.select_surf, rect)

    def get_neighbors(self, node):
        neighbors = []
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        for dx, dy in directions:
            new_col, new_row = node.col + dx, node.row + dy
            if 0 <= new_col < len(self.grid[0]) and 0 <= new_row < len(self.grid):
                if self.matrix[new_row][new_col] == 1:
                    neighbors.append(self.grid[new_row][new_col])
        return neighbors

    def heuristic(self, node, end):
        return abs(node.col - end.col) + abs(node.row - end.row)
    
    def a_star_search(self, start_node, end_node):
        pq = PriorityQueue()
        start_node.distance = 0
        pq.add(start_node)
        distance = {node: float('inf') for row in self.grid for node in row}
        distance[start_node] = 0
        parent = {node: None for row in self.grid for node in row}


        visited = set()

        while not pq.is_empty():
            current_node = pq.poll()
            visited.add(current_node)
            if current_node == end_node:
                break

            for neighbor in self.get_neighbors(current_node):
                if neighbor in visited:
                    continue
                dx, dy = abs(neighbor.col - current_node.col), abs(neighbor.row - current_node.row)
                move_cost = 1.41 if dx == 1 and dy == 1 else 1
                tentative_distance = distance[current_node] + move_cost
                if tentative_distance < distance[neighbor]:
                    distance[neighbor] = tentative_distance
                    parent[neighbor] = current_node
                    neighbor.distance = tentative_distance + self.heuristic(neighbor, end_node)
                    if not pq.contains(neighbor):
                        pq.add(neighbor)

        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = parent[current]
        return path[::-1]

    def iter_deepening_a_star_search(self, start_node, end_node):
        # Initial threshold is the heuristic cost from the start to the end node
        threshold = self.heuristic(start_node, end_node)

        while True:
            # Initialize the distance and parent dictionaries for this iteration
            distance = {node: float('inf') for row in self.grid for node in row}
            distance[start_node] = 0
            parent = {node: None for row in self.grid for node in row}

            # Perform the depth-limited search with the current threshold
            temp = self.search(start_node, 0, threshold, end_node, distance, parent)
            if temp == float('inf'):
                # If temp is infinity, no path was found within the threshold
                return None
            elif temp == -1:
                # If temp is -1, a path to the goal was found
                # Reconstruct and return the path
                path = []
                current = end_node
                while current:
                    path.append(current)
                    current = parent[current]
                return path[::-1]
            else:
                # Increase the threshold for the next iteration
                # This is the minimum f-cost that exceeded the current threshold
                threshold = temp

    def search(self, node, g, threshold, end_node, distance, parent):
        # Calculate the f-cost for the node (g-cost + heuristic)
        f = g + self.heuristic(node, end_node)
        if f > threshold:
            # If the f-cost exceeds the current threshold, return the f-cost
            return f
        if node == end_node:
            # If the goal node is reached, return -1 to indicate success
            return -1

        # Initialize the minimum threshold for the next iteration
        min_threshold = float('inf')
        for successor in self.get_neighbors(node):
            # Explore the neighbors of the current node
            if distance[successor] > g + 1:
                distance[successor] = g + 1
                parent[successor] = node
                # Recursively search from the neighbor
                temp = self.search(successor, g + 1, threshold, end_node, distance, parent)

                if temp == -1:
                    # If the goal is found in the recursive search, return -1
                    return -1
                if temp < min_threshold:
                    # Update the minimum threshold for the next iteration
                    min_threshold = temp

        return min_threshold

 
    def jump_point_search(self, start_node, end_node):
        pq = PriorityQueue()
        start_node.distance = 0
        pq.add(start_node)
        distance = {node: float('inf') for row in self.grid for node in row}
        distance[start_node] = 0
        parent = {node: None for row in self.grid for node in row}

        visited = set()

        while not pq.is_empty():
            current_node = pq.poll()
            visited.add(current_node)
            if current_node == end_node:
                break

            for neighbor in self.get_jps_neighbors(current_node, end_node):
                if neighbor in visited:
                    continue
                dx, dy = abs(neighbor.col - current_node.col), abs(neighbor.row - current_node.row)
                move_cost = 1.41 if dx == 1 and dy == 1 else 1
                tentative_distance = distance[current_node] + move_cost
                if tentative_distance < distance[neighbor]:
                    distance[neighbor] = tentative_distance
                    parent[neighbor] = current_node
                    neighbor.distance = tentative_distance + self.heuristic(neighbor, end_node)
                    if not pq.contains(neighbor):
                        pq.add(neighbor)

        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = parent[current]
        return path[::-1]

    def get_jps_neighbors(self, node, end_node):
        neighbors = []

        # Get forced neighbors
        forced_neighbors = self.get_forced_neighbors(node, end_node)

        # Add forced neighbors to the list
        for neighbor in forced_neighbors:
            neighbors.append(neighbor)

        # Add natural neighbors to the list
        natural_neighbors = self.get_neighbors(node)
        for neighbor in natural_neighbors:
            if neighbor not in neighbors and self.matrix[neighbor.row][neighbor.col] == 1:
                neighbors.append(neighbor)

        return neighbors

    def get_forced_neighbors(self, node, end_node):
        forced_neighbors = []

        dx = end_node.col - node.col
        dy = end_node.row - node.row

        # Diagonal movement
        if dx != 0 and dy != 0:
            if dx > 0 and dy > 0:
                if self.matrix[node.row + 1][node.col] == 1 and self.matrix[node.row][node.col + 1] == 1:
                    forced_neighbors.append(self.grid[node.row + 1][node.col + 1])
            elif dx > 0 and dy < 0:
                if self.matrix[node.row - 1][node.col] == 1 and self.matrix[node.row][node.col + 1] == 1:
                    forced_neighbors.append(self.grid[node.row - 1][node.col + 1])
            elif dx < 0 and dy > 0:
                if self.matrix[node.row + 1][node.col] == 1 and self.matrix[node.row][node.col - 1] == 1:
                    forced_neighbors.append(self.grid[node.row + 1][node.col - 1])
            elif dx < 0 and dy < 0:
                if self.matrix[node.row - 1][node.col] == 1 and self.matrix[node.row][node.col - 1] == 1:
                    forced_neighbors.append(self.grid[node.row - 1][node.col - 1])

        # Horizontal movement
        elif dx != 0:
            if dx > 0:
                if self.matrix[node.row][node.col + 1] == 1:
                    forced_neighbors.append(self.grid[node.row][node.col + 1])
            elif dx < 0:
                if self.matrix[node.row][node.col - 1] == 1:
                    forced_neighbors.append(self.grid[node.row][node.col - 1])

        # Vertical movement
        elif dy != 0:
            if dy > 0:
                if self.matrix[node.row + 1][node.col] == 1:
                    forced_neighbors.append(self.grid[node.row + 1][node.col])
            elif dy < 0:
                if self.matrix[node.row - 1][node.col] == 1:
                    forced_neighbors.append(self.grid[node.row - 1][node.col])

        return forced_neighbors

    def dijkstra_search(self, start_node, end_node):
        pq = PriorityQueue()
        start_node.distance = 0
        pq.add(start_node)
        distance = {node: float('inf') for row in self.grid for node in row}
        distance[start_node] = 0
        parent = {node: None for row in self.grid for node in row}

        visited = set()

        while not pq.is_empty():
            current_node = pq.poll()
            visited.add(current_node)
            if current_node == end_node:
                break

            for neighbor in self.get_neighbors(current_node):
                if neighbor in visited:
                    continue
                move_cost = 1
                tentative_distance = distance[current_node] + move_cost

                if tentative_distance < distance[neighbor]:
                    distance[neighbor] = tentative_distance
                    parent[neighbor] = current_node
                    neighbor.distance = tentative_distance
                    if not pq.contains(neighbor):
                        pq.add(neighbor)

        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = parent[current]
        return path[::-1]


    def bfs_search(self, start_node, end_node):
        queue = deque([start_node])
        visited = {start_node}
        parent = {start_node: None}

        while queue:
            current_node = queue.popleft()

            if current_node == end_node:
                break

            for neighbor in self.get_neighbors(current_node):
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = current_node
                    queue.append(neighbor)
            # Reconstruct path from end_node to start_node
        path = []
        current = end_node
        while current:
            path.append(current)
            current = parent.get(current)
        return path[::-1]
    
    def bestfs_search(self, start_node, end_node):

        pq = PriorityQueue()
        start_node.distance = self.heuristic(start_node, end_node)  # Initialize distance using heuristic
        pq.add(start_node)

        visited = set()
        parent = {start_node: None}

        while not pq.is_empty():
            current_node = pq.poll()
            visited.add(current_node)

            if current_node == end_node:
                break

            for neighbor in self.get_neighbors(current_node):
                if neighbor not in visited:
                    parent[neighbor] = current_node
                    neighbor.distance = self.heuristic(neighbor, end_node)
                    pq.add(neighbor)

        # Reconstruct path from end_node to start_node
        path = []
        current = end_node
        while current:
            path.append(current)
            current = parent.get(current)
        return path[::-1]

    def dfs_search(self, start_node, end_node):
        stack = [start_node]
        visited = set()
        parent = {start_node: None}

        while stack:
            current_node = stack.pop()

            if current_node == end_node:
                break

            if current_node not in visited:
                visited.add(current_node)
                for neighbor in self.get_neighbors(current_node):
                    if neighbor not in visited:
                        parent[neighbor] = current_node
                        stack.append(neighbor)

        # Reconstruct path from end_node to start_node
        path = []
        current = end_node
        while current:
            path.append(current)
            current = parent.get(current)
        return path[::-1]

    def create_path(self):
        global walksound, walking
        if not walking:
            walking = True
            mixer.Sound.play(walksound, -1)
        start_x, start_y = self.roomba.sprite.get_coord()
        start_node = self.grid[start_y][start_x]
        mouse_pos = pygame.mouse.get_pos()
        end_x, end_y = mouse_pos[0] // 32, mouse_pos[1] // 32
        end_node = self.grid[end_y][end_x]

        if self.algorithm == 1:
            self.path = self.dijkstra_search(start_node, end_node)
        elif self.algorithm == 2:
            self.path = self.a_star_search(start_node, end_node)
        elif self.algorithm == 3:
            self.path = self.bfs_search(start_node, end_node)  
        elif self.algorithm == 4:
            self.path = self.bestfs_search(start_node, end_node)  
        elif self.algorithm == 5:
            self.path = self.dfs_search(start_node, end_node) 
        elif self.algorithm == 6:
            self.path = self.iter_deepening_a_star_search(start_node, end_node) 
        elif self.algorithm == 7:
            self.path = self.jump_point_search(start_node, end_node) 

        self.roomba.sprite.set_path(self.path)

    def draw_path(self):
        if self.path and len(self.path) > 1:
            points = [(node.col * 32 + 16, node.row * 32 + 16) for node in self.path]
            pygame.draw.lines(screen, '#4a4a4a', False, points, 5)

    def update(self):
        self.draw_active_cell()
        self.draw_path()

        # roomba updating and drawing
        self.roomba.update()
        self.roomba.draw(screen)


class Roomba(pygame.sprite.Sprite):
    def __init__(self, empty_path):
        self.spriteCounter = 0
        self.spriteNum = 1
        self.algorithm = 0
        
        # basic
        super().__init__()
        self.face = ""
        self.sprites = [pygame.image.load('up1.png'), pygame.image.load('up2.png'), pygame.image.load('down1.png'),
						pygame.image.load('down2.png'), pygame.image.load('left1.png'), pygame.image.load('left2.png'),
						pygame.image.load('right1.png'), pygame.image.load('right2.png')]
        self.current_sprite = 0
        self.image = self.sprites[self.current_sprite]
        self.rect = self.image.get_rect(center=(320, 256))

        # movement
        self.pos = self.rect.center
        self.speed = 3
        self.direction = pygame.math.Vector2(0, 0)

        # path
        self.path = []
        self.collision_rects = []
        self.empty_path = empty_path

        self.has_path = False
        self.start_time = None
        self.elapsed_time = 0
    def get_coord(self):
        col = self.rect.centerx // 32
        row = self.rect.centery // 32
        return (col, row)

    def set_path(self, path):
        if path:
            self.path = path
            self.create_collision_rects()
            self.get_direction()
            # Start the timer
            self.start_time = time.time()
            self.has_path = True  # Flag to indicate that the path is being traversed
        else:
            self.has_path = False

    def create_collision_rects(self):
        self.collision_rects = [(node.col * 32 + 16, node.row * 32 + 16) for node in self.path]


    def get_direction(self):
        self.get_current_direction()
        if self.collision_rects:
            start = pygame.math.Vector2(self.pos)
            end = pygame.math.Vector2(self.collision_rects[0])
            self.direction = (end - start).normalize()
        else:
            self.direction = pygame.math.Vector2(0, 0)
            self.path = []

    def check_collisions(self):
        if self.collision_rects:
            target_point = pygame.math.Vector2(self.collision_rects[0])
            if target_point.distance_to(pygame.math.Vector2(self.pos)) < self.speed:
                del self.collision_rects[0]
                self.get_direction()
        else:
            self.empty_path()


    def get_current_direction(self):

        angle = pygame.math.Vector2(0, -1).angle_to(self.direction)
        if angle >= 355 or angle <= 10:
            self.face = "up"
        if (angle >= 15 and angle <= 180):
            self.face = "right"
        if angle >= 190 and angle <= 345 or angle <= -30:
            self.face = "left"
        if angle >= 175 and angle <= 185:
            self.face = "down"

    def update(self):
        global walksound, pathfinder, walking
        if not self.path and self.has_path:
            walking = False
            mixer.Sound.fadeout(walksound, 1)
            # Path traversal is complete
            self.elapsed_time = time.time() - self.start_time
            algo_names = pathfinder.get_algorithm_name()
            print(f"Chosen Algorithm: {algo_names}")
            print(f"Time taken to reach the endpoint: {self.elapsed_time:.2f} seconds")
            self.has_path = False  # Stop timing
        
        if pathfinder.algorithm == 1:
            self.sprites = [pygame.image.load('up1.png'), pygame.image.load('up2.png'), pygame.image.load('down1.png'),
						pygame.image.load('down2.png'), pygame.image.load('left1.png'), pygame.image.load('left2.png'),
						pygame.image.load('right1.png'), pygame.image.load('right2.png')]
        if pathfinder.algorithm == 2:
            original_image = pygame.image.load('char2/Male.png')

            rows, columns = 3, 4

            sub_image_size = (original_image.get_width() // columns, original_image.get_height() // rows)
            frame_size = (48, 48)
            final_images = []

            for row in range(rows):
                for col in range(columns):
                    x = col * sub_image_size[0]
                    y = row * sub_image_size[1]
                    sub_image = original_image.subsurface(pygame.Rect(x, y, sub_image_size[0], sub_image_size[1]))

                    frame = pygame.Surface(frame_size, pygame.SRCALPHA)
                    frame_x = (frame_size[0] - sub_image_size[0]) // 2
                    frame_y = (frame_size[1] - sub_image_size[1]) // 2

                    frame.blit(sub_image, (frame_x, frame_y))
                    final_images.append(frame)

            self.sprites = [final_images[0], final_images[1], final_images[2], final_images[3], final_images[0], final_images[1], final_images[2], final_images[3]]
            self.sprites[4] = pygame.transform.flip(self.sprites[4], True, False)
            self.sprites[5] = pygame.transform.flip(self.sprites[5], True, False)
        if pathfinder.algorithm == 3:
            original_image = pygame.image.load('char2/Female.png')

            rows, columns = 3, 4

            sub_image_size = (original_image.get_width() // columns, original_image.get_height() // rows)
            frame_size = (48, 48)
            final_images = []

            for row in range(rows):
                for col in range(columns):
                    x = col * sub_image_size[0]
                    y = row * sub_image_size[1]
                    sub_image = original_image.subsurface(pygame.Rect(x, y, sub_image_size[0], sub_image_size[1]))

                    frame = pygame.Surface(frame_size, pygame.SRCALPHA)
                    frame_x = (frame_size[0] - sub_image_size[0]) // 2
                    frame_y = (frame_size[1] - sub_image_size[1]) // 2

                    frame.blit(sub_image, (frame_x, frame_y))
                    final_images.append(frame)

            self.sprites = [final_images[0], final_images[1], final_images[2], final_images[3], final_images[0], final_images[1], final_images[2], final_images[3]]
            self.sprites[4] = pygame.transform.flip(self.sprites[4], True, False)
            self.sprites[5] = pygame.transform.flip(self.sprites[5], True, False)
        
        if pathfinder.algorithm == 4:
            original_image = pygame.image.load('char2/bear_polar.png')

            rows, columns = 2, 4

            sub_image_size = (original_image.get_width() // columns, original_image.get_height() // rows)
            frame_size = (48, 48)
            final_images = []

            for row in range(rows):
                for col in range(columns):
                    x = col * sub_image_size[0]
                    y = row * sub_image_size[1]
                    sub_image = original_image.subsurface(pygame.Rect(x, y, sub_image_size[0], sub_image_size[1]))
                    
                    frame = pygame.Surface(frame_size, pygame.SRCALPHA)
                    frame_x = (frame_size[0] - sub_image_size[0]) // 2
                    frame_y = (frame_size[1] - sub_image_size[1]) // 2

                    frame.blit(sub_image, (frame_x, frame_y))
                    final_images.append(frame)

            self.sprites = [final_images[4], final_images[5], final_images[6], final_images[7], final_images[4], final_images[5], final_images[6], final_images[7]]
            self.sprites[4] = pygame.transform.flip(self.sprites[4], True, False)
            self.sprites[5] = pygame.transform.flip(self.sprites[5], True, False)
        
        if pathfinder.algorithm == 5:
            original_image = pygame.image.load('char2/penguin.png')

            rows, columns = 1, 4

            sub_image_size = (original_image.get_width() // columns, original_image.get_height() // rows)
            frame_size = (48, 48)
            final_images = []

            for row in range(rows):
                for col in range(columns):
                    x = col * sub_image_size[0]
                    y = row * sub_image_size[1]
                    sub_image = original_image.subsurface(pygame.Rect(x, y, sub_image_size[0], sub_image_size[1]))
                    
                    frame = pygame.Surface(frame_size, pygame.SRCALPHA)
                    frame_x = (frame_size[0] - sub_image_size[0]) // 2
                    frame_y = (frame_size[1] - sub_image_size[1]) // 2

                    frame.blit(sub_image, (frame_x, frame_y))
                    final_images.append(frame)

            self.sprites = [final_images[0], final_images[1], final_images[2], final_images[3], final_images[0], final_images[1], final_images[2], final_images[3]]
            self.sprites[4] = pygame.transform.flip(self.sprites[4], True, False)
            self.sprites[5] = pygame.transform.flip(self.sprites[5], True, False)
            
        self.spriteCounter += 1
        if self.spriteCounter > 17:
            if self.spriteNum == 1:
                self.spriteNum = 2
            elif self.spriteNum == 2:
                self.spriteNum = 1
            self.spriteCounter = 0

        if self.face == "up":
            if self.spriteNum == 1:
                self.current_sprite = 0
            elif self.spriteNum == 2:
                self.current_sprite = 1
        if self.face == "down":
            if self.spriteNum == 1:
                self.current_sprite = 2
            elif self.spriteNum == 2:
                self.current_sprite = 3
        if self.face == "right":
            if self.spriteNum == 1:
                self.current_sprite = 6
            elif self.spriteNum == 2:
                self.current_sprite = 7
        if self.face == "left":
            if self.spriteNum == 1:
                self.current_sprite = 4
            elif self.spriteNum == 2:
                self.current_sprite = 5

        # print(self.spriteNum)

        self.image = self.sprites[self.current_sprite]
        self.pos += self.direction * self.speed
        self.check_collisions()
        self.rect.center = self.pos


# pygame setup
pygame.init()
screen = pygame.display.set_mode((960, 640))
clock = pygame.time.Clock()

# game setup
bg_surf = pygame.image.load('map_2.png').convert()
matrix = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
    [0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1],
    [0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1],
    [0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1],
    [0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0],
    [1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]]

pathfinder = Pathfinder(matrix)
font = pygame.font.SysFont("comicsansms", 25)

onMusic = True

music = mixer.music.load("sound/BGM.wav")
mixer.music.play(-1)
mixer.music.set_volume(0.5)

walksound = mixer.Sound("sound/walk.wav")
walking = False

dijkstraImage = pygame.image.load("Icons/Dikstra.png")
AStarImage = pygame.image.load("Icons/Star_Icon.png")
BFSImage = pygame.image.load("Icons/BFS_ICON.png")
DFSImage = pygame.image.load("Icons/DFS_ICON.png")
BestFSImage = pygame.image.load("Icons/BestFS_ICON.png")


algo = 1

while True:
    screen.blit(bg_surf, (0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            pathfinder.create_path()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_1:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 1
            elif event.key == pygame.K_2:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 2
            elif event.key == pygame.K_3:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 3
            elif event.key == pygame.K_4:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 4
            elif event.key == pygame.K_5:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 5
            elif event.key == pygame.K_6:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 6
            elif event.key == pygame.K_7:
                mixer.Sound.play(mixer.Sound("sound/confirm.wav"))
                pathfinder.algorithm = 7
            elif event.key == pygame.K_m:
                if onMusic:
                    mixer.music.unpause()
                else:
                    mixer.music.pause()
                onMusic = False if onMusic else True
    if pathfinder.algorithm == 1:
        screen.blit(dijkstraImage, (600, 400))
    elif pathfinder.algorithm == 2:
        screen.blit(AStarImage, (600, 400))
    elif pathfinder.algorithm == 3:
        screen.blit(BFSImage, (600, 400))
    elif pathfinder.algorithm == 4:
        screen.blit(BestFSImage, (600, 400))
    elif pathfinder.algorithm == 5:
        screen.blit(DFSImage, (600, 400))
    pathfinder.update()
    pygame.display.update()
    clock.tick(60)