import pygame, sys 
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

class Pathfinder:
	def __init__(self,matrix):

		# setup
		self.matrix = matrix
		self.grid = Grid(matrix = matrix)
		self.select_surf = pygame.image.load('selection.png').convert_alpha()

		# pathfinding
		self.path = []

		# Roomba
		self.roomba = pygame.sprite.GroupSingle(Roomba(self.empty_path))

	def empty_path(self):
		self.path = []

	def draw_active_cell(self):
		mouse_pos = pygame.mouse.get_pos()
		row =  mouse_pos[1] // 32
		col =  mouse_pos[0] // 32
		current_cell_value = self.matrix[row][col]
		if current_cell_value == 1:
			rect = pygame.Rect((col * 32,row * 32),(32,32))
			screen.blit(self.select_surf,rect)

	def create_path(self):

		# start
		start_x, start_y = self.roomba.sprite.get_coord()
		start = self.grid.node(start_x,start_y)

		# end
		mouse_pos = pygame.mouse.get_pos()
		end_x,end_y =  mouse_pos[0] // 32, mouse_pos[1] // 32  
		end = self.grid.node(end_x,end_y) 

		# path
		finder = DijkstraFinder(diagonal_movement = DiagonalMovement.always)
		self.path,_ = finder.find_path(start,end,self.grid)
		self.grid.cleanup()
		self.roomba.sprite.set_path(self.path)

	def draw_path(self):
		if self.path:
			points = []
			for point in self.path:
				x = (point.x * 32) + 16
				y = (point.y * 32) + 16
				points.append((x,y))

			pygame.draw.lines(screen,'#4a4a4a',False,points,5)

	def update(self):
		self.draw_active_cell()
		self.draw_path()

		# roomba updating and drawing
		self.roomba.update()
		self.roomba.draw(screen)

class Roomba(pygame.sprite.Sprite):
	def __init__(self,empty_path):
		self.spriteCounter = 0
		self.spriteNum = 1
		# basic
		super().__init__()
		self.face = ""
		self.sprites = []
		self.sprites.append(pygame.image.load('up1.png'))
		self.sprites.append(pygame.image.load('up2.png'))
		self.sprites.append(pygame.image.load('down1.png'))
		self.sprites.append(pygame.image.load('down2.png'))
		self.sprites.append(pygame.image.load('left1.png'))
		self.sprites.append(pygame.image.load('left2.png'))
		self.sprites.append(pygame.image.load('right1.png'))
		self.sprites.append(pygame.image.load('right2.png'))
		self.current_sprite = 0
		self.image = self.sprites[self.current_sprite]
		self.rect = self.image.get_rect(center = (320, 256))

		# movement 
		self.pos = self.rect.center
		self.speed = 3
		self.direction = pygame.math.Vector2(0,0)

		# path
		self.path = []
		self.collision_rects = []
		self.empty_path = empty_path

	

	def get_coord(self):
		col = self.rect.centerx // 32
		row = self.rect.centery // 32
		return (col,row)

	def set_path(self,path):
		self.path = path
		self.create_collision_rects()
		self.get_direction()

	def create_collision_rects(self):
		if self.path:
			self.collision_rects = []
			for point in self.path:
				x = (point.x * 32) + 16
				y = (point.y * 32) + 16
				rect = pygame.Rect((x - 2,y - 2),(4,4))
				self.collision_rects.append(rect)

	def get_direction(self):
		self.get_current_direction()
		if self.collision_rects:
			start = pygame.math.Vector2(self.pos)
			end = pygame.math.Vector2(self.collision_rects[0].center)
			self.direction = (end - start).normalize()
		else:
			self.direction = pygame.math.Vector2(0,0)
			self.path = []

	def check_collisions(self):
		if self.collision_rects:
			for rect in self.collision_rects:
				if rect.collidepoint(self.pos):
					del self.collision_rects[0]
					self.get_direction()
		else:
			self.empty_path()

	def get_current_direction(self):
		
		angle = pygame.math.Vector2(0, -1).angle_to(self.direction)
		if angle >= 355 or angle <= 10 :
			self.face = "up"
		if (angle >= 15 and angle <= 180) :
			self.face = "right"
		if angle >= 190 and angle <= 345 or angle <= -30:
			self.face = "left"
		if angle >= 175 and angle <= 185 :
			self.face = "down"
		


	def update(self):
		self.spriteCounter+=1
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

		print(self.spriteNum)

		self.image = self.sprites[self.current_sprite]
		self.pos += self.direction * self.speed
		self.check_collisions()
		self.rect.center = self.pos

# pygame setup
pygame.init()
screen = pygame.display.set_mode((960,640))
clock = pygame.time.Clock()

# game setup
bg_surf = pygame.image.load('map_2.png').convert()
matrix = [
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,0],
    [0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,1],
    [0,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1],
    [0,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1],
    [0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,0,1,0,0],
    [0,0,0,0,0,0,1,0,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,1,1,1,1,0,0],
    [0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,0,0,1,1,1,0,1,1,1,1,0,1],
    [0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,0,0,1,0,0,1,1],
    [0,1,1,1,0,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,0,0,1,1,0,0,1,1],

	[0,1,0,1,0,0,1,1,0,1,1,1,1,1,1,1,1,0,0,0,0,1,0,0,0,1,1,1,1,0],
	[1,0,1,1,1,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0],
	[0,0,0,0,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0],
	[0,0,0,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0],
	[0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
	[0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
	[0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
	[0,0,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0]]

pathfinder = Pathfinder(matrix)

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			sys.exit()
		if event.type == pygame.MOUSEBUTTONDOWN:
			pathfinder.create_path()
	screen.blit(bg_surf,(0,0))
	pathfinder.update()
	pygame.display.update()
	clock.tick(60)