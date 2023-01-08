from numpy import size
from numpy.random import randint
from pygame import Rect
from pygame.draw import rect
from pygame.draw import circle
from pygame.draw import line
from pygame.font import SysFont
from support.Target import Target
from support.Target import reshuffle_target_if_overlap
from support.Obstacle import Obstacle
from support.Obstacle import reshuffle_obstacles_if_overlap
from support.Node import Node
from support.Node import reshuffle_init_nodes_if_in_Cobs
from support.Edge import Edge
from support.functions import distance_points
from support.functions import distance_edge
from support.functions import intersect_point_line_perpendicular
from support.functions import distance_nodes
from support.functions import points2line
from support.functions import find_intersect
from support.functions import compare_angular_coeff
from support.variables import screen_width
from support.variables import screen_height
from support.variables import scale
from support.variables import check_edge

class Graph:
	def __init__(self,V,pos_node_start,E,Cobs): # Vertices, Edges
		print('Graph = (V=' + str(V) + ', E=' + str(E) + ')')
		if size(Cobs) == 0: # Cobs not initialized
			Cobs = [Obstacle('random','random') for i in range(5)]
			Cobs = reshuffle_obstacles_if_overlap(Cobs)
		sizeV = 1
		V = [Node(pos_node_start, 0)] # group 0
		V = reshuffle_init_nodes_if_in_Cobs(V,Cobs)
		sizeE = 0
		E = []
		self.V = V
		self.E = E
		self.sizeV = sizeV
		self.sizeE = sizeE
		self.Cobs = Cobs

	def generate_node(self,coord,group):
		"""#generate random node, or node with given coordinates, or node in direction of target coordinates"""
		if type(coord) == str: # case: coord = 'random'
			node_new = Node('random', group)
		else:
			node_new = Node(coord, group)
		return node_new
	
	def add_vertex(self,node):
		"""#adds 1 vertex"""
		self.V.append(node)
		self.sizeV += 1

	def add_edge(self,node1,node2):
		"""#uses Node classes"""
		if type(node2) == str: # case: node2 = 'last'
			node2 = self.V[self.sizeV-1]
		self.E.append(Edge(node1,node2))
		self.sizeE += 1
	
	def remove_edge(self,i):
		"""#uses index"""
		del self.E[i]
		self.sizeE -= 1
	
	def near(self,node_new,r):
		"""#returns all the nodes nearer than radius r to node_new"""
		pointP = node_new.x, node_new.y
		V,E,sizeV,sizeE = self.V,self.E,self.sizeV,self.sizeE
		U = []
		for i in range(sizeV): # computes distance nodes
			pointI = V[i].x, V[i].y # x,y di i
			d_vertex = distance_points(pointP, pointI)
			if d_vertex < r:
				U.append(V[i])
		return U

	def check_Cfree(self,node): # not used
		"""#if node in Cfree returns -1, if in Cobs returns index of Obstacle; Obstacle border not considered part of the Obstacle"""
		x,y = node.x,node.y
		Cobs = self.Cobs
		for i in range(size(Cobs)):
			xo,yo,lx,ly = Cobs[i].x,Cobs[i].y,Cobs[i].lx,Cobs[i].ly
			if (x > xo and x < xo+lx) and (y > yo and y < yo+ly):
				return i
		return -1
	
	def obstacle_free(self,node_attach,node_new):
		"""#returns 1 if yes, 0 if no ("node_attach" in code doesn't refer to anything, don't want to change it from previous version)"""
		x,y = node_attach.x,node_attach.y
		point_new = [node_new.x,node_new.y]
		line_edge = points2line([x,y], point_new)
		for i in range(size(self.Cobs)): # checks every Obstacle for edge overlaps
			obs = self.Cobs[i]
			x1,y1,x4,y4 = obs.x, obs.y, obs.x+obs.lx, obs.y+obs.ly # obstacle coordinates (upper left, lower right)
			if (node_attach.x <= x1 and node_new.x <= x1) or (node_attach.x >= x4 and node_new.x >= x4) or (node_attach.y <= y1 and node_new.y <= y1) or (node_attach.y >= y4 and node_new.y >= y4):
				continue # if node_attach and node_new are both on one side with respect to the obstacle, skip
			else:
				ul,ur,dl,dr = [x1,y1], [x4,y1], [x1,y4], [x4,y4] # obstacle corners (up left, up right, down left, down right)
				left,above,right,below = [ul,dl], [ul,ur], [ur,dr], [dl,dr] # obstacle sides
				side = [left, above, right, below]
				# first check adjacent, then corners
						# adjacent
				if   y <= y1 and x >= x1 and x <= x4: p1,p2 = above	# adjacent, above
				elif y >= y4 and x >= x1 and x <= x4: p1,p2 = below	# adjacent, below
				elif x <= x1 and y >= y1 and y <= y4: p1,p2 = left	# adjacent, left
				elif x >= x4 and y >= y1 and y <= y4: p1,p2 = right	# adjacent, right
				# corners
				else:
					if   x < x1 and y < y1: c,s = ul,0				# corner up left
					elif x > x4 and y < y1: c,s = ur,1				# corner up right
					elif x > x4 and y > y4: c,s = dr,2				# corner down right
					elif x < x1 and y > y4: c,s = dl,3				# corner down left
					line_check = points2line(c, point_new)
					i = compare_angular_coeff(line_check, line_edge)
					if i == -1: continue # ignores iteration if vertical lines in corner
					s = s + i
					if s > 3: s = 0
					p1,p2 = side[s]
				line_obs = points2line(p1, p2)
				xi,yi = find_intersect(line_edge, line_obs)
				if xi == -1: continue # ignores iteration if the lines are parallel
				if xi >= x1 and xi <= x4 and yi >= y1 and yi <= y4:
					return 0
		return 1
	
	def steer(self,node_attach,node_new,eta):
		"""#finds intersect among the line between node_attach and node_new
			#and a circumference with center in node_new and radius eta"""
		if distance_nodes(node_attach,node_new) <= eta:
			return node_new
		a,b,c = points2line([node_attach.x,node_attach.y],[node_new.x,node_new.y])
		if b != 0:
			m, q = -a/b, -c/b
			x_c, y_c, r = node_attach.x, node_attach.y, eta
			A, B, C = -2*x_c, -2*y_c, x_c**2 + y_c**2 - r**2
			x1 = -(A + B*m + 2*m*q + (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2))/(2*(m**2 + 1))
			x2 = -(A + B*m + 2*m*q - (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2))/(2*(m**2 + 1))
			y1 = q - (m*(A + B*m + 2*m*q + (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2)))/(2*(m**2 + 1))
			y2 = q - (m*(A + B*m + 2*m*q - (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2)))/(2*(m**2 + 1))
		else: # vertical
			x1 = node_attach.x
			x2 = x1
			y1 = node_attach.y + eta
			y2 = node_attach.y - eta
		if distance_points([node_new.x,node_new.y],[x1,y1]) < distance_points([node_new.x,node_new.y],[x2,y2]):
			node_new.x, node_new.y = round(x1), round(y1)
		else:
			node_new.x, node_new.y = round(x2), round(y2)
		return node_new

	def check_target(self):
		"""#checks every node in target (if node.parent not in target) and goes backwards to node_start, then chooses the road with the smallest cumulative cost"""
		V,sizeV,target,bc = self.V,self.sizeV,self.target,self.best_cost
		nodes_to_target = [] # since the RRT* algorithm can accidently make a worse path, you can't keep the old one or update only when a better one is found, it has to be updated every time
		index,cost = -1,-1
		for i in range(1,sizeV): # does not check node_start
			for j in range(size(target)):
				and1 = (V[i].x > target[j].x and V[i].x < target[j].x + target[j].lx) and (V[i].y > target[j].y and V[i].y < target[j].y + target[j].ly) # node inside target
				and2 = (V[i].parent.x <= target[j].x or V[i].parent.x >= target[j].x + target[j].lx) or (V[i].parent.y <= target[j].y or V[i].parent.y >= target[j].y + target[j].ly) # parent outside target
				if (and1 and and2) and (self.V[i].cost < cost or cost == -1):
						if round(self.V[i].cost,1) < round(bc,1) or bc == -1: # first path found or new better path found ; could be changed to "if self.V[i].cost < bc-10 or bc == -1:" 
							self.progressive_number = sizeV # resets pn to sizeV (for probability in generate_node)
						cost = self.V[i].cost
						self.current_cost = cost # cumulative cost of the entire road
						if cost < bc or bc == -1:
							self.best_cost = cost # updates bc only if the new path is the best until then
						index = i # i == V[i].index
		if index != -1:
			nodes_to_target.append(index)
			while index != 0: # or "while type(V[index].parent) != str" --> case: V[index].parent = 'none' --> node_start reached
				index = V[index].parent.index
				nodes_to_target.append(index)
		return nodes_to_target
	
	def draw(self,screen,new):
		"""#draws Graph (Nodes, Edges, Obstacles)"""
		if new == 0: # only once, at the start, draws Obstacles
			Cobs = self.Cobs
			for i in range(size(Cobs)):
				rect(screen, 'red', Rect(Cobs[i].x, Cobs[i].y, Cobs[i].lx, Cobs[i].ly), 1)
		V = self.V
		font = SysFont(None, 15)
		# draw last e in E
		for i in range(new):
			i = self.sizeE-1-i
			E = self.E
			line(screen, 'blue', [ E[i].node1.x, E[i].node1.y ], [ E[i].node2.x, E[i].node2.y ])
		# draw last v in V (or last 2)
		if new == 2: # draw second-to-last
			i = self.sizeV-2
			circle(screen, 'blue', [ V[i].x, V[i].y ], 1)
			# screen.blit(font.render(str(i+1) + ' ' + str(int(V[i].x)) + ',' + str(int(V[i].y)), True, 'red'), (V[i].x + 5, V[i].y - 5)) # numbers on the nodes
		i = self.sizeV-1
		circle(screen, 'blue', [ V[i].x, V[i].y ], 1)
		# screen.blit(font.render(str(i+1) + ' ' + str(int(V[i].x)) + ',' + str(int(V[i].y)), True, 'red'), (V[i].x + 5, V[i].y - 5)) # numbers on the nodes

	def print_V(self):
		V = self.V
		for i in range(self.sizeV):
			print('V[' + str(i) + '] = (' + str(V[i].x) + ',' + str(V[i].y) + ')')
			print('group = ', V[i].group)
	
	def print_E(self):
		E = self.E
		for i in range(self.sizeE):
			print('E[' + str(i) + '] = (' + str(E[i].node1.x) + ',' + str(E[i].node1.y) + ')(' + str(E[i].node2.x) + ',' + str(E[i].node2.y) + ')')
	
	def print_Cobs(self):
		Cobs = self.Cobs
		for i in range(size(Cobs)):
			print('Cobs[' + str(i) + '] = (' + str(Cobs[i].x) + ',' + str(Cobs[i].y) + ')(' + str(Cobs[i].lx) + ',' + str(Cobs[i].ly) + ')')
	
	def print_groups(self,max_group):
		V,sizeV = self.V, self.sizeV
		count = 0 
		for k in range(max_group):
			for i in range(sizeV):
				if V[i].group == k:
					count += 1
					break

		print('number of different groups = ', count)
		return count
	