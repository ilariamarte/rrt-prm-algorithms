#!/usr/bin/env python

# --------------------------------------------------------------------------- #
#            New nodes are attached only to nodes, never to edges             #
# --------------------------------------------------------------------------- #
#                        draw obstacles with mouse,                           #
#         then press q and click to enter qstart and run the algorithm        #
# --------------------------------------------------------------------------- #
#           if no obstacles initialized, default random: 5 obstacles          #
# --------------------------------------------------------------------------- #
#        click space during execution to print current number of nodes        #
# --------------------------------------------------------------------------- #

from support.imports_PRM import *


def init_pygame():
	pygame.init()
	pygame.display.set_caption("PRM")
	screen = pygame.display.set_mode((screen_width,screen_height))
	clock = pygame.time.Clock()
	return screen,clock


def init_PRM(Cobs,pos_node_start):
	V = [1]
	E = []
	new = 0 # number of new vertices added in one iteration
	G = Graph(V,pos_node_start,E,Cobs)
	print(f'node_start = {G.V[0].x},{G.V[0].y}')
	return G,new


def PRM(G,radius,new_group,new):
	q_rand = G.generate_node('random', new_group)
	if G.check_Cfree(q_rand) == -1: # if the node is in Cobs, skip
		U = G.near(q_rand,radius)
		G.add_vertex(q_rand)
		while size(U) != 0:
			u, U = min_distance(U, q_rand)
			u_group = u.group
			if q_rand.group != u.group and G.obstacle_free(u, q_rand): # if q_rand and u are not in the same connected component of G
				G.add_edge(u, q_rand)
				new += 1
				if G.V[G.sizeV-1].group > u.group: # changes every "group" variable in the group to the smaller between those of u and q_rand
					for i in range(G.sizeV):
						if G.V[i].group == G.V[G.sizeV-1].group:
							G.V[i].group = u_group
					q_rand.group = u_group
				else:
					for i in range(G.sizeV):
						if G.V[i].group == u_group:
							G.V[i].group = G.V[G.sizeV-1].group
		new_group += 1
	return new_group, new

if __name__ == '__main__':
	loop = True
	screen,clock = init_pygame()
	radius = 50
	mux = 1
	Cobs = []
	pos_node_start = [50,50] # default
	x,y,lx,ly = 0,0,0,0
	pos_dim = x,y,lx,ly
	is_qstart = False
	start_algorithm = False
	U = []
	new_group = 1
	while loop:
		for event in pygame.event.get():
			if event.type == pygame.QUIT or pygame.key.get_pressed()[pygame.K_ESCAPE]:
				loop = False
		
		if start_algorithm == False:
			screen.fill((0, 0, 0)) # screen refresh
			Cobs,pos_node_start,pos_dim,mux,is_qstart,start_init = check_obstacles_and_qstart(Cobs,pos_node_start,pos_dim,mux,is_qstart)
			draw_obstacles_and_qstart(screen,Cobs,pos_dim,mux,is_qstart)
		
		if start_init == True:
			G,new = init_PRM(Cobs,pos_node_start)
			start_init = False
			start_algorithm = True
		
		if start_algorithm == True:
			new = 0
			new_group,new = PRM(G, radius, new_group, new)
			G.draw(screen, new)
			if pygame.key.get_pressed()[pygame.K_SPACE]:
				print('') # carriage return
				print('number of nodes = ', G.sizeV)
				print('radius = ', round(radius))
				G.print_groups(new_group)
			if pygame.key.get_pressed()[pygame.K_a]:
				adjust_time -= 1
			if pygame.key.get_pressed()[pygame.K_s]:
				adjust_time += 1

		pygame.display.flip()
		# clock.tick(adjust_time)
pygame.quit()