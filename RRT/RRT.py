#!/usr/bin/env python

# --------------------------------------------------------------------------- #
#            New nodes are attached only to nodes, never to edges             #
# --------------------------------------------------------------------------- #
# draw obstacles with mouse, then press t (target) and draw target with mouse #
# when finished, press q and click to enter qstart and run the algorithm      #
# --------------------------------------------------------------------------- #
# if no obstacles or target initialized, default random: 5 obstacles 1 target #
# --------------------------------------------------------------------------- #
#        click space during execution to print current number of nodes        #
# --------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------- #
# check_edge = 0 --> SIMPLE RDT: new nodes are attached only to nodes, never to edges #
# ----------------------------------------------------------------------------------- #
# check_edge = 1 --> POOR PERFORMANCE: checks distance to every node and every edge   #
# ----------------------------------------------------------------------------------- #

from support.imports_RRT import *
check_edge = 0


def init_pygame():
	pygame.init()
	pygame.display.set_caption("RRT")
	screen = pygame.display.set_mode((screen_width,screen_height))
	clock = pygame.time.Clock()
	return screen,clock


def init_RRT(Cobs,target,pos_node_start,check_edgek):
	V = [1]
	E = []
	G = Graph(V,pos_node_start,E,Cobs,target,check_edgek)
	print(f'node_start = {G.V[0].x},{G.V[0].y}')
	return G


def RRT(G):
	q_rand = G.generate_node('random')
	q_nearest = G.nearest_check_edge(q_rand)
	q_new = G.steer(q_nearest,q_rand,eta)
	q_new_in_Cfree = G.stopping_configuration(q_nearest,q_new)
	if q_new_in_Cfree.x != q_nearest.x or q_new_in_Cfree.y != q_nearest.y:
		q_new = q_new_in_Cfree
		q_new.cost = q_nearest.cost + distance_nodes(q_nearest, q_new)
		G.add_vertex(q_new)
		q_new.parent = q_nearest
		G.add_edge(q_nearest, q_new)


if __name__ == '__main__':
	loop = True
	screen,clock = init_pygame()
	eta = 100
	mux = 1
	Cobs = []
	target = []
	pos_node_start = [50,50] # default
	x,y,lx,ly = 0,0,0,0
	pos_dim = x,y,lx,ly
	is_target = False
	is_qstart = False
	start_algorithm = False
	while loop:
		for event in pygame.event.get():
			if event.type == pygame.QUIT or pygame.key.get_pressed()[pygame.K_ESCAPE]:
				loop = False
		
		screen.fill((0, 0, 0)) # screen refresh

		if start_algorithm == False:
			Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart,start_init = check_obstacles_and_target_and_qstart(Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart)
			draw_obstacles_and_target_and_qstart(screen,Cobs,target,pos_dim,mux,is_target,is_qstart)
		
		if start_init == True:
			G = init_RRT(Cobs,target,pos_node_start,check_edge)
			start_init = False
			start_algorithm = True

		if start_algorithm == True:
			RRT(G)
			nodes_to_target, cumulative_cost = G.check_target()
			G.draw(screen, nodes_to_target)
			if pygame.key.get_pressed()[pygame.K_SPACE]:
				print('number of nodes = ', G.sizeV)
				print('radius_V = ', round(radius_V))
				print('cost of road to target = ', cumulative_cost)
			if pygame.key.get_pressed()[pygame.K_a]:
				adjust_time -= 1
			if pygame.key.get_pressed()[pygame.K_s]:
				adjust_time += 1

		pygame.display.flip()
		# clock.tick(adjust_time)
pygame.quit()