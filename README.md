# RRT-PRM-algorithms
2D routing and path planning RRT, PRM and PRM* algorithms  

## RRT
Rapidly-exploring Random Tree  

Progressively covers the area in a net of nodes, but doesn't dinamically improve and rewire edges of the graph like RRT*  
The alternative version connects new nodes to edges, not just to other nodes  

![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/RRT/images/rrt1.PNG)
![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/RRT/images/rrt2.PNG)
![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/RRT/images/rrt3.PNG)

## PRM & PRM*
Probabilistic RoadMap  

A different approach than RDT (Rapidly-exploring Dense Trees, which RRT/RRT* belong to)  
Creates various groups of nodes that connect when they get close enough to each other  
Covers areas with big obstacles faster since it can generate new nodes everywhere, and it's not stopped by walls even if they leave little room to maneuver around them  

![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/PRM/images/prm1.PNG)
![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/PRM/images/prm2.PNG)
![alt text](https://github.com/ilariamarte/rrt-prm-algorithms/blob/main/PRM/images/prm3.PNG)
