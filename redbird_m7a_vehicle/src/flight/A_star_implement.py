from A_Star import Map
from A_Star import Node
from A_Star import Utilities as util
import numpy as np


#mapx is size of map in x direction, mapy is size of map in y direction, obstacle_pos is an array of obstacle position, start and end pos are tuples with coordinates of positions.
def AStar(mapx, mapy, obstacles_pos, start_pos, end_pos):

    mapp = Map(mapx,mapy)
    start = mapp.get_node_at_location(start_pos)
    goal = mapp.get_node_at_location((end_pos))
    
    o = []
    for obstacle in obstacles_pos:
        ob = []
        ob.append((obstacle[0],obstacle[1]))

        #first ring
        ob.append((obstacle[0]+1,obstacle[1]))
        ob.append((obstacle[0]+1,obstacle[1]-1))
        ob.append((obstacle[0],obstacle[1]-1))
        ob.append((obstacle[0]-1,obstacle[1]-1))
        ob.append((obstacle[0]-1,obstacle[1]))
        ob.append((obstacle[0]-1,obstacle[1]+1))
        ob.append((obstacle[0],obstacle[1]+1))
        ob.append((obstacle[0]+1,obstacle[1]+1))

        #second ring
        ob.append((obstacle[0] -2,obstacle[1] +2))
        ob.append((obstacle[0] -1,obstacle[1] +2))
        ob.append((obstacle[0],obstacle[1] +2))
        ob.append((obstacle[0] +1,obstacle[1] +2))
        ob.append((obstacle[0] +2,obstacle[1] +2))
        ob.append((obstacle[0] +2,obstacle[1] +1))
        ob.append((obstacle[0] +2,obstacle[1]))
        ob.append((obstacle[0] +2,obstacle[1] -1))
        ob.append((obstacle[0] +2,obstacle[1] -2))
        ob.append((obstacle[0] +1,obstacle[1] -2))
        ob.append((obstacle[0],obstacle[1] -2))
        ob.append((obstacle[0] -1,obstacle[1] -2))
        ob.append((obstacle[0] -2,obstacle[1] -2))
        ob.append((obstacle[0] -2,obstacle[1] -1))
        ob.append((obstacle[0] -2,obstacle[1]))
        ob.append((obstacle[0] -2,obstacle[1] +1))

        flag = True
        for pt in ob:
            if pt == start_pos:
                flag = False

        if flag:
            o.extend(ob)

    p = util.duplicates(o)
    d = util.boundary_eliminate(p, mapx, mapy)

    mapp.add_obstacles(d)   
       
    print d
                
    
    

    
    

    # path = [start.get_location]
    # temp = start.get_location()
    # for pts in o:
    #     if pts == temp:
    #         dx = temp[0] - pts[0]
    #         dy = temp[1] - pts[1]
    #         if dx >= dy:
    #             if dx > 0:
    #                 for x in range(abs(dx)):
    #                     path.append(temp[0] + 1, temp[1])
    #                     temp = (temp[0] + 1, temp[1])
    #             if dx < 0:
    #                 for x in range(abs(dx)):
    #                     path.append(temp[0] - 1, temp[1])
    #                     temp = (temp[0] - 1, temp[1])
                
     



    print "Start node: ", start.get_id()
    print "Goal node: ", goal.get_id()

    for node in mapp.get_nodes():
        node.set_g_cost(node.calculate_g_cost(goal))
        node.set_h_cost(node.calculate_h_cost(goal))

    open_list = [start]
    closed_list = []

    # Loop and perform calculations to determine path
    while open_list:
        # Current node is the node with the shortest distance to the goal node
        current_node = util.get_lowest_cost(open_list, goal)

        # Remove the current node from the open list and place on closed list
        open_list.remove(current_node)
        closed_list.append(current_node)

        # Get all adjacent nodes to the current node
        for node in mapp.get_adjacent_nodes(current_node.get_location()):
            # Create a cost variable consisting of the current node's
            # current G cost + the cost to get to the loop node
            cost = current_node.get_g_cost() + current_node.calculate_g_cost(node.get_location())

            # If the adjacent node is in the open list and the calculated
            # cost is less the cost of the adjacent node, remove it from
            # the open list (new path is better)
            if node in open_list and cost < node.get_g_cost():
                # Remove node from open, new path is better
                open_list.remove(node)
            # More or less the same as above, though this should rarely
            # be encountered
            if node in closed_list and cost < node.get_g_cost():
                # Remove node from open, new path is better
                closed_list.remove(node)
            # If the adjacent node is not in the open list or the closed
            # list then set its G cost to the previously calculated cost
            # and append it to the open list, then set its parent to the
            # id of the current node
            if node not in open_list and node not in closed_list:
                node.set_g_cost(cost)
                open_list.append(node)
                node.set_parent(current_node.get_id())

    # Grab the final path and display working backwards from the
    # goal node to the start node
    try:
        print "Final path from start node to goal node:"
        final_path = []
        current_node = goal
        while current_node.get_location() != start.get_location():
            final_path.append(current_node.get_location())
            current_node = mapp.get_node_by_id(current_node.get_parent())



        print start.get_location(), "->",
        for location in reversed(final_path):
            print location,
            if location != list(reversed(final_path))[len(final_path)-1]:
                print "->",

        return list(reversed(final_path))
    except:
         return False

#1.5 m from obstacles... create fuction to remove adjacent nodes near obstacles.

row = 40
col = 40
obstacles = [(5,5)]
current_pos = (0,0)
target = (6,6)

print AStar(row,col, obstacles, current_pos, target)