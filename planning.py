
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import copy
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps


def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""

	speed_instance = speed_mmps(speed)
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

class node:
    def __init__(self,coord):
        self.coord = coord
        self.parent = None
        self.H = 0
        self.G = 0
    def __lt__(self, other):
        return (self.H + self.G) < (other.H + other.G)

def neighbor_is_in_closed_set(node_coord,grid):
    for element in grid.getVisited():
        if element == node_coord:
            return True
    return False

def neighbor_is_in_open_set(node_coord,open_set):
    for element in open_set.queue:
        element_node = element
        if node_coord == element_node.coord:
            return True
    return False


def get_node_from_open_set(node_coord, open_set):
    for element in open_set.queue:
        element_node = element
        if node_coord == element_node.coord:
            return element_node

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    open_set = PriorityQueue()

    start_node = node(grid.getStart())
    goal_node = node(grid.getGoals()[0])

    start_node.G = 0
    start_node.H = heuristic(start_node.coord, goal_node.coord)
    F = start_node.G + start_node.H

    current_node = start_node
    open_set.put(start_node)

    while not open_set.empty(): # if there is none inside the open set, program just ends.

        current_node = open_set.get()
#        print("open_set_tuple", current_node)
        grid.addVisited(current_node.coord)

#        print("current node coordinate", current_node.coord)

#        print("current_node.cood", current_node.coord)

        if current_node.coord == goal_node.coord:
#            print("finally goal!")
            return reconstruct_path(current_node, grid)


        # Check if the current node is goal node. If it is, we create a path
        # if check_points(current_node, goal_node): # if the current node is the goal node, we report out path
        #
        #     while current_node.parent: # as far as the current node has a parent
        #         grid.setPath(current_node.point)
        #         current_node = current_node.parent
        #
        #     grid.setPath(current_node.point) # the start node does not have a parent, but we should add it
        #
        #     break

        # If the current noise is not the goal node, we check neighbors of the current node

        for neighbor in grid.getNeighbors(current_node.coord): # neighbor = [x,y, delta:value]

 #           print("neighbors", grid.getNeighbors(current_node.coord))
  #          print("neighbor[1]",neighbor[1])

            neighbor_coord = neighbor[0]
            diff_neighbor_current = neighbor[1]

            if neighbor_is_in_closed_set(neighbor_coord, grid):
   #             print("neighbor is in closed set")
                continue

            if not neighbor_is_in_open_set(neighbor_coord, open_set):
#                print("let's put neighbor to the open set")
                neighbor_node = node(neighbor_coord)
                neighbor_node.G = current_node.G + diff_neighbor_current
                neighbor_node.H = heuristic(neighbor_node.coord, goal_node.coord)
                neighbor_node.parent = current_node

#                print("F", F)

                open_set.put(neighbor_node)
                continue

            # If neighbor is in open set
            neighbor_node = get_node_from_open_set(neighbor_coord, open_set)

            tentative_G = current_node.G + diff_neighbor_current

            if tentative_G >= neighbor_node.G:
                continue

            neighbor_node.parent = current_node
            neighbor_node.G = tentative_G
            neighbor_node.H = heuristic(neighbor_node.coord, goal_node.coord)

def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """



#    print("current", current)
#    print("goal", goal)
#    print(((current[0]-goal[0])**2 + (current[1] - goal[1])**2) **0.5)
    return ((current[0]-goal[0])**2 + (current[1] - goal[1])**2) **0.5


def reconstruct_path(current_node, grid):

#    print("let's start reconstruct_path")
    while current_node.parent:
#        print("grid.setPath:",current_node.coord)
        grid._path.append(current_node.coord)
#        print("path", grid.getPath())
        current_node = current_node.parent

    grid._path.append(current_node.coord)
    grid._path.reverse()


# Check if the current node is goal node. If it is, we create a path
# if check_points(current_node, goal_node): # if the current node is the goal node, we report out path
#
#     while current_node.parent: # as far as the current node has a parent
#         grid.setPath(current_node.point)
#         current_node = current_node.parent
#
#     grid.setPath(current_node.point) # the start node does not have a parent, but we should add it
#
#     break

def scan_update(cubes_in_view, grid, goal_found):

    cube1_found = False
    c_space = 2
    center = (math.ceil(grid.width / 2), math.ceil(grid.height / 2))

    if (len(cubes_in_view) > 0):
        for cube_in_view in cubes_in_view:
            cube_coord = (math.floor(cube_in_view.pose.position.x / grid.scale) + 3,
                          math.floor(cube_in_view.pose.position.y / grid.scale) + 2)

            if cube_in_view.object_id == 3:
                cube1_found = True
                grid.clearGoals()
                print("cube_coord", cube_coord)
                grid.addGoal(cube_coord)

            else:
                cube1_found = False

                for c_space_x in range(-1*c_space, c_space+1):
                    for c_space_y in range(-1*c_space, c_space+1):
                        obstacle_coord = (cube_coord[0] + c_space_x, cube_coord[1] + c_space_y)
                        grid.addObstacle(obstacle_coord)

                if (not len(grid.getGoals())) and (not goal_found): # there was no goal
                    if (center[0] > (cube_coord[0] - (c_space+3))) and (center[0] < (cube_coord[0]+(c_space+3))) and (center[1] > (cube_coord[1] - (c_space+3))) and (center[1] < (cube_coord[1]+(c_space+3))):
                        grid.addGoal((cube_coord[0]+10,cube_coord[1]-3))
                    else:
                        grid.addGoal(center)

    return cube1_found


#def new_cube_found(robot,cubes_in_areas):

#    lst = list(robot.world.visible_objects)

def move_robot_one_grid(robot,current_robot_coord, current_robot_angle, path_coord):

    x_coord_diff = path_coord[0] - current_robot_coord[0]
    y_coord_diff = path_coord[1] - current_robot_coord[1]

    print(x_coord_diff)
    print(y_coord_diff)

    if x_coord_diff == 0 and y_coord_diff == 1:
        target_angle = 90
    elif x_coord_diff == 1 and y_coord_diff == 1:
        target_angle = 45
    elif x_coord_diff == 1 and y_coord_diff == 0:
        target_angle = 0
    elif x_coord_diff == 1 and y_coord_diff == -1:
        target_angle = -45
    elif x_coord_diff == 0 and y_coord_diff == -1:
        target_angle = -90
    elif x_coord_diff == -1 and y_coord_diff == -1:
        target_angle = -135
    elif x_coord_diff == -1 and y_coord_diff == 0:
        target_angle = 180
    else:
        target_angle = 135

    angle = target_angle - current_robot_angle

    print("rotation angle",angle)

    # Let's rotate the robot first
    cozmo_turn_in_place(robot, angle, 50)

    # Let's move the robot by one grid
    dist = ((x_coord_diff * grid.scale) ** 2 + (y_coord_diff * grid.scale) ** 2) ** 0.5

    cozmo_drive_straight(robot, dist, 75)


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent

    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()

    cube1_reached = False
    rotated_angle = 0
    need_rotation = True
    goal_found = False

    while (not cube1_reached):

        cubes_in_view = list(robot.world.visible_objects)
        cube1_found = scan_update(cubes_in_view, grid, goal_found)
        print("cube1 found",cube1_found)

        if cube1_found == True:
            need_rotation = False
            goal_found = True

        if (need_rotation == True) and rotated_angle < 360:
            robot.turn_in_place(degrees(30), speed=degrees(20)).wait_for_completed()
            rotated_angle += 30

        else:
            need_rotation = False
            rotated_angle = 0

            current_robot_coord = (math.floor(robot.pose.position.x / grid.scale)+3, math.floor(robot.pose.position.y / grid.scale)+2)
            current_robot_angle = robot.pose.rotation.angle_z.degrees

            grid.clearStart()
            grid.setStart(current_robot_coord)

            print("grid.start", grid.getStart())
            print("grid.goal", grid.getGoals())
            astar(grid, heuristic)
            next_path_coord = grid.getPath()[1]

            move_robot_one_grid(robot, current_robot_coord, current_robot_angle, next_path_coord)

            grid.clearPath()
            grid.clearVisited()

            goal_coord = grid.getGoals()[0]

            robot_coord = (math.floor(robot.pose.position.x/grid.scale)+3, math.floor(robot.pose.position.y/grid.scale)+2)
            print("robot coord",robot_coord)
            dist_robot_goal = ((robot_coord[0]-goal_coord[0]) **2 + (robot_coord[1]-goal_coord[1])**2) ** 0.5
            print("robot_goal", goal_coord[0], goal_coord[1])
            print("goal_found", goal_found)

            if (dist_robot_goal < 3) and (goal_found == True):
                cube1_reached = 1
            elif (dist_robot_goal < 3) and (goal_found == False):
                need_rotation = True
                grid.clearObstacles()
                grid.clearGoals()




 #   while not stopevent.is_set():



#        robot_coord = [math.floor(robot.pose.position.x/grid.scale),math.floor(robot.pose.position.y/grid.scale)]
#        print(robot_coord)
#        grid.setStart(robot_coord)
#        visualizer.update()



    #      while new_cube_found() == False:
 #        print("robot coordinate", robot_coord)

#        visualizer.update()

#        cozmo_drive_straight(robot, 50, 50)






#        visualizer.update()




        # Cozmo camera works. Measure relative position and pose. Check if there is any change in location

        # if a cube is not detected,
            # Go to the center of the area and find a cube until it sees

        # else: it found the cube
            # Update the map (cube 1: obstacle, set the goal).
            # Find the path by A*
            # Start to move to the next node in path (how to control the wheel speed then?)

    pass # Your code here


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

