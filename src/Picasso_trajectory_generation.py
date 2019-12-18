#!/usr/bin/env python
'''
State Machine of the trajectory generator
    1. Go to the center of the paper and wait for 5 s. Adjusting paper position as needed
    2. ready_4_new_line (load a new path and draw status list, including standoff point)
        if path list is empty, go to step 6. Else, proceed to 3
    3. fly: 2s to the next stand off
    4. up_down: 1s to move up/down to/from the current standoff position, after/before the drawing process
    5. draw: each waypoint takes 0.1s to draw
    6. go back to 2
    7. idle
'''
import intera_interface
import rospy
import numpy as np
from Rico_DFS import get_total_path_list

DIST_THRE = 0.003
#TEST
# DRAW_TIME = 1.0
# UP_DOWN_TIME = 0.2
# FLY_TIME = 0.5
DRAW_TIME = 0.02
UP_DOWN_TIME = 1.0
FLY_TIME = 2.5
CHECKER_CENTER_COORD = np.array([0.7267684830590353, 0.04111839299227235])+ np.array([0.012, 0.020])
IMAGE_X_NUM = 200
IMAGE_Y_NUM = 227
PAPER_X_SIDE = 0.215 - 0.03    #unit: m. This side is parallel to the robot's x axis. The robot x axis is coming out of the screen
PAPER_Y_SIDE = 0.28 *PAPER_X_SIDE/0.215     #unit: m. This side is parallel to the robot's y axis. The robot y axis points to the left of the x axis

def mat_to_robot_frame(point):
    '''
    Converts a point's position in the image matrix to its robot frame position
    '''
    global IMAGE_X_NUM, IMAGE_Y_NUM, PAPER_X_SIDE, PAPER_Y_SIDE
    #frame1 has the same orientation as the robot frame, but is located in the origin of the matrix, which is the upper left corner of the paper
    gw1 = np.array([[1.0, 0.0, PAPER_X_SIDE/2.0],
                    [0.0, 1.0, PAPER_Y_SIDE/2.0],
                    [0.0, 1.0, 1.0]])
    #frame r has the same orientation and the origin as the matrix, but everything in this frame is in meters
    g1r = np.array([[-1.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0],
                    [0.0, 0.0, 1.0]])
    point_r = np.array([point[0]*PAPER_X_SIDE/IMAGE_X_NUM,point[1]*PAPER_Y_SIDE/IMAGE_Y_NUM, 1.0])
    point_w = gw1.dot(g1r.dot(point_r))
    return [point_w[0]+CHECKER_CENTER_COORD[0], point_w[1]+CHECKER_CENTER_COORD[1]]

class TrajGen(object):
    def __init__(self):
        global DRAW_TIME, DIST_THRE, UP_DOWN_TIME, CHECKER_CENTER_COORD, FLY_TIME

        self._limb = intera_interface.Limb("right")

                ##Test
        self.go_to_paper_center()
        rospy.sleep(5)      # for testing

        self.operating_mode = 'ready_4_new_line'
        self.draw_status_list = []
        self.init_time = None
        self.current_path = []
        self.total_path_list = get_total_path_list()
        self.current_target = None
        self.line_init_pose = None
        self.action_time = None
        self.s = 0



    def go_to_paper_center(self):
        joint_angles = { 'right_j0': -0.570514648437,
                         'right_j1': -0.405889648437,
                         'right_j2': -2.72518945313,
                         'right_j3': -1.56860742187,
                         'right_j4': -0.524416015625,
                         'right_j5': -0.679075195313,
                         'right_j6': -1.51012792969}
        self._limb.move_to_joint_positions(joint_angles)

    def update_trajectory_status(self):
        '''
        This is the main state machine
        State Machine of the trajectory generator
        1. Go to the center of the paper and wait for 5 s. Adjusting paper position as needed
        2. ready_4_new_line (load a new path and draw status list, including standoff point)
            if path list is empty, go to step 6. Else, proceed to 3
        3. fly: 2s to the next stand off
        4. up_down: 1s to move up/down to/from the current standoff position, after/before the drawing process
        5. go back to 2
        6. idle
        '''
        if self.operating_mode == 'ready_4_new_line':
            if len(self.total_path_list) != 0:
                self.operating_mode = 'fly'
                self.new_path_params_setup()
                self.operation_setup('fly')
            else:
                self.operating_mode = 'idle'

        elif self.operating_mode == 'fly':
            if  rospy.Time.now().to_sec() - self.init_time > FLY_TIME:
                self.operating_mode = 'up_down'
                self.operation_setup('up_down')

        elif self.operating_mode == 'up_down':
            if rospy.Time.now().to_sec() - self.init_time > UP_DOWN_TIME:
                self.operating_mode = 'draw'
                self.operation_setup('draw')

        elif self.operating_mode == "draw":
            if rospy.Time.now().to_sec() - self.init_time > DRAW_TIME:
                if self.draw_status_list[0] == 1:       #we still have points to draw
                    self.operating_mode = 'draw'
                    self.operation_setup('draw')
                else:
                    self.operating_mode = 'ready_4_new_line'


    def operation_setup(self, mode):
        '''
        Mode: fly, up_down, draw
        '''
        if mode == 'fly':
            self.action_time = FLY_TIME
        elif mode == 'up_down':
            self.action_time = UP_DOWN_TIME
        elif mode == 'draw':
            self.action_time = DRAW_TIME

        self.init_time = rospy.Time.now().to_sec()
        self.line_init_pose = self.update_current_pose()
        #test
        print 'current_pose: ', self.line_init_pose
        self.current_target = self.current_path[0]
        self.current_draw_status = self.draw_status_list[0]
        self.current_path.pop(0)
        self.draw_status_list.pop(0)

    def new_path_params_setup(self):
        '''
        Set up params for generating a new path
        '''
        self.current_path = self.generate_new_path()
        self.draw_status_list = self.generate_draw_status_list()
        self.total_path_list.pop(0)


    def generate_new_path(self):
        #convert coordinates
        current_path = self.total_path_list[0]
        for index in range(len(current_path)):
            current_path[index] = mat_to_robot_frame(current_path[index])
        #append standoff positions
        current_path.insert(0, current_path[0])
        current_path.insert(-1, current_path[-1])

        #Test
        # print ("new path is: ", current_path)
        return current_path

    def generate_draw_status_list(self):
        #draw_status: -1 move up, 0 move horizontally, 1 move down(force control)
        draw_status_list = [1]*len(self.current_path)
        draw_status_list[0] = 0
        draw_status_list[-1] = -1
        #test
        # print ("new draw status list: ", draw_status_list)
        return draw_status_list

    def get_xy(self):
        '''
        Returns the waypoint [x,y] for the next instant.
        '''
        current_pose = self.update_current_pose()
        if self.operating_mode == 'ready_4_new_line':
            coord = current_pose

        elif self.operating_mode == 'idle':
            coord = current_pose

        else:       #for fly, draw, up_down states
            t = rospy.Time.now().to_sec() - self.init_time
            self.s = 10 * (1.0 * t / self.action_time) ** 3 - 15 * (1.0 * t / self.action_time) ** 4 + 6 * (1.0 * t / self.action_time) ** 5
            if t> self.action_time:
                self.s = 1

            coord = self.s * np.array( self.current_target ) + (1 - self.s) * np.array(self.line_init_pose)

        return coord

    def update_current_pose(self):
        #Returns the current pose in an array
        current_pose = self._limb.endpoint_pose()
        full_return_pose = [current_pose['position'].x , current_pose['position'].y,current_pose['position'].z,current_pose['orientation'].x, current_pose['orientation'].y, current_pose['orientation'].z, current_pose['orientation'].w ]
        partial_return_pose = full_return_pose[:2]
        return partial_return_pose


    def get_draw_status(self):
        return self.current_draw_status

def main():
    rospy.init_node("sawyer")
    trajgen = TrajGen()

    r = rospy.Rate(5)
    for i in range(100):
        trajgen.update_trajectory_status()
        draw_status = trajgen.get_draw_status()
        coord = trajgen.get_xy()
        print "x, y is ", coord, ", draw status is ", draw_status
        r.sleep()

if __name__=='__main__':
    main()
