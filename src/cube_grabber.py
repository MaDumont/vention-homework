#!/usr/bin/env python

###################################
# This file provides a complete example of a pick and place task
# using the moveit_commander python api. 
###################################

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from math import pi
from moveit_msgs import msg
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
import moveit_commander
from std_msgs.msg import String


class CubeMover():

    def __init__(self, initial_cube_pose):
        # Initialise moveit and ROS node
        roscpp_initialize(sys.argv)
        rospy.init_node('python_pick_place_tutorial', anonymous=True)


        self.planning_scene_interface = PlanningSceneInterface()
        robot = RobotCommander(robot_description='/robot_description')

        # Select planning group
        self.group = robot.get_group('panda_arm')
        # Set a liberal planner timeout 
        self.group.set_planning_time(seconds=45.0)

        # IMPORTANT: you must call sleep after the previous line 
        # to ensure the planning scene object is initialised before
        # using it.
        rospy.sleep(2)

        self.actual_pose_cube = initial_pose_cube
        rospy.Subscriber("/move_group/monitored_planning_scene", msg.PlanningScene, self.callback_cube_position)

        rospy.Timer(rospy.Duration(0.5), self.timer_callback)


    def timer_callback(self, event):

        rospy.loginfo("Actual position cube : %s", self.actual_pose_cube.position)
        rospy.loginfo("Vector UP %s", self.actual_pose_cube.orientation)


    def callback_cube_position(self, data):
        if len(data.robot_state.attached_collision_objects) > 0:
            self.actual_pose_cube = data.robot_state.attached_collision_objects[0].object.pose


    def closedGripper(self, posture):
        """Specifies closed grasp posture for the pick and place motions"""
        posture.joint_names.append('panda_finger_joint1')
        posture.joint_names.append('panda_finger_joint2')
        pt = JointTrajectoryPoint()

        # now specify where they should be positioned in order to 
        # close the hand
        pt.positions.append(0.02)
        pt.positions.append(0.02)
        # ... and how long it should take
        pt.time_from_start = rospy.Duration(0.5)
        posture.points.append(pt)

    def openGripper(self, posture):
        """Specifies open grasp posture for the pick and place motions"""
        # specify which end effector joints are involved in the grasp
        posture.joint_names.append('panda_finger_joint1')
        posture.joint_names.append('panda_finger_joint2')
        pt = JointTrajectoryPoint()

        # now specify where they should be positioned in order to 
        # open the hand
        pt.positions.append(0.06)
        pt.positions.append(0.06)
        # ... and how it should take to open the hand
        pt.time_from_start = rospy.Duration(0.5)
        posture.points.append(pt)

    def pick(self, pose_object):
        """Specifies, plans, and executes a pick operation."""
        # Create empty grasp object to parameterise the pick motion
        grasp = msg.Grasp()

        # A grasp has to be specified relative to some frame
        # of reference. Here we specify the base of the robot
        # as the frame for the grasp
        #grasp.grasp_pose.header.frame_id = 'base_link'
        grasp.grasp_pose.header.frame_id = 'panda_link0'

        # First we specify the pose for the grasp itself in terms 
        # of both the orientation and position
        q = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
        grasp.grasp_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        grasp.grasp_pose.pose.position.x = pose_object.position.x - 0.095
        grasp.grasp_pose.pose.position.y = pose_object.position.y
        grasp.grasp_pose.pose.position.z = pose_object.position.z + 0.04/2

        # Next we specify the pre-grasp approach i.e. how should the
        # end-effector approach the object. 
        grasp.pre_grasp_approach.direction.header.frame_id = 'panda_link0'
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.115

        # Then we specify the post-grasp retreat  i.e. how should the
        # end-effector lift the object from the support surface
        grasp.post_grasp_retreat.direction.header.frame_id = 'panda_link0'
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.25

        # Finally we specify the parameters for the open and close
        # posture of the grasp 
        self.openGripper(grasp.pre_grasp_posture)

        self.closedGripper(grasp.grasp_posture)

        # When the robot picks up the object it then includes the object
        # as part of its planning i.e. to ensure both the robot and 
        # object do not come into collision with another object. Since
        # the object is in contact with table1 we must let the planner
        # know that it can ignore table1 initially during planning the 
        # lift.
        #move_group.set_support_surface_name('table1')

        # Finally we can plan and execute the pick operation
        self.group.pick('object',grasp)

    def place(self, drop_zone_pose):
        """Specifies, plans, and executes a place operation."""
        # Create empty PlaceLocation object to specify the place motion 
        placeLocation = msg.PlaceLocation()

        # similiar to frame_id in pick motion
        placeLocation.place_pose.header.frame_id = "panda_link0"

        # First we specify the pose for the place itself in terms 
        # of both the orientation and position
        q = quaternion_from_euler(0, 0, pi / 2)
        placeLocation.place_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        placeLocation.place_pose.pose.position = drop_zone_pose.position

        ## Setting pre-place approach ##
        # Defined with respect to frame_id
        placeLocation.pre_place_approach.direction.header.frame_id = "panda_link0"
        # Direction is set as negative z axis
        placeLocation.pre_place_approach.direction.vector.z = - 1.0  
        placeLocation.pre_place_approach.min_distance = 0.095
        placeLocation.pre_place_approach.desired_distance = 0.115

        ## Setting post-grasp retreat
        # Defined with respect to frame_id
        placeLocation.post_place_retreat.direction.header.frame_id = "panda_link0"
        # Direction is set as negative y axis
        placeLocation.post_place_retreat.direction.vector.y = -1.0      
        placeLocation.post_place_retreat.min_distance = 0.1
        placeLocation.post_place_retreat.desired_distance = 0.25

        ## Setting posture of eef after placing object
        # Similar to the pick case
        self.openGripper(placeLocation.post_place_posture)

        ## Set support surface as table2 ##
        #group.set_support_surface_name("table2")

        ## Call place to place the object using the place locations given. ##
        self.group.place("object", placeLocation)

    def addCollisionObjects(self, pose_cube):
        """Add two tables and an object to the planning scene."""
        # clean the scene in case the script has already been executed
        self.planning_scene_interface.remove_world_object("object")

        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = "panda_link0"#'base_link'
        p.pose.orientation.w = 1.0
        p.pose.position = pose_cube.position
        self.planning_scene_interface.add_box("object", p, (0.04, 0.04, 0.04))


    def removeCollisionObject(self):
        self.planning_scene_interface.remove_world_object("object")


    def pick_and_place(self, initial_pose_cube, drop_zone_pose):

        # Setup scene for the pick and place example
        self.addCollisionObjects(initial_pose_cube)

        rospy.sleep(1)

        # Plan and execute the pick operation
        self.pick(initial_pose_cube)

        rospy.sleep(1)

        # Plan and execute the place operaation
        self.place(drop_zone_pose)

        self.removeCollisionObject()

if __name__=='__main__':
        initial_pose_cube = Pose()
        initial_pose_cube.position.x = 0.5
        initial_pose_cube.position.y = 0.2
        initial_pose_cube.position.z = 0.0 

        drop_zone_pose = Pose()
        drop_zone_pose.position.x = -0.2
        drop_zone_pose.position.y = 0.7
        drop_zone_pose.position.z = 0.02 

        cube_mover = CubeMover(initial_pose_cube)

        rospy.loginfo("Initiale Pose Cube %s", initial_pose_cube)
        rospy.loginfo("Drop zone Pose %s", drop_zone_pose)

        cube_mover.pick_and_place(initial_pose_cube, drop_zone_pose)
