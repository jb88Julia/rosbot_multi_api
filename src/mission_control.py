#! /usr/bin/env python
import rospy
# for the path planning
from enum import Enum
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback


class Robot_State(Enum):
    idle = 1
    lost = 2
    searching = 3
    rescued = 4


# Node that will control the whole flow of the mission:
# -request move_base actions from each robot
# -get statuses of each robot
class RobotCore:

    robot_name = None
    node_name = None
    mb_client = None
    status = None

    def __init__(self, robot_name) -> None:
        
        self.robot_name = robot_name
        self.node_name =  robot_name+'_core'
        self.status =Robot_State.idle

        # create the connections to the Move Base action server
        self.connect_move_base()


    def connect_move_base(self):
        # create the connections to the Move Base action server
        self.mb_client = actionlib.SimpleActionClient(self.robot_name + '/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.mb_client.wait_for_server()




def main():
    
    rospy.init_node("mission_control")
    r1 = RobotCore("robot1")
    r2 = RobotCore("robot2")
    r3 = RobotCore("robot3")

    # Start mission
    rospy.loginfo("Starting mission... Robot 2 getting lost")
    

    rospy.spin()


if __name__=="__main__":
    #if len(sys.argv) < 2:
    #    print("usage: robot_core.py robot_name")
    #else:
    main()

