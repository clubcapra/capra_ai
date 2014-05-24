#! /usr/bin/env python

import roslib; roslib.load_manifest('capra_ai')
import rospy
import actionlib
from ai_base import AIBase
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class LazyAI(AIBase):
    def __init__(self):
        super(self.__class__, self).__init__("capra_ai")
        
    def run(self):
        super(self.__class__, self).run()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = 20

        self.send_goal(goal, True)                
        print self.get_state()  
        
    def estop_listener(self, status):
        super(self.__class__, self).estop_listener(status)
        print status
              
if __name__ == '__main__':
    try:
        ai = LazyAI()
        ai.run()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
