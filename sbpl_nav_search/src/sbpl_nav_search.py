#!/usr/bin/env python

package_name = 'sbpl_nav_search'
import roslib
roslib.load_manifest(package_name)
import rospy
from rospkg import RosPack

import actionlib
#from pr_msgs.msg import *
#import herbpy
#import openrave_exports
#env_vars_changed=openrave_exports.export()
import openravepy
import numpy
import time

from rospkg.rospack import RosPack

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

'''
rp = RosPack()
package_path = rp.get_path(package_name)
librarian_or_data_path = package_path+'/../librarian_ordata/ordata' # Change this to match your system.
'''

class SbplNavSearchHandler:
  def __init__(self):
    print('__init__ start')
    self.sbpl_navsearch_init()
    self.run_problem_sbpl_navsearch()


  # test function
  def run_problem_sbpl_navsearch(self):
    print('run_problem_sbpl_navsearch start')
    # Get goal
    '''
    test example
    header: 
      seq: 0
      stamp: 
        secs: 47
        nsecs: 600000000
      frame_id: /map
    pose: 
      position: 
        x: 13.7327480316
        y: 3.89822363853
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.997698115284
        w: 0.0678120251762
    '''
    sbpl_goal = PoseStamped()
    sbpl_goal.header.frame_id = "/map"
    sbpl_goal.pose.position.x = 13.7327480316
    sbpl_goal.pose.position.y = 3.89822363853
    sbpl_goal.pose.position.z = 0
    #q = transformations.quaternion_from_matrix(book_in_world)
    sbpl_goal.pose.orientation.x = 0 #q[0]
    sbpl_goal.pose.orientation.y = 0 #q[1]
    sbpl_goal.pose.orientation.z = 0.997698115284 #q[2]
    sbpl_goal.pose.orientation.w = 0.0678120251762 #q[3]
    
    # Path goal to sbpl planner and wait x seconds
    self.sbpl_navsearch(sbpl_goal, 10)


  def sbpl_navsearch_init(self):
    print 'sbpl_navsearch_init start'
    
    # initialise member valuables
    self.repeat_cnt = 0
    
    # initialise publisher and subscriber    
    self.sbpl_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    #self.sbpl_path = rospy.Subscriber("/move_base_node/TrajectoryPlannerROS/global_plan", Path, self.sbpl_getpath) # register callback
    self.sbpl_path = rospy.Subscriber("/odom", Odometry, self.sbpl_getpath) # register callback
    
    print 'sbpl_navsearch_init done'

  # find sbpl navigation path within x seconds 
  def sbpl_navsearch(self, goal, timout=10):
    print 'sbpl_navsearch start'
    
    # pass a goal to sbpl planner
    print 'goal pos x %s, y %s, z %s' %(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)
    print 'goal ori x %s, y %s, z %s, w %s' \
      %(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
    
    # I do not know why it has to publish two times ..
    for i in range(2):
      self.sbpl_goal.publish(goal)
      print 'publish goal'
      time.sleep(1)
    
    '''
    # check the sbpl path counter
    self.repeat_cnt = 0
    repeat_period = 0.01
    while(1):
      time.sleep(repeat_period)
      # return if there is no callback from sbpl planner
      if self.repeat_cnt > repeat_period*1000: # 10 seconds
        print "no path found"
        return None
      # increase the repeat counter
      # callback function reset this value whenever the path is published by sbpl planner
      self.repeat_cnt += 1
      
      # call ros.spin to allow ros callback
    '''    
    print 'sbpl_navsearch done'
     
  
  # pass a goal to get path
  def sbpl_getpath(self, path):
    print 'sbpl_getpath start'
    #print 'path size %s' %path.size()
    
    #for i in range(path.size()):
    #  print 'path x %s y %s z %s' %(path[i].pose.position.x, path[i].pose.position.y, path[i].pose.position.z)
    
    print 'odom pose x %s y %s z %s' %(path.pose.pose.position.x, path.pose.pose.position.y, path.pose.pose.position.z)
    print 'odom ori x %s, y %s, z %s, w %s' \
      %(path.pose.pose.orientation.x, path.pose.pose.orientation.y, path.pose.pose.orientation.z, path.pose.pose.orientation.w)
      
    print 'odom twist x %s y %s z %s' %(path.twist.twist.linear.x, path.twist.twist.linear.y, path.twist.twist.angular.z)
    
    print 'sbpl_getpath done\n'
    

if __name__ == '__main__':
    # initialise ros node
    rospy.init_node(package_name, anonymous=True) 
    sbpl = SbplNavSearchHandler()
    rospy.spin()
