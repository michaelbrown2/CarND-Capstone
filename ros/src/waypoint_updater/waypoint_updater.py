#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
#MAX_DECEL = 0.5
#STOP_DIST = 5.0
#TARGET_SPEED_MPH = 20

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        max_speed = rospy.get_param('/waypoint_loader/velocity')
        self.max_speed = max_speed * 1000.0 / 3600.0  #k/h to m/s

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.cur_pose = None
        self.waypoints = None  
        self.redlight_waypoint_idx = None
        self.velocity = 0.0

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.cur_pose = msg.pose 
        if self.waypoints is not None:                
            self.publish()

    def waypoints_cb(self, msg):
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = msg.waypoints  

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.redlight_waypoint_idx = msg.data        
        if self.redlight_waypoint_idx > -1:
            self.publish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x #m/s

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def distance(self, p1, p2):
        #From waypoint_loader code
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
    
    def closest_waypoint(self, pose, waypoints):
        # Code from the path planning module , reimplement in python
        closestlen = 100000 #large number
        closestwaypoint = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for idx, waypoint in enumerate(waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closestlen):
                closestlen = dist
                closestwaypoint = idx
                
        return closestwaypoint
    
    def next_waypoint(self, pose, waypoints):
        # Code from the path planning module , reimplement in python
        closestwaypoint = self.closest_waypoint(pose, waypoints)
        map_x = waypoints[closestwaypoint].pose.pose.position.x
        map_y = waypoints[closestwaypoint].pose.pose.position.y

        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        roll,pitch,yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)

        if angle > (math.pi / 4):        
            closestwaypoint += 1
        
        return closestwaypoint


    
    def decelerate(self, waypoints, redlight_index):
        # from Waypoint_loader code
        if len(waypoints) < 1:
            return []
          
        last = waypoints[redlight_index]
        last.twist.twist.linear.x = 0.
        for idx, wp in enumerate(waypoints):

            if idx > redlight_index:
                vel = 0
            else:
                dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
                dist = max(0, (dist - 5.0) )                
                vel  = math.sqrt(2 * 0.5 * dist) 
                if vel < 1.:
                    vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

        return waypoints

    def publish(self):
        
        if self.cur_pose is not None:
            next_waypoint_idx = self.next_waypoint(self.cur_pose, self.waypoints)
            lookahead_waypoints = self.waypoints[next_waypoint_idx:next_waypoint_idx+LOOKAHEAD_WPS]

            if self.redlight_waypoint_idx is None or self.redlight_waypoint_idx < 0:
            
                # set the velocity for lookahead waypoints
                for i in range(len(lookahead_waypoints) - 1):
                    #v=(TARGET_SPEED_MPH * 1609.34) / (60 * 60) # convert  miles per hour to meters per sec
                    v = min(self.velocity+1.0,self.max_speed)
                    self.set_waypoint_velocity(lookahead_waypoints, i, v) 

            else:                
                redlight_lookahead_idx = max(0, self.redlight_waypoint_idx - next_waypoint_idx)
                lookahead_waypoints = self.decelerate(lookahead_waypoints, redlight_lookahead_idx)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = lookahead_waypoints
            
            self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
