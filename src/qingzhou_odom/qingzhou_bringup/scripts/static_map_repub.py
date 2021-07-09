import rospy
from nav_msgs import OccupancyGrid

def map_repuber_callback(msg):
    new_map_data = msg
    pass

map_suber = rospy.Subscriber("/map",OccupancyGrid,map_repuber_callback,queue_size=1)
