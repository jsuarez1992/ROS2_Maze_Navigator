import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_msgs.msg import Header
import numpy as np

#Basically the first one imports the NOdes, and the second one imports the string message type.
#This dependencies will also have to be added to package.xml too

class Occupancy_grid_pub(Node):
    def __init__(self):
        super().__init__('Occupancy_grid_pub_node')
        self.grid_publisher_ = self.create_publisher(OccupancyGrid,'map',10)
        self.timer = self.create_timer(0.5,self.og_pub_callback)

    def og_pub_callback(self):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header= Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map_frame'

        occupancy_grid_msg.info.resolution = 1.0
        occupancy_grid_msg.info.width =3
        occupancy_grid_msg.info.height=3

        occupancy_grid_msg.info.origin.position.x = 0.0
        occupancy_grid_msg.info.origin.position.y = 0.0
        occupancy_grid_msg.info.origin.position.z = 0.0
        array = np.array([0,1,1,0,0,0,0,0,1],dtype=np.int8)
        occupancy_grid_msg.data = array.tolist()
        self.grid_publisher_.publish(occupancy_grid_msg)

def main(args=None):
    rclpy.init(args=args)
    OG_publisher = Occupancy_grid_pub()
    print("Publishing Map")
    rclpy.spin(OG_publisher)
    #destroys the node otherwise it is done automatically later on
    OG_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
