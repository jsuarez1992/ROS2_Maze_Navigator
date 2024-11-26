# yellow_light_sdf=$HOME/.gazebo/models/yellow_light/model.sdf
# ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_light 0.0 0.0

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main():
    argv=sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("Spawning_Node")
    client= node.create_client(SpawnEntity,"/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("connected to spawner")
    sdf_path = argv[0]
    request = SpawnEntity.Request()
    request.name = argv[1]

    #use user defined positions if they are provided at the beginning
    if len(argv)>3:
        request.initial_pose.position.x = float(argv[2])
        request.initial_pose.position.y = float(argv[3])
    request.xml= open(sdf_path,'r').read()

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node,future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r'% future.exception())
    
    node.get_logger.info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
