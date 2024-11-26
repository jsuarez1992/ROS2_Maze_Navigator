#! /usr/bin/env python3
'''
This is a Python script that uses the nav2_simple_commander package to navigate a robot
through a sequence of pre-defined poses in a ROS 2 environment. The BasicNavigator class
is used to control the robot's movement, and the PoseStamped class is used to represent each goal pose.
The setInitialPose() method is used to set the robot's initial position, and goThroughPoses() is used to navigate
the robot through the sequence of goals.
 The script waits for the task to complete before printing the result and shutting down the navigator.
'''
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    goals=[]
    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -5.18
    initial_pose.pose.position.y = -6.58
    initial_pose.pose.orientation.z =0.0
    initial_pose.pose.orientation.w =0.99
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()


    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x =-1.23
    goal_pose.pose.position.y =-2.1
    goal_pose.pose.orientation.w =0.99
    goals.append(goal_pose)

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x =-7.4
    goal_pose_1.pose.position.y =-1.17
    goal_pose_1.pose.orientation.w =0.99
    goals.append(goal_pose_1)


    navigator.goThroughPoses(goals)
    while not navigator.isTaskComplete():
        pass


    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
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

    exit(0)


if __name__ == '__main__':
    main()