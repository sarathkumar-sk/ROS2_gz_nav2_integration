import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.task import Future

from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros

from sfr_coursework2_interface_package.action import DroneControl


class MissionControllerNode(Node):
    
    def __init__(self):
        super().__init__('plasma_droid_controller_node')
       
    
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/model/box/cmd_vel',
            1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.world_frame = 'shapes'
        self.box_frame = 'box'
        self.target_frame = 'target_0'
    
        self.action_server = ActionServer(
            self,
            DroneControl,
            'plasma_droid/set_pose', 
            self.execute_callback,
        )

        self.action_client = ActionClient(
            self,
            DroneControl,
            'plasma_droid/set_pose',
        )

        self.attempt_count = 0
        self.max_attempts = 4

        self.mission_timer = self.create_timer(1.0, self.wait_for_server)
        
        self.get_logger().info('Plasma droid Controller node started')


    def distance(self, p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received action goal')

        desired_pose: PoseStamped = goal_handle.request.desired_pose

        feedback = DroneControl.Feedback()
        result = DroneControl.Result()

        start_time = time.time()
        
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

        while rclpy.ok():

            if time.time() - start_time > 5.0:
                self.get_logger().warn('Action failed: timeout (5s limit reached)')
                goal_handle.abort()
                result.success = False
                self.cmd_vel_publisher.publish(stop_cmd)
                return result

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.box_frame,
                    rclpy.time.Time()
                )
             
                current_pose = PoseStamped()
                current_pose.header.stamp = self.get_clock().now().to_msg()
                current_pose.header.frame_id = self.world_frame
                current_pose.pose.position.x = transform.transform.translation.x
                current_pose.pose.position.y = transform.transform.translation.y
                current_pose.pose.position.z = transform.transform.translation.z
                current_pose.pose.orientation = transform.transform.rotation

                feedback.current_pose = current_pose
                goal_handle.publish_feedback(feedback)

                dist = self.distance(
                    current_pose.pose.position,
                    desired_pose.pose.position
                )

                if dist <= 0.1:
                    self.get_logger().info('Target reached')
                    goal_handle.succeed()
                    result.success = True
                    self.cmd_vel_publisher.publish(stop_cmd)
                    return result
               
                cmd = Twist()
                cmd.linear.x = desired_pose.pose.position.x - current_pose.pose.position.x
                cmd.linear.y = desired_pose.pose.position.y - current_pose.pose.position.y
                cmd.linear.z = desired_pose.pose.position.z - current_pose.pose.position.z

                MAX_VEL = 0.5
                cmd.linear.x = max(min(cmd.linear.x, MAX_VEL), -MAX_VEL)
                cmd.linear.y = max(min(cmd.linear.y, MAX_VEL), -MAX_VEL)
                cmd.linear.z = max(min(cmd.linear.z, MAX_VEL), -MAX_VEL)
                
                self.cmd_vel_publisher.publish(cmd)

            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'Could not get transform: {e}')

            time.sleep(0.01)


    def wait_for_server(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available, mission aborted.')
            return

        self.get_logger().info('Action server ready.')
        self.mission_timer.cancel()
        self.send_mission_goal()

    def send_mission_goal(self):
        self.attempt_count += 1
        
        if self.attempt_count > self.max_attempts:
            self.get_logger().error(f'Max attempts ({self.max_attempts}) reached. Mission Aborted.')
            return

        self.get_logger().info(f'Starting Attempt {self.attempt_count}/{self.max_attempts}')

        try:
            target_transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'Could not get transform for {self.target_frame}: {e}')
            return

        desired_pose = PoseStamped()
        desired_pose.header.stamp = self.get_clock().now().to_msg()
        desired_pose.header.frame_id = self.world_frame
        
        desired_pose.pose.position.x = target_transform.transform.translation.x
        desired_pose.pose.position.y = target_transform.transform.translation.y
        desired_pose.pose.position.z = target_transform.transform.translation.z + 1.0

        desired_pose.pose.orientation = target_transform.transform.rotation 
        
        goal_msg = DroneControl.Goal()
        goal_msg.desired_pose = desired_pose
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future: Future):
        result = future.result().result
        
        if result.success:
            self.get_logger().info('SUCCESS! Target 1m above target_0 reached.')
        else:
            self.get_logger().warn(f'Attempt {self.attempt_count} Failed.')
            
            if self.attempt_count < self.max_attempts:
                self.get_logger().info(f'Retrying mission in 1 second...')
                time.sleep(1.0) 
                self.send_mission_goal()
            else:
                self.get_logger().error('All attempts used. Mission failed.')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    try:
        rclpy.init(args=args)
        executor = rclpy.executors.MultiThreadedExecutor()
        
        node = MissionControllerNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == '__main__':
    main()