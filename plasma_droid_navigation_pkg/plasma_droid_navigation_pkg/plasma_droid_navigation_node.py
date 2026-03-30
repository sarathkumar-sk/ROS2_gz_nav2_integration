import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

class PlasmaNode(Node):
    def __init__(self):
        super().__init__('plasma_droid_navigation_node')

        self._initial_pose_topic = '/initialpose'
        self._nav_action_topic = '/navigate_to_pose'
        self.timeout_sec = 60.0
        self.retry_delay_sec = 1.0

        self.goal_handle: ClientGoalHandle = None
        self.start_time = None
        self.current_goal_pose = None
        self.navigation_active = False

        self.publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=self._initial_pose_topic,
            qos_profile=1
        )
        
        self.action_client = ActionClient(self, NavigateToPose, self._nav_action_topic)
        self.timer = self.create_timer(0.01, self.control_loop)
        self._wait_for_amcl_subscriber()

    def _wait_for_amcl_subscriber(self):

        subscription_count = 0
        while subscription_count < 1:
            subscription_count = self.publisher.get_subscription_count()
            print(f"Waiting for Nav2 to subscribe")
            print(f"Subscriber count is {subscription_count}.")
            time.sleep(1)
        self.get_logger().info("Nav2 is ready to receive initial pose.")

    def control_loop(self):
        
        if self.navigation_active and self.start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            if elapsed_time > self.timeout_sec:
                self.get_logger().warn(f"Timeout of {self.timeout_sec}s exceeded. Initiating cancellation.")
                self.cancel_current_goal()

    def send_initial_pose_with_covariance(self):
        self.get_logger().info(f"Publishing initial pose to: {self._initial_pose_topic}")

        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = self.get_clock().now().to_msg()
        pwcs.header.frame_id = 'map'

        pwcs.pose.pose.position.x = 0.0
        pwcs.pose.pose.position.y = 0.0
        pwcs.pose.pose.position.z = 0.0
        pwcs.pose.pose.orientation.w = 1.0

        pwcs.pose.covariance[0] = 0.25   # X
        pwcs.pose.covariance[7] = 0.25   # Y
        pwcs.pose.covariance[14] = 0.25  # Z
        pwcs.pose.covariance[35] = 0.068 # Yaw

        self.publisher.publish(pwcs)

    def send_goal_async(self, desired_pose: Pose, behaviour_tree: str = "") -> None:
        self.current_goal_pose = desired_pose
        
        if not self.navigation_active:
             self.start_time = self.get_clock().now()
             self.navigation_active = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = desired_pose
        goal_msg.behavior_tree = behaviour_tree

        if not self.action_client.server_is_ready():
            self.get_logger().info('Action server not available, waiting...')
            self.action_client.wait_for_server()

        self.get_logger().info(f'Sending navigation goal...')
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.action_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected. Retrying...')
            time.sleep(self.retry_delay_sec) 
            self.send_goal_async(self.current_goal_pose)
            return

        self.get_logger().info('Goal accepted by server.')
        self.goal_handle = goal_handle
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.action_result_callback)

    def action_feedback_callback(self, feedback_msg: NavigateToPose.Feedback) -> None:
        feedback = feedback_msg.feedback

    def cancel_current_goal(self):
        if self.goal_handle:
            self.get_logger().info("Cancelling goal...")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_response_callback)
        self.navigation_active = False

    def cancel_response_callback(self, future):
        self.get_logger().info("Goal cancellation request processed.")

    def action_result_callback(self, future: Future) -> None:
        result = future.result()
        status = result.status
        
        if status == 4: # STATUS_SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
        elif status == 5: # STATUS_CANCELED
            self.get_logger().info('Goal was canceled.')
        elif status == 6: # STATUS_ABORTED
            self.get_logger().error('Goal aborted by Nav2.')
        
        self.navigation_active = False
        self.start_time = None

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlasmaNode()
     
        node.send_initial_pose_with_covariance()
        
        desired_pose = Pose()
        desired_pose.position.x = 22.00
        desired_pose.position.y = -7.10
        desired_pose.orientation.w = 1.0
        time.sleep(0.5) 
        node.send_goal_async(desired_pose)

        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    

if __name__ == '__main__':
    main()