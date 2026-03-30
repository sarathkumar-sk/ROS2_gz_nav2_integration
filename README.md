This project demonstrates the integration between the Gazebo Sim environment and ROS2 Humble/Jazzy. It covers the creation of custom communication bridges, an action-based drone controller using tf2 transforms, and high-level autonomous navigation using the Nav2 stack.

Core Functionality:

    bridge_package: The communication layer.

        Implements a custom launch configuration using ros_gz_bridge.

        Maps Gazebo topics to ROS2 topics, including velocity commands (cmd_vel) and coordinate transformations (/tf) for multiple entities (drone and targets).

    controller_package: The flight control system.

        TF Listener: Real-time pose tracking of the drone relative to the simulation world.

        DroneControl Action Server: Manages complex movement goals. It calculates Euclidean distance to targets and executes velocity commands to hover precisely 1 meter above a dynamic target (target_0).

        Precision: Implements feedback loops to maintain a 0.1 m tolerance with a 5-second timeout safety protocol.

    navigation_package: The autonomous navigation stack.

        Nav2 Integration: Interfaces with the TurtleBot4 Nav2 stack.

        Initial Pose Estimation: Automates the "2D Pose Estimate" by publishing a covariance-aware initial pose to the localization system.

        Goal Pursuit: Communicates with the Nav2 Action Server to navigate the robot to specific map coordinates with a 0.5 m tolerance within a strict 60-second time limit.

Technical Stack: 

    Framework: ROS2 (Ament Python)

    Simulation: Gazebo Sim / Ignition

    Navigation: Nav2 (Navigation 2 Stack)

    Key Libraries: tf2_ros (Transformations), rclpy (Python Client Library), geometry_msgs, nav2_msgs.

    Key Concepts: Covariance Matrices for Pose Estimation, Action Servers, Bi-directional Gazebo Bridges.
