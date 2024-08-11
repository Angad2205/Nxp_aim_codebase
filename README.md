LINEFOLLOWER CODE INTRO - 
    Initialization:
        The LineFollower class, derived from rclpy.Node, initializes several publishers and subscriptions:
            Subscribes to /edge_vectors for edge detection data.
            Publishes to /cerebri/in/joy to send movement commands.
            Subscribes to /traffic_status for traffic-related information.
            Subscribes to /scan for LIDAR data.

    Movement Control:
        The rover_move_manual_mode method sends movement commands (speed and turn) to the rover.
        The reverse_direction method is used to reverse the rover’s direction.

    Callbacks:
        edge_vectors_callback: Handles edge detection data to adjust speed and turning based on the number and position of detected vectors. It also checks for stop signs and ramps.
        traffic_status_callback: Updates the traffic status based on received messages.
        lidar_callback: Processes LIDAR data to detect obstacles and ramps. It calculates the minimum distances to obstacles in various directions (front left, front right, side left, side right) and adjusts the rover’s speed and turning direction based on detected obstacles and ramps.

    Obstacle and Ramp Detection:
        The lidar_callback method handles complex obstacle avoidance logic. It detects obstacles by checking distances against predefined thresholds and determines the best turning direction based on where obstacles are detected.
        It also detects ramps by analyzing LIDAR data patterns over time.

    Main Function:
        The main function initializes the ROS 2 client library, creates an instance of LineFollower, and starts spinning the node to process incoming data and execute callbacks.

Overall, the code is designed to autonomously navigate a rover by processing sensor data and making real-time adjustments to its movement based on detected obstacles, traffic signals, and terrain conditions.
