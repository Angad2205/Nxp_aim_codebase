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

EDGE VECTOR - 
The provided code defines a ROS 2 node named EdgeVectorsPublisher for processing camera images to detect road edges and publish edge vectors. It utilizes the sensor_msgs and synapse_msgs packages to handle image data and publish detected edge vectors.
Key Components:

    Imports and Constants:
        Imports necessary libraries including rclpy, cv2, and numpy.
        Defines constants for colors, image processing parameters, and vector magnitude threshold.

    EdgeVectorsPublisher Class:
        Initialization (__init__):
            Subscribes to a compressed camera image topic (/camera/image_raw/compressed).
            Publishes edge vectors on /edge_vectors, and debug images on /debug_images/thresh_image and /debug_images/vector_image.
        publish_debug_image Method:
            Converts an image to a compressed format and publishes it for debugging.
        get_vector_angle_in_radians Method:
            Calculates the angle of a vector in radians relative to the x-axis.
        compute_vectors_from_image Method:
            Processes a binary threshold image to detect contours and compute vectors.
            Discards vectors with insufficient magnitude and computes their distance from the rover.
        process_image_for_edge_vectors Method:
            Converts the image to grayscale and applies a threshold to isolate edges.
            Processes the threshold image to detect road edge vectors and sorts them by distance from the rover.
            Selects one vector each from the left and right halves of the image, if available.
            Publishes debug images for visualization.
        camera_image_callback Method:
            Receives and decodes a camera image.
            Processes the image to detect road edge vectors and publishes the vectors.

    main Function:
        Initializes ROS 2, creates an instance of EdgeVectorsPublisher, and enters the ROS event loop.
        Cleans up the node and shuts down ROS 2 upon completion.

Purpose:

The node is designed to detect and publish vectors representing road edges using camera images. These vectors can be used for autonomous navigation and obstacle detection in robotic systems.

Object Recog - For object Recog we Use Yolov5 model 
