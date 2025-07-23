% Create a ROS 2 node in MATLAB
ros2("domain", 0) % Set the ROS2 domain ID (default is 0)
ros2Node = ros2node("my_ros2_node");


% Initialize the ROS 2 node
ros2Node.initialize();

% Create a publisher for a topic
ros2Publisher = ros2publisher(ros2Node, "my_topic", "std_msgs/String");

% Publish a message to the topic
msg = ros2message(ros2Publisher);
msg.Data = "Hello, ROS 2!";
send(ros2Publisher, msg);
