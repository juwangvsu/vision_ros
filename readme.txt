R_inspire_21.urdf
	currently 12 joints: 4 for thumb, 2 each the rest. no wrist joint

tbd: map rokoko joint to this to test in (1) rviz, (2) isaac sim

ros2_ws/src:
	ln -sn /workspace/urdf_r6bot/allegro_hand/allegro_hand_description_ros2/
	git clone this repo
	source install/setup.bash; rviz2; select world frame
	ros2 run joint_state_publisher_gui joint_state_publisher_gui
	
-----------FAQ ------------------------------------
(1) [launch]: Caught exception in launch (see debug for traceback): executable '[<launch.substitutions.text_substitution.TextSubstitution object at 0x76efe0799960>]' not found on the PATH

	fix: apt install ros-humble-xacro

(2) turtlebot ...
	fix: apt install ros-humble-turtlebot3-manip*
	
(3) other pkgs
	apt install ros-humble-rqt-robot-steering
	apt install ros-humble-joint-state-publisher-gui
