# ugv_packages

  

### Description:

UGV ROS packages for UAV-UGV collaboration for warehouse inventory missions found in https://github.com/RafaelaChaffilla/aerial_ground_robotic_system_for_warehouse_inventory. These packages were designed for ROS Melodic.
A description of each package is given in [Documentation](#documentation).

### Installation:
Perform the following steps to install these packages in your catkin_workspace:

    cd catkin_ws/src
    git clone --recurse-submodules https://github.com/RafaelaChaffilla/ugv_packages.git
    cd ../
    catkin_make

To integrate the localization and warehouse simulation, you can include the following spinet within a desired launch file:

    <include file="$(find ugv_localization)/launch/ugv_localization.launch">
	    <arg  name="simulation"  value="true"/>
	    <arg  name="config"  value="8_areas_full_bottom"  />
	    <arg  name="robot_name"  value="rovid"  />
	    <arg  name="init_x"  value="-9.5"  />
	    <arg  name="init_y"  value="-3.5"  />
	    <arg  name="init_z"  value="0.2"  />
	    <arg  name="init_th"  value="0.0"  />
    </include>

where the parameters indicate:
 - *simulation*: True if the gazebo and sensor's noise nodes should be launched;
 - *config*: warehouse world configuration (8_areas_full_bottom, 8_areas_half_bottom or 8_areas_empty_bottom);
 - *robot_name*: name of robot, used for prefixes;
 - *init_x, init_y, init_z, init_th:* UGV's initial x, y, z and yaw positions in map frame;

This file launches the gazebo simulation with the intended warehouse world model, launches the sensors noise simulation and the localization nodes, such that the transform from the map to the ugv frame is published in the /tf tree. 

### Documentation<a name="documentation"></a>:
 1. **ekf_ugv**: The Extended Kalman Filter module fuses the UGV's sensors to obtain the robot's position estimate. It fuses IMU, wheel odometry and scan to map matching measurements. 
	 - **Parameters:**
		 - *imu_topic*: Topic where IMU data is published. (sensor_msgs/IMU);
		 - *odom_topic*: Topic where wheel odometry data is published. (nav_msgs/Odometry);
		 - *position_topic*: Topic where the scan to map matching output is published. (nav_msgs/Odometry);
		 - *publish_topic*: Topic where EKF output will be published.  (nav_msgs/Odometry);
		 - *publish_tf*: Boolean indicating the EKF estimate should publish the transformation from the map to the UGV frame in the /tf tree;
		 - *base_frame*: string with UGV's base frame name in /tf;
		 - *odom_frame*: string with UGV's odom name in /tf;
		 - *init_x, init_y, init_th:* EKF's initial x, y, and yaw variables;
		 - *process_noise_x, process_noise_y, process_noise_th, process_noise_vx, process_noise_vy, process_noise_vth, process_noise_acc_x:* Entries of the diagonal of Q for each state variable;
		 -  *init_noise_P_value*: Diagonal entry for all variables of $P_0$.
		 - *sensor_timeout*: If the time between the last received message and current time from sensor is higher than *sensor_timeout*, discard all further readings from this sensor;
	 - **Publishes to:**
		 - "/rovid/ekf_ugv" or *publish_topic* defined in parameters: publishes the estimated UGV position in the map frame (nav_msgs/Odometry).
 2. **noise_simulator**: realistic noise simulator for IMU and Wheel odometry sensors.
	 - **IMU Parameters:**
		 - *angular_velocity_stddev, linear_acceleration_stddev, orientation_stddev*: Standard deviation of white gaussian noise to be added to the gyroscope, accelerometer and compass outputs (rad/s, m/s$^2$, rad);
		 - *gaussian_noise*: Bias of white gaussian noise to be added to IMU's output;
		 - *max_meas_ang, max_meas_lin*: Maximum measurable angular rate and linear acceleration by IMU (rad/s, m/s$^2$);
		 - *min_meas_ang, min_meas_lin*: Minimum measurable angular rate and linear acceleration by IMU (rad/s, m/s$^2$);
		 - *no_noise_imu_topic*: topic that publishes the perfect IMU data to be transformed. The node will subscribe to this topic. (sensor_msgs/IMU);
	 - **Wheel Odometry Parameters:**
		 - *wheel_radius*: Radius of the used wheels;
		 - *wheel_dist*: Track distance between wheels (considering differential drive configuration);
		 - *wheel_max_accel*: Wheel's maximum acceleration (m/s$^2$);
		 - *speed_stddev*: Standard deviation of white gaussian noise to be added to wheel odometry's linear speed output (m/s);
		 - *gaussian_noise*: Bias of white gaussian noise to be added to wheel odometry's linear speed output (m/s);
		 - *radius_bias_left, radius_bias_right, track_width_bias*: Error bias of left and right wheel's radius and track's width (m);
		 - *slipage_lin, slipage_ang*: Linear velocity's and angular velocity's slip ratios;
		 - *init_x, init_y, init_th:* Initial x, y, and yaw states of UGV;
	 -  **Publishes to:**
		 - "/rovid/idmind_motors/odom": Simulated noisy wheel odometry measurements (nav_msgs/Odometry);
		 - "/rovid/idmind_imu/imu": Simulated noisy IMU measurements (sensor_msgs/IMU).
 3. **s2mm_ugv**: Package that performs scan to map matching.
	 - **Parameters:**
		 - *scan_topic*: Topic that publishes the merged laser scans output (sensor_msgs/LaserScan);
		 - *odom_topic*: Topic that publishes the ekf output (nav_msgs/Odometry);
		 - *map_topic*: Topic that publishes the occupancy grid map (nav_msgs/OccupancyGrid);
		 - *publish_to_topic*: Topic where the s2mm result should be published (nav_msgs/Odometry);
		 - *init_x, init_y, init_th:* Initial x, y, and yaw states of UGV;
		 - *frequency*: output frequency of the s2mm algorithm;
	 - **Publishes to:**
		 - "/rovid/s2mm_odom" or *publish_to_topic* defined in parameters: publishes the estimated UGV position in the map frame from the s2mm algorithm (nav_msgs/Odometry).
 4. **ugv_localization**: Package that encapsulates the full localization algorithm in a single launch file. It also includes the gazebo world files for the different warehouse configurations and their respective maps.