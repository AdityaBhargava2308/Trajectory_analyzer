## Steps for using the package.

  ###  step 1 - Build the package in your workspace:
        cd workspace
        colcon build --packages-select trajectory_analyzer --symlink-install
        source install/setup.bash

    ### step 2 - Publish and save the trajectory during robot's navigation/mapping/path_planning.
        Terminal 1 - launch the gazebo world/bringup file for the robot (following is the example for turtlebot):
            ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        
        Terminal 2 - launch the navigation/mapping/pathplanning file foe the robot (following is the example for navigation of turtlebot):
            ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

        Terminal 3 - run the trajectory publisher and saver node:
            ros2 run trajectory_analyzer trajectory_writer 

        Terminal 4 - call the service with file name and time duration as parameters to save the trajectory as a csv file in the csv folder of the package:
            ros2 service call /save_trajectory trajectory_analyzer/srv/SaveTrajectory "{filename: 'trajectory1.csv', duration: 20.0}"

   ###  step 3 - Read the saved trajectory and visualise it on rviz wrt odom frame (give the trajectory file name as parameter to view that particular trajectory):
            ros2 launch trajectory_analyzer TrajectoryViewer.launch.py trajectory_file:=trajectory1.csv


 ### Pseudo code for publishing and saving the trajectory.

    BEGIN TrajectorySaverNode

        DECLARE and GET parameter "marker_topic"
        SUBSCRIBE to "/odom" (odom_callback)
        PUBLISH to marker_topic_
        CREATE service "save_trajectory" (save_trajectory_callback)
        START timer to call publish_marker_array every 0.05s
        SET start_time_

        FUNCTION odom_callback(msg)
            EXTRACT position, orientation, timestamp
            STORE in trajectory_data_
        END FUNCTION

        FUNCTION publish_marker_array()
            CREATE MarkerArray
            FOR each data in trajectory_data_
                CREATE and CONFIGURE Marker
                ADD Marker to MarkerArray
            PUBLISH MarkerArray
        END FUNCTION

        FUNCTION save_trajectory_callback(request, response)
            GET filename, duration
            COMPUTE valid data within duration
            IF no valid data:
                SET response (failure, message)
                RETURN
            TRY
                SAVE data to CSV
                SET response (success, message)
            CATCH error
                SET response (failure, message)
                LOG error
            LOG "Service completed"
        END FUNCTION

    END TrajectorySaverNode

    MAIN FUNCTION
        INIT ROS 2
        CREATE and SPIN TrajectorySaverNode
        SHUTDOWN ROS 2
    END MAIN FUNCTION

Pseudo code for reading the trajectory.

    BEGIN TrajectoryVisualizerNode  
        SET trajectory_file_ from parameter  
        SET file_path_ from package directory  
        CREATE CSV folder if missing  
        INIT publisher "/visualized_trajectory"  

        IF loadTrajectoryData() THEN  
            START timer (1s) → publishTrajectory()  
        ELSE  
            LOG error, EXIT  
    END  

    FUNCTION loadTrajectoryData()  
        OPEN file_path_, RETURN false if failed  
        SKIP header  
        PARSE each line → STORE valid 8-column rows in trajectory_data_  
        RETURN true  
    END  

    FUNCTION publishTrajectory()  
        INIT MarkerArray  
        FOR every 10th row in trajectory_data_  
            CREATE ARROW Marker (pose, scale, color) → ADD to MarkerArray  
        ENDFOR  
        PUBLISH MarkerArray  
    END  

    MAIN  
        INIT ROS 2, RUN Node, SHUTDOWN  
    END  
