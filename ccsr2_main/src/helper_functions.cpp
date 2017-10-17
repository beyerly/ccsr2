   bool cameraRunning = false;
   ros::V_string nodes;
   ros::init(argc, argv, "image_converter");

   while (not cameraRunning) {
      ros::master::getNodes(nodes);

      for (ros::V_string::iterator it = nodes.begin() ; it != nodes.end(); it++) {
         const std::string& info = *it;
         if (info == "/camera/camera") {
            cameraRunning = true;
         }
      }
      ROS_INFO("Waiting for camera node");
      sleep(1);
   }
