# liovil_sam
This is a repository for improving VILSAM algortihm by combining it with LIOSAM. 

Currently, VILSAM and LIOSAM run using a single launch file. Moreover, VILSAM uses gtsam 4.0.2, the original VILSAM works with gtsam 4.0.2. I have chosed gtsam 4.0.2 because of LIOSAM, its files were uses as basis for upgrading VILSAM.

To run the liovil_sam:
  - Download the code into /catkin_ws/srs/liovil_sam
  - build it: catkin_make
  - roslaunch liovil_sam run.launch
    
The code is organized as follow:
  - liovil_sam/
    - Config/
      - params.yaml  --> contains configurable parameter for: visual_frontend, vio, lidar_mappin, and liosam
    - include/ (TODO most of the header files for VILSAM are a copy with minor modifications. They could be merged in single header files)
      - lidar_mapping/ --> header files for lidar mapping (loam)
      - nanoflann/     --> extra header files for visual_frontend
      - range-tree/    --> extra header files for vio (visual inertial odometry) 
      - vio/           --> header files for vio
      - visual_fronend/  --> header files for visual_frontend
      - utility.h  --> liosam include file
    - launch/
      - run.launch  --> main launch file
      - include/    --> extra launcher files
    - msg/    --> message files used by VILSAM and LIOSAM
    - scripts/ --> python scripts implemented by VILSAM (They are not used)
    - src/
      -  lidar_mapping/       --> surce files for performing loam 
      -  liosam/              --> source files used by LIOSAM 
      -  vio/                 --> source files for performing the visual inertial odometry 
      -  visual_frontend/     --> source files for retrieve stereo matches
    - srv/ --> services used by LIOSAM for saving the reconstructed map
