# liovil_sam

LIOSAM dependencies:

ROS (tested Melodic)

    sudo apt-get install -y ros-kinetic-navigation
    sudo apt-get install -y ros-kinetic-robot-localization
    sudo apt-get install -y ros-kinetic-robot-state-publisher

gtsam (Georgia Tech Smoothing and Mapping library)
    wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
    cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
    cd ~/Downloads/gtsam-4.0.2/
    mkdir build && cd build
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
    sudo make install -j8
    
VILSAM dependencies:
  Install velodyne driver:
    sudo apt-get install ros-melodic-velodyne

  Install opencv and checkout 3.3.1 tag
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 3.3.1 -b v3.3.1
    cd ..
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib
    git checkout 3.3.1 -b v3.3.1
    cd ..

  Now build it
    
    mkdir opencv_build; cd opencv_build; mkdir ../opencv_install
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=../opencv_install \
            -D INSTALL_C_EXAMPLES=ON \
            -D INSTALL_PYTHON_EXAMPLES=ON \
            -D WITH_TBB=ON \
            -D WITH_V4L=ON \
            -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-py3/lib/python3.5/site-packages \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON ../opencv

    make -j8
    make install

  Update CMakeLists.txt in liovil_sam/src/CMakeLists.txt
    Change opencv line to
    find_package(OpenCV 3.3.1 REQUIRED HINTS <point_to_the_opencv_installation folder>)

Similar dependencies
  Install Eigen3 (Usually is already installed in Linux)
  
Ceres solver is not used in the current implementation. Ceres is used by LidarMapping node, yet loam is performed by LIOSAM.  
