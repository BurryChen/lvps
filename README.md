
# lvps
## lidar_visual_slam
## An slam with Lidar-visual fusion and graph optimization

## 1. node introduction
### 1.1 ndt_omp & ndt_pca 
Follow [ndt_omp Repositoty](https://github.com/koide3/ndt_omp,https://github.com/BurryChen/ndt_omp).
### classical ndt, weighted ndt

### 1.2 lidar_odometry 

### 1.3 global_graph
### A global graph lidar slam using visual loop dectection


## 2 Requirements
***lvps*** requires the following libraries:
- OpenMP
- PCL
- g2o
- suitesparse
- Sophus
- OpenCV 
- DBoW3
- A-LOAM

### 2.0. base
```
    sudo apt-get install -y --no-install-recommends \
         libsuitesparse-dev \
         ros-$ROS_DISTRO-geodesy ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-nmea-msgs ros-$ROS_DISTRO-libg2o
```

### 2.1. **Ceres Solver** installed to /usr/local/libname
```
    sudo apt-get install -y --no-install-recommends \
         cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev 
```
```  
    #下载 ceres-solver-1.14.0.tar.gz并解压
    # http://www.ceres-solver.org/installation.html#linux
    cd ceres-solver-1.14.0
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/ceres-solver ..
    make -j12
    sudo make install
   // cd ..&& sudo rm -r ./build
```

### 2.2. **g2o** installed to default path
```
    sudo apt-get install ros-$ROS_DISTRO-libg2o
```

### 2.3. **Sophus** installed to /usr/local/libname
```
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout a621ff
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/Sophus ..
    make -j12
    sudo make install
    //cd ..&& sudo rm -r ./build
```

### 2.4. **DBow3** installed to /usr/local/libname
**OpenCV**
```
   #version default is 3.2.0 in ubuntu18.04, 4.5.0 in ubuntu20.04
   sudo apt install libopencv-dev python3-opencv
```

**DBow3**
```
    git clone https://github.com/rmsalinas/DBow3
    cd DBow3
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/DBow3 ..
    make -j12
    sudo make install
    // cd ..&& sudo rm -r ./build
```

### 2.5. **A-LOAM**
Follow [A-LOAM Repositoty](https://github.com/BurryChen/A-LOAM).

```
cd workspace
cd src
git clone https://github.com/BurryChen/A-LOAM.git
cd A-LOAM
#set status on 1df43a1 in ubuntu18.04, master in ubuntu20.04
git checkout 1df43a1 
cd ../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="aloam_velodyne" --build='./build/A-LOAM' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install  install
```

### 2.6. **evo**
Follow [evo Repositoty](https://github.com/MichaelGrupp/evo).


## 3. Build 
Clone the repository and catkin_make:

```
    cd ~/lvps_ws/src
    git clone https://github.com/BurryChen/lvps.git
    cd ../
catkin_make -DCATKIN_WHITELIST_PACKAGES="lvps" --build='./build/lvps' -DCATKIN_DEVEL_PREFIX=./devel -DCMAKE_INSTALL_PREFIX=./install  install
    source ~/lvps_ws/devel/setup.bash
```
Download [lvps_data](https://drive.google.com/drive/folders/1YTTXD4QmFrA6LQrfslRlUzATxYG0u32R?usp=sharing) to YOUR_DATASET_FOLDER.

## 4. Example dlo_lfa_ggo_kitti

Download [velo_img_04](https://drive.google.com/file/d/1PD9RHqhYCuFaXSo95ARs3ALMUDD9y5mn/view?usp=sharing) to YOUR_DATASET_FOLDER.
### 4.1 dlo_lfa_kitti
```
# realtive path
roslaunch lvps dlo_lfa_kitti.launch  calib_file:='$(find lvps)/config/kitti_calib/calib04-12.txt'     odom_file:='$(find lvps)/data/kitti_lv_dlo_lfa/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='$(find lvps)/data/kitti_lv_dlo_lfa'
rosbag play --clock './src/lvps/data/velo_img_04.bag'    -r 1.0
```

```
    roslaunch lvps dlo_lfa_kitti.launch  calib_file:='/home/chenshoubin/code_ws/lvps_ws/src/lvps/config/kitti_calib/calib04-12.txt'   odom_file:='/home/chenshoubin/code_ws/lvps_ws/src/lvps/data/kitti_lv_dlo_lfa/dlo_lfa_global/data/KITTI_04_odom.txt'  seq:=04  lfa_output_path:='/home/chenshoubin/code_ws/lvps_ws/src/lvps/data/kitti_lv_dlo_lfa'
    rosbag play --clock '/home/chenshoubin/data/data_source_KITTI/velostereobag/velo_img_04.bag'    -r 1.0

    #cpp ./evaluate_odometry_seq '/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa/aft_mapped_to_init_high_frec_file' 04
    #seq 04 (t_avg,r_avg)=(0.003118,0.000026)
```

### 4.2 dlo_lfa_ggo_kitti
```
# realtive path
roslaunch lvps dlo_lfa_ggo_kitti.launch  calib_file:='$(find lvps)/config/kitti_calib/calib04-12.txt'     odom_file:='$(find lvps)/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='$(find lvps)/data/kitti_lv_dlo_lfa_ggo'
rosbag play --clock './src/lvps/data/velo_img_04.bag'    -r 1.0
```

```
    roslaunch lvps dlo_lfa_ggo_kitti.launch  calib_file:='/home/chenshoubin/lvps_ws/src/lvps/config/kitti_calib/calib04-12.txt'     odom_file:='/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' seq:=04  lfa_output_path:='/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo'
    rosbag play --clock '/home/chenshoubin/data/data_source_KITTI/velostereobag/velo_img_04.bag'    -r 1.0
    
    rosservice call /global_graph/dump "destination: '/home/chenshoubin/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_06'  "
    rosservice call /global_graph/save_map "{resolution: 0.05, destination: '/home/chenshoubin/data/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_06/map.pcd'}"

    evo_traj kitti '/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/KITTI_04_odom.txt' '/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo/aft_mapped_to_init_high_frec_file/data/KITTI_04_odom.txt'   '/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_04/ggo_wf_odom.txt'      --plot_mode=xz  --ref='/home/chenshoubin/data/data_source_KITTI/gt/04.txt'   -p --save_plot  '/home/chenshoubin/data/lvps_kitti/kitti_lv_dlo_lfa_ggo/dlo_lfa_global/data/dump_04/ggo_wf_odom.pdf'
```