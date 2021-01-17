# Long Term Relocalization

# 1 Dependencies
- System: ubuntu 16.04
- C++ version: c++17
- g++/gcc >= 7.0
    - [How to upgrade your g++ and gcc?](https://www.zybuluo.com/iStarLee/note/1260368)
- cmake >= 3.10
    - [upgrade your cmake](https://www.zybuluo.com/iStarLee/note/1739997)
- clang-format-6.0
```bash
sudo apt-get install clang-format-6.0
# 在vscode打开工程后，在根目录下执行如下命令
/usr/bin/clang-format-6.0  -style=llvm -dump-config > .clang-format
# vscode 安装格式化插件，设置clang-format路径，自行google
```
- Third Parties
    - gtest[https://github.com/google/googletest]
    ```bash
    git clone https://github.com/google/googletest.git -b release-1.10.0
    cd googletest        # Main directory of the cloned repository.
    mkdir build          # Create a directory to hold the build output.
    cd build
    cmake ..             # Generate native build scripts for GoogleTest.
    ```
    - ros-kinetic
    - absl(compiled with c++17 settings)
    ```bash
    # 1. add `set(CMAKE_CXX_STANDARD 17)` to CMakeLists.txt of abseil-cpp
    # 2. compile 
    mkdir build
    cd build
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
    make
    sudo make install
    ```
    - libnabo(https://github.com/ethz-asl/libnabo)
    ```bash
    git clone https://github.com/ethz-asl/libnabo
    cd libnabo
    mkdir build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/usr ..
    make
    sudo make install
    ```

# 2 Long Term Relocalization 数据处理流程
## 2.1 语义点云处理
- 跑lio_sam 保存keyframe, pose, timestamp
    - 参数调整注意
    `src/third_parties/lio_sam/config/params.yaml` 中`savePCD`打开；`savePCDDirectory`设置保存位置 `"/home/YOUR_USER_NAME/offline_process/sequences/00/"`；`surroundingkeyframeAddingDistThreshold`设置关键帧之间的距离。以`savePCDDirectory = ~/offline_process/`为例。
    - 在三个终端内先后执行以下命令
    ```bash
    # terminal1
    roslaunch lio_sam run.launch #会自动生成savePCDDirectory文件夹

    # terminal2
    ./sh/record.sh
    
    # terminal3
    rosbag play xx.bag --clock # 一定要带clock
    ```
注意在lio_sam回环之前关闭，因为回环会造成位姿误差，导致建图有重影，这个问题未来可以考虑使用`src/third_parties/interactive_slam`来解决

- 离线后处理
    ```bash
    ./sh/offline_process.sh
    ```
    至此我们的semantic.bag中有了以下topic:
    - /lio_sam/mapping/odometry 用于长航时定位和NDT定位
    - /navgps 用于定位的evaluation
    - /navodom 保留
    - /imu 用于NDT定位预测
    - /odom 用于NDT定位预测

- 使用`pcl_viewer`打开生成的全局点云
```bash
pcl_viewer cloudGlobal.pcd
```
如果发现打不开，报错如下
```
> Loading cloudGlobal.pcd [pcl::PCDReader::read] Number of points read (2801499) is different than expected (16759678)
```
采用如下[方法](http://www.pcl-users.org/Can-t-read-pcd-file-td4044649.html)解决:
使用编辑器打开该pcd文件，修改`POINTS`为终端提示的2801499，如下所示
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 2801499
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 2801499
DATA ascii
```
## 2.2 Evalution
- 评估轨迹
```
cd PATH/long_term_slam/src/long_term_slam/data

evo_traj tum localization.txt --ref=gt.txt -p --plot_mode=xy --align
```
- 评估误差
```
evo_ape tum ground_true.txt proposed.txt -a -p --plot_mode=xy
```
- 评估重定位距离
P99
- 评估全局聚类数量（x）与重定位距离（y）
    - 路径周围的聚类数量，
- 聚类缺失到什么程度，重定位会失败（20m之内无法重定位）
- 重定位的得分
    - 聚类匹配数量
    - 参与匹配数量
    - 匹配得到的距离
    - 匹配中聚类物体的被观测次数


## 2.3 Run long term relocalization
### 2.3.1 mapping
```
# terminal 1
cd /home/nrsl/offline_process/sequences/00
rosbag play semantic-1.bag --clock
# terminal 2
roslaunch long_term_relocalization mapping.launch # modify `mode` to localization
# terminal 3: save map
rosrun long_term_relocalization save_cluster_map
```

save map code details:
```cpp
// src/long_term_relocalization/src/relocalization/relocalization.cc
void Relocalization::SaveMapCmdCallback(const std_msgs::Int32ConstPtr &msg) {
  LOG(INFO) << GREEN << "msg data: " << msg->data << COLOR_END;
  if (msg->data == 1) {
    semantic_cluster_map_->mutable_cluster_manager()->SaveClustersToFile("/tmp/clusters_map.bin");
  }
}
```

### 2.3.2 relocalization
```
# terminal 1
cd /home/nrsl/offline_process/sequences/00
rosbag play semantic-2.bag --clock
# terminal 2
roslaunch long_term_relocalization relocalization.launch # modify `mode` to relocalization
```
