#!/usr/bin/zsh
source devel/setup.zsh

# 使用RangeNet处理刚才保存的点云数据
cd src/other_pkgs/lidar-bonnetal/train/tasks/semantic
echo "run RangeNet++..."
rm -rf  ~/logs/
python3 infer.py -d ~/offline_process/ -m ../../../model/trunk_pole -l ~/logs/

# 把RangeNet++预测的结果文件拷贝到velodyne文件夹下
echo "copy predictions to labels..."
cp -r ~/logs/sequences/00/predictions ~/offline_process/sequences/00/labels

# 把处理后的关键帧语义点云和对应的关键帧位姿pose.txt写入 semantic.bag
# 使用point laber矫正部分杆子树干分割错误
echo "write semantic cloud and pose to rosbag..."
cd ~/offline_process/sequences/00/
cp semantic.bag semantic_copy.bag
rosrun long_term_relocalization semantic_bag_writer_main --dataset_dir ~/offline_process/sequences/00/

echo "offline process done"
