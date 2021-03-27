# Long Term Localization
Pole-like Objects Mapping and Long-Term Robot Localization in Dynamic Urban Scenarios

# 1 Settings
- System: ubuntu 16.04
- C++ version: c++17
- g++/gcc >= 7.0
    - [How to upgrade your g++ and gcc?](https://www.zybuluo.com/iStarLee/note/1260368)
- cmake >= 3.10
    - [upgrade your cmake](https://www.zybuluo.com/iStarLee/note/1739997)

# 2 Build
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/HITSZ-NRSL/long-term-localization.git
git clone https://github.com/lisilin013/common.git
git clone https://github.com/lisilin013/third_parities.git

cd ..
# When you build this ws for the first time, it may take a long time, be patient please.
catkin build
```


