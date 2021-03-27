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

## 2.1 Install dependencies
Please take a look the [README](https://github.com/HITSZ-NRSL/long-term-localization/blob/master/src/common/README.md) in `long-term-localization/src/common` folder.

## 2.2 Build
```
git clone https://github.com/HITSZ-NRSL/long-term-localization.git
cd long-term-localization/src
git clone https://github.com/lisilin013/third_parities.git

cd ..
# When you build this ws for the first time, it may take a long time, be patient please.
catkin build
```


# 3 Run
## 3.1 Localization
Modify the following code in config file `src/long_term_relocalization/config/long_term_relocalization_params.yaml`
change to `relocalization` mode.
```
relocalization:
  mode: relocalization
```

```
# copy pole cluster map to another folder.
cp src/long_term_relocalization/maps/clusters_map.bin /tmp/

# launch relocalization nodes.
roslaunch long_term_relocalization relocalization.launch
```


## 3.2 Mapping
Modify the following code in config file `src/long_term_relocalization/config/long_term_relocalization_params.yaml`
change to `localizaion` mode.
```
relocalization:
  mode: localization
```

```
# launch mapping nodes.
roslaunch long_term_relocalization mapping.launch

# when you want to save pole cluster map, open another terminal and run the following cmd.
rosrun long_term_relocalization save_cluster_map 
```

