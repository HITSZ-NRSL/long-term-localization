# common

## 1 Dependencies
- System: ubuntu 16.04
- C++ version: c++17
- g++/gcc >= 7.0
    - [How to upgrade your g++ and gcc?](https://www.zybuluo.com/iStarLee/note/1260368)
- cmake >= 3.10
    - [upgrade your cmake](https://www.zybuluo.com/iStarLee/note/1739997)
- clang-format-6.0
```bash
sudo apt-get install clang-format-6.0
# cd your workspaces floder
/usr/bin/clang-format-6.0  -style=llvm -dump-config > .clang-format
# install the clang-format plugin in vscode extensions and then set the path of clage-formate, if you don't know how, google will help you.
```
- Third Parties
    - gtest[https://github.com/google/googletest](or use offline mode in "third_parties/googletest")
    ```bash
    git clone https://github.com/google/googletest.git -b release-1.10.0
    cd googletest        # Main directory of the cloned repository.
    mkdir build          # Create a directory to hold the build output.
    cd build
    cmake ..             # Generate native build scripts for GoogleTest.
    make
    sudo make install
    ```
    - ros-kinetic
    - absl(compiled with c++17 settings, the code has been in "third_parties/abseil-cpp-master")
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

