# README #

## Branches
- master 主分支，包含整个室内外定位方案
- xbot 炸弹人项目的demo代码，与master差异较大（使用不同的传感器且目标功能不同）

## How do I get set up? ##

```bash
# 安装 pcl 库
# ubuntu 16.04 以前的系统
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

# ubuntu 16.04 及以后的版本
sudo apt install libpcl-dev 

# 安装其他库
sudo apt install libatlas-base-dev \
    libpcap-dev \
    libopencv-dev \
    libbluetooth-dev

# lpms
cd ~/Downloads
wget https://bitbucket.org/lpresearch/openmat/downloads/LpSensor-1.3.5-Linux-x86-64.tar.gz
tar -zxvf LpSensor-1.3.5-Linux-x86-64.tar.gz
cd LpSensor-1.3.5-Linux-x86-64
sudo dpkg -i liblpsensor-1.3.5-Linux.deb
dpkg -L liblpsensor

# slam
cd ~
git clone http://...current...git...address... cart_cpp
cd cart_cpp
make -j4

# doc 
# 如果需要生成文档，首先需要安装 doxygen
sudo apt install doxygen
make doc
# 过程中的 warning 是 doxygen 的版本问题导致的，可以忽略
# 结束之后打开 $PWD/doc/html/index.html 即可在浏览器中查看项目的说明文档

# style
# 如果需要格式化代码，需要安装 clang-format
sudo apt install clang-format
make style

# for arm
# 首先需要配置好交叉编译环境 gcc-arm-gnuenihf-5.4.*
# make arch=arm -j4

# 编译完成，得到三个可执行程序： driver carto starter
```

## Tunning
google 官方提供了一些关于调试的一些建议 —— [调试文档](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)，这里是我针对一些个人认为对算法效果影响比较大的参数的调试建议。
首先，Cartographer 的算法是分前端和后端的，那么调试同样也是针对前端和后端的，在参数里面体现的分别是 Trajectory_builder 和 pose_gragh 两个部分。
### TRAJECTORY_BUILDER

#### use_imu_data
- 在 2d slam 算法中，imu并不是必须的，不过我还是强烈建议把这个参数设为 true。这个参数使用之后会在算法考虑到重力方向的因素对传入的 2d 点云进行平面上的映射，对一些有坡度地面的还原是有明显效果的。
- 3d slam中 imu 是必须的，所以这个参数必须设为 true，否则程序会报错。

#### min_range & max_range 
- 对 2d/3d 点云原始数据先做简单的筛选处理，不在范围内的点被直接剔除。
- max_range 变大会使得计算量明显增大，在室内一般在10m～30m范围比较合适，而室外一般在30m～60m比较合适。
- min_range 通常可以用来剔除机器人本体上的点，去除机器人本身对地图的影响。

#### submaps.num_range_data
- 该参数的意义： 一个submap中包含的点云数据帧数，例如：10hz的3D点云数据，在该参数设为50的情况下（不考虑丢弃数据）大概5秒会生成一个新的submap。
- 这个参数通常结合机器人的速度和点云数据更新的频率来设定，通常来说一个submap中机器人的运动在1.5m~4m比较合适，例如： 机器人运动速度 0.5m/s，数据更新 10hz， 那么这个参数设定 30 (1.5/0.5×10) ~ 80 (4/0.5×10) 之间比较合适，不过这个范围在 max_range 很大的情况下可以适当增大。
- 这个参数越大，submap的量更少，后端的计算压力会有所降低。

#### use_online_correlative_scan_matching
- 该参数决定了前端的 real time correlative scan matcher 是否启用，目前看来这个 scan matcher 的效果很好，通常在2D slam中开启会提升建图和定位的效果。
- 在3D slam中该 scan matcher 非常耗资源，所以在3D slam中需要谨慎使用，可能会导致被迫丢弃很多点云数据（尤其在 max_range 设置的比较大的情况下）。

#### real_time_correlative_scan_matcher
- use_online_correlative_scan_matching 设为 true 的话，此参数就会发挥作用，限制这个scan matcher在 local scan matching 时的搜索范围。
- 搜索范围包括 linear_search_window, angular_search_window。前面我们也说了，通常在2D情况下我们建议开启这个 scan matcher，而这两个search window参数的设定同样对计算量影响很大，我们需要在保证质量的情况下尽可能的缩小这两个参数，例如： 机器人运动速度 0.5m/s，数据更新 10hz，那么就是在两次数据更新之间的运动距离大概在 0.5/10 = 0.05m，考虑到数据删减等因素，linear search window 可以设置为 0.1 ~ 0.15之间比较合适；角度同理，考虑角速度。

#### voxel_filter_size
- 这个参数是直接对原始的点云数据进行的体素滤波参数，建议在 0.05 ~ 0.2 之间，参数增大会减小计算量，但是同时会降低精度

#### pure_localization
- 纯定位模式，这个模式下不会更新载入的地图信息（如果有地图的话），同样在程序结束也不会修改已有地图。

### POSE_GRAPH
Cartographer 的后端基于图优化的思路，所以需要先建立一个 pose graph。这其中包括 node， contraint 等概念具体可以参考一些资料：
> [A Tutorial on Graph-Based SLAM](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)  
> [Efficient Sparse Pose Adjustment for 2D Mapping](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.231.1772&rep=rep1&type=pdf)  
> [Course on SLAM](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/toolbox/courseSLAM.pdf)

#### constraint_builder

- min_score 该参数决定了在 fast correlative scan matcher 得到什么样的分数时才会认定构建新的 constraint ，建议设置在 0.5 ~ 0.6 之间，通常室内环境可以设置稍高一点，如0.6，室外环境设置低一些，如0.52。该参数设置过小会导致后端运算量过大，而设置过大可能会导致后端完全发挥不了作用。
  - 针对3d场景，我在Cartographer的代码中专门加了自动调整该参数的逻辑，因而引入了一套参数 —— constraint_builder_self_adjust_options_3d；
  - enabled 使能该功能，默认为 false（不开启）；
  - min_score & max_score 为自动调整的范围；
  - step 每次自动调整时的调整 step，建议设定值在 0.005 ~ 0.01 之间;
  - 剩余两个参数建议保留默认值
- max_constraint_distance 这个参数通常和建图的大小有关，建图面积越大则设置越大。

#### optimize_every_n_nodes

- 该参数决定了后端进行优化的频率。
- 通常在建图时设置与submaps.num_range_data相等即可，而在纯定位模式下这个参数可以小一些保证后端的高频率优化，从而保证定位位置的实时性。保证定位低延迟的调试建议在官方的[调试文档](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)里面讲得很详细，这里就不赘述了。




