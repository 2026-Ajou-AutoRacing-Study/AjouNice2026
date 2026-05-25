# ubuntu20.04에 ROS Noetic 설치하기
### [공식 문서](https://wiki.ros.org/noetic/Installation/Ubuntu)를 참고하거나, 아래 스크립트를 순서대로 실행
```bash
# Ubuntu 20.04 초기 설정 -> 이미 했으면 건너뛰기
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

sudo apt install -y \
  git
  # wget \ # 선택
  # vim 선택
```

```bash
# ROS Noetic 설치
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

# ros 설치에 5분 이상 소요될 수 있음.
sudo apt install -y ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 설치 잘 됐는지 확인
roscore
# 다른 터미널에서 실행
rostopic list 
# /rosout \ /rosout_agg 이 나오면 성공


# 기본 tool들 설치
sudo apt install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential \
  python3-catkin-tools # 이건 공식 문서엔 없는데, 나중에 catkin_make 대신 catkin build를 쓸 수 있어서 더 편함
# python3-catkin-tools를 설치했으니 앞으로의 빌드는 아래처럼 수행할 수 있게 됨:
# cd ~/base_ws
# catkin init
# catkin build

# rosdep 초기화
sudo rosdep init
rosdep update
# updated cache in ... 이 나오면 성공.
```

```bash
# perception 기준으로 필요할 수 있는 패키지들 -> morai에서 토픽을 어떻게 보내주냐에 따라 몇 개는 필요 없을 수도 있긴 함.
# 우리는 ros-noetic-desktop-full로 설치했기에, 실제로는 ros-noetic-rqt만 추가로 설치됐고 나머진 이미 설치되어 있음.
sudo apt install -y \
  ros-noetic-tf \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-image-transport-plugins \
  ros-noetic-compressed-image-transport \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-rviz \
  ros-noetic-rqt \
  ros-noetic-rqt-graph \
  ros-noetic-rqt-tf-tree
  # 후순위(실제 물리적 센서 사용한다면 필요한데, 우리는 morai에서 센싱해 주니까 불필요할듯?)
  # ros-noetic-velodyne \
  # ros-noetic-velodyne-pointcloud \

# 특정 패키지가 설치되어 있는지 확인하는 법:
# dpkg -l | grep ros-noetic-velodyne
```

```bash
# 편의성 빌드 단축키
# 이제 bs만 입력해도 빌드, 셋업 소스 후 원래 경로로 복귀함
cat <<'EOF' >> ~/.bashrc

bs() {
  pushd ~/AjouNice2026/base_ws > /dev/null || return
  catkin build
  source devel/setup.bash
  popd > /dev/null || return
}
EOF

source ~/.bashrc
```


## 아래는 git clone하고, 비어 있는 workspace를 처음부터 init 후 채운다는 가정

```bash
cd base_ws
catkin init
catkin build

echo 'source ~/AjouNice2026/base_ws/devel/setup.bash' >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
source ~/.bashrc

# 확인
echo $ROS_PACKAGE_PATH
# 아직은 /opt/ros/noetic/share 같은 거만 출력될 거임.
```

```bash
# 초기 패키지들 생성
cd ~/AjouNice2026/base_ws/src
# catkin_create_pkg란? -> ros에서 패키지 만드는 명령어

# 우리 custom message 정의용 패키지 -> BEVFusion 등에 사용
catkin_create_pkg msgs \
  std_msgs geometry_msgs nav_msgs sensor_msgs message_generation message_runtime

# launch 파일 모음 -> roslaunch bringup sensing.launch 또는 full_stack.launch 식으로 사용하자
# sensing, localization, perception, full_stack.launch -> 일단 센싱부터 만들기
catkin_create_pkg bringup \
  rospy std_msgs

# morai 센서 토픽 -> 우리 커스텀 센서 토픽으로 변경 
# msg는 sensor_msgs 유지하고 웬만해선 굳이 바꾸진 x
# 근데 토픽명 변경 굳이일수도 -> 일단 해보고 고민 -> 그래도 바꾸는게 좋긴 해보임
catkin_create_pkg sensor_interface \
  rospy std_msgs sensor_msgs geometry_msgs tf tf2_ros

# morai (센서 제외한) 토픽 -> 우리 커스텀 토픽으로 변경 
# + msg 변경(아마 morai 어쩌고로 들어올 듯)
catkin_create_pkg morai_ros_bridge \
  rospy std_msgs sensor_msgs geometry_msgs nav_msgs
```

```bash
# 각 패키지 안에 우리가 쓸 폴더들 만들어주기
cd ~/AjouNice2026/base_ws/src

mkdir -p bringup/launch bringup/config

mkdir -p sensor_interface/launch sensor_interface/config sensor_interface/scripts
mkdir -p sensor_interface/src/sensor_interface
touch sensor_interface/src/sensor_interface/__init__.py

mkdir -p morai_ros_bridge/launch morai_ros_bridge/config morai_ros_bridge/scripts
mkdir -p morai_ros_bridge/src/morai_ros_bridge
touch morai_ros_bridge/src/morai_ros_bridge/__init__.py

mkdir -p msgs/msg
```