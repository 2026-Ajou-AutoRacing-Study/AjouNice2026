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