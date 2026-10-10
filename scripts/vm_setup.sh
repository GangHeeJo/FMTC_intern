#!/usr/bin/env bash
set -e

echo "=== 1. locale ==="
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== 2. ROS2 apt repo ==="
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== 3. ROS2 humble + deps (takes a while) ==="
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
sudo apt install -y ros-humble-usb-cam ros-humble-cv-bridge ros-humble-joy ros-humble-rmw-cyclonedds-cpp
sudo apt install -y build-essential cmake python3-colcon-common-extensions python3-pip git v4l-utils

echo "=== 4. serial/camera port permission ==="
sudo usermod -aG dialout,video "$USER"

echo "=== 5. python packages (setuptools pin to avoid colcon build breakage) ==="
pip install --no-cache-dir ultralytics torch torchvision
pip install "setuptools==58.2.0"
pip uninstall -y opencv-python opencv-python-headless 2>/dev/null || true
pip install "numpy<2.0" pyserial

echo "=== 6. .bashrc env ==="
grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -qxF "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc || echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

echo "=== 7. clone repo + build ==="
cd ~
if [ ! -d "FMTC_intern" ]; then
  git clone https://github.com/GangHeeJo/FMTC_intern.git
fi
cd FMTC_intern
git checkout ganghee
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install

echo "=== 8. udev rules ==="
sudo cp udev/99-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "=== DONE ==="
echo "터미널 새로 열거나 'source ~/.bashrc' 하면 바로 ros2 명령 사용 가능"
