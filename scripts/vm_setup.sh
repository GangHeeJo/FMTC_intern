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
pip install "numpy<2.0" pyserial scikit-learn

echo "=== 6. .bashrc env ==="
grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# CycloneDDS는 WSL2/VM에서 노드끼리 서로 못 찾는 문제를 피하려고 쓴다. 네이티브 Ubuntu(NATIVE=1)에서는
# 오히려 큰 이미지 토픽(카메라)이 전달되지 않으므로 넣지 않는다 (2026-10-11 확인, 기본 FastDDS 사용).
if [ "${NATIVE:-0}" != "1" ]; then
  grep -qxF "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc || echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
fi
# 새 터미널에서 이 워크스페이스를 자동으로 불러오기 (빌드 후 install/ 이 생김)
grep -qxF "source ~/FMTC_intern/install/setup.bash" ~/.bashrc || echo "source ~/FMTC_intern/install/setup.bash" >> ~/.bashrc

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

echo "=== 9. (선택) 네이티브 실차 머신: USB 자동 절전 끄기 ==="
# Arduino/허브가 몇 초마다 끊기는 원인이 USB 자동 절전(autosuspend)이었음. NATIVE=1 bash vm_setup.sh 로 실행하면 적용.
if [ "${NATIVE:-0}" = "1" ]; then
  if ! grep -q "usbcore.autosuspend=-1" /etc/default/grub; then
    sudo sed -i 's/^GRUB_CMDLINE_LINUX_DEFAULT="\(.*\)"/GRUB_CMDLINE_LINUX_DEFAULT="\1 usbcore.autosuspend=-1"/' /etc/default/grub
    sudo update-grub
    echo "적용됨. 재부팅해야 완전히 반영됩니다."
  else
    echo "이미 적용되어 있음"
  fi
else
  echo "건너뜀 (VM이면 필요 없음. 실차 노트북이면 NATIVE=1 로 실행)"
fi

echo "=== DONE ==="
echo "터미널 새로 열거나 'source ~/.bashrc' 하면 바로 ros2 명령 사용 가능"
