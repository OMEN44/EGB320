sudo apt update
sudo apt upgrade -y

sudo snap install zellij --classic

# Setup ros stuff
sudo apt install software-properties-common gh git curl btop openssh-server -y
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install ros-dev-tools -y

sudo apt update
sudo apt upgrade

sudo apt install ros-jazzy-ros-base -y

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Setup v4l2loopback stuff
sudo apt update
sudo apt install build-essential linux-headers-$(uname -r) git dkms

git clone https://github.com/umlaeute/v4l2loopback.git
cd v4l2loopback

sudo dkms add .
sudo dkms build v4l2loopback/0.15.1   # replace with version if needed
sudo dkms install v4l2loopback/0.15.1

echo "Close this terminal and open a new one to start using ROS Jazzy."
