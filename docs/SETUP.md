# Raspberry Pi + ROS 2 setup

This guide installs SHARKY on a Raspberry Pi Zero 2 W using **Ubuntu Server
24.04 LTS (64-bit)** and **ROS 2 Jazzy Jalisco**. Jazzy is the ROS 2 long-term
support release for Ubuntu 24.04 and provides native arm64 packages.

## 1. Install Ubuntu Server

1. Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Choose **Raspberry Pi Zero 2 W** as the device.
3. Choose **Ubuntu Server 24.04 LTS (64-bit)** as the operating system.
4. In Imager customisation, set your hostname, user, Wi-Fi, locale, and SSH key.
5. Write the image, insert the card, boot the Pi, and connect over SSH.

Use your own credentials in Imager. Do not add passwords, private keys, network
names, or the completed `config/sharky.env` file to this repository.

Update the Pi:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

Reconnect after the reboot and confirm that Ubuntu reports `aarch64`:

```bash
uname -m
```

## 2. Check the Ubuntu repositories

ROS 2's Raspberry Pi guide requires the `updates` and `backports` suites. Check
the active Ubuntu source:

```bash
grep -A4 '^Types: deb' /etc/apt/sources.list.d/ubuntu.sources
```

For Ubuntu 24.04, the `Suites:` line should include:

```text
noble noble-updates noble-backports
```

If either suite is missing, edit the file before continuing:

```bash
sudo nano /etc/apt/sources.list.d/ubuntu.sources
```

## 3. Install ROS 2 Jazzy

Set a UTF-8 locale and enable Ubuntu Universe:

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

Install the official ROS repository configuration package:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install the lightweight ROS base and development tools:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep \
  python3-flask python3-opencv python3-psutil git i2c-tools
```

Load ROS automatically in new Bash sessions:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

Initialise `rosdep` if this is a fresh image:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that its sources file already exists, continue with
`rosdep update`.

Check the installation:

```bash
ros2 --help
printenv ROS_DISTRO
```

The second command should print `jazzy`.

## 4. Enable I2C

Add the Raspberry Pi I2C overlay if it is not already present:

```bash
grep -q '^dtparam=i2c_arm=on' /boot/firmware/config.txt || \
  echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/firmware/config.txt
sudo usermod -aG i2c "$USER"
sudo reboot
```

Reconnect and check that the I2C bus exists:

```bash
ls -l /dev/i2c-1
i2cdetect -y 1
```

The PCA9685 normally appears at address `0x40`.

## 5. Build SHARKY

Clone the repository and create a ROS workspace:

```bash
git clone https://github.com/lgib2u/SHARKY.git
mkdir -p ~/sharky_ws/src
cp -R SHARKY/software/sharky_control ~/sharky_ws/src/
cp -R SHARKY/software/ros2_pca9685 ~/sharky_ws/src/
cd ~/sharky_ws
```

Install package dependencies and build sequentially to suit the Pi Zero 2 W:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --executor sequential
source install/setup.bash
```

Add the workspace to future Bash sessions:

```bash
echo 'source ~/sharky_ws/install/setup.bash' >> ~/.bashrc
```

## 6. Install the gait profile

```bash
mkdir -p ~/.config/sharky/gaits
cp ~/SHARKY/config/smooth_crawl_v1.json \
  ~/.config/sharky/gaits/active.json
```

If you cloned the repository somewhere other than `~/SHARKY`, change the source
path in the command.

## 7. Launch SHARKY

Start the motion stack:

```bash
source /opt/ros/jazzy/setup.bash
source ~/sharky_ws/install/setup.bash
ros2 launch sharky_control sharky_full.launch.py
```

The web interface binds to `127.0.0.1:5000` by default. To open it from another
computer without exposing the controller directly, create an SSH tunnel:

```bash
ssh -L 5000:127.0.0.1:5000 YOUR_PI_USER@YOUR_PI_HOST
```

Then open [http://127.0.0.1:5000](http://127.0.0.1:5000) in your browser.

## Useful checks

```bash
ros2 node list
ros2 service list | grep pca9685
ros2 topic list
```

Official references:

- [ROS 2 Jazzy Ubuntu packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 on Raspberry Pi](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [Raspberry Pi Imager and headless setup](https://www.raspberrypi.com/documentation/computers/getting-started.html)
