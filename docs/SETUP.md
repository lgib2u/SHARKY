# Software setup

This guide builds the workspace without starting the robot. Commands are run on
the Raspberry Pi unless a step says otherwise.

## 1. Prepare the computer

Use a Raspberry Pi Zero 2 W (or another Linux SBC with compatible I2C) and a
supported 64-bit Linux image. Install ROS 2 by following the official
instructions for the ROS distribution and operating-system combination you
choose. A minimal `ros-base` installation is sufficient.

You will also need Git, a C/C++ toolchain, Python 3, `rosdep`, and `colcon`.
Package names vary between operating-system releases, so prefer your ROS
distribution's official installation guide over copying an old package list.

Do not connect servo power during operating-system or software setup.

## 2. Create a private local configuration

Clone the repository and make an untracked local environment file:

```bash
git clone https://github.com/lgib2u/SHARKY.git
cd SHARKY
cp config/sharky.env.example config/sharky.env
```

Edit `config/sharky.env` for your own network and ROS environment. Never commit
that file. The example deliberately contains placeholders rather than working
credentials or machine identifiers.

Load your edited values into the current shell:

```bash
set -a
. config/sharky.env
set +a
```

The dashboard binds to `127.0.0.1` by default because its hardware controls do
not provide built-in authentication. If remote access is required, place it
behind an authenticated local reverse proxy; do not expose port 5000 directly.

## 3. Build the ROS workspace

Create a workspace and copy the two packages into it:

```bash
mkdir -p ~/sharky_ws/src
cp -R software/sharky_control ~/sharky_ws/src/
cp -R software/ros2_pca9685 ~/sharky_ws/src/
cd ~/sharky_ws
```

Resolve available dependencies and build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Building does not start ROS nodes and does not command hardware.

## 4. Install the example gait profile

The controller looks for a local active profile first. Install the example with
ordinary user permissions:

```bash
mkdir -p ~/.config/sharky/gaits
cp /path/to/SHARKY/config/smooth_crawl_v1.json \
  ~/.config/sharky/gaits/active.json
```

Review every angle against the mechanical build before using it. Right-side
joint flips and channel limits are implemented in the controller; they are not
universal wiring instructions.

## 5. Inspect without motors

With servo power disconnected, source the workspace and inspect the package:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/sharky_ws/install/setup.bash
ros2 pkg executables sharky_control
ros2 pkg executables ros2_pca9685
```

At this point you can also review the launch file and node parameters. Do not
launch the full stack merely to see whether the build succeeded.

## 6. First hardware test

Before powering the servo rail, complete every item in
[`SAFETY.md`](SAFETY.md). Raise and restrain the robot so no foot can reach a
person, cable, or table. Be ready to remove servo power independently of the Pi.

The full stack can then be started explicitly:

```bash
ros2 launch sharky_control sharky_full.launch.py
```

Launching creates the hardware driver and motion interface, but the controller
starts disarmed. An explicit arm command enables outputs and moves to the stand
pose. Do not send it until the channel map and neutral positions are verified.

## 7. Stop safely

Use the dashboard's disarm control or the local command interface before ending
a session. Confirm that PWM outputs are disabled, stop the ROS processes, and
then disconnect servo power.

## Troubleshooting

- If `rosdep` cannot resolve a dependency, verify that the selected ROS release
  supports your operating system and architecture.
- If the PCA9685 service is missing, check I2C enablement and wiring with servo
  power disconnected.
- If a joint would move the wrong way, stop and correct the mapping or linkage;
  do not increase angle limits to force it.
- If the camera is unavailable, test the camera independently before launching
  the combined web interface.
