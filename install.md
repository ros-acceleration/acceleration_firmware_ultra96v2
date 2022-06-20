This release was tested with :

- Ubuntu 20.04 Focal Fossa operating system [download](https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso)
- the Vitis 2021.2 suite (Vitis, Vivado, Vitis HLS) [install instructions](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis/2021-2.html)
- the ROS 2 Foxy distribution [install instructions](https://docs.ros.org/en/foxy/Installation.html)

```

###################################################
# 0. install Vitis 2021.2, and ROS 2 Foxy,
      #  see above
###################################################

###################################################
# 1. install some dependencies you might be missing
###################################################
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv

###################################################
# 2. create a new ROS 2 workspace
###################################################
mkdir -p ~/krs_ws/src; cd ~/krs_ws

###################################################
# 3. Create file with KRS alpha release
###################################################
cat << 'EOF' > krs_alpha_ultra96v2.repos
repositories:
  acceleration/acceleration_firmware:
    type: git
    url: https://github.com/ros-acceleration/acceleration_firmware
    version: 0.4.0
  acceleration/colcon-acceleration:
    type: git
    url: https://github.com/ros-acceleration/colcon-acceleration
    version: 0.3.0
  acceleration/ros2acceleration:
    type: git
    url: https://github.com/ros-acceleration/ros2acceleration
    version: 0.2.0
  acceleration/ament_vitis:
    type: git
    url: https://github.com/ros-acceleration/ament_vitis
    version: 0.5.0
  acceleration/vitis_common:
    type: git
    url: https://github.com/ros-acceleration/vitis_common
    version: 0.1.0
EOF

###################################################
# 4. import repos of KRS alpha release
###################################################
vcs import src --recursive < krs_alpha_ultra96v2.repos  # about 2 mins

###################################################
# 5. modifiy ros2 acceleration list verb
###################################################
pico ./src/acceleration/ros2acceleration/ros2acceleration/verb/list.py

Replace 
        run(cmd, shell=True)
With
        outs, errs = run(cmd, shell=True)
        if outs:
            print(outs)

###################################################
# 6. get acceleration_firmware_ultra96v2 (1.7 GB Download)
###################################################
cd src && wget https://github.com/pimartos/acceleration_firmware_ultra96v2/releases/download/acceleration_firmware_ultra96v2-krs_alpha_v1.0.0/acceleration_firmware_ultra96v2.zip && unzip acceleration_firmware_ultra96v2.zip && cd ..

###################################################
# 7. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2021.2/settings64.sh  # source Xilinx tools
source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation.
# Note: The path above is valid if one installs ROS 2 from a pre-built
# package. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_foxy/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
colcon build --merge-install  # about 2 mins

###################################################
# 8. source the overlay to enable all features
###################################################
source install/setup.bash

###################################################
# 8. Copy the rootfs.cpio.gz file by hand (due it's size, sometimes it doesn't install automatically)
###################################################
$ cp ./src/acceleration_firmware_ultra96v2/firmware/rootfs.cpio.gz ./acceleration/firmware/ultra96v2/
$ sync


```

Now we can run [the examples](examples.md) 
