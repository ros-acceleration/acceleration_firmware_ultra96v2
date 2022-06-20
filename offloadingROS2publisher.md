# 3. Offloading ROS 2 publisher

|   | Source code |
|---|----------|
| [`offloaded_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/offloaded_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/offloaded_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`accelerated_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/offloaded_doublevadd_publisher/src/offloaded_doublevadd_publisher.cpp) |


```eval_rst
.. sidebar:: Before you begin
   
    This example builds on top of a prior one:

    - `0. ROS 2 publisher - doublevadd_publisher, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the ultra96v2 and is only able to publish at :code:`2 Hz`.

```

This example leverages KRS to offload the `vadd` function operations to the FPGA, showing how easy it is for ROS package maintainers to extend their packages to include hardware acceleration, and create deterministic kernels. The objective is to publish the resulting vector at 10 Hz.  

The source code is taken from `doublevadd_publisher` purposely *as is*, and HLS transforms the C++ code directly to RTL, creating a dedicated hardware circuit in the form of a kernel that offloads the CPU from the heavy vadd computations and provides deterministic responses.

Though deterministic, the resulting kernel computation is slower than its CPUs counterpart. The reason behind this is that the code is taken *as is*, and the kernel doesn't really exploit any parallelism, nor optimizes the computation flow. Given that the kernel clock (`4 ns`) is slower much than the one of the Arm CPUs, this leads altogether to an actual worse performance than the only-CPU case previously studied at [0. ROS 2 publisher](ROS2publisher.md).

The ultimate objective of this example is to illustrate roboticists how **more deterministic (connected to real-time) does not necessarily lead to faster (lower latency) or better performance, quite the opposite**. Future examples will demonstrate how to achieve both, determinism and low latency.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to the install instructions.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `offloaded_doublevadd_publisher`

The kernel is identical to the one presented in [0. ROS 2 publisher](ROS2publisher.md) with a small modification in the vector size

Let's setup the envirnonment in the workstation:

```
bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

```
Source Code:
There is a change in the vector size en line 17 (./src/acceleration_examples/offloaded_doublevadd_publisher/src/vadd.cpp)
```
cpp 
/*
        ____  ____
       /   /\/   /
      /___/  \  /   Copyright (c) 2021, Xilinx®.
      \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
       \   \
       /   /
      /___/   /\
      \   \  /  \
       \___\/\___\

Inspired by the Vector-Add example.
See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis

*/

// Vector size change
// #define DATA_SIZE 4096
#define DATA_SIZE 4032

// TRIPCOUNT identifier
const int c_size = DATA_SIZE;

extern "C" {
    void vadd(
            const unsigned int *in1,  // Read-Only Vector 1
            const unsigned int *in2,  // Read-Only Vector 2
            unsigned int *out,        // Output Result
            int size                  // Size in integer
            )
    {
        for (int j = 0; j < size; ++j) {  // stupidly iterate over
                                          // it to generate load
        #pragma HLS loop_tripcount min = c_size max = c_size
            for (int i = 0; i < size; ++i) {
            #pragma HLS loop_tripcount min = c_size max = c_size
              out[i] = in1[i] + in2[i];
            }
        }
    }
}
```
We need to change the vector size and make the buffers four times bigger in `offloaded_doublevadd_pubisher.cpp` (/src/acceleration_examples/offloaded_doublevadd_publisher/src) 

in line 9
```
#define DATA_SIZE 4032
```
in lines 91-96
```
  cl::Buffer in1_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(int) * DATA_SIZE * 4, NULL, &err);
  cl::Buffer in2_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(int) * DATA_SIZE * 4, NULL, &err);
  cl::Buffer out_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY,
    sizeof(int) * DATA_SIZE * 4, NULL, &err);
```
in line 103 add the fourth argument to the kernel (vector size)
```
krnl_vector_add.setArg(3, DATA_SIZE);
```
in lines 106-108
```
int *in1 = (int *)q.enqueueMapBuffer(in1_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE * 4);  // NOLINT
int *in2 = (int *)q.enqueueMapBuffer(in2_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE * 4);  // NOLINT
int *out = (int *)q.enqueueMapBuffer(out_buf, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, sizeof(int) * DATA_SIZE * 4);  // NOLINT
```


The only difference in this package is that it declares a kernel on its CMakeLists.txt file using the `vitis_acceleration_kernel` CMake macro:

```cmake 
# vadd kernel
vitis_acceleration_kernel(
  NAME vadd
  FILE src/vadd.cpp
  CONFIG src/ultra96v2.cfg
  INCLUDE
    include
  TYPE
    hw
  PACKAGE
)
```

Create the ultra96v2.cfg file:
```
bash
$ pico ~/krs_wk/src/acceleration_examples/offloaded_doublevadd_publisher/src/ultra96v2.cfg

# ultra96v2.cfg
platform=xilinx_ultra96v2_base_202020_1
save-temps=1
debug=1

# Enable profiling of data ports
[profile]
data=all:all:all
```

Modify the CMakeLists.txt file (~krs_ws/src/acceleration_examples/offloaded_doublevadd_publisher/CMakeLists.txt)
```
Replace in line 59:
      CONFIG src/kv260.cfg
with
      CONFIG src/ultra96v2.cfg

```

Let's build it:
```
bash
# select ultra96v2 firmware:
$ colcon acceleration select ultra96v2

# build offloaded_doublevadd_publisher (about 15 min)
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis ros2acceleration offloaded_doublevadd_publisher

# copy to SD card rootfs:
$ sudo cp -r install-ultra96v2/* /media/usuario/vos_2/krs_ws
$ sync
```

Since this package contains a kernel and we're using the Vitis `hw` build target (*more on Vitis build targets in future tutorials*), it'll take a bit longer to build the package. In an *Intel i7 10th with 32GB RAM* it took **15 minutes**, so go for a cup of coffe.

Note also the process is *slightly different* this time since we have an acceleration kernel. Before launching the binary in the CPUs, we need to load the kernel in the FPGA. For that, we'll be using some of the extensions KRS provides to the ROS 2 CLI tooling, particularly the `ros2 acceleration` suite:

When you boot the ultra96v2 board you will find DFX-MGR a missing file error:
```
DFX-MGRD> ERROR: initBaseDesign: could not open /lib/firmware/xilinx/base/shell.json err No such file or directory
```
This is because every accelerator needs a `shell.json` file. We have a `base` and `offloaded_doublevadd_publisher` accelerators, so we need a shell.json file for each one:
```
bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /krs_ws/local_setup.bash     # source the ROS 2 overlay workspace we just 
                                  # created. Note it has been copied to the SD 
                                  # card image while being created.

# restart the daemon that manages the acceleration kernels to create /lib/firmware/xilinx/offloaded_doublevadd_publisher
$ ros2 acceleration stop
$ ros2 acceleration start

# stop the daemon to create the shell.json files
$ ros2 acceleration stop

# add shell.json files
vi /lib/firmware/xilinx/base/shell.json
{
    "shell_type" : "XRT_FLAT",
    "num_slots": "1"
}

vi /lib/firmware/xilinx/offloaded_doublevadd_publisher/shell.json
{
    "shell_type" : "XRT_FLAT",
    "num_slots": "1"
}

# reboot
$ shutdown -r now

$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /krs_ws/local_setup.bash     # source the ROS 2 overlay workspace we just 
                                  # created. Note it has been copied to the SD 
                                  # card image while being created.

# Change to the accelerator directory
$ cd /krs_ws/lib/offloaded_doublevadd_publisher

# Load the accelerator 
$ ros2 acceleration select offloaded_doublevadd_publisher

# restart the daemon that manages the acceleration kernels
$ ros2 acceleration stop
$ ros2 acceleration start

# list the accelerators
$ ros2 acceleration list
Accelerator                            Base                            Type             #slots         Active_slot

  offloaded_doublevadd_publisher  offloaded_doublevadd_publisher       XRT_FLAT         0                  -1
                            base                            base       XRT_FLAT         0                  -1

# select the offloaded_doublevadd_publisher
$ ros2 acceleration select offloaded_doublevadd_publisher

# launch binary 
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run offloaded_doublevadd_publisher offloaded_doublevadd_publisher

[INFO] [1520599663.156563610] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 6'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00048s window: 6
[INFO] [1520599664.949324690] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 7'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00053s window: 7
[INFO] [1520599666.742386630] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 8'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00050s window: 8
[INFO] [1520599668.535174830] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00052s window: 9
[INFO] [1520599670.328085390] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00049s window: 10
[INFO] [1520599672.121017030] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
average rate: 0.558
        min: 1.792s max: 1.794s std dev: 0.00047s window: 10
[INFO] [1520599673.913815520] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'

...
```

The publishing rate is `0.5 Hz`, which is lower than the `2.2 Hz` obtained in [0. ROS 2 publisher](0_ros2_publisher/). As introduced before and also in example [2. HLS in ROS 2](2_hls_ros2/), the rationale behind this is a combination of two aspects:
- First, the CPU clock is generally faster than the FPGA one, which means that pure offloading of operations (unless dataflow is optimized) are deterministic, but most of the time subject to be coherent with the slower clock.
- Second, the computation needs to be adapted to the dataflow and parallelism exploited (if available).
- Third, we are using buffers that are four times bigger, the transfer time needs to be taken into acount

NOTE:
The change in the vector size and bigger buffers are necessary due to a some kind of misalignment between the physical memory in the accelerator, the memory object in the openCL runtime and the userland pointers. At some position we get a 0 result from the accelerator. With these changes, things work as expected (the new vector size is a multiple of a blockram core size). Probably this happens because we are using direct mapping (a userland pointer that access directly the memory in the accelerator). Perhaps the use of a dedicated openCL buffer and a separated userland buffer with transfers between them should be a better approach
