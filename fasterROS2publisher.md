# 5. Faster ROS 2 publisher

|   | Source code |
|---|----------|
| [`faster_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/faster_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/faster_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`faster_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/faster_doublevadd_publisher/src/faster_doublevadd_publisher.cpp) |

```eval_rst
.. sidebar:: Before you begin
   
    Past examples of this series include: 

    - `4. Accelerated ROS 2 publisher - offloaded_doublevadd_publisher <4_accelerated_ros2_publisher.html>`_, which offloads and accelerates the `vadd` operation to the FPGA, optimizing the dataflow and leading to a deterministic vadd operation with an improved publishing rate of 5 Hz.
    - `3. Offloading ROS 2 publisher - offloaded_doublevadd_publisher <3_offloading_ros2_publisher.html>`_, which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of :code:`1.935 Hz`.
    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the ultra96v2 and is only able to publish at 2 Hz.

```

This example is the last one of the *ROS 2 publisher series*. It features a trivial vector-add ROS 2 publisher, which adds two vector inputs in a loop, and tries to publish the result at 10 Hz. This example will leverage KRS to produce an acceleration kernel that a) optimizes the dataflow and b) leverages parallelism via loop unrolling to meet the initial goal established by the `doublevadd_publisher`.

- [4. Accelerated ROS 2 publisher - `offloaded_doublevadd_publisher`](4_accelerated_ros2_publisher/), which offloads and accelerates the `vadd` operation to the FPGA, optimizing the dataflow and leading to a deterministic vadd operation with an improved publishing rate of `5 Hz`.
- [3. Offloading ROS 2 publisher - `offloaded_doublevadd_publisher`](3_offloading_ros2_publisher/), which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of `1.935 Hz`.
- [0. ROS 2 publisher - `doublevadd_publisher`](0_ros2_publisher/), which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the ultra96v2 and is only able to publish at 2 Hz.


```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `accelerated_doublevadd_publisher`

Let's take a look at the [kernel source code](https://github.com/ros-acceleration/acceleration_examples/blob/main/faster_doublevadd_publisher/src/vadd.cpp) first:
We need to change the vector size to 4032.

```cpp 
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

#define DATA_SIZE 4032
//#define DATA_SIZE 4096
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
#pragma HLS INTERFACE m_axi port=in1 bundle=aximm1
#pragma HLS INTERFACE m_axi port=in2 bundle=aximm2
#pragma HLS INTERFACE m_axi port=out bundle=aximm1

        for (int j = 0; j <size; ++j) {  // stupidly iterate over
                                          // it to generate load
        #pragma HLS loop_tripcount min = c_size max = c_size
            for (int i = 0; i<(size/16)*16; ++i) {
            #pragma HLS UNROLL factor=16
            #pragma HLS loop_tripcount min = c_size max = c_size
              out[i] = in1[i] + in2[i];
            }
        }
    }
}
```

Besides the dataflow optimizations between input and output arguments in the PL-PS interaction, *line 37* utilizes a pragma to unroll the inner `for` loop by a factor of `16`, executing `16` sums *in parallel*, within the same clock cycle. The value of `16` is not arbitrary, but selected specifically to consume the whole bandwidth (`512-bits`) of the `m_axi` ports at each clock cycle available after previous dataflow optimizations (see [4. Accelerated ROS 2 publisher](4_accelerated_ros2_publisher/) to understand more about the dataflow optimizations). To fill in `512` bits, we pack together `16 unsigned int` inputs, each of `4 bytes`:

```eval_rst
.. math::

   \texttt{16 unsigned int}  \cdot 4 \frac{bytes}{\texttt{unsigned int}} \cdot 8 \frac{bits}{byte} = 512 \texttt{ bits}
```

Altogether, this leads to the most optimized form of the `vadd` kernel, *delivering both dataflow optimizations and code parallelism*, getting a 6.7 Hz publish frequency. Overall, when compared to the initial example [0. ROS 2 publisher - `doublevadd_publisher`](0_ros2_publisher/), the one presented in here obtains a **speedup of 3x** (*In fact, the speedup is higher however the ROS 2 `WallRate` instance is set to `100ms`, so the kernel is idle waiting for new data to arrive, discarding further acceleration opportunities.*).

We need to change the vector size and make the buffers four times bigger in `faster_doublevadd_pubisher.cpp` (/src/acceleration_examples/faster_doublevadd_publisher/src) 

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
Create the ultra96v2.cfg file:
```
bash
$ pico ~/krs_wk/src/acceleration_examples/faster_doublevadd_publisher/src/ultra96v2.cfg

# ultra96v2.cfg
platform=xilinx_ultra96v2_base_202020_1
save-temps=1
debug=1

# Enable profiling of data ports
[profile]
data=all:all:all
```

Modify the CMakeLists.txt file (~krs_ws/src/acceleration_examples/faster_doublevadd_publisher/CMakeLists.txt)
```
Replace in line 59:
      CONFIG src/kv260.cfg
with
      CONFIG src/ultra96v2.cfg

```

Let's build it:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# if not done before, fetch the source code of examples
$ git clone https://github.com/ros-acceleration/acceleration_examples src/acceleration_examples

# build the workspace to deploy KRS components
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash

# select ultra96v2 firmware (in case you've been experimenting with something else)
$ colcon acceleration select ultra96v2

# build accelerated_doublevadd_publisher
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis ros2acceleration faster_doublevadd_publisher

# copy to SD card rootfs:
$ sudo cp -r install-ultra96v2/* /media/usuario/vos_2/krs_ws
$ sync
```

and run it:

Every accelerator needs a `shell.json` file:

```
bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /krs_ws/local_setup.bash     # source the ROS 2 overlay workspace we just 
                                  # created. Note it has been copied to the SD 
                                  # card image while being created.

# restart the daemon that manages the acceleration kernels to create /lib/firmware/xilinx/faster_doublevadd_publisher
$ ros2 acceleration stop
$ ros2 acceleration start

# stop the daemon to create the shell.json files
$ ros2 acceleration stop

# add shell.json files
vi /lib/firmware/xilinx/faster_doublevadd_publisher/shell.json
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
$ cd /krs_ws/lib/faster_doublevadd_publisher

# Load the accelerator 
$ ros2 acceleration select faster_doublevadd_publisher

# restart the daemon that manages the acceleration kernels
$ ros2 acceleration stop
$ ros2 acceleration start

# list the accelerators
$ ros2 acceleration list
Accelerator                            Base                            Type             #slots         Active_slot

  faster_doublevadd_publisher  	    faster_doublevadd_publisher        XRT_FLAT         0                  -1
  accelerated_doublevadd_publisher  accelerated_doublevadd_publisher   XRT_FLAT         0                  -1
  offloaded_doublevadd_publisher  offloaded_doublevadd_publisher       XRT_FLAT         0                  -1
                            base                            base       XRT_FLAT         0                  -1

# select the accelerated_doublevadd_publisher
$ ros2 acceleration select faster_doublevadd_publisher

# launch binary 
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run faster_doublevadd_publisher faster_doublevadd_publisher

[INFO] [1520601618.052719680] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1520601618.252669520] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1520601618.386791040] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
[INFO] [1520601618.520793390] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1520601618.654800060] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
[INFO] [1520601618.854827450] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1520601618.988827330] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
average rate: 6.500
        min: 0.134s max: 0.200s std dev: 0.03028s window: 10
[INFO] [1520601619.122914260] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'
[INFO] [1520601619.257509340] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1520601619.457075040] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1520601619.591478010] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 19'
[INFO] [1520601619.726018860] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 20'
[INFO] [1520601619.860117600] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 21'
[INFO] [1520601620.060155240] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 22'
average rate: 6.495
        min: 0.134s max: 0.200s std dev: 0.03007s window: 10
[INFO] [1520601620.195554070] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 23'
[INFO] [1520601620.330336770] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 24'
[INFO] [1520601620.464646740] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 25'
[INFO] [1520601620.664534150] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 26'
[INFO] [1520601620.798578020] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 27'
[INFO] [1520601620.932545890] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 28'
[INFO] [1520601621.066545900] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 29'
average rate: 6.783
        min: 0.134s max: 0.200s std dev: 0.02638s window: 10
...
```
