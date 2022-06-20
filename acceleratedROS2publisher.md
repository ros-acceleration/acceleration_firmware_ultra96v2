# 4. Accelerated ROS 2 publisher

|   | Source code |
|---|----------|
| [`accelerated_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/accelerated_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/accelerated_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`accelerated_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/accelerated_doublevadd_publisher/src/accelerated_doublevadd_publisher.cpp) |


```eval_rst
.. sidebar:: Before you begin
   
    This example builds on top of two prior ones: 

    - `3. Offloading ROS 2 publisher - offloaded_doublevadd_publisher <3_offloading_ros2_publisher.html>`_, which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of 0.5 Hz.
    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the ultra96v2 and is only able to publish at 2.2 Hz.

```


This example leverages KRS to offload and accelerate the `vadd` function operations to the FPGA, showing how easy it is for ROS package maintainers to extend their packages, include hardware acceleration and create deterministic kernels. The objective is to publish the resulting vector at 10 Hz. 

This example upgrades the previous offloading operation at [3. Offloading ROS 2 publisher](offloadingROS2publisher.md), and includes an optimization for the dataflow. This allows the publisher to improve its publishing rate from 2 Hz, up to 5 Hz. For that, the HLS `INTERFACE` pragma is used. The HLS INTERFACE specifies how RTL ports are created from the function definition during interface synthesis. Sharing ports helps save FPGA resources by eliminating AXI interfaces, but it can limit the performance of the kernel because all the memory transfers have to go through a single port. The `m_axi` port has  independent READ and WRITE channels, so with a single m_axi port, we can do reads and writes simultaneously but since the kernel (`vadd`) has two vectors from where its reading (simultaneously), we can optimize the dataflows by simply asking for an extra AXI interface.
    
After this dataflow optimization in the kernel, the `accelerated_doublevadd_publisher` ROS 2 package is able to publish at 5 Hz. *For a faster kernel that meets the 10 Hz goal, refer to [5. Faster ROS 2 publisher](fasterROSpublisher.md)*.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `accelerated_doublevadd_publisher`

Let's take a look at the kernel source code this time (./src/acceleration_examples/accelerated_doublevadd_publisher/src/vadd.cpp)
change DATA_SIZE to 4032

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

#define DATA_SIZE 4096
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

Note the aforementioned `INTERFACE` pragma as the only relevant change introduced, which specifies how RTL ports are created from the function definition during interface synthesis.

```eval_rst

.. admonition:: Short explanation of the `HLS INTERFACE` pragma

    The parameters of the software functions defined in a HLS design are synthesized into ports in the RTL code [1]_ that group multiple signals to encapsulate the communication protocol between the HLS design and things external to the design.  The :code:`HLS INTERFACE` specifies how RTL ports are created from the function definition during interface synthesis [2]_. Sharing ports helps save FPGA resources by eliminating AXI interfaces, but it can limit the performance of the kernel because all the memory transfers have to go through a single port. The :code:`m_axi` port has independent READ and WRITE channels, so with a single :code:`m_axi` port, we can do reads and writes simultaneously but since we have two vectors from where we're reading (simultaneously), we can optimize the dataflows by simply asking for an extra AXI interface. 
    
    Note the bandwidth and throughput of the kernel can be increased by creating multiple  ports, using different bundle names, to connect multiple memory banks but this cames at the cost of resources in the PL fabric.
    
    
    To understand this better, it's important to also understand that in RTL design, input and output operations must be performed through a port in the design interface and typically operate using a specific I/O (input-output) protocol. The implementation of a function-level protocol is indicated in :code:`<mode>` after the :code:`#pragma HLS INTERFACE <mode>`. In this case, the pragma is using the :code:`m_axi` mode which corresponds with all ports as an AXI4 interface. The complete syntax of the pragma is as follows:

    :code:`#pragma HLS interface <mode> port=<name> bundle=<string>`

    where:

    - `<mode>` corresponds with the function-level protocol for the input/output operations through the RTL port
    - `<port>` specifies the name of the function argument, return value or global variable the pragma applies to
    - `<bundle>` groups function arguments into AXI interface ports. By default, HLS groups all function arguments specified as an AXI4 (:code:`m_axi` mode) interface into a single AXI4 port. This option explicitly groups all interface ports with the same :code:`bundle=<string>` into the same AXI interface port and names the RTL port the value specified by :code:`<string>`.

    In other words, the pragmas below define the :code:`INTERFACE` standards for the RTL ports of the :code:`vadd` function.
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

# build offloaded_doublevadd_publisher (about 18 min)
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis ros2acceleration accelerated_doublevadd_publisher

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

# restart the daemon that manages the acceleration kernels to create /lib/firmware/xilinx/accelerated_doublevadd_publisher
$ ros2 acceleration stop
$ ros2 acceleration start

# stop the daemon to create the shell.json files
$ ros2 acceleration stop

# add shell.json files
vi /lib/firmware/xilinx/accelerated_doublevadd_publisher/shell.json
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
$ cd /krs_ws/lib/accelerated_doublevadd_publisher

# Load the accelerator 
$ ros2 acceleration select accelerated_doublevadd_publisher

# restart the daemon that manages the acceleration kernels
$ ros2 acceleration stop
$ ros2 acceleration start

# list the accelerators
$ ros2 acceleration list
Accelerator                            Base                            Type             #slots         Active_slot

  accelerated_doublevadd_publisher  accelerated_doublevadd_publisher   XRT_FLAT         0                  -1
  offloaded_doublevadd_publisher  offloaded_doublevadd_publisher       XRT_FLAT         0                  -1
                            base                            base       XRT_FLAT         0                  -1

# select the accelerated_doublevadd_publisher
$ ros2 acceleration select accelerated_doublevadd_publisher

# launch binary 
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run accelerated_doublevadd_publisher offloaded_doublevadd_publisher

average rate: 4.914
        min: 0.203s max: 0.204s std dev: 0.00018s window: 6
[INFO] [1520600204.921584580] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 7'
[INFO] [1520600205.125141880] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 8'
[INFO] [1520600205.328703630] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1520600205.532187640] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1520600205.735649740] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
average rate: 4.914
        min: 0.203s max: 0.204s std dev: 0.00019s window: 10
[INFO] [1520600205.939115520] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1520600206.142593740] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
[INFO] [1520600206.346257330] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1520600206.550009410] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
[INFO] [1520600206.753585530] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'
average rate: 4.913
        min: 0.203s max: 0.204s std dev: 0.00033s window: 10
[INFO] [1520600206.957099840] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1520600207.160623620] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1520600207.364335140] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 19'
[INFO] [1520600207.568091420] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 20'
[INFO] [1520600207.771709770] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 21'

...
```

The optimizations in the dataflow introduced in the kernel via the use of the HLS `INTERFACE` pragma lead to a `5 Hz` publishing rate, which is about 10 times better than what was obtained before, but still insuficient to meet the `10 Hz` goal.

Next, in [5. Faster ROS 2 publisher](fasterROS2publisher.md), we'll review an even faster kernel that meets the 10 Hz publishing goal, by optimizing both the dataflow (as in this example) and exploiting `vadd` parallelism (via loop unrolling).

<!-- references -->
[^1]: 
[^2]: 


```eval_rst

.. [1] Managing Interface Synthesis. Retrieved from https://www.xilinx.com/html_docs/xilinx2020_2/vitis_doc/managing_interface_synthesis.html#jro1585107736856

.. [2] pragma HLS interface. Retrieved from https://www.xilinx.com/html_docs/xilinx2021_1/vitis_doc/hls_pragmas.html#jit1504034365862.

```
