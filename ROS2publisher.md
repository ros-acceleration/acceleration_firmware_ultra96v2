# 0. ROS 2 publisher

|   | Source code |
|---|----------|
| [`vadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/vadd_publisher) | |
| publisher | [`vadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd_publisher.cpp) |
| [`doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/doublevadd_publisher) | |
| publisher | [`doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/doublevadd_publisher/src/doublevadd_publisher.cpp) |


This first example presents a trivial vector-add ROS 2 publisher, which adds two vector inputs in a loop, and tries to publish the result at 10 Hz. The ROS 2 package runs in the scalar processors (the CPUs). Step-by-step, the process is documented, walking through the different actions required to run the ROS 2 package in hardware, leveraging KRS capabilities. Afterwards, a slighly modified version of the publisher is presented which has additional computation demands. With these modifications, it becomes clear how the publisher isn't able to meet the publication goal anymore, which motivates the use of hardware acceleration.

The ultimate objective of this example is to generate a simple ROS 2-centric example that creates a CPU baseline to understand the value of hardware acceleration and how KRS facilitates it. Next examples will build upon this one.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to the install instrucctions

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `vadd_publisher`

<!-- KRS aims to provide a ROS 2-centric experience (see [here](/features/ros2centric/) for more) and instead of using external tools to build ROS 2 workspaces, extensions to ROS 2 build system (`ament`) and ROS build tools (`colcon`) are implemented which facilitate the process. -->
### Prepare the environment and fetch the example

We start by preparing the environment and fetching the source code of the example into our KRS workspace:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# fetch the source code of examples
$ git clone https://github.com/ros-acceleration/acceleration_examples src/acceleration_examples -b 0.2.0

# build the workspace
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash
```

### Inspecting the ROS 2 publisher

The publisher is a CPU-based average one. The source code of the publisher has been split between the `vadd` function ([vadd.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd.cpp)) and the rest ([vadd_publisher.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/vadd_publisher/src/vadd_publisher.cpp)) for simplicity.
The `vadd` (vector-add) function is as follows:

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
        for (int i = 0; i < size; ++i) {
        #pragma HLS loop_tripcount min = c_size max = c_size
            out[i] = in1[i] + in2[i];
        }
    }
}

```

The [`loop_tripcount`](https://www.xilinx.com/html_docs/xilinx2021_1/vitis_doc/hls_pragmas.html#sty1504034367099) is for analysis only and the pragma doesn't impact the function logic in any way. Instead, it allows HLS to identify how many iterations are expected in the loop to make time estimations. This will come handy if we want to run synthesis tests to estimate the timing it'll take for this function to run on a dedicated circuit in the FPGA.

### Building, creating the raw image and running in hardware

Let's build the example, create a raw SD card image and run it in the ultra96v2 board. First, let's select the firmware for the target hardware, ultra96v2:

```bash
$ colcon acceleration select ultra96v2
```

To verify that we indeed that the right firmware selected, look for the one marked with a "*" at the end of its name:
```bash
$ colcon acceleration list
ultra96v2*
```

Let's now build the package targeting the ultra96v2:

```bash
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis vadd_publisher
```

Let's now create a raw disk image for the SD card with PetaLinux's rootfs, a vanilla Linux 5.4.0 kernel and the ROS 2 overlay workspace we've just created for the ultra96v2. First we run a sudo command because the sd image creation needs root privileges so doing this the creation process won't ask for the password in the middle of the process

```bash
$ sudo ls -all
$ colcon acceleration linux vanilla --install-dir install-ultra96v2
if you receive a 
Something went wrong while creating sd card image.
Review the output: gzip: stdin: unexpected end of file
This means that the rootfs.cpio.gz file wasn't installed correctly, do a 
$ cp ~/krs_ws/src/acceleration/acceleration_firmware_ultra96v2/firmware/rootfs.cpio.gz ~/krs_ws/acceleration/firmware/ultra96v2/rootfs.cpio.gz
$ sync
and run the command again
```

We're now ready to run it on hardware. For that, we need to flash the `~/krs_ws/acceleration/firmware/select/sd_card.img` file into the SD card. One quick way to do it is as follows:

```bash
# first, find out where your SD card has been mapped, in my case, /dev/rdisk2
$ sudo diskutil umount /dev/rdisk2s1  # umount mounted partition
$ pv <your-path-to>/krs_ws/acceleration/firmware/select/sd_card.img | sudo dd of=/dev/rdisk2 bs=4M  # dd the image
```

There are other methods (for example [Balena Etcher](https://www.balena.io/etcher/)). **Make sure to flash `~/krs_ws/acceleration/firmware/select/sd_card.img` we just generated, and not some other image**.


Once flashed, connect the board to the computer via its USB/UART/JTAG FTDI adapter and power it on. Then, launch your favority serial console (e.g. `sudo putty /dev/ttyUSB1 -serial -sercfg 115200,8,n,1,N`). You should get the prompt (no need a password, it's a root prompt):
(wait about 50 seconds because the system tries to initialize the bluetooth device)

```bash
PetaLinux 2021.2 ultra96v2-petalinux ttyPS0


root@ultra96v2-petalinux:~# [   18.254730] Bluetooth: hci0: command 0x1001 tx timeout
[   26.446723] Bluetooth: hci0: Reading TI version information failed (-110)
[   26.453534] Bluetooth: hci0: download firmware failed, retrying...
[   29.006715] Bluetooth: hci0: command 0x1001 tx timeout
[   37.198720] Bluetooth: hci0: Reading TI version information failed (-110)
[   37.205525] Bluetooth: hci0: download firmware failed, retrying...
[   39.758710] Bluetooth: hci0: command 0x1001 tx timeout
[   47.950726] Bluetooth: hci0: Reading TI version information failed (-110)
[   47.957528] Bluetooth: hci0: download firmware failed, retrying...

root@ultra96v2-petalinux:~#
```

Then, we can launch the example:

```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /krs_ws/local_setup.bash     # source the ROS 2 overlay workspace we just
                                  # created. Note it has been copied to the SD
                                  # card image while being created.

$ ros2 topic hz /vector --window 10 &
$ ros2 run vadd_publisher vadd_publisher
...
average rate: 0.366
        min: 0.100s max: 26.450s std dev: 7.90509s window: 10
[INFO] [1520599329.775573550] [vadd_publisher]: Publishing: 'vadd finished, iteration: 2'
[INFO] [1520599329.875554600] [vadd_publisher]: Publishing: 'vadd finished, iteration: 3'
[INFO] [1520599329.975557770] [vadd_publisher]: Publishing: 'vadd finished, iteration: 4'
[INFO] [1520599330.075576430] [vadd_publisher]: Publishing: 'vadd finished, iteration: 5'
[INFO] [1520599330.175546340] [vadd_publisher]: Publishing: 'vadd finished, iteration: 6'
[INFO] [1520599330.275544080] [vadd_publisher]: Publishing: 'vadd finished, iteration: 7'
[INFO] [1520599330.375542480] [vadd_publisher]: Publishing: 'vadd finished, iteration: 8'
[INFO] [1520599330.475541450] [vadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1520599330.575540140] [vadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1520599330.675558050] [vadd_publisher]: Publishing: 'vadd finished, iteration: 11'
[INFO] [1520599330.775543820] [vadd_publisher]: Publishing: 'vadd finished, iteration: 12'
average rate: 10.003
        min: 0.100s max: 0.100s std dev: 0.00019s window: 10
[INFO] [1520599330.875545320] [vadd_publisher]: Publishing: 'vadd finished, iteration: 13'
[INFO] [1520599330.975541440] [vadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1520599331.075542830] [vadd_publisher]: Publishing: 'vadd finished, iteration: 15'
[INFO] [1520599331.175541270] [vadd_publisher]: Publishing: 'vadd finished, iteration: 16'
[INFO] [1520599331.275536760] [vadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1520599331.375539270] [vadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1520599331.475533740] [vadd_publisher]: Publishing: 'vadd finished, iteration: 19'
[INFO] [1520599331.575534500] [vadd_publisher]: Publishing: 'vadd finished, iteration: 20'
[INFO] [1520599331.675542850] [vadd_publisher]: Publishing: 'vadd finished, iteration: 21'
[INFO] [1520599331.775537550] [vadd_publisher]: Publishing: 'vadd finished, iteration: 22'
[INFO] [1520599331.875534850] [vadd_publisher]: Publishing: 'vadd finished, iteration: 23'
...
```

You should see that the ROS 2 publisher successfully publishes at approximately 10 Hz.

### Bonus: Getting a time intuition of a `vadd` acceleration kernel
What if instead of running the `vadd` in the scalar CPUs, we create a specific hardware circuit with the FPGA that runs this function? What'd would be the associated timing? In other words, a *robot chip* for the `vadd` computation.

Though going all the way down to implementing the harware cirtuit for `vadd` is beyond the scope of this example, we can get a quick intuition with HLS capabilities integrated in KRS. Let's get such intuition:

```bash
$ colcon acceleration hls --run --synthesis-report vadd_publisher
Found Tcl script "project_vadd_publisher.tcl" for package: vadd_publisher
Executing /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/vadd_publisher/project_vadd_publisher.tcl
 Project:  project_vadd_publisher
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/vadd_publisher/project_vadd_publisher
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
		- Synthesis report: /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/vadd_publisher/project_vadd_publisher/solution_4ns/syn/report/vadd_csynth.rpt
 			
  			
  			================================================================
  			== Vitis HLS Report for 'vadd'
  			================================================================
  			* Date:           Wed May 18 12:39:09 2022
  			
  			* Version:        2021.2 (Build 3367213 on Tue Oct 19 02:47:39 MDT 2021)
  			* Project:        project_vadd_publisher
  			* Solution:       solution_4ns (Vitis Kernel Flow Target)
  			* Product family: zynquplus
  			* Target device:  xczu3eg-sbva484-1-e
  			
  			
  			================================================================
  			== Performance Estimates
  			================================================================
  			+ Timing: 
  			    * Summary: 
  			    +--------+---------+----------+------------+
  			    |  Clock |  Target | Estimated| Uncertainty|
  			    +--------+---------+----------+------------+
  			    |ap_clk  |  4.00 ns|  2.920 ns|     1.08 ns|
  			    +--------+---------+----------+------------+
  			
  			+ Latency: 
  			    * Summary: 
  			    +---------+---------+----------+-----------+-----+------+---------+
  			    |  Latency (cycles) |  Latency (absolute)  |  Interval  | Pipeline|
  			    |   min   |   max   |    min   |    max    | min |  max |   Type  |
  			    +---------+---------+----------+-----------+-----+------+---------+
  			    |        2|     8338|  8.000 ns|  33.352 us|    3|  8339|       no|
  			    +---------+---------+----------+-----------+-----+------+---------+
  			
  			    + Detail: 
  			        * Instance: 
  			        +-----------------------------------------+-------------------------------+---------+---------+-----------+-----------+------+------+---------+
  			        |                                         |                               |  Latency (cycles) |   Latency (absolute)  |   Interval  | Pipeline|
  			        |                 Instance                |             Module            |   min   |   max   |    min    |    max    |  min |  max |   Type  |
  			        +-----------------------------------------+-------------------------------+---------+---------+-----------+-----------+------+------+---------+
  			        |grp_vadd_Pipeline_VITIS_LOOP_30_1_fu_97  |vadd_Pipeline_VITIS_LOOP_30_1  |     8267|     8267|  33.068 us|  33.068 us|  8267|  8267|       no|
  			        +-----------------------------------------+-------------------------------+---------+---------+-----------+-----------+------+------+---------+
  			
  			        * Loop: 
  			        N/A
  			
  			
  			
  			================================================================
  			== Utilization Estimates
  			================================================================
  			* Summary: 
  			+-----------------+---------+-----+--------+-------+-----+
  			|       Name      | BRAM_18K| DSP |   FF   |  LUT  | URAM|
  			+-----------------+---------+-----+--------+-------+-----+
  			|DSP              |        -|    -|       -|      -|    -|
  			|Expression       |        -|    -|       0|     26|    -|
  			|FIFO             |        -|    -|       -|      -|    -|
  			|Instance         |        4|    -|    1200|   1454|    0|
  			|Memory           |        -|    -|       -|      -|    -|
  			|Multiplexer      |        -|    -|       -|    568|    -|
  			|Register         |        -|    -|     295|      -|    -|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Total            |        4|    0|    1495|   2048|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Available        |      432|  360|  141120|  70560|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Utilization (%)  |       ~0|    0|       1|      2|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			
  			+ Detail: 
  			    * Instance: 
  			    +-----------------------------------------+-------------------------------+---------+----+-----+-----+-----+
  			    |                 Instance                |             Module            | BRAM_18K| DSP|  FF | LUT | URAM|
  			    +-----------------------------------------+-------------------------------+---------+----+-----+-----+-----+
  			    |control_s_axi_U                          |control_s_axi                  |        0|   0|  291|  490|    0|
  			    |gmem_m_axi_U                             |gmem_m_axi                     |        4|   0|  512|  580|    0|
  			    |grp_vadd_Pipeline_VITIS_LOOP_30_1_fu_97  |vadd_Pipeline_VITIS_LOOP_30_1  |        0|   0|  397|  384|    0|
  			    +-----------------------------------------+-------------------------------+---------+----+-----+-----+-----+
  			    |Total                                    |                               |        4|   0| 1200| 1454|    0|
  			    +-----------------------------------------+-------------------------------+---------+----+-----+-----+-----+
  			
  			    * DSP: 
  			    N/A
  			
  			    * Memory: 
  			    N/A
  			
  			    * FIFO: 
  			    N/A
  			
  			    * Expression: 
  			    +---------------------+----------+----+---+----+------------+------------+
  			    |    Variable Name    | Operation| DSP| FF| LUT| Bitwidth P0| Bitwidth P1|
  			    +---------------------+----------+----+---+----+------------+------------+
  			    |ap_block_state2_io   |       and|   0|  0|   2|           1|           1|
  			    |ap_block_state72     |       and|   0|  0|   2|           1|           1|
  			    |icmp_ln30_fu_107_p2  |      icmp|   0|  0|  20|          32|           1|
  			    |ap_block_state1      |        or|   0|  0|   2|           1|           1|
  			    +---------------------+----------+----+---+----+------------+------------+
  			    |Total                |          |   0|  0|  26|          35|           4|
  			    +---------------------+----------+----+---+----+------------+------------+
  			
  			    * Multiplexer: 
  			    +---------------+-----+-----------+-----+-----------+
  			    |      Name     | LUT | Input Size| Bits| Total Bits|
  			    +---------------+-----+-----------+-----+-----------+
  			    |ap_NS_fsm      |  377|         73|    1|         73|
  			    |ap_done        |    9|          2|    1|          2|
  			    |gmem_ARVALID   |    9|          2|    1|          2|
  			    |gmem_AWADDR    |   14|          3|   64|        192|
  			    |gmem_AWBURST   |    9|          2|    2|          4|
  			    |gmem_AWCACHE   |    9|          2|    4|          8|
  			    |gmem_AWID      |    9|          2|    1|          2|
  			    |gmem_AWLEN     |   14|          3|   32|         96|
  			    |gmem_AWLOCK    |    9|          2|    2|          4|
  			    |gmem_AWPROT    |    9|          2|    3|          6|
  			    |gmem_AWQOS     |    9|          2|    4|          8|
  			    |gmem_AWREGION  |    9|          2|    4|          8|
  			    |gmem_AWSIZE    |    9|          2|    3|          6|
  			    |gmem_AWUSER    |    9|          2|    1|          2|
  			    |gmem_AWVALID   |   14|          3|    1|          3|
  			    |gmem_BREADY    |   14|          3|    1|          3|
  			    |gmem_RREADY    |    9|          2|    1|          2|
  			    |gmem_WVALID    |    9|          2|    1|          2|
  			    |gmem_blk_n_AW  |    9|          2|    1|          2|
  			    |gmem_blk_n_B   |    9|          2|    1|          2|
  			    +---------------+-----+-----------+-----+-----------+
  			    |Total          |  568|        115|  129|        427|
  			    +---------------+-----+-----------+-----+-----------+
  			
  			    * Register: 
  			    +------------------------------------------------------+----+----+-----+-----------+
  			    |                         Name                         | FF | LUT| Bits| Const Bits|
  			    +------------------------------------------------------+----+----+-----+-----------+
  			    |ap_CS_fsm                                             |  72|   0|   72|          0|
  			    |ap_done_reg                                           |   1|   0|    1|          0|
  			    |ap_rst_n_inv                                          |   1|   0|    1|          0|
  			    |ap_rst_reg_1                                          |   1|   0|    1|          0|
  			    |ap_rst_reg_2                                          |   1|   0|    1|          0|
  			    |grp_vadd_Pipeline_VITIS_LOOP_30_1_fu_97_ap_start_reg  |   1|   0|    1|          0|
  			    |icmp_ln30_reg_167                                     |   1|   0|    1|          0|
  			    |trunc_ln30_1_reg_181                                  |  62|   0|   62|          0|
  			    |trunc_ln30_2_reg_186                                  |  62|   0|   62|          0|
  			    |trunc_ln30_reg_171                                    |  31|   0|   31|          0|
  			    |trunc_ln_reg_176                                      |  62|   0|   62|          0|
  			    +------------------------------------------------------+----+----+-----+-----------+
  			    |Total                                                 | 295|   0|  295|          0|
  			    +------------------------------------------------------+----+----+-----+-----------+
  			
  			
  			
  			================================================================
  			== Interface
  			================================================================
  			* Summary: 
  			+-----------------------+-----+-----+---------------+--------------+--------------+
  			|       RTL Ports       | Dir | Bits|    Protocol   | Source Object|    C Type    |
  			+-----------------------+-----+-----+---------------+--------------+--------------+
  			|s_axi_control_AWVALID  |   in|    1|          s_axi|       control|        scalar|
  			|s_axi_control_AWREADY  |  out|    1|          s_axi|       control|        scalar|
  			|s_axi_control_AWADDR   |   in|    6|          s_axi|       control|        scalar|
  			|s_axi_control_WVALID   |   in|    1|          s_axi|       control|        scalar|
  			|s_axi_control_WREADY   |  out|    1|          s_axi|       control|        scalar|
  			|s_axi_control_WDATA    |   in|   32|          s_axi|       control|        scalar|
  			|s_axi_control_WSTRB    |   in|    4|          s_axi|       control|        scalar|
  			|s_axi_control_ARVALID  |   in|    1|          s_axi|       control|        scalar|
  			|s_axi_control_ARREADY  |  out|    1|          s_axi|       control|        scalar|
  			|s_axi_control_ARADDR   |   in|    6|          s_axi|       control|        scalar|
  			|s_axi_control_RVALID   |  out|    1|          s_axi|       control|        scalar|
  			|s_axi_control_RREADY   |   in|    1|          s_axi|       control|        scalar|
  			|s_axi_control_RDATA    |  out|   32|          s_axi|       control|        scalar|
  			|s_axi_control_RRESP    |  out|    2|          s_axi|       control|        scalar|
  			|s_axi_control_BVALID   |  out|    1|          s_axi|       control|        scalar|
  			|s_axi_control_BREADY   |   in|    1|          s_axi|       control|        scalar|
  			|s_axi_control_BRESP    |  out|    2|          s_axi|       control|        scalar|
  			|ap_local_block         |  out|    1|  ap_ctrl_chain|          vadd|  return value|
  			|ap_clk                 |   in|    1|  ap_ctrl_chain|          vadd|  return value|
  			|ap_rst_n               |   in|    1|  ap_ctrl_chain|          vadd|  return value|
  			|interrupt              |  out|    1|  ap_ctrl_chain|          vadd|  return value|
  			|m_axi_gmem_AWVALID     |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWREADY     |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWADDR      |  out|   64|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWID        |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWLEN       |  out|    8|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWSIZE      |  out|    3|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWBURST     |  out|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWLOCK      |  out|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWCACHE     |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWPROT      |  out|    3|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWQOS       |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWREGION    |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_AWUSER      |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WVALID      |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WREADY      |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WDATA       |  out|   32|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WSTRB       |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WLAST       |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WID         |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_WUSER       |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARVALID     |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARREADY     |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARADDR      |  out|   64|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARID        |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARLEN       |  out|    8|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARSIZE      |  out|    3|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARBURST     |  out|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARLOCK      |  out|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARCACHE     |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARPROT      |  out|    3|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARQOS       |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARREGION    |  out|    4|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_ARUSER      |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RVALID      |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RREADY      |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RDATA       |   in|   32|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RLAST       |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RID         |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RUSER       |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_RRESP       |   in|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_BVALID      |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_BREADY      |  out|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_BRESP       |   in|    2|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_BID         |   in|    1|          m_axi|          gmem|       pointer|
  			|m_axi_gmem_BUSER       |   in|    1|          m_axi|          gmem|       pointer|
  			+-----------------------+-----+-----+---------------+--------------+--------------+
  			
...

```

The output will show you that the `vadd` function with all its complexity can run with a `4 ns` clock (we could ask for other clocks) in a total of `33.336 us`. This result is **completely deterministic** from the vadd viewpoint, after all, the FPGA would create a specialized hardware cirtuit for the `vadd` computations. For what concerns vadd, there's no more deterministic execution than this. This is just a brief introduction, *if you wish to learn more about HLS and KRS, head to the second example: [1. Hello Xilinx](HelloXilinx.md)*.

## `doublevadd_publisher`

We've seen that as a simple ROS 2 package, `vadd_publisher` runs perfectly fine and meets the 10 Hz publishing objective. But what if the `vadd` has bigger vectors, or has operations that involve many more iterations? This second section explores this with a more computationally expensive publisher, the `doublevadd_publisher`. The source code of the new `vadd` function is presented below:

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

Note how instead of one `for` loop, we now have two, simulating a more complex computation. Let's try this out in hardware. Provided that the image was previously created, we just need to replace the ROS 2 workspace, the rest should be identical. You can do this in various ways (physically mounting the raw image in the SD card, `scp`-ing the ROS 2 workspace, etc.).

Briefly, in the workstation:

First we need to rebuild the vitis_common library for the ultra96 (when you do a "colcon build --merge-install", it builds for x86_64

```
bash
# generate the vitis_common.a library and copy it to the install directory
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select vitis_common
$ cp ./install-ultra96v2/lib/libvitis_common.a ./install/lib
```
Now we build the `doublevadd_publisher`

```bash
# generate the workspace with doublevadd_publisher (if exists already, add to it)
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select ament_vitis doublevadd_publisher

# copy to rootfs in SD card, e.g.
$ sudo cp -r install-ultra96v2/* /media/usuario/vos_2/krs_ws
$ sync
```
And in the board:

```
# Launch doublevadd_publisher
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /ros2_ws/local_setup.bash     # source the ROS 2 overlay workspace we just
                                # created. Note it has been copied to the SD
                                # card image while being created.

$ ros2 topic hz /vector --window 10 &
$ ros2 run doublevadd_publisher doublevadd_publisher

...
average rate: 2.002
        min: 0.499s max: 0.500s std dev: 0.00048s window: 7
[INFO] [1520602117.155020380] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 8'
[INFO] [1520602117.654437760] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1520602118.153748470] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
average rate: 2.002
        min: 0.499s max: 0.500s std dev: 0.00048s window: 10
[INFO] [1520602118.653036080] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
[INFO] [1520602119.152649810] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1520602119.652019040] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
average rate: 2.002
        min: 0.499s max: 0.500s std dev: 0.00049s window: 10
[INFO] [1520602120.151378600] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1520602120.650732750] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
[INFO] [1520602121.150203450] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'

...
```

This new publisher achieves only `2.0 Hz`, quite far from the `10 Hz` targeted. Using hardware acceleration, future examples will demonstrate how to build a custom compute pipeline that offloads computations to a kernel. *If you wish to jump directly into hardware acceleration with `doublevadd_publisher`, head to: [3. Offloading ROS 2 publisher](offloadingROS2publisher.md)*.
