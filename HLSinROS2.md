# 2. HLS in ROS 2

|   | Source code |
|---|----------|
| [`simple_adder`](https://github.com/ros-acceleration/acceleration_examples/tree/main/simple_adder) | |
| **adder1** | |
| kernel 1 | [`adder1.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/simple_adder/src/adder1.cpp) |
| testbench 1 | [`testbench1.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/simple_adder/src/testbench1.cpp) |
| **adder2** | |
| kernel 2 | [`adder2.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/simple_adder/src/adder2.cpp) |
| testbench 2 | [`testbench2.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/simple_adder/src/testbench2.cpp) |


HLS is at the core of C++ hardware acceleration. KRS allows to maintain a ROS 2-centric view while leveraging HLS, optimizing the flow and empowering roboticists toperform everything from the CLI, attaching nicely to the ROS 2 meta build system (`ament`) and the ROS 2 meta build tools (`colcon`).

This example demonstrates how KRS helps to transition from the Xilinx's Vitis-centric flow to the ROS 2 flow.  The source code performs a trivial add operation. HLS is used directly from the ROS 2 CLI build tools. We explore the possiblities of offloading this operation to the Programmable Logic (PL). The latencies derived from the offloading operation are then analyzed both with the ROS 2 CLI tooling, and later, with the Vitis HLS GUI.

**Though we'll be targeting ultra96v2 hardware platform, no physical hardware is needed for this example.**

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to the install instructions.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## Prepare the environment 

```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
```

## A bit of background on HLS, RTL and FPGAs

```eval_rst
.. admonition:: About C simulation

    HLS C Simulation performs a pre-synthesis validation and checks that that the C program correctly implements the required functionality. This allows you to compile, simulate, and debug the C/C++ algorithm. C simulation allows for the function to be synthesized (e.g. `simple_adder` above) to be validated with a test bench using C simulation. A C test bench includes a main() top-level function, that calls the function to be synthesized by the Vitis HLS project. The test bench can also include other functions.

    Refer to `Verifying Code with C Simulation <https://www.xilinx.com/html_docs/xilinx2020_2/vitis_doc/verifyingcodecsimulation.html#amz1583532308782/>`_ for more.


.. admonition:: About Synthesis

    Synthesis transforms the C or C++ function into RTL code for acceleration in programmable logic. HLS allows you to control synthesis process using optimization pragmas to create high-performance implementations.

    Refer to `Synthesizing the Code <https://www.xilinx.com/html_docs/xilinx2020_2/vitis_doc/synthesizingcode.html#dtu1584226824270/>`_ for more.


.. admonition:: A further understanding of the *synthesis summary view*

    The *synthesis summary view* provides lots of information with different fields which may not always be self-explanatory. The list below tries to clarify some of the most relevant ones:

    - **Slack**: displays any timing issues in the implementation.
    - **Latency(cycles)/Latency(ns)**: displays the number of cycles it takes to produce the output. The same information is also displayed in ns.
    - **Iteration Latency**: latency of a single iteration for a loop.
    - **Interval**: Interval or Initiation Interval (II) is the number of clock cycles before new inputs can be applied.
    - **Trip Count**: displays the number of iterations of a specific loop in the implemented hardware. This reflects any unrolling of the loop in hardware.

.. admonition:: A quick review of the resources in an FPGA

    Xilinx FPGA designs contain the following core components: Configurable Logic Blocks (CLBs), Programmable Routing Blocks (PRBs), I/O Blocks (IOBs) and Digital Signal Processors (DSPs). Within CLBs we have 4 sub-components: Flip-flops (FFs), Loop-up-tables (LUTs), multiplexers (MUXs) and SRAM.

    When accounting for resources consumed, we focus particularly on the following which are further defined below:

    - **DSP**: DSPs have been optimized to implement various common digital signal processing functions with maximum performance and minimum logic resource utilization. They have functions to provide multipliers, adders, substractors, accumulators, coefficient register storage or a summation unit, amongst others.
    - **FF**: they are the smallest storage resource on the FPGA and each FF within a CLB is a binary register used to save logic states between clock cycles on an FPGA circuit.
    - **LUT**: stores a predefined list of outputs for every combination of inputs. LUTs provide a fast way to retrieve the output of a logic operation because possible results are stored and then referenced, rather than calculated.
    - **URAM**: UltraRAM is a large, lightweight memory block intended to allow the replacement of off-board memories enabling better overall performance.

```


## `simple_adder1`: a quick look into the flow

[`simple_adder`](https://github.com/ros-acceleration/acceleration_examples/blob/main/simple_adder/src/adder1.cpp) is pretty straightforward:

```cpp 
int simple_adder(int a, int b) {
    int c;
    c = a + b;
    return c;
}
```

We can build this example packaged as a ROS package (though it doesn't interact with the computational graph) as follows:

```bash
$ colcon acceleration select ultra96v2  # select the ultra96v2 firmware
$ colcon build  --merge-install --build-base=build-ultra96v2 --install-base=install-ultra96v2 --mixin ultra96v2 --packages-select simple_adder
```

The `simple_adder` package uses the `vitis_hls_generate_tcl` macro on its CMakeLists.txt. This macro commands the build system (`ament`) that when the package is built it should generate a Tcl script for `simple_adder`. This Tcl script will allow architects with HLS experience to use the traditional HLS interface to further optimize and investigate the kernel through C simulation, synthesis, RTL simulation, etc.

Here's a peek to the macro syntax used:

```cmake 
vitis_hls_generate_tcl(
  PROJECT 
    project_simpleadder1
  SRC 
    src/adder1.cpp
  HEADERS
    include
  TESTBENCH 
    src/testbench.cpp
  TOPFUNCTION 
    simple_adder
  CLOCK 
    4
  SYNTHESIS
)
```

??? note "The Xilinx Tcl traditional path"

    When using the `vitis_hls_generate_tcl` macro above, a script is automatically generated under `~/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2.tcl`:

```
    open_project -reset project_simpleadder2
    add_files  /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/adder2.cpp
    add_files -tb /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/testbench2.cpp -cflags "  -I /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/include -I /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/include"
    set_top simple_adder
    # solution_4ns
    open_solution -flow_target vitis solution_4ns
    set_part {xczu3eg-sbva484-1-e}
    create_clock -period 4
    csim_design -ldflags "-lOpenCL" -profile
    csynth_design

    # solution_10ns
    open_solution -flow_target vitis solution_10ns
    set_part {xczu3eg-sbva484-1-e}
    create_clock -period 10
    csim_design -ldflags "-lOpenCL" -profile
    csynth_design
    
    exit
```
    The resulting Tcl script can directly be launched with `vitis_hls -f <file-name>` allowing to perform C simulation and synthesis (in this particular case).

```
    bash
    $ vitis_hls -f ./build-ultra96v2/simple_adder/project_simpleadder2.tcl

   ****** Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
  **** SW Build 3367213 on Tue Oct 19 02:47:39 MDT 2021
  **** IP Build 3369179 on Thu Oct 21 08:25:16 MDT 2021
    ** Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.

source /tools/Xilinx/Vitis_HLS/2021.2/scripts/vitis_hls/hls.tcl -notrace
INFO: Applying HLS Y2K22 patch v1.2 for IP revision
INFO: [HLS 200-10] Running '/tools/Xilinx/Vitis_HLS/2021.2/bin/unwrapped/lnx64.o/vitis_hls'
INFO: [HLS 200-10] For user 'usuario' on host 'Laptop-i7' (Linux_x86_64 version 5.13.0-41-generic) on Wed May 18 16:24:27 -03 2022
INFO: [HLS 200-10] On os Ubuntu 20.04.4 LTS
INFO: [HLS 200-10] In directory '/media/usuario/SeagateLinux/krs_ws'
Sourcing Tcl script './build-ultra96v2/simple_adder/project_simpleadder2.tcl'
INFO: [HLS 200-1510] Running: open_project -reset project_simpleadder2 
INFO: [HLS 200-10] Creating and opening project '/media/usuario/SeagateLinux/krs_ws/project_simpleadder2'.
INFO: [HLS 200-1510] Running: add_files /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/adder2.cpp 
INFO: [HLS 200-10] Adding design file '/media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/adder2.cpp' to the project
INFO: [HLS 200-1510] Running: add_files -tb /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/testbench2.cpp -cflags   -I /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/include -I /media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/include 
INFO: [HLS 200-10] Adding test bench file '/media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/testbench2.cpp' to the project
INFO: [HLS 200-1510] Running: set_top simple_adder 
INFO: [HLS 200-1510] Running: open_solution -flow_target vitis solution_4ns 
INFO: [HLS 200-10] Creating and opening solution '/media/usuario/SeagateLinux/krs_ws/project_simpleadder2/solution_4ns'.
INFO: [HLS 200-1505] Using flow_target 'vitis'
Resolution: For help on HLS 200-1505 see www.xilinx.com/cgi-bin/docs/rdoc?v=2021.2;t=hls+guidance;d=200-1505.html
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_latency=64
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_alignment_byte_size=64
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_max_widen_bitwidth=512
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_rtl -register_reset_num=3
INFO: [HLS 200-1510] Running: set_part xczu3eg-sbva484-1-e 
INFO: [HLS 200-1611] Setting target device to 'xczu3eg-sbva484-1-e'
INFO: [HLS 200-1510] Running: create_clock -period 4 
INFO: [SYN 201-201] Setting up clock 'default' with a period of 4ns.
INFO: [HLS 200-1510] Running: csim_design -ldflags -lOpenCL -profile 
INFO: [SIM 211-2] *************** CSIM start ***************
INFO: [SIM 211-4] CSIM will launch CLANG as the compiler.
   Compiling ../../../../src/acceleration_examples/simple_adder/src/testbench2.cpp in debug mode
   Compiling ../../../../src/acceleration_examples/simple_adder/src/adder2.cpp in debug mode
   Generating csim.exe
Expected result: 10000, Got Result: 10000
Expected result: 10405, Got Result: 10405
Expected result: 10824, Got Result: 10824
Expected result: 11263, Got Result: 11263
Expected result: 11728, Got Result: 11728
Expected result: 12225, Got Result: 12225
Expected result: 12760, Got Result: 12760
Expected result: 13339, Got Result: 13339
Expected result: 13968, Got Result: 13968
Expected result: 14653, Got Result: 14653
   Generating dot files
INFO: [SIM 211-1] CSim done with 0 errors.
INFO: [SIM 211-3] *************** CSIM finish ***************
INFO: [HLS 200-111] Finished Command csim_design CPU user time: 1.58 seconds. CPU system time: 0.33 seconds. Elapsed time: 1.2 seconds; current allocated memory: -937.109 MB.
INFO: [HLS 200-1510] Running: csynth_design 
INFO: [HLS 200-111] Finished File checks and directory preparation: CPU user time: 0 seconds. CPU system time: 0 seconds. Elapsed time: 0 seconds; current allocated memory: 110.141 MB.
INFO: [HLS 200-10] Analyzing design file '/media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/adder2.cpp' ... 
INFO: [HLS 200-111] Finished Source Code Analysis and Preprocessing: CPU user time: 0.22 seconds. CPU system time: 0.17 seconds. Elapsed time: 1.19 seconds; current allocated memory: 111.574 MB.
INFO: [HLS 200-777] Using interface defaults for 'Vitis' flow target.
INFO: [HLS 214-284] Auto array partition mode is set into default.
INFO: [HLS 214-270] Starting automatic array partition analysis...
INFO: [HLS 200-111] Finished Compiling Optimization and Transform: CPU user time: 3.33 seconds. CPU system time: 0.25 seconds. Elapsed time: 3.64 seconds; current allocated memory: 112.266 MB.
INFO: [HLS 200-111] Finished Checking Pragmas: CPU user time: 0 seconds. CPU system time: 0 seconds. Elapsed time: 0 seconds; current allocated memory: 112.281 MB.
INFO: [HLS 200-10] Starting code transformations ...
INFO: [HLS 200-111] Finished Standard Transforms: CPU user time: 0.01 seconds. CPU system time: 0.01 seconds. Elapsed time: 0.07 seconds; current allocated memory: 112.773 MB.
INFO: [HLS 200-10] Checking synthesizability ...
INFO: [HLS 200-111] Finished Checking Synthesizability: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.01 seconds; current allocated memory: 112.840 MB.
INFO: [HLS 200-111] Finished Loop, function and other optimizations: CPU user time: 0.02 seconds. CPU system time: 0.01 seconds. Elapsed time: 0.04 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-111] Finished Architecture Synthesis: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.01 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-10] Starting hardware synthesis ...
INFO: [HLS 200-10] Synthesizing 'simple_adder' ...
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [HLS 200-42] -- Implementing module 'simple_adder' 
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [SCHED 204-11] Starting scheduling ...
INFO: [SCHED 204-11] Finished scheduling.
INFO: [HLS 200-111] Finished Scheduling: CPU user time: 0.03 seconds. CPU system time: 0.02 seconds. Elapsed time: 0.06 seconds; current allocated memory: 133.125 MB.
INFO: [BIND 205-100] Starting micro-architecture generation ...
INFO: [BIND 205-101] Performing variable lifetime analysis.
INFO: [BIND 205-101] Exploring resource sharing.
INFO: [BIND 205-101] Binding ...
INFO: [BIND 205-100] Finished micro-architecture generation.
INFO: [HLS 200-111] Finished Binding: CPU user time: 0.04 seconds. CPU system time: 0 seconds. Elapsed time: 0.04 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [HLS 200-10] -- Generating RTL for module 'simple_adder' 
INFO: [HLS 200-10] ----------------------------------------------------------------
WARNING: [RTGEN 206-101] Design contains AXI ports. Reset is fixed to synchronous and active low.
INFO: [RTGEN 206-500] Setting interface mode on port 'simple_adder/a' to 's_axilite & ap_none'.
INFO: [RTGEN 206-500] Setting interface mode on port 'simple_adder/b' to 's_axilite & ap_none'.
INFO: [RTGEN 206-500] Setting interface mode on function 'simple_adder' to 's_axilite & ap_ctrl_chain'.
INFO: [RTGEN 206-100] Bundling port 'return', 'b' and 'ap_local_deadlock' to AXI-Lite port control.
INFO: [RTGEN 206-100] Generating core module 'mul_32s_32s_32_3_1': 3 instance(s).
INFO: [RTGEN 206-100] Finished creating RTL model for 'simple_adder'.
INFO: [HLS 200-111] Finished Creating RTL model: CPU user time: 0.03 seconds. CPU system time: 0 seconds. Elapsed time: 0.03 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-111] Finished Generating all RTL models: CPU user time: 0.2 seconds. CPU system time: 0.01 seconds. Elapsed time: 0.3 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-111] Finished Updating report files: CPU user time: 0.19 seconds. CPU system time: 0 seconds. Elapsed time: 0.22 seconds; current allocated memory: 133.125 MB.
INFO: [VHDL 208-304] Generating VHDL RTL for simple_adder.
INFO: [VLOG 209-307] Generating Verilog RTL for simple_adder.
INFO: [HLS 200-789] **** Estimated Fmax: 372.72 MHz
INFO: [HLS 200-111] Finished Command csynth_design CPU user time: 4.1 seconds. CPU system time: 0.47 seconds. Elapsed time: 5.64 seconds; current allocated memory: 22.984 MB.
INFO: [HLS 200-1510] Running: open_solution -flow_target vitis solution_10ns 
INFO: [HLS 200-10] Creating and opening solution '/media/usuario/SeagateLinux/krs_ws/project_simpleadder2/solution_10ns'.
INFO: [HLS 200-1505] Using flow_target 'vitis'
Resolution: For help on HLS 200-1505 see www.xilinx.com/cgi-bin/docs/rdoc?v=2021.2;t=hls+guidance;d=200-1505.html
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_latency=64
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_alignment_byte_size=64
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_interface -m_axi_max_widen_bitwidth=512
INFO: [HLS 200-435] Setting 'open_solution -flow_target vitis' configuration: config_rtl -register_reset_num=3
INFO: [HLS 200-1510] Running: set_part xczu3eg-sbva484-1-e 
INFO: [HLS 200-1510] Running: create_clock -period 10 
INFO: [SYN 201-201] Setting up clock 'default' with a period of 10ns.
INFO: [HLS 200-1510] Running: csim_design -ldflags -lOpenCL -profile 
INFO: [SIM 211-2] *************** CSIM start ***************
INFO: [SIM 211-4] CSIM will launch CLANG as the compiler.
   Compiling ../../../../src/acceleration_examples/simple_adder/src/testbench2.cpp in debug mode
   Compiling ../../../../src/acceleration_examples/simple_adder/src/adder2.cpp in debug mode
   Generating csim.exe
Expected result: 10000, Got Result: 10000
Expected result: 10405, Got Result: 10405
Expected result: 10824, Got Result: 10824
Expected result: 11263, Got Result: 11263
Expected result: 11728, Got Result: 11728
Expected result: 12225, Got Result: 12225
Expected result: 12760, Got Result: 12760
Expected result: 13339, Got Result: 13339
Expected result: 13968, Got Result: 13968
Expected result: 14653, Got Result: 14653
   Generating dot files
INFO: [SIM 211-1] CSim done with 0 errors.
INFO: [SIM 211-3] *************** CSIM finish ***************
INFO: [HLS 200-111] Finished Command csim_design CPU user time: 0.42 seconds. CPU system time: 0.12 seconds. Elapsed time: 0.56 seconds; current allocated memory: 0.000 MB.
INFO: [HLS 200-1510] Running: csynth_design 
INFO: [HLS 200-111] Finished File checks and directory preparation: CPU user time: 0 seconds. CPU system time: 0 seconds. Elapsed time: 0 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-10] Analyzing design file '/media/usuario/SeagateLinux/krs_ws/src/acceleration_examples/simple_adder/src/adder2.cpp' ... 
INFO: [HLS 200-111] Finished Source Code Analysis and Preprocessing: CPU user time: 0.12 seconds. CPU system time: 0.11 seconds. Elapsed time: 0.22 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-777] Using interface defaults for 'Vitis' flow target.
INFO: [HLS 214-284] Auto array partition mode is set into default.
INFO: [HLS 214-270] Starting automatic array partition analysis...
INFO: [HLS 200-111] Finished Compiling Optimization and Transform: CPU user time: 3.18 seconds. CPU system time: 0.24 seconds. Elapsed time: 3.42 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-111] Finished Checking Pragmas: CPU user time: 0 seconds. CPU system time: 0 seconds. Elapsed time: 0 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-10] Starting code transformations ...
INFO: [HLS 200-111] Finished Standard Transforms: CPU user time: 0 seconds. CPU system time: 0 seconds. Elapsed time: 0 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-10] Checking synthesizability ...
INFO: [HLS 200-111] Finished Checking Synthesizability: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.01 seconds; current allocated memory: 133.125 MB.
INFO: [HLS 200-111] Finished Loop, function and other optimizations: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.02 seconds; current allocated memory: 146.219 MB.
INFO: [HLS 200-111] Finished Architecture Synthesis: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.01 seconds; current allocated memory: 146.219 MB.
INFO: [HLS 200-10] Starting hardware synthesis ...
INFO: [HLS 200-10] Synthesizing 'simple_adder' ...
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [HLS 200-42] -- Implementing module 'simple_adder' 
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [SCHED 204-11] Starting scheduling ...
INFO: [SCHED 204-11] Finished scheduling.
INFO: [HLS 200-111] Finished Scheduling: CPU user time: 0.03 seconds. CPU system time: 0.01 seconds. Elapsed time: 0.05 seconds; current allocated memory: 146.219 MB.
INFO: [BIND 205-100] Starting micro-architecture generation ...
INFO: [BIND 205-101] Performing variable lifetime analysis.
INFO: [BIND 205-101] Exploring resource sharing.
INFO: [BIND 205-101] Binding ...
INFO: [BIND 205-100] Finished micro-architecture generation.
INFO: [HLS 200-111] Finished Binding: CPU user time: 0.02 seconds. CPU system time: 0 seconds. Elapsed time: 0.02 seconds; current allocated memory: 146.219 MB.
INFO: [HLS 200-10] ----------------------------------------------------------------
INFO: [HLS 200-10] -- Generating RTL for module 'simple_adder' 
INFO: [HLS 200-10] ----------------------------------------------------------------
WARNING: [RTGEN 206-101] Design contains AXI ports. Reset is fixed to synchronous and active low.
INFO: [RTGEN 206-500] Setting interface mode on port 'simple_adder/a' to 's_axilite & ap_none'.
INFO: [RTGEN 206-500] Setting interface mode on port 'simple_adder/b' to 's_axilite & ap_none'.
INFO: [RTGEN 206-500] Setting interface mode on function 'simple_adder' to 's_axilite & ap_ctrl_chain'.
INFO: [RTGEN 206-100] Bundling port 'return', 'b' and 'ap_local_deadlock' to AXI-Lite port control.
INFO: [RTGEN 206-100] Generating core module 'mul_32s_32s_32_1_1': 3 instance(s).
INFO: [RTGEN 206-100] Finished creating RTL model for 'simple_adder'.
INFO: [HLS 200-111] Finished Creating RTL model: CPU user time: 0.01 seconds. CPU system time: 0 seconds. Elapsed time: 0.02 seconds; current allocated memory: 146.219 MB.
INFO: [HLS 200-111] Finished Generating all RTL models: CPU user time: 0.14 seconds. CPU system time: 0 seconds. Elapsed time: 0.15 seconds; current allocated memory: 146.219 MB.
INFO: [HLS 200-111] Finished Updating report files: CPU user time: 0.18 seconds. CPU system time: 0.02 seconds. Elapsed time: 0.19 seconds; current allocated memory: 146.219 MB.
INFO: [VHDL 208-304] Generating VHDL RTL for simple_adder.
INFO: [VLOG 209-307] Generating Verilog RTL for simple_adder.
INFO: [HLS 200-789] **** Estimated Fmax: 164.39 MHz
INFO: [HLS 200-111] Finished Command csynth_design CPU user time: 3.72 seconds. CPU system time: 0.38 seconds. Elapsed time: 4.13 seconds; current allocated memory: 13.094 MB.
INFO: [HLS 200-112] Total CPU user time: 11.26 seconds. Total CPU system time: 1.99 seconds. Total elapsed time: 14.05 seconds; peak allocated memory: 146.219 MB.
INFO: [Common 17-206] Exiting vitis_hls at Wed May 18 16:24:41 2022...
    
```

With KRS, we can avoid using Tcl and use instead the ROS 2 extensions to `colcon` to easily perform C simulation, synthesis, implementation and more. Before running things, ROS 2 tools will dump the following status:

```
bash
$ colcon acceleration hls simple_adder
 Project:  project_simpleadder2
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2
 	- Solution:  solution_4ns
 		- C Simulation:               Not Run
 		- C Synthesis:                Not Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
 	- Solution:  solution_10ns
 		- C Simulation:               Not Run
 		- C Synthesis:                Not Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
 Project:  project_simpleadder1
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1
 	- Solution:  solution_4ns
 		- C Simulation:               Not Run
 		- C Synthesis:                Not Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run

```

Let's run it and get some results:

```bash
$ colcon acceleration hls simple_adder --run
Found Tcl script "project_simpleadder2.tcl" for package: simple_adder
Executing /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2.tcl
Found Tcl script "project_simpleadder1.tcl" for package: simple_adder
Executing /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1.tcl
 Project:  project_simpleadder2
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
 	- Solution:  solution_10ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
 Project:  project_simpleadder1
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run

```

If we want to inspect the results from the CLI, we can add the `--synthesis-report` flag:

```bash
$ colcon acceleration hls simple_adder --synthesis-report

...

 Project:  project_simpleadder2
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
		- Synthesis report: /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2/solution_4ns/syn/report/simple_adder_csynth.rpt
 			
  			
  			================================================================
  			== Vitis HLS Report for 'simple_adder'
  			================================================================
  			* Date:           Wed May 18 16:32:14 2022
  			
  			* Version:        2021.2 (Build 3367213 on Tue Oct 19 02:47:39 MDT 2021)
  			* Project:        project_simpleadder2
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
  			    |ap_clk  |  4.00 ns|  2.683 ns|     1.08 ns|
  			    +--------+---------+----------+------------+
  			
  			+ Latency: 
  			    * Summary: 
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			    |  Latency (cycles) |   Latency (absolute)  |  Interval | Pipeline|
  			    |   min   |   max   |    min    |    max    | min | max |   Type  |
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			    |        7|        7|  28.000 ns|  28.000 ns|    8|    8|       no|
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			
  			    + Detail: 
  			        * Instance: 
  			        N/A
  			
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
  			|Expression       |        -|    -|       0|     41|    -|
  			|FIFO             |        -|    -|       -|      -|    -|
  			|Instance         |        0|    9|     649|    381|    0|
  			|Memory           |        -|    -|       -|      -|    -|
  			|Multiplexer      |        -|    -|       -|     58|    -|
  			|Register         |        -|    -|     108|      -|    -|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Total            |        0|    9|     757|    480|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Available        |      432|  360|  141120|  70560|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Utilization (%)  |        0|    2|      ~0|     ~0|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			
  			+ Detail: 
  			    * Instance: 
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |        Instance       |       Module       | BRAM_18K| DSP|  FF | LUT | URAM|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |control_s_axi_U        |control_s_axi       |        0|   0|  151|  234|    0|
  			    |mul_32s_32s_32_3_1_U1  |mul_32s_32s_32_3_1  |        0|   3|  166|   49|    0|
  			    |mul_32s_32s_32_3_1_U2  |mul_32s_32s_32_3_1  |        0|   3|  166|   49|    0|
  			    |mul_32s_32s_32_3_1_U3  |mul_32s_32s_32_3_1  |        0|   3|  166|   49|    0|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |Total                  |                    |        0|   9|  649|  381|    0|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			
  			    * DSP: 
  			    N/A
  			
  			    * Memory: 
  			    N/A
  			
  			    * FIFO: 
  			    N/A
  			
  			    * Expression: 
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |  Variable Name  | Operation| DSP| FF| LUT| Bitwidth P0| Bitwidth P1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |ap_return        |         +|   0|  0|  39|          32|          32|
  			    |ap_block_state1  |        or|   0|  0|   2|           1|           1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |Total            |          |   0|  0|  41|          33|          33|
  			    +-----------------+----------+----+---+----+------------+------------+
  			
  			    * Multiplexer: 
  			    +-----------+----+-----------+-----+-----------+
  			    |    Name   | LUT| Input Size| Bits| Total Bits|
  			    +-----------+----+-----------+-----+-----------+
  			    |ap_NS_fsm  |  49|          9|    1|          9|
  			    |ap_done    |   9|          2|    1|          2|
  			    +-----------+----+-----------+-----+-----------+
  			    |Total      |  58|         11|    2|         11|
  			    +-----------+----+-----------+-----+-----------+
  			
  			    * Register: 
  			    +-------------------+----+----+-----+-----------+
  			    |        Name       | FF | LUT| Bits| Const Bits|
  			    +-------------------+----+----+-----+-----------+
  			    |ap_CS_fsm          |   8|   0|    8|          0|
  			    |ap_done_reg        |   1|   0|    1|          0|
  			    |ap_rst_n_inv       |   1|   0|    1|          0|
  			    |ap_rst_reg_1       |   1|   0|    1|          0|
  			    |ap_rst_reg_2       |   1|   0|    1|          0|
  			    |mul_ln20_1_reg_78  |  32|   0|   32|          0|
  			    |mul_ln20_2_reg_83  |  32|   0|   32|          0|
  			    |mul_ln20_reg_73    |  32|   0|   32|          0|
  			    +-------------------+----+----+-----+-----------+
  			    |Total              | 108|   0|  108|          0|
  			    +-------------------+----+----+-----+-----------+
  			
  			
  			
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
  			|ap_local_block         |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_clk                 |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_rst_n               |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|interrupt              |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			+-----------------------+-----+-----+---------------+--------------+--------------+
  			
  	- Solution:  solution_10ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
		- Synthesis report: /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2/solution_10ns/syn/report/simple_adder_csynth.rpt
 			
  			
  			================================================================
  			== Vitis HLS Report for 'simple_adder'
  			================================================================
  			* Date:           Wed May 18 16:32:19 2022
  			
  			* Version:        2021.2 (Build 3367213 on Tue Oct 19 02:47:39 MDT 2021)
  			* Project:        project_simpleadder2
  			* Solution:       solution_10ns (Vitis Kernel Flow Target)
  			* Product family: zynquplus
  			* Target device:  xczu3eg-sbva484-1-e
  			
  			
  			================================================================
  			== Performance Estimates
  			================================================================
  			+ Timing: 
  			    * Summary: 
  			    +--------+----------+----------+------------+
  			    |  Clock |  Target  | Estimated| Uncertainty|
  			    +--------+----------+----------+------------+
  			    |ap_clk  |  10.00 ns|  6.083 ns|     2.70 ns|
  			    +--------+----------+----------+------------+
  			
  			+ Latency: 
  			    * Summary: 
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			    |  Latency (cycles) |   Latency (absolute)  |  Interval | Pipeline|
  			    |   min   |   max   |    min    |    max    | min | max |   Type  |
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			    |        1|        1|  10.000 ns|  10.000 ns|    2|    2|       no|
  			    +---------+---------+-----------+-----------+-----+-----+---------+
  			
  			    + Detail: 
  			        * Instance: 
  			        N/A
  			
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
  			|Expression       |        -|    -|       0|     41|    -|
  			|FIFO             |        -|    -|       -|      -|    -|
  			|Instance         |        0|    9|     151|    294|    0|
  			|Memory           |        -|    -|       -|      -|    -|
  			|Multiplexer      |        -|    -|       -|     32|    -|
  			|Register         |        -|    -|      70|      -|    -|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Total            |        0|    9|     221|    367|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Available        |      432|  360|  141120|  70560|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Utilization (%)  |        0|    2|      ~0|     ~0|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			
  			+ Detail: 
  			    * Instance: 
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |        Instance       |       Module       | BRAM_18K| DSP|  FF | LUT | URAM|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |control_s_axi_U        |control_s_axi       |        0|   0|  151|  234|    0|
  			    |mul_32s_32s_32_1_1_U1  |mul_32s_32s_32_1_1  |        0|   3|    0|   20|    0|
  			    |mul_32s_32s_32_1_1_U2  |mul_32s_32s_32_1_1  |        0|   3|    0|   20|    0|
  			    |mul_32s_32s_32_1_1_U3  |mul_32s_32s_32_1_1  |        0|   3|    0|   20|    0|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			    |Total                  |                    |        0|   9|  151|  294|    0|
  			    +-----------------------+--------------------+---------+----+-----+-----+-----+
  			
  			    * DSP: 
  			    N/A
  			
  			    * Memory: 
  			    N/A
  			
  			    * FIFO: 
  			    N/A
  			
  			    * Expression: 
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |  Variable Name  | Operation| DSP| FF| LUT| Bitwidth P0| Bitwidth P1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |c_fu_60_p2       |         +|   0|  0|  39|          32|          32|
  			    |ap_block_state1  |        or|   0|  0|   2|           1|           1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |Total            |          |   0|  0|  41|          33|          33|
  			    +-----------------+----------+----+---+----+------------+------------+
  			
  			    * Multiplexer: 
  			    +-----------+----+-----------+-----+-----------+
  			    |    Name   | LUT| Input Size| Bits| Total Bits|
  			    +-----------+----+-----------+-----+-----------+
  			    |ap_NS_fsm  |  14|          3|    1|          3|
  			    |ap_done    |   9|          2|    1|          2|
  			    |ap_return  |   9|          2|   32|         64|
  			    +-----------+----+-----------+-----+-----------+
  			    |Total      |  32|          7|   34|         69|
  			    +-----------+----+-----------+-----+-----------+
  			
  			    * Register: 
  			    +-----------------+----+----+-----+-----------+
  			    |       Name      | FF | LUT| Bits| Const Bits|
  			    +-----------------+----+----+-----+-----------+
  			    |ap_CS_fsm        |   2|   0|    2|          0|
  			    |ap_done_reg      |   1|   0|    1|          0|
  			    |ap_return_preg   |  32|   0|   32|          0|
  			    |ap_rst_n_inv     |   1|   0|    1|          0|
  			    |ap_rst_reg_1     |   1|   0|    1|          0|
  			    |ap_rst_reg_2     |   1|   0|    1|          0|
  			    |mul_ln20_reg_71  |  32|   0|   32|          0|
  			    +-----------------+----+----+-----+-----------+
  			    |Total            |  70|   0|   70|          0|
  			    +-----------------+----+----+-----+-----------+
  			
  			
  			
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
  			|ap_local_block         |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_clk                 |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_rst_n               |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|interrupt              |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			+-----------------------+-----+-----+---------------+--------------+--------------+
  			
  Project:  project_simpleadder1
 Path:  /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
		- Synthesis report: /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1/solution_4ns/syn/report/simple_adder_csynth.rpt
 			
  			
  			================================================================
  			== Vitis HLS Report for 'simple_adder'
  			================================================================
  			* Date:           Wed May 18 16:32:26 2022
  			
  			* Version:        2021.2 (Build 3367213 on Tue Oct 19 02:47:39 MDT 2021)
  			* Project:        project_simpleadder1
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
  			    |ap_clk  |  4.00 ns|  2.203 ns|     1.08 ns|
  			    +--------+---------+----------+------------+
  			
  			+ Latency: 
  			    * Summary: 
  			    +---------+---------+----------+----------+-----+-----+---------+
  			    |  Latency (cycles) |  Latency (absolute) |  Interval | Pipeline|
  			    |   min   |   max   |    min   |    max   | min | max |   Type  |
  			    +---------+---------+----------+----------+-----+-----+---------+
  			    |        0|        0|      0 ns|      0 ns|    1|    1|       no|
  			    +---------+---------+----------+----------+-----+-----+---------+
  			
  			    + Detail: 
  			        * Instance: 
  			        N/A
  			
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
  			|Expression       |        -|    -|       0|     41|    -|
  			|FIFO             |        -|    -|       -|      -|    -|
  			|Instance         |        0|    -|     151|    234|    0|
  			|Memory           |        -|    -|       -|      -|    -|
  			|Multiplexer      |        -|    -|       -|     18|    -|
  			|Register         |        -|    -|      37|      -|    -|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Total            |        0|    0|     188|    293|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Available        |      432|  360|  141120|  70560|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			|Utilization (%)  |        0|    0|      ~0|     ~0|    0|
  			+-----------------+---------+-----+--------+-------+-----+
  			
  			+ Detail: 
  			    * Instance: 
  			    +-----------------+---------------+---------+----+-----+-----+-----+
  			    |     Instance    |     Module    | BRAM_18K| DSP|  FF | LUT | URAM|
  			    +-----------------+---------------+---------+----+-----+-----+-----+
  			    |control_s_axi_U  |control_s_axi  |        0|   0|  151|  234|    0|
  			    +-----------------+---------------+---------+----+-----+-----+-----+
  			    |Total            |               |        0|   0|  151|  234|    0|
  			    +-----------------+---------------+---------+----+-----+-----+-----+
  			
  			    * DSP: 
  			    N/A
  			
  			    * Memory: 
  			    N/A
  			
  			    * FIFO: 
  			    N/A
  			
  			    * Expression: 
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |  Variable Name  | Operation| DSP| FF| LUT| Bitwidth P0| Bitwidth P1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |c_fu_44_p2       |         +|   0|  0|  39|          32|          32|
  			    |ap_block_state1  |        or|   0|  0|   2|           1|           1|
  			    +-----------------+----------+----+---+----+------------+------------+
  			    |Total            |          |   0|  0|  41|          33|          33|
  			    +-----------------+----------+----+---+----+------------+------------+
  			
  			    * Multiplexer: 
  			    +-----------+----+-----------+-----+-----------+
  			    |    Name   | LUT| Input Size| Bits| Total Bits|
  			    +-----------+----+-----------+-----+-----------+
  			    |ap_done    |   9|          2|    1|          2|
  			    |ap_return  |   9|          2|   32|         64|
  			    +-----------+----+-----------+-----+-----------+
  			    |Total      |  18|          4|   33|         66|
  			    +-----------+----+-----------+-----+-----------+
  			
  			    * Register: 
  			    +----------------+----+----+-----+-----------+
  			    |      Name      | FF | LUT| Bits| Const Bits|
  			    +----------------+----+----+-----+-----------+
  			    |ap_CS_fsm       |   1|   0|    1|          0|
  			    |ap_done_reg     |   1|   0|    1|          0|
  			    |ap_return_preg  |  32|   0|   32|          0|
  			    |ap_rst_n_inv    |   1|   0|    1|          0|
  			    |ap_rst_reg_1    |   1|   0|    1|          0|
  			    |ap_rst_reg_2    |   1|   0|    1|          0|
  			    +----------------+----+----+-----+-----------+
  			    |Total           |  37|   0|   37|          0|
  			    +----------------+----+----+-----+-----------+
  			
  			
  			
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
  			|ap_local_block         |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_clk                 |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|ap_rst_n               |   in|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			|interrupt              |  out|    1|  ap_ctrl_chain|  simple_adder|  return value|
  			+-----------------------+-----+-----+---------------+--------------+--------------+

```


Results show how the `simple_adder` function can be implemented with only a few LUTs and using only one single cycle (0 is counted as the first cycle). The FPGA can be programmed to run with a clock at `2.016 ns` (estimated). This means that **by using solely 293 LUTs and 188 FFs of the FPGA to build specialized circuitry, we can get deterministic responses when launching the `simple_adder` function with a maximum response period of (`2.203 ns` (estimated) + `1.08 ns` (uncertainty)). In other words, `3.283` ns**.

Let's compare this with the results we may get if we were to run this (only) on the PS system without relying on the FPGA. The Zynq UltraScale+ MPSoC targeted uses Quad-core Arm Cortex-A53 which has a CPU frequency of up to 1.5 GHz. Since the clock of the processor can theoretically run at 1.5 GHz, this means that one cycle has a period of approximately `0.666 ns`. If the CPU were to be able to fit the whole function into one cycle, this would is indeed better than the `3.283 ns` we estimated before when using the FPGA, however, things aren't that easy. Neither Von-Neumann-based CPU architectures can fit all operations into one cycle nor adaptive SoCs like the Zynq UltraScale+ MPSoC can leverage hardware acceleration without considering the interfacing with the FPGA. The following considerations should be taken into account:

1. CPUs control-driven machine model is based on a token of control, which indicates when a statement should be executed. This gives CPUs full control to implement easily complex data and control structures however, this also comes at the cost of being less efficient since every operation needs to push data in-and-out of ALUs (each operation needs to be managed in this control flow mechanism). Ultimately this leads to multiple cycles for even the simplest operations like what's illustrated in `simple_adder` above.

2. Besides the additional cycles required for the sole computation, the PS deals with tons of complex aspects that may very easily interrupt the computation, produce a switch of context and get back to the computation of `simple_adder` afterwards. Even in a somewhat ideal scenario, with a soft/firm real-time operating system running on the PS, the kernel can incur on delays of several tenths of microseconds.

3. FPGAs offer a deterministic response which can be specially exploited when relying onits resources, e.g. while driving I/O directly. In adaptive SoCs, interfacing the Processing System (PS, the CPU), with the Programmable Logic (PL, the FPGA) has a time/cycle cost which should be considered. If the output of the function/kernel is to return to the PS,  one should account for the complexity of such function/kernel. In very simple examples like the `simple_adder` above, this cost of interfacing PS-PL is generally much bigger than the optimizations one could get and the determinism is essentially degraded due to the involvement of the CPU.


### Using the Vitis HLS GUI

The project files generated by ROS 2 CLI tools can also be opened with `vitis_hls` GUI by pointing to the new project folder created. Within the Vitis HLS GUI, the various reports generated can be inspected graphically:

| Image | Comments |
|---|---|
| ![](../../_images/1.png) | The *synthesis summary view* shows that the target clock is `10ns`, as specified in the first solution in the Tcl script above. Note however that the synthesized clock ends up being much lower. The *Performance & Resource Estimates* section summarizes that overall timing characteristics. Note that the timing characteristics show a 0 latency |
| ![](../../_images/2.png) | The *schedule viewer view* shows how the two read operation are executed in the same clock and then get fed into the add operation. Everything gets executed in the same cycle. |
| ![](../../_images/3.png) | The *pre-synthesis control flow view* shows that this function has a trivial control flow.|
| ![](../../_images/4.png) | The *synthesis details view* shows a summary of the latency and also the resources consumed to synthesis the function. In this case only 39 LUTs.|


The data that we just inspected through Vitis HLS GUI is available in reports which can be parsed and exposed in a CLI interface. KRS does exactly this. KRS provides a series of CLI verbs and subverbs that allow to fetch this information directly from the CLI allowing ROS developers to create their own development flows.

Let's now complicate a bit more the `simple_adder` function and see how faster clocks aren't alwasy better. Specially, we show how FPGAs can be optimized to fit operations in less cycles delivering lower latencies.

## `simple_adder2`: optimizing FPGA synthesis for lower latency responses

The source code of `simple_adder2` will now be the following:


```cpp 
int simple_adder(int a, int b) {
    int c;
    c = a*a*a + b*b;
    return c;
}    
```

The CMakeLists.txt file uses now a different testbench and kernel source code:

```cmake 
vitis_hls_generate_tcl(
  PROJECT 
    project_simpleadder2
  SRC 
    src/adder2.cpp
  HEADERS
    include
  TESTBENCH 
    src/testbench2.cpp
  TOPFUNCTION 
    simple_adder
  CLOCK 
    4 5 6 7 8 9 10
  SYNTHESIS
)
```

Note that the macro will generate one solution per each `CLOCK` (in ns) argument provided. 

Let's compare the *Schedule Viewer* of the `4 ns` and `10 ns` clock solutions.


| Image | Comments |
|---|---|
| ![](../../_images/5.png) | Targeting `4 ns` clock. Note the whole operation takes 4 cycles. |
| ![](../../_images/6.png) | Targeting `10 ns` clock. Note the whole operation takes 2 cycles. |

Futher inspecting the solutions with `colcon` CLI extensions (see output above)

```bash
$ colcon acceleration hls simple_adder --synthesis-report
```

**We observe how using a target `10 ns` clock (which is slower than `4 ns`) leads to a) the use of less LUT and FF resources and b) a lower latency (due to a smaller number of cycles required)**. It's pretty interesting to note that getting a higher frequency in the clock does not necesarily mean we'll obtain a lower period for the function. This happens very clearly in this case.


`colcon` CLI tooling also allows to obtain a quick summary of all the solutions to evaluate time and use of resources:

```
bash
$ colcon acceleration hls simple_adder --summary
# /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder2.tcl
Solution#	tar.clk	est.clk		latency_max	BRAM_18K	DSP	FF		LUT
solution_10ns	10.00	6.083		10.000		0 (0%)		9 (2%)	221 (~0%)	367 (~0%)	
solution_4ns	4.00	2.683		28.000		0 (0%)		9 (2%)	757 (~0%)	480 (~0%)	
# /media/usuario/SeagateLinux/krs_ws/build-ultra96v2/simple_adder/project_simpleadder1.tcl
Solution#	tar.clk	est.clk		latency_max	BRAM_18K	DSP	FF		LUT
solution_4ns	4.00	2.203		0		0 (0%)		0 (0%)	188 (~0%)	293 (~0%)	
```

