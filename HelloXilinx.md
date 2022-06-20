# 1. Hello Xilinx

|   | Source code |
|---|----------|
| [`publisher_xilinx`](https://github.com/ros-acceleration/acceleration_examples/tree/main/publisher_xilinx) | |
| publisher | [`member_function_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/publisher_xilinx/member_function_publisher.cpp) |

This example lets you experience KRS further, walking you through the process of building and launching a ROS 2 package across different targets: in the workstation, and in the real hardware (the emulation is not supported yet)

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to the install instrucctions.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```
Let's setup the envirnonment in the workstation:

```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

```

Source Code:
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

*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
MinimalPublisher()
: Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

private:
void timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, Xilinx! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
};

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MinimalPublisher>());
rclcpp::shutdown();
return 0;
}

```

## Launch `hello_xilinx` example in the workstation
```bash
$ ros2 run publisher_xilinx member_function_publisher
[INFO] [1618407842.800443167] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618407843.300407127] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618407843.800389187] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```


## Launch `hello_xilinx` example in the ultra96v2 hardware

First, let's select the firmware for the target hardware:

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
$ colcon build --build-base=build-ultra96v2 --install-base=install-ultra96v2 --merge-install --mixin ultra96v2 --packages-select publisher_xilinx
```

Now copy to the rootfs in the sd card:
```bash
$ sudo cp -r install-ultra96v2/* /media/usuario/vos_2/krs_ws
$ sync
```

To launch the `hello_xilinx` package on the ultra96v2 board:

```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation
$ . /ros2_ws/local_setup.bash   # source the ROS 2 overlay workspace
$ ros2 run publisher_xilinx member_function_publisher  # launch the hello_xilinx example

[INFO] [1618478074.116887661] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618478074.611878275] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618478075.112175121] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```


