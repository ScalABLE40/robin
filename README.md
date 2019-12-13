# ROBIN

[![Build status](https://travis-ci.org/ScalABLE40/robin.svg?branch=master)](https://travis-ci.org/ScalABLE40/robin) [![ROBIN project](https://img.shields.io/badge/project-ROBIN-informational)](https://rosin-project.eu/ftp/robin)

A ROS-CODESYS shared memory bridge to map CODESYS variables to ROS topics.

This bridge is the result of the [ROBIN](https://rosin-project.eu/ftp/robin) project, a Focused Technical Project (FTP) of the [ROSIN](https://rosin-project.eu/) European project.

<!-- 
## Table of contents

* [Getting started](#getting-started)
    * [Prerequisites](#prerequisites)
    * [Installation](#installation)
* [Usage](#usage)
    * [Examples](#examples)
* [License](#license) -->


<!-- TODO -->
<!-- ## About -->


<!-- TODO -->
<!-- ### Built With -->


## Getting started

<!-- The bridge maps CODESYS variables to ROS topics through shared memory. -->

<!-- It uses shared memory for interprocess communication therefore, both sides of the bridge (ROS and CODESYS) must be running on the same system. -->

The bridge is made up of two components:
* A ROS package that doesn't require any manual configuration other than the installation of its dependencies. The package contains a ROS node that reads/writes data from/to shared memory spaces and publishes/receives messages to/from ROS topics.
* A CODESYS library to be used in a CODESYS project created by the user. An example project is provided in [__robin_updater/src/robin_updater/cfg/codesys_project.xml__](https://github.com/ScalABLE40/robin/blob/develop/robin_updater/cfg/codesys_project.xml). The library contains a _Robin_ function block that reads/writes data from/to shared memory spaces and writes/reads it to CODESYS user-defined variables.

The following IEC 61131-3 data types are currently supported:
* BOOL
* BYTE
* SINT, INT, DINT, LINT, USINT, UINT, UDINT, ULINT
* REAL, LREAL
* CHAR, STRING

As well as __arrays__ and __custom structs__. The following standard ROS message packages are already defined as CODESYS structs and available on the Robin CODESYS library: <!-- TODO list msg pkgs -->
* [std_msgs](http://wiki.ros.org/std_msgs)
* [geometry_msgs](http://wiki.ros.org/geometry_msgs)

These variables have to be defined on both the CODESYS project and the ROS package. For arrays or for structs with string or array members, because these data types are handled as non-POD (Plain Old Data) objects in C++, the mapping between the C++ variables and the ROS messages has to be explicitly defined. However, an updater application was developed to automate most of this process. The user simply needs to define its desired variables on the CODESYS project and run the updater.

<!-- The bridge was tested on [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) with [ROS Melodic](http://wiki.ros.org/melodic) and [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) with [ROS Kinetic](http://wiki.ros.org/kinetic). -->

### Prerequisites

* [Ubuntu 18.04](http://releases.ubuntu.com/18.04/)/[16.04](http://releases.ubuntu.com/16.04/) system (may work on other distros as well) with:
    * [ROS Melodic](http://wiki.ros.org/melodic)/[Kinetic](http://wiki.ros.org/kinetic)
    * CODESYS Control SoftPLC application:
        * [Debian/Ubuntu](https://store.codesys.com/codesys-control-for-linux-sl.html?___store=en)
        * [Raspberry Pi](https://store.codesys.com/codesys-control-for-raspberry-pi-sl.html?___store=en)
        * [BeagleBone](https://store.codesys.com/codesys-control-for-beaglebone-sl.html?___store=en)

* Windows system with:
    * [CODESYS Development System V3](https://store.codesys.com/codesys.html?___store=en) (developed and tested with version 3.5.15.0)

<!-- TODO? prerequisites installation instructions (links?) -->

### Installation

1. Install CODESYS library:
    1. Open CODESYS Development System V3
    2. Go to _Tools->Library Repository->Install_
    3. Find and select _robin.library_ from the repo
    4. Close the _Library Repository_ dialog

2. Create catkin workspace (if non-existent):
    ```sh
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    ```

3. Clone repository into catkin workspace (eg. __\~/catkin_ws__):
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/ScalABLE40/robin
    ```

4. Install updater package dependencies:
    ```sh
    rosdep install robin_updater
    ```

5. Compile bridge package:
    ```sh
    cd ~/catkin_ws
    catkin_make robin  # or 'catkin build robin'
    ```
<!-- TODO 'source' line needed? -->


## Usage

1. Create CODESYS project. You can either:
    * Create your own project and add the Robin library to it.
        1. In the _Devices_ tree, double click _Library Manager_ and open the _Add Library_ dialog
        2. Find and select the previously installed _Robin_ library and click _OK_
        3. You can now use the _Robin_ function block as shown in the [Examples](#examples) section
    * Create a new __empty__ project and import the example project from [__codesys_project.xml__](https://github.com/ScalABLE40/robin/blob/develop/robin_updater/cfg/codesys_project.xml).
        1. Go to _Project->Import PLCopenXML..._
        2. Find and select the XML file
        3. Select all items and click _OK_

    <!-- Start the definition of custom CODESYS structs with the line: `{attribute 'pack_mode' := '0'}`. -->

    Variable length arrays are only partially supported in CODESYS. To make the updater interpret a regular fixed length array as a ROS variable length array, preceed its declaration with the line: `{attribute 'robin_var_len'}`.

2. Launch ROS node:
    ```sh
    rosrun robin_bridge robin_node
    ```

3. Run the updater application:
    1. Go to _Tools->Scripting->Execute Script File..._
    2. Open the script file [__robin_updater/src/robin_updater/src/robin_updater/start_update.py__](https://github.com/ScalABLE40/robin/blob/develop/robin_updater/src/robin_updater/start_update.py)
        * If you don't have access to it from CODESYS, first copy it to your Windows system
    3. Input the requested information and follow the script's execution

4. Restart the codesyscontrol service (if using SoftPLC):
    ```sh
    sudo systemctl restart codesyscontrol
    ```
    
    To avoid having to manually restart codesyscontrol after each update run:
    ```sh
    echo "$USER ALL=(ALL:ALL) NOPASSWD: /bin/systemctl restart codesyscontrol" | sudo EDITOR="tee" visudo -f /etc/sudoers.d/allow_restart_codesyscontrol
    ```
    This will allow the command `systemctl restart codesyscontrol` to be run with `sudo` without having to input a password. The user must be in the _sudo_ group.

<!-- ### Without updater

1. Update ROS package:
    1. Define any custom structs and messages in [__include/robin/structs.h__](https://github.com/ScalABLE40/robin/blob/release_manual/include/robin/structs.h) and [__msg/__](https://github.com/ScalABLE40/robin/blob/release_manual/msg) respectively.
    2. If using strings or arrays, define the mapping between the C++ variables and the ROS messages in [__src/robin/robin_inst.cpp__](https://github.com/ScalABLE40/robin/blob/release_manual/src/robin/robin_inst.cpp)
    3. Instantiate the _Robin_ classes used by adding a line such as the one below to [__robin_inst.cpp__](https://github.com/ScalABLE40/robin/blob/release_manual/src/robin/robin_inst.cpp).
        ```c++
        template class RobinSubscriber<double, std_msgs::Float64>;
        ```

2. Compile ROS package and run node:
    ```sh
    cd ~/catkin_ws
    catkin_make robin  # or 'catkin build robin'
    rosrun robin robin
    ``` -->

<!-- TODO -->
### Examples

![Example 1](https://raw.githubusercontent.com/ScalABLE40/robin/develop/doc/examples/usage_example1.png)

<!-- TODO -->
<!-- ## Running the tests -->


<!-- TODO -->
<!-- ## Development setup -->


<!-- TODO -->
<!-- ## Deployment -->


<!-- TODO -->
<!-- ## Release history -->


<!-- TODO -->
<!-- ## Roadmap -->


<!-- TODO -->
<!-- ## Contributing -->


<!-- TODO -->
<!-- ## Authors -->


## License

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


<!-- TODO -->
<!-- ## Contact -->


<!-- TODO -->
<!-- ## Acknowledgements -->
