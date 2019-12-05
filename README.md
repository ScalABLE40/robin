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


<!-- TODO -->
## Getting started

<!-- The bridge maps CODESYS variables to ROS topics through shared memory. -->

<!-- It uses shared memory for interprocess communication therefore, both sides of the bridge (ROS and CODESYS) must be running on the same system. -->

The bridge is made up of two components:
* A ROS package that doesn't require any manual configuration other than the installation of its dependencies. The package contains a ROS node that reads/writes data from/to shared memory spaces and publishes/receives messages to/from ROS topics.
* A CODESYS library to be used in a CODESYS project created by the user. An example project is provided. <!-- TODO point to example --> The library contains a 'Robin' function block that reads/writes data from/to shared memory spaces and writes/reads it to CODESYS user-defined variables.

The following IEC 61131-3 data types are currently supported:
* BOOL
* BYTE
* SINT, INT, DINT, LINT, USINT, UINT, UDINT, ULINT
* REAL, LREAL
* CHAR, STRING

As well as custom structs and arrays. Some standard ROS message packages are already defined as CODESYS structs and available on the CODESYS library. <!-- TODO list msg pkgs -->

These structs have to be defined on both the CODESYS project and the ROS package. For arrays or for structs with string or array members, because these data types are handled as non-POD (Plain Old Data) objects in C++, the mapping between the C++ variables and the ROS messages has to be explicitly defined. However, an updater application was developed to automate most of this process. The user simply needs to define its desired variables on the CODESYS project and run the updater.

<!-- TODO necessary? -->
The bridge was tested on [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) with [ROS Melodic](http://wiki.ros.org/melodic) and [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) with [ROS Kinetic](http://wiki.ros.org/kinetic).

<!-- TODO -->
### Prerequisites

* [Ubuntu 18.04](http://releases.ubuntu.com/18.04/)/[16.04](http://releases.ubuntu.com/16.04/) (may work on other distros as well)
* [ROS Melodic](http://wiki.ros.org/melodic)/[Kinetic](http://wiki.ros.org/kinetic)
* [CODESYS Development System V3](https://store.codesys.com/codesys.html?___store=en) (tested with version 3.5.15.0)
* CODESYS based PLC/SoftPLC

A CODESYS SoftPLC can be easily installed on the following systems:

* [Debian/Ubuntu](https://store.codesys.com/codesys-control-for-linux-sl.html?___store=en)
* [Raspberry Pi](https://store.codesys.com/codesys-control-for-raspberry-pi-sl.html?___store=en)
* [BeagleBone](https://store.codesys.com/codesys-control-for-beaglebone-sl.html?___store=en)

<!-- TODO prerequisites installation instructions (links?) -->

<!-- TODO -->
### Installation

1. Clone the repository into your catkin workspace (eg. _\~/catkin_ws_):
    ```sh
    cd ~/catkin_ws
    git clone https://github.com/ScalABLE40/robin
    ```

2. Install the package dependencies:
    ```sh
    rosdep install robin
    ```

* (optional) To avoid having to manually restart codesyscontrol after each update run:
    ```sh
    echo "$USER ALL=(ALL:ALL) NOPASSWD: /bin/systemctl restart codesyscontrol" | sudo EDITOR="tee" visudo -f /etc/sudoers.d/allow_restart_codesyscontrol
    ```
    This will allow the command `systemctl restart codesyscontrol` to be run with `sudo` without having to input a password. The user must belong to the _sudo_ group.


<!-- TODO -->
## Usage

<!-- Launch robin node -->

<!-- TODO -->
### Examples


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
