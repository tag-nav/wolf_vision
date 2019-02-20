WOLF - Windowed Localization Frames
===================================

Overview
--------

Wolf is a library to solve localization problems in mobile robotics, such as SLAM, map-based localization, or visual odometry. The approach contemplates the coexistence of multiple sensors of different kind, be them synchronized or not. It is thought to build state vectors formed by a set of key-frames (window), defining the robot trajectory, plus other states such as landmarks or sensor parameters for calibration, and compute error vectors given the available measurements in that window.

Wolf is mainly a structure for having the data accessible and organized, plus some functionality for managing this data. It requires, on one side, several front-ends (one per sensor, or per sensor type), and a back-end constituted by the solver.

Wolf may be interfaced with many kinds of solvers, including filters and nonlinear optimizers. It can be used with EKF, error-state KF, iterative EKF, and other kinds of filters such as information filters. It can also be used with nonlinear optimizers, most especially -- though not necessarily -- in their sparse flavors, and in particular the incremental ones.

The task of interfacing Wolf with these solvers is relegated to wrappers, which are coded out of Wolf. We provide a wrapper to Google CERES. We also provide an experimental wrapper-less solver for incremental, sparse, nonlinear optimization based on the QR decomposition (implementing the iSAM algorithm).

The basic Wolf structure is a tree of base classes reproducing the elements of the robotic problem. This is called the Wolf Tree. It has a robot, with sensors, with a trajectory formed by keyframes, and the map with its landmarks. These base classes can be derived to build the particularizations you want. You have the basic functionality in the base classes, and you add what you want on top. The Wolf Tree connectivity is augmented with the constraints linking different parts of it, becoming a real network of relations. This network is equivalent to the factor graph that would be solved by graphical models and nonlinear optimization. Wrappers are the ones transferring the Wolf structure into a factor graph that can be provided to the solver. See the documentation for a proper rationale and all the details.

Wolf can be used within ROS for an easy integration. We provide examples of ROS nodes using Wolf. Wolf can also be used in other robotics frameworks.

#### Features

-   Keyframe based
-   Multi-sensor
-   Pose-SLAM, landmark-based SLAM, or visual odometry
-   Different state types -- the state blocks
-   Different measurement models -- the constraint blocks
-   Built with polymorphic classes using virtual inheritance and templates.
-   Solver agnostic: choose your solver and build your wrapper to Wolf.

#### Some preliminary documentation

-   You can visit this [Wolf inspiring document](https://docs.google.com/document/d/1_kBtvCIo33pdP59M3Ib4iEBleDDLcN6yCbmwJDBLtcA). Contact [Joan](mailto:jsola@iri.upc.edu) if you need permissions for the link.
-   You can also have a look at the [Wolf tree](https://docs.google.com/drawings/d/1jj5VVjQThddswpTPMLG2xv87vtT3o1jiMJo3Mk1Utjg), showing the organization of the main elements in the Wolf project. Contact [Andreu](mailto:acorominas@iri.upc.edu) if you need permissions for the link.
-   You can finally visit this [other inspiring document](https://docs.google.com/document/d/18XQlgdfTwplakYKKsfw2YAoaVuSyUEQoaU6bYHQDNpU) providing the initial motivation for the Wolf project. Contact [Joan](mailto:jsola@iri.upc.edu) if you need permissions for the link.

Dependencies
------------

! Please notice that we are detailing two installation procedures below. If you are familiar with `ROS` and more especially the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/index.html) package then you may jump directly to the 'Using the `catkin_tools` package' section.

#### CMake 
Building tool used by Wolf and by some of its dependencies. In order to install *cmake* please follow the instructions at [cmake site](https://cmake.org/install/)

#### Autoreconf

    $ sudo apt install dh-autoreconf

#### Eigen

[Eigen](http://eigen.tuxfamily.org). Linear algebra, header library. Eigen 3.2 is also a depencency of ROS-Hydro. In case you don't have ROS in your machine, you can install Eigen by typing:

    $ sudo apt-get install libeigen3-dev

#### Ceres (5 steps)

[Ceres](http://www.ceres-solver.org/) is an optimization library. Currently, this dependency is optional, so the build procedure of Wolf skips part of compilation in case this dependency is not found on the system. **Installation** is described at [Ceres site](http://www.ceres-solver.org/building.html). However we report here an alternative step by step procedure to install Ceres.

**(1)** Skip this step if Cmake 2.8.0+ and Eigen3.0+ are already installed. Otherwise install them with *apt-get*.

**(2) GFLAGS**

-   Git clone the source:

        $ git clone https://github.com/gflags/gflags.git
    
-   Build and install with:

        $ cd gflags
        $ mkdir build
        $ cd build
        $ cmake -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" -DGFLAGS_NAMESPACE="google" ..
        $ make
        $ sudo make install 
    
libgflags.a will be installed at **/usr/local/lib**

**(3) GLOG**

-   Git clone the source:

        $ git clone https://github.com/google/glog.git
    
-   Build and install with:

        $ cd glog
        $ ./autogen.sh
        $ ./configure --with-gflags=/usr/local/
        $ make
        $ sudo make install

libglog.so will be installed at **/usr/local/lib**

-   Tourbleshooting:

    * If ./autogen.sh fails with './autogen.sh: autoreconf: not found'

    In a fresh installation you will probably need to install autoreconf running
    
        $ sudo apt-get install dh-autoreconf 

    * If `make` command fails with the error: `/bin/bash: aclocal-1.14: command not found`
    
    Install Glog with the following commands:
        
        $ cd glog
        $ sudo apt-get install autoconf
        $ autoreconf --force --install
        $ ./configure --with-gflags=/usr/local/
        $ make
        $ sudo make install
    
**(4) SUITESPARSE**

-   Easy way!:

        $ sudo apt-get install libsuitesparse-dev
    
**(5) CERES**
    
-   Git clone the source:

        $ git clone https://ceres-solver.googlesource.com/ceres-solver
          
-   Build and install with:

        $ cd ceres-solver
        $ mkdir build
        $ cd build
        $ cmake -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" ..
        $ make
        $ sudo make install 
    
libceres.a will be installed at **/usr/local/lib** and headers at **/usr/local/include/ceres**

#### Yaml-cpp 

Wolf uses YAML files for configuration and for saving and loading workspaces. To obtain it run:

-   Ubuntu:

        $ sudo apt-get install libyaml-cpp-dev
    
-   Mac:

        $ brew install yaml-cpp
        
We are shipping the CMAKE file `FindYamlCpp.cmake` together with Wolf. Find it at `[wolf]/cmake_modules/FindYamlCpp.cmake`

#### spdlog

Wolf uses spdlog macros. Right now Wolf is only compatible with spdlog version 0.17.0. To install it:

-   Git clone the source:

        $ git clone https://github.com/gabime/spdlog.git
        
-   Build and install with:

        $ cd spdlog
        $ git checkout v0.17.0
        $ mkdir build
        $ cd build
        $ cmake -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" ..
        $ make
        $ sudo make install 

#### Optional: Vision Utils (Install only if you want to use IRI's vision utils)

This library requires OpenCV. If it is not installed in your system or you are unsure, please follow the installation steps at https://gitlab.iri.upc.edu/mobile_robotics/vision_utils

**(1)** Git clone the source:

        $ git clone https://gitlab.iri.upc.edu/mobile_robotics/vision_utils.git
    
**(2)** Build and install:

        $ cd vision_utils/build
        $ cmake ..
        $ make
        $ sudo make install
        
**(3)** Optionally run tests

        $ ctest
        
#### Optional: Laser Scan Utils (Install only if you want to use IRI's laser scan utils)

**(1)** Git clone the source:

        $ git clone https://gitlab.iri.upc.edu/mobile_robotics/laser_scan_utils.git
    
**(2)** Build and install:

        $ cd laser_scan_utils
        $ mkdir build && cd build
        $ cmake ..
        $ make
        $ sudo make install
            
#### Optional: Raw GPS Utils (Install only if you want to use IRI's raw gps utils)

**(1)** Git clone the source:

    $ git clone https://github.com/pt07/raw_gps_utils.git
    
**(2)** Build and install:

    $ cd raw_gps_utils
    $ mkdir build && cd build
    $ cmake ..
    $ make
    $ sudo make install
    
Download and build
------------------

#### Wolf C++ Library

**Download:**
    
    $ git clone https://gitlab.iri.upc.edu/mobile_robotics/wolf.git
    
**Build:**

    $ cd wolf
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install //optional in case you want to install wolf library
    

#### Wolf ROS Node

-   Git clone the source (inside your ROS workspace):

    $ git clone https://github.com/IRI-MobileRobotics/wolf_ros.git

Using the `catkin_tools` package
--------------------------------

**(1)**  Install `catkin_tools` :

[`installation webpage.`](https://catkin-tools.readthedocs.io/en/latest/installing.html)

-   Installing on Ubuntu with `apt-get`

        $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
        $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        $ sudo apt-get update
        $ sudo apt-get install python-catkin-tools
    
-   Installing with [`pip`](https://pip.pypa.io/en/stable/installing/)

        $ sudo pip install -U catkin_tools
    
**(2)**  Create a `catkin workspace` :

    $ cd 
    $ mkdir -p wolf_ws/src
    $ cd wolf_ws/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    
**(3)** Setup your `bash_rc`:
    
Add at the end of the ~/.bashrc file with the following command:
    
    $ echo "source ~/wolf_ws/devel/setup.bash" >> ~/.bashrc
    
Source your bash:
    
    source ~/.bashrc
    
**(3)**  Download `Ceres` :

In the previously created directory `~/my_workspace_directory/wolf_ws/src/` clone `Ceres` & `wolf`.

    $ git clone https://github.com/artivis/ceres_solver.git

**(4)**  Download `wolf` :

    $ git clone https://gitlab.iri.upc.edu/mobile_robotics/wolf.git

At this point you might need to switch to the `catkin_build` branch of the wolf project.

    $ cd wolf
    $ git checkout catkin_build

(optional) Download `wolf_ros` :

    $ git clone https://github.com/IRI-MobileRobotics/Wolf_ros.git

**(5)**  Let's Compile !

    The command below can be launch from any sub-directory in `~/my_workspace_directory/wolf_ws/`.

    $ catkin build

**(6)**  Run tests:

    $ catkin run_tests
    
Troubleshooting
---------------

#### Boost

We have made our best to keep being boost-independent.
However, in case you run into a boost installation issue at execution time, check that you have boost installed. 
If needed, install it with:

[Boost](http://www.boost.org/). Free peer-reviewed portable C++ source libraries.

    $ sudo apt-get install libboost-all-dev



Inspiring Links
---------------

-   [Basics on code optimization](http://www.eventhelix.com/realtimemantra/basics/optimizingcandcppcode.htm)

-   [Headers, Includes, Forward declarations, ...](http://www.cplusplus.com/forum/articles/10627/)

-   Using Eigen quaternion and CERES: [explanation](http://www.lloydhughes.co.za/index.php/using-eigen-quaternions-and-ceres-solver/) & [GitHub CERES extension](https://github.com/system123/ceres_extensions)

Useful tools
------------

#### Profiling with Valgrind and Kcachegrind

Kcachegrind is a graphical frontend for profiling your program and optimizing your code.

- Ubuntu:

Get the programs with

    $ sudo apt-get install valgrind kcachegrind

- Mac OSX

In Mac, you can use qcachegrind instead. To get it through Homebrew, type

    $ brew install valgrind qcachegrind

I don't know if these packages are available through MacPorts. Try

    $ ports search --name valgrind
    $ ports search --name qcachegrind

If they are available, just do

    $ sudo port install valgrind qcachegrind

#### Do the profiling and watch the reports

_Remember:_ For a proper profiling, compile Wolf and related libraries 
in RELEASE mode. Profiling code compiled in DEBUG mode is not going to take you 
anywhere, and in the reports you will mostly see the overhead of the DEBUG mode.

Type in your `wolf/bin/` directory:

    $ cd bin/
    $ valgrind --tool=callgrind ./my_program <my_prg_params>

this produces a log report called `callgrind.out.XXXX`, where XXXX is a number. Then type 

- Ubuntu

    ```shell
    $ kcachegrind callgrind.out.XXXX
    ```

- Mac OSX

    ```shell
    $ qcachegrind callgrind.out.XXXX
    ```

and enjoy.
