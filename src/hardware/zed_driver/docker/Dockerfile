ARG L4T_MINOR_VERSION=6.1

FROM nvcr.io/nvidia/l4t-base:r32.${L4T_MINOR_VERSION}


# ==============================================================================
#                           Install ROS2 from source
# ==============================================================================

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L5
ARG ROS_PKG=ros_base
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

WORKDIR /tmp

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L18
# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L27
# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L44
# install development packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
        # Fast-RTPS dependencies
		libasio-dev \
		libtinyxml2-dev \
        # Cyclone DDS dependencies
		libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# install some pip packages needed for testing
RUN python3 -m pip install -U \
		argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L88
# install OpenCV (with CUDA)
ARG OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
ARG OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz

RUN apt-get purge -y '*opencv*' || echo "previous OpenCV installation not found" && \
    mkdir opencv && \
    cd opencv && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${OPENCV_URL} -O ${OPENCV_DEB} && \
    tar -xzvf ${OPENCV_DEB} && \
    dpkg -i --force-depends *.deb && \
    apt-get update && \
    apt-get install -y -f --no-install-recommends && \
    dpkg -i *.deb && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    cd ../ && \
    rm -rf opencv && \
    cp -r /usr/include/opencv4 /usr/local/include/opencv4 && \
    cp -r /usr/lib/python3.6/dist-packages/cv2 /usr/local/lib/python3.6/dist-packages/cv2

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L111
# compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
RUN git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L122
# download/build ROS from source
RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \

    # https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
    # ==== List modified to REMOVE: ====
    # (not required)
    #   - demo_nodes_cpp
    #   - demo_nodes_py
    #   - example_interfaces
    # (supplied by image_common, installed version 3.0.0 below)
    #   - camera_calibration_parsers
    #   - camera_info_manager
    #   - image_transport
    # (causes issues according to https://github.com/stereolabs/zed-ros2-wrapper#known-issues)
    #   - compressed_image_transport
    #   - compressed_depth_image_transport
    # ==== List modified to ADD: ====
    # (required by zed camera node)
    #   - xacro
    # (dev tools)
    #   - rviz2
    # (dev message)
    #   - ackermann_msgs

    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
		launch_xml \
		launch_yaml \
		launch_testing \
		launch_testing_ament_cmake \
		cv_bridge \
		v4l2_camera \
		vision_opencv \
		vision_msgs \
		image_geometry \
		image_pipeline \
        xacro \
        rviz2 \
        ackermann_msgs \
		> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \

    cd ${ROS_ROOT}/src/ && \

    # install https://github.com/ros-perception/image_common/releases/tag/3.0.0
    rm -rf image_common && \
    git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --depth 1 && \

    # install https://github.com/ros/diagnostics.git branch foxy
    rm -rf diagnostics && \
    git clone https://github.com/ros/diagnostics.git --branch foxy --depth 1 && \

    # install https://github.com/ament/google_benchmark_vendor/releases/tag/0.0.6
    rm -rf google_benchmark_vendor && \
    git clone https://github.com/ament/google_benchmark_vendor.git --branch 0.0.6 --depth 1 && \

    #  patch libyaml - https://github.com/dusty-nv/jetson-containers/issues/41#issuecomment-774767272
    rm ${ROS_ROOT}/src/libyaml_vendor/CMakeLists.txt && \
    wget --no-check-certificate https://raw.githubusercontent.com/ros2/libyaml_vendor/master/CMakeLists.txt -P ${ROS_ROOT}/src/libyaml_vendor/ && \

    # install dependencies using rosdep
    apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
        --ignore-src \
        --from-paths src \
        --rosdistro ${ROS_DISTRO} \
	    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \

    # build it!
    colcon build --merge-install && \

    # remove build files
    rm -rf ${ROS_ROOT}/src && \
    rm -rf ${ROS_ROOT}/logs && \
    rm -rf ${ROS_ROOT}/build && \
    rm ${ROS_ROOT}/*.rosinstall

# https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.foxy#L192
# Set the default DDS middleware to cyclonedds
# https://github.com/ros2/rclcpp/issues/1335
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


# ==============================================================================
#                           Install ZED Drivers
# https://github.com/stereolabs/zed-docker/blob/master/3.X/jetpack_4.X/tools-devel/Dockerfile
# ==============================================================================

# value comes from above, see https://stackoverflow.com/a/56748289
ARG L4T_MINOR_VERSION

# ZED_SDK=3.6
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=6

# JETPACK=4.6
ARG JETPACK_MAJOR=4
ARG JETPACK_MINOR=6

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update -y && apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https -y && \
    echo "# R32 (release), REVISION: ${L4T_MINOR_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/jp${JETPACK_MAJOR}${JETPACK_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so


# ==============================================================================
#                      Install ZED ROS nodes
#     Following instructions from https://www.stereolabs.com/docs/ros2/
# ==============================================================================
WORKDIR /ros_zed_ws/src
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git --branch foxy-v3.5.1

WORKDIR /ros_zed_ws
# symlink libusb for the build
RUN ln -s /lib/aarch64-linux-gnu/libusb-1.0.so.0 /usr/lib/aarch64-linux-gnu/libusb-1.0.so
# build packages, extra cuda library options come from https://github.com/stereolabs/zed-ros2-wrapper/issues/18
RUN source ${ROS_ROOT}/install/setup.bash; \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined

# copy calibration file
COPY ./src/hardware/zed_driver/docker/SN33580719.conf /usr/local/zed/settings/SN33580719.conf

# copy perception and run a build
WORKDIR /driverless_ws
COPY ./src/hardware/zed_driver ./src/zed_driver
RUN source ${ROS_ROOT}/install/setup.bash; \
    colcon build --symlink-install && \
    rm -rf ./src

# setup entrypoint
COPY ./src/hardware/zed_driver/docker/jetson_ros_zed_entrypoint.sh /jetson_ros_zed_entrypoint.sh
ENTRYPOINT ["/jetson_ros_zed_entrypoint.sh"]
CMD ["ros2", "launch", "zed_driver", "zed2i.launch.py"]
