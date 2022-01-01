.PHONY: all help build_docker_base_1804 build_docker_base_l4t build run

include .env
export  # export all variables from .env

export USERNAME=developer
export HOST_UID=$(shell id -u)
export CURR_DIR=$(notdir $(shell pwd))

# UNLESS YOU ARE UPDATING THE MAKEFILE FUNCTIONALITY
# This file should rarely need to be touched.

# ADDING A NEW TARGET
# Targets are defined in docker/docker-compose.yml (at the bottom, called "Services")
# To add a new target:
#     1. add the target to the docker compose file
#     2. document the target option in the `all` rule in this file below

all:
	@echo "usage: make [COMMAND]"
	@echo
	@echo "COMMAND options:"
	@echo "    help"
	@echo "        - show this message"
	@echo "    build_docker_base_[IMAGE_TYPE]"
	@echo "        - build the base docker container"
	@echo "        - IMAGE_TYPE options:"
	@echo "            - 1804 (for most environments)"
	@echo "            - l4t (for nvidia jetson)"
	@echo "            - prebuilt (for jetson with Galactic + Pytorch)"
	@echo "    build target=[ENVIRONMENT_TARGET]"
	@echo "        - build a target (see ENVIRONMENT_TARGET options)"
	@echo "    run target=[ENVIRONMENT_TARGET]"
	@echo "        - run a target (see ENVIRONMENT_TARGET options)"
	@echo
	@echo "ENVIRONMENT_TARGET options:"
	@echo "    roscube"
	@echo "    jetson"
	@echo "    unreal_sim"
	@echo "    zed_camera_driver"
	@echo "    prebuilt"
	@echo

help: all

build_docker_base_1804:
	docker build -f Dockerfile.base -t qutms_driverless_base:18.04 \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
	--build-arg BASE_IMAGE=ubuntu:18.04 \
	..

build_docker_base_l4t:
	docker build -f Dockerfile.base -t qutms_driverless_base:l4t-r32.6.1 \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
	--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1 \
	..

build_docker_base_prebuilt:
	docker build -f Dockerfile.ros.pytorch -t qutms_driverless:ros-pytorch \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
	--build-arg BASE_IMAGE=dustynv/jetson-inference:r32.6.1 \
	..

# use make build target=<value>
build:
	@echo "ensure make build_docker_base_1804 or make build_docker_base_l4t is run before running a build"
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless build $(target)

# use make run target=<value>
run:
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless run --rm $(target)
