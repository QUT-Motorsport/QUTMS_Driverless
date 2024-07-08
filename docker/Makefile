.PHONY: all help build_docker_base_jetson build run panda_srv jetson_srv

export USERNAME=developer
export HOST_UID=$(shell id -u)

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
	@echo "    build target=[DEVICE]"
	@echo "        - build a target (see DEVICE options)"
	@echo "    run target=[DEVICE]"
	@echo "        - run a target (see DEVICE options)"
	@echo "    jetson_srv a=[ACTION]"
	@echo "        - run systemctl service action (see ACTION options)"
	@echo "    panda_srv a=[ACTION]"
	@echo "        - run systemctl service action (see ACTION options)"
	@echo
	@echo "DEVICE options:"
	@echo "    jetson (requires base jetson image)"
	@echo "    jetson_terminal (requires base jetson image)"
	@echo "    panda (requires nothing, just build)"
	@echo "    panda_terminal (requires nothing, just build)"
	@echo
	@echo "ACTION options:"
	@echo "    status"
	@echo "    start"
	@echo "    stop"
	@echo "    restart"
	@echo

help: all

# build_docker_base_roscube:
# 	docker build -f Dockerfile.base -t qutms_driverless_base:ubuntu-22.04 \
# 	--build-arg USERNAME=$(USERNAME) \
# 	--build-arg HOST_UID=$(HOST_UID) \
# 	--build-arg BASE_IMAGE=ubuntu:22.04 \
# 	..

# build_docker_base_jetson:
# 	docker build -f Dockerfile.base -t qutms_driverless_base:l4t-jetpack-r35.3.1 \
# 	--build-arg USERNAME=$(USERNAME) \
# 	--build-arg HOST_UID=$(HOST_UID) \
# 	--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.3.1 \
# 	..

build_docker_base_jetson_new:
	@echo "ensure 'jetson-containers' has been cloned into the 'repos' directory"
	~/dev/repos/jetson-containers/build.sh --name=qutms_driverless_base pytorch torchvision zed ros:humble-ros-base --skip-tests=all

# use make build target=<value>
build:
	@echo "ensure a 'make build_docker_base_XXX' is run before running a build"
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless build $(target)

# use make run target=<value>
run:
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless run --rm $(target)

# roscube_srv:
# 	systemctl $(a) roscube_startup.service

jetson_srv:
	systemctl $(a) jetson_startup.service

panda_srv:
	systemctl $(a) panda_startup.service
