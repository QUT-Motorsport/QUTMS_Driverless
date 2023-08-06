.PHONY: all help build_docker_base_jetson build_docker_base_roscube build run roscube_srv jetson_srv

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
	@echo "    build_docker_base_[IMAGE_TYPE]"
	@echo "        - build the base docker container"
	@echo "        - IMAGE_TYPE options:"
	@echo "            - roscube"
	@echo "            - jetson"
	@echo "    build target=[ENVIRONMENT_TARGET]"
	@echo "        - build a target (see ENVIRONMENT_TARGET options)"
	@echo "    run target=[ENVIRONMENT_TARGET]"
	@echo "        - run a target (see ENVIRONMENT_TARGET options)"
	@echo "    roscube_srv a=[ACTION]"
	@echo "        - run systemctl service action (see ACTION options)"
	@echo "    jetson_srv a=[ACTION]"
	@echo "        - run systemctl service action (see ACTION options)"
	@echo
	@echo "ENVIRONMENT_TARGET options:"
	@echo "    roscube (requires IMAGE_TYPE=roscube)"
	@echo "    roscube_terminal (requires IMAGE_TYPE=roscube)"
	@echo "    jetson (requires IMAGE_TYPE=jetson)"
	@echo "    jetson_terminal (requires IMAGE_TYPE=jetson)"
	@echo
	@echo "ACTION options:"
	@echo "    status"
	@echo "    start"
	@echo "    stop"
	@echo "    restart"
	@echo

help: all

build_docker_base_roscube:
	docker build -f Dockerfile.base -t qutms_driverless_base:ubuntu-22.04 \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
	--build-arg BASE_IMAGE=ubuntu:22.04 \
	..

build_docker_base_jetson:
	docker build -f Dockerfile.base -t qutms_driverless_base:l4t-jetpack-r35.3.1 \
	--build-arg USERNAME=$(USERNAME) \
	--build-arg HOST_UID=$(HOST_UID) \
	--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.3.1 \
	..

build_docker_base_jetson_new:
	@echo "ensure 'jetson-containers' has been cloned into the 'repos' directory"
	~/dev/repos/jetson-containers/build.sh --name=qutms_driverless_base ros:humble-base pytorch zed

# use make build target=<value>
build:
	@echo "ensure a 'make build_docker_base_XXX' is run before running a build"
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless build $(target)

# use make run target=<value>
run:
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless run --rm $(target)

roscube_srv:
	systemctl $(a) roscube_startup.service

jetson_srv:
	systemctl $(a) jetson_startup.service
