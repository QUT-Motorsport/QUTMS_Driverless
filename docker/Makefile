.PHONY: all help build_docker_base build run ssh

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
#     2. document the target option to the `all` rule in this file below

all:
	@echo "usage: make [COMMAND]"
	@echo
	@echo "COMMAND options:"
	@echo "    help"
	@echo "        - show this message"
	@echo "    build_docker_base"
	@echo "        - build the base docker container"
	@echo "    build target=[ENVIRONMENT_TARGET]"
	@echo "        - build a target (see ENVIRONMENT_TARGET options). Not supplying a target will build all environments."
	@echo "    run target=[ENVIRONMENT_TARGET]"
	@echo "        - run a target (see ENVIRONMENT_TARGET options)"
	@echo "    ssh target=[SSH_TARGET]"
	@echo "        - ssh into a target (see SSH_TARGET options)"
	@echo
	@echo "ENVIRONMENT_TARGET options:"
	@echo "    roscube"
	@echo "    jetson"
	@echo "    unreal_sim"
	@echo

help: all

build_docker_base:
	docker build -f Dockerfile.base -t qutms_driverless_base \
	--build-arg USERNAME=$(USERNAME) --build-arg HOST_UID=$(HOST_UID) .

# use make build target=<value>
build: build_docker_base
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build $(target)

# use make build target=<value>
run:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless run --rm $(target)
