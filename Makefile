.PHONY: build run

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
	@echo "usage: make [COMMAND] target=[TARGET]"
	@echo "COMMAND options:"
	@echo "    build"
	@echo "    run"
	@echo "TARGET options:"
	@echo "    navigation"
	@echo "    perception"
	@echo "    unreal_sim"
	@echo "    rosboard"
	@echo "    jetson"
	@echo "    jetson_processing"
	@echo "Not supplying a target for 'make build' will build all environments."

build_docker_base:
	docker build -f docker/Dockerfile -t qutms_driverless_base \
	--build-arg USERNAME=$(USERNAME) --build-arg HOST_UID=$(HOST_UID) .

# use make build target=<value>
build: build_docker_base
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build $(target)

# use make build target=<value>
run:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless run --rm $(target)
