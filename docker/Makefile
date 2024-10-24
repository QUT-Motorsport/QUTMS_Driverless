.PHONY: all help build run panda_srv

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
	@echo
	@echo "DEVICE options:"
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

# use make build target=<value>
build:
	@echo "ensure a 'make build_docker_base_XXX' is run before running a build"
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless build $(target)

# use make run target=<value>
run:
	docker-compose -f ./docker-compose.yml -p QUTMS_Driverless run --rm $(target)

panda_srv:
	systemctl $(a) panda_startup.service
