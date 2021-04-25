.PHONY: build run run_core

include .env

export USERNAME=developer
export HOST_UID=$(shell id -u)
export CURR_DIR=$(notdir $(shell pwd))

all:
	@echo Hi!
	@echo Check the Makefile for commands you can run

build:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build ros

build_sim_bridge:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build sim_bridge


run_core:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up -d core

run:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless run ros

run_sim_bridge:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up -d ros_1_bridge
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up sim_bridge
