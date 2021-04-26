.PHONY: build run_ros run_sim_bridge

include .env

export USERNAME=developer
export HOST_UID=$(shell id -u)
export CURR_DIR=$(notdir $(shell pwd))

all:
	@echo Hi!
	@echo Check the Makefile for commands you can run

build:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build ros
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build sim_bridge

run_ros:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless run ros

run_sim_bridge:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up -d core
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up -d ros_1_bridge
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up sim_bridge
