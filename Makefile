.PHONY: build run run_core

# include .env

export USERNAME=developer
export HOST_UID=$(shell id -u)
export CURR_DIR=$(notdir $(shell pwd))

all:
	@echo Hi!
	@echo Check the Makefile for commands you can run

build:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless build ros

run:
	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless run ros

# run_core:
# 	docker-compose -f ./docker/docker-compose.yml -p QUTMS_Driverless up -d core
