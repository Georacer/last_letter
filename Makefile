# Script arguments.
USE_DOCKER ?= true
USE_SUDO ?= false
HEADLESS ?= false
UAV_NAME ?= skywalker_2013
CUSTOM_UAV_PATH ?= ''

.DEFAULT_GOAL = run

.PHONY: help
help:
	@echo '*** last_letter simulator ***'
	@echo 'Usage: make [TARGET] [ARGUMENTS]'
	@echo 'TARGETS:'
	@echo '	help		Display help message.'
	@echo '	run		Run the simulation.'
	@echo 'ARGUMENTS:'
	@echo '	USE_DOCKER	Run the simulation in a Docker container [true].'
	@echo '	USE_SUDO	Invoke docker with sudo [false].'
	@echo '	HEADLESS	Do not launch graphics [false].'
	@echo '	UAV_NAME	Select the UAV model to launch [skywalker_2013].'

# Activate the xhost environment to share it with a Docker container.
define xhost_activate
	@echo "Enabling local xhost sharing:"
	@echo "  Display: $(DISPLAY)"
	@-DISPLAY=$(DISPLAY) xhost  +local:docker
	@-xhost  +local:docker
endef

ROS_ARGS = \
	uav_name:=$(UAV_NAME)

ifneq ($(CUSTOM_UAV_PATH), '')
	mount_custom_uav:=--volume=$(CUSTOM_UAV_PATH):/root/last_letter_models/models/$(UAV_NAME)
endif

ifeq ($(USE_SUDO), true)
	DOCKER_SUDO = sudo 
endif

ifeq ($(USE_DOCKER), true)
	DOCKER_CMD = $(DOCKER_SUDO) docker run -it --rm \
		--volume=/tmp/.X11-unix/:/tmp/.X11-unix/ \
		$(mount_custom_uav) \
		--device=/dev/dri:/dev/dri \
		-e DISPLAY \
		-p 14570:14570 \
		-p 4560:4560 \
		last_letter 
endif

# Build the docker image.
.PHONY: run
run: 
ifeq ($(HEADLESS), false)
ifeq ($(USE_DOCKER), true)
	@$(call xhost_activate)
endif
endif
	@$(DOCKER_CMD) ros2 launch last_letter default.launch.py $(ROS_ARGS)
	
# world_file_path:=<world_path>