# Script arguments.
USE_DOCKER ?= true
DETACHED ?= false
CONTAINER_NAME ?= last_letter_container
USE_SUDO ?= false
HEADLESS ?= false
UAV_NAME ?= skywalker_2013
CUSTOM_MODELS_FOLDER ?= ''

.DEFAULT_GOAL = run

.PHONY: help
help:
	@echo '*** last_letter simulator ***'
	@echo 'Usage: make [TARGET] [ARGUMENTS]'
	@echo 'TARGETS:'
	@echo '	help			Display help message.'
	@echo '	run			Run the simulation.'
	@echo 'ARGUMENTS:'
	@echo '	USE_DOCKER		Run the simulation in a Docker container [true].'
	@echo '	DETACHED		Run the Docker container detached [false].'
	@echo '	CONTAINER_NAME		Assign a name to the Docker container [last_letter_container]'
	@echo '	USE_SUDO		Invoke docker with sudo [false].'
	@echo '	HEADLESS		Do not launch graphics [false].'
	@echo '	UAV_NAME		Select the UAV model to launch [skywalker_2013].'
	@echo '	CUSTOM_MODELS_FOLDER	Select the UAV model to launch [skywalker_2013].'

# Activate the xhost environment to share it with a Docker container.
define xhost_activate
	@echo "Enabling local xhost sharing:"
	@echo "  Display: $(DISPLAY)"
	@-DISPLAY=$(DISPLAY) xhost  +local:docker
	@-xhost  +local:docker
endef

ROS_ARGS = \
	uav_name:=$(UAV_NAME)

ifneq ($(CUSTOM_MODELS_FOLDER), '')
	mount_custom_uav:=--volume=$(CUSTOM_MODELS_FOLDER)/$(UAV_NAME):/root/last_letter_models/models/$(UAV_NAME)
endif

ifeq ($(USE_SUDO), true)
	DOCKER_SUDO = sudo 
endif

ifeq ($(DETACHED), true)
	DOCKER_DETACHED = -d 
endif

ifeq ($(USE_DOCKER), true)
	DOCKER_CMD = $(DOCKER_SUDO) docker run --rm \
		$(DOCKER_DETACHED) \
		--name $(CONTAINER_NAME) \
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

# Stop a detached docker container.
.PHONY: stop
stop:
	$(DOCKER_SUDO) docker stop $(CONTAINER_NAME)
	
# world_file_path:=<world_path>