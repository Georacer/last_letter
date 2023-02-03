FROM ros:foxy-ros-base 

ARG INSTALL_PATH=/ros_ws/src/last_letter

COPY . $INSTALL_PATH

RUN bash $INSTALL_PATH/tools/install_dependencies.sh

RUN bash $INSTALL_PATH/tools/build_project.sh

RUN bash $INSTALL_PATH/tools/install_project.sh

# This doesn't do anything.
RUN bash /ros_ws/install/setup.bash

RUN ls ~/last_letter_models

EXPOSE 4560/tcp
EXPOSE 14540
EXPOSE 14550
EXPOSE 14560
EXPOSE 14570