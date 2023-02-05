FROM ros:foxy-ros-base 

ARG INSTALL_PATH=/ros_ws/src/last_letter

COPY . $INSTALL_PATH

RUN bash $INSTALL_PATH/tools/install_dependencies.sh

RUN bash $INSTALL_PATH/tools/build_project.sh

RUN bash $INSTALL_PATH/tools/install_project.sh

COPY tools/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
COPY tools/environment.sh /environment.sh

# RUN echo "" >> ~/.bashrc
# RUN echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$HOME/.gazebo/models:" >> ~/.bashrc
# RUN echo "export GAZEBO_RESOURCE_PATH=:/usr/share/gazebo-11:/usr/share/gazebo_models:" >> ~/.bashrc
# RUN echo "export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ~/.bashrc
# RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc
RUN cat /environment.sh >> ~/.bashrc
# Perhaps source: "source <install_path>/share/gazebo/setup.sh"

# Remove the apt cache to reduce image size.
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2 launch last_letter default.launch.py"]

EXPOSE 4560/tcp
EXPOSE 14540
EXPOSE 14550
EXPOSE 14560
EXPOSE 14570