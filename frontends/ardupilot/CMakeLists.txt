# ArduPilot SITL bridge. Launched by ArduPilot's SITL::last_letter backend, it
# drives a UavModel over the ArduPilot JSON backend protocol (UDP). Needs no
# dependency beyond POSIX sockets and last_letter_lib itself.
#
# Added by the top-level CMakeLists.txt only when LAST_LETTER_BUILD_ARDUPILOT
# is ON.

add_executable(last_letter_ardupilot
  udp_socket.cpp
  json_connector.cpp
  main.cpp
)
target_link_libraries(last_letter_ardupilot last_letter_lib)
# liblast_letter_lib.so installs to <prefix>/${LAST_LETTER_LIB_DESTINATION}
# (see the install rules in lib/CMakeLists.txt), so the binary in <prefix>/bin
# needs to be pointed at it explicitly. LAST_LETTER_LIB_DESTINATION is set by
# the top-level CMakeLists.txt, since a value set in lib/ would not reach here.
set_target_properties(last_letter_ardupilot PROPERTIES
  INSTALL_RPATH "$ORIGIN/../${LAST_LETTER_LIB_DESTINATION}"
  BUILD_RPATH "$ORIGIN"
)
install(TARGETS last_letter_ardupilot RUNTIME DESTINATION bin)
