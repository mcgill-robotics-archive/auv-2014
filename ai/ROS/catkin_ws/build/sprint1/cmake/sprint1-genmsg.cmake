# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sprint1: 3 messages, 0 services")

set(MSG_I_FLAGS "-Isprint1:/home/alex/catkin_ws/src/sprint1/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Isprint1:/home/alex/catkin_ws/src/sprint1/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sprint1_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/CVQuery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1
)
_generate_msg_cpp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Pressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1
)
_generate_msg_cpp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Depth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1
)

### Generating Services

### Generating Module File
_generate_module_cpp(sprint1
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sprint1_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sprint1_generate_messages sprint1_generate_messages_cpp)

# target for backward compatibility
add_custom_target(sprint1_gencpp)
add_dependencies(sprint1_gencpp sprint1_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sprint1_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/CVQuery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1
)
_generate_msg_lisp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Pressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1
)
_generate_msg_lisp(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Depth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1
)

### Generating Services

### Generating Module File
_generate_module_lisp(sprint1
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sprint1_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sprint1_generate_messages sprint1_generate_messages_lisp)

# target for backward compatibility
add_custom_target(sprint1_genlisp)
add_dependencies(sprint1_genlisp sprint1_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sprint1_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/CVQuery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1
)
_generate_msg_py(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Pressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1
)
_generate_msg_py(sprint1
  "/home/alex/catkin_ws/src/sprint1/msg/Depth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1
)

### Generating Services

### Generating Module File
_generate_module_py(sprint1
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sprint1_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sprint1_generate_messages sprint1_generate_messages_py)

# target for backward compatibility
add_custom_target(sprint1_genpy)
add_dependencies(sprint1_genpy sprint1_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sprint1_generate_messages_py)


debug_message(2 "sprint1: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sprint1
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sprint1_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(sprint1_generate_messages_cpp sprint1_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sprint1
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sprint1_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(sprint1_generate_messages_lisp sprint1_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sprint1
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sprint1_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(sprint1_generate_messages_py sprint1_generate_messages_py)
