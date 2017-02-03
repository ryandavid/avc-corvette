cmake_minimum_required(VERSION 2.8.3)
project(pru_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pru_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES pru_example
    CATKIN_DEPENDS roscpp pru_utils
#   DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(pru_example
#   src/${PROJECT_NAME}/pru_example.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pru_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_custom_target(compile_pru_asm
#  COMMAND ${CATKIN_DEVEL_PREFIX}/../pru_utils/lib/pru_utils/pasm -V3 ${CMAKE_CURRENT_SOURCE_DIR}/src/PRU_memAccessPRUDataRam.p ${CMAKE_CURRENT_SOURCE_DIR}/src/PRU_memAccessPRUDataRam
#  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#  COMMENT "Running pasm. ${CATKIN_DEVEL_PREFIX}/../pru_utils/lib/pru_utils/pasm -V3 ${CMAKE_CURRENT_SOURCE_DIR}/src/PRU_memAccessPRUDataRam.c ${CMAKE_CURRENT_SOURCE_DIR}"
#  SOURCES ${test2_SOURCES}
#)


add_custom_command(OUTPUT main.d
    COMMAND clpru -v3 -Ooff -I=${CMAKE_CURRENT_SOURCE_DIR}/include/ \
        -I=${CMAKE_CURRENT_SOURCE_DIR}/include/pru_icss -I=/usr/share/ti/cgt-pru/include/ \
        --define=am3359 --define=pru0 --diag_warning=225 --diag_wrap=off --display_error_number \
        --hardware_mac=on --endian=little --preproc_with_compile --preproc_dependency=main.d ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    COMMENT "Compiling with clpru: ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    VERBATIM
)

add_custom_command(OUTPUT main.obj
    COMMAND clpru -v3 -Ooff --define=am3359 --define=pru0 --diag_warning=225 --diag_wrap=off \
        --display_error_number --hardware_mac=on --endian=little -z -mmain.map --heap_size=0x100 \
        --stack_size=0x100 -i/usr/share/ti/cgt-pru/lib/ -i/usr/share/ti/cgt-pru/include/ --reread_libs \
        --display_error_number --warn_sections --diag_wrap=off --xml_link_info=main.xml --rom_model -o main.out \
        main.obj ${CMAKE_CURRENT_SOURCE_DIR}/src/AM335x_PRU.cmd -llibc.a
    COMMENT "Linking with clpru: ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    DEPENDS main.d
    VERBATIM
)

add_custom_target(hexpru ALL
    COMMAND hexpru ${CMAKE_CURRENT_SOURCE_DIR}/src/bin.cmd main.out
    COMMENT "Generating binaries with hexpru."
    DEPENDS main.obj
    VERBATIM
)



## Declare a C++ executable
#add_executable(PRU_memAccessPRUDataRam src/PRU_memAccessPRUDataRam.c)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(PRU_memAccessPRUDataRam compile_pru_asm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(PRU_memAccessPRUDataRam
#    ${catkin_LIBRARIES}
#)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pru_example pru_example_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pru_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
