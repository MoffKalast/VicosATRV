cmake_minimum_required(VERSION 2.8.3)
project(face_led)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
	demo.py
	led_controller.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

