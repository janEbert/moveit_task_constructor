find_package(PkgConfig)
pkg_check_modules(LIBYAML REQUIRED yaml-0.1)

add_executable(${PROJECT_NAME}_yaml
	# subscribe_example.cpp
	stage.cpp
)
# add_library(${PROJECT_NAME}_yaml
# 	# ${PROJECT_INCLUDE}/yaml/
#
# 	subscribe_example.cpp
# )
target_include_directories(${PROJECT_NAME}_yaml
	PUBLIC ${catkin_INCLUDE_DIRS}
	PUBLIC ${LIBYAML_INCLUDE_DIRS}
)
# link_directories(${LIBYAML_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}_yaml ${catkin_LIBRARIES} ${LIBYAML_LIBRARIES})
# target_include_directories(${PROJECT_NAME}_yaml
# )

# install(TARGETS ${PROJECT_NAME}_yaml
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
