add_library(${PROJECT_NAME}_type_introspection
	${PROJECT_INCLUDE}/type_introspection/parse_property.h

	parse_property.cpp
)
add_executable(test_rti
	test_rti.cpp
)

target_include_directories(${PROJECT_NAME}_type_introspection
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_type_introspection ${catkin_LIBRARIES})

target_include_directories(test_rti
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(test_rti ${catkin_LIBRARIES} ${PROJECT_NAME}_type_introspection)

install(TARGETS ${PROJECT_NAME}_type_introspection
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS test_rti
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
