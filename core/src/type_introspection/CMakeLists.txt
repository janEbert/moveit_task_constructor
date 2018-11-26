add_library(${PROJECT_NAME}_type_introspection
	${PROJECT_INCLUDE}/type_introspection/

	parse_property.cpp
)
target_include_directories(${PROJECT_NAME}_type_introspection
	PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_type_introspection ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_type_introspection
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
