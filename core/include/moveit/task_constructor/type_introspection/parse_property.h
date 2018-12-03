#pragma once

#include <ros_type_introspection/ros_introspection.hpp>

#include <ros/message_traits.h>

#include <moveit_task_constructor_msgs/Property.h>

namespace ri = RosIntrospection;

namespace moveit {
namespace task_constructor {

class TypeIntrospector {

public:
	template <typename T>
	inline void registerMsgType(const T& msg) {
		registerMsgType<T>();
	}

	template <typename T>
	inline void registerMsgType() {
		parser.registerMessageDefinition(ros::message_traits::datatype<T>(),
				ri::ROSType(ros::message_traits::datatype<T>()),
				ros::message_traits::definition<T>());
	}

	ri::RenamedValues extractFromPropertyMsg(const moveit_task_constructor_msgs::Property& prop);


// private:

// The following two functions are deliberately kept separate at the moment.

/** Return extracted values of a serialized message as a flat vector.
``name`` is the name by which the message was registered, while ``sermsg`` is the serialized message. */
	ri::RenamedValues extract(const std::string& name, const std::string& sermsg);

/** Return extracted values of a serialized msg in a buffer as a flat vector.
``name`` is the name by which the message was registered, while ``sermsg`` is a buffer containing the serialized message. */
	ri::RenamedValues extract(const std::string& name, std::vector<uint8_t>& sermsg);

	ri::Parser parser;
};

}
}

