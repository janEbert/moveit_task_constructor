#pragma once

#include <ros/message_traits.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <ros_type_introspection/ros_introspection.hpp>

#include <moveit_task_constructor_msgs/Property.h>

namespace ri = RosIntrospection;

namespace moveit_rviz_plugin {

class TreeConstructor { // {{{1

public:
	static rviz::Property* parseToTree(
	        const std::vector<std::pair<std::string, ri::Variant>>& msgValues,
	        const std::string& name,
	        const std::string& description,
	        rviz::Property* old = 0);


private:
	static void treeFromValue(const std::pair<std::string, ri::Variant>& value,
	                          rviz::Property* root, const size_t startIx);

	static rviz::Property* getOrCreateChild(const std::string& name, rviz::Property* parent);

	static rviz::Property* createLeaf(const std::pair<std::string, ri::Variant>& value, rviz::Property* parent, const size_t startIx);

	template <typename T, typename V>
	static rviz::Property* createLeafOfType(const std::pair<std::string, ri::Variant>& value,
	                                        rviz::Property* parent, const size_t startIx);

	template <typename V>
	static rviz::StringProperty* createStringLeaf(const std::pair<std::string, ri::Variant>& value,
	                                              rviz::Property* parent, const size_t startIx);

	static rviz::StringProperty* timeLeaf(const std::pair<std::string, ri::Variant>& value,
	                                      rviz::Property* parent, const size_t startIx);

	static rviz::StringProperty* durationLeaf(const std::pair<std::string, ri::Variant>& value,
	                                          rviz::Property* parent, const size_t startIx);

};


class TypeIntrospector { // {{{1

public:
	/** Construct a type introspector registering a default message. */
	TypeIntrospector() {
		registerMsgType<moveit_task_constructor_msgs::Property>();
	}

	/** Register the given message to the parser.
	Registering the same message twice is allowed. */
	template <typename T>
	inline void registerMsgType(const T& msg) {
		registerMsgType<T>();
	}

	/** Register the given type to the parser.
	Registering the same message twice is allowed. */
	template <typename T>
	inline void registerMsgType() {
		parser.registerMessageDefinition(ros::message_traits::datatype<T>(),
		                                 ri::ROSType(ros::message_traits::datatype<T>()),
		                                 ros::message_traits::definition<T>());
	}

	/** Return the root node of an `rviz::Property` tree constructed
	from the given property message.
	Optionally give a property to insert the tree under. */
	inline rviz::Property* createTree(const moveit_task_constructor_msgs::Property& prop,
	                                  rviz::Property* old = 0) {
		const std::string& name = prop.type;
		const ri::RenamedValues& flatMsg = extract(name, prop.value);
		return TreeConstructor::parseToTree(flatMsg, name, prop.description, old);
	}


private:
	/** Return extracted values of the message contained in the given
	property. */
	inline ri::RenamedValues extractFromPropertyMsg(
	        const moveit_task_constructor_msgs::Property& prop) {
		return extract(prop.type, prop.value);
	}

/** Return extracted values of a serialized message as a flat vector.
`name` is the name by which the message was registered, while
`sermsg` is the serialized message. */
	ri::RenamedValues extract(const std::string& name, const std::string& sermsg);

/** Return extracted values of a serialized msg in a buffer as a flat
vector.
`name` is the name by which the message was registered, while
`sermsg` is a buffer containing the serialized message. */
	ri::RenamedValues extract(const std::string& name, std::vector<uint8_t>& sermsg);


	// TODO make static? is it safe?
	/** The parser containing all registered message types. */
	ri::Parser parser;
}; // }}}1

} // namespace moveit_rviz_plugin

