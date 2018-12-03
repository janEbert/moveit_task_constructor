#include <moveit/task_constructor/type_introspection/parse_property.h>

#include <iostream>

#include <topic_tools/shape_shifter.h>

namespace ri = RosIntrospection;

namespace moveit {
namespace task_constructor {

ri::RenamedValues TypeIntrospector::extractFromPropertyMsg(const moveit_task_constructor_msgs::Property& prop) {
	return extract(prop.type, prop.value);
}

/** Return extracted values of a serialized message as a flat vector.
``name`` is the name by which the message was registered, while ``sermsg`` is the serialized message. */
ri::RenamedValues TypeIntrospector::extract(const std::string& name, const std::string& sermsg) {
	constexpr uint32_t MAX_ARRAY_SIZE = 10;

	static ri::FlatMessage      msgContent;
	static ri::RenamedValues    msgValues;
	static std::vector<uint8_t> buffer(sermsg.begin(), sermsg.end());

	std::cout << "type: " << name << std::endl;

	parser.deserializeIntoFlatContainer(name, absl::Span<uint8_t>(buffer),
	                                    &msgContent, MAX_ARRAY_SIZE);
	std::cout << "deserialize success" << std::endl;
	parser.applyNameTransform(name, msgContent, &msgValues);

	// test
	for (const auto v : msgValues) {
		std::cout << v.first << std::endl;
	}

	return msgValues;
}

/** Return extracted values of a serialized message as a flat vector.
``name`` is the name by which the message was registered, while ``sermsg`` is the serialized message. */
ri::RenamedValues TypeIntrospector::extract(const std::string& name, std::vector<uint8_t>& sermsg) {
	constexpr uint32_t MAX_ARRAY_SIZE = 10;

	static ri::FlatMessage      msgContent;
	static ri::RenamedValues    msgValues;

	std::cout << "type: " << name << std::endl;

	parser.deserializeIntoFlatContainer(name, absl::Span<uint8_t>(sermsg),
	                                    &msgContent, MAX_ARRAY_SIZE);
	std::cout << "deserialize success" << std::endl;
	parser.applyNameTransform(name, msgContent, &msgValues);

	// test
	for (const auto v : msgValues) {
		std::cout << v.first << std::endl;
	}

	return msgValues;
}

} // namespace task_constructor
} // namespace moveit

