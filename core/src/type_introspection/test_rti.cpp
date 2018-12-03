#include <moveit/task_constructor/type_introspection/parse_property.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros_type_introspection/ros_introspection.hpp>

#include <moveit_task_constructor_msgs/Property.h>

namespace ri = RosIntrospection;

using namespace moveit::task_constructor;

int main(int argc, char** argv) {
	TypeIntrospector inspector;

	rosbag::Bag bag;
	bag.open("/homes/jebert/2018-11-28-14-00-08.bag");
	rosbag::View bag_view(bag);
	// register (only once at the beginning) the type of messages
	for (const rosbag::ConnectionInfo* connection : bag_view.getConnections()) {
		// const std::string& topic_name = connection->topic;
		const std::string& datatype   = connection->datatype;
		const std::string& definition = connection->msg_def;
		// register the type using the topic_name as identifier.
		inspector.parser.registerMessageDefinition(datatype, ri::ROSType(datatype), definition);
	}

	std::vector<uint8_t> buffer;
	for (const rosbag::MessageInstance& msg_instance : bag_view) {
		// write into buffer
		const size_t msg_size  = msg_instance.size();
		buffer.resize(msg_size);
		ros::serialization::OStream stream(buffer.data(), buffer.size());
		msg_instance.write(stream);

		std::cout << "type: " << msg_instance.getDataType() << std::endl;
		ri::RenamedValues vals = inspector.extract(msg_instance.getDataType(), buffer);
	}

	// const moveit_task_constructor_msgs::Property msg = moveit_task_constructor_msgs::Property();

	// inspector.registerMsgType(msg);
	// ri::RenamedValues vals = inspector.extractFromPropertyMsg(msg);
}

