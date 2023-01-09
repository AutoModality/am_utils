/*
 * key_value_stamped.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/key_value_stamped.h>

using boost::assign::map_list_of;

const boost::unordered_map<KeyValueType, uint8_t> param_type_2_msg = map_list_of
		(KeyValueType::INT, 0)
		(KeyValueType::BOOL, 1)
		(KeyValueType::DOUBLE, 2)
		(KeyValueType::STRING, 3);

const boost::unordered_map<KeyValueType, std::string> param_type_2_string = map_list_of
		(KeyValueType::INT, "INT")
		(KeyValueType::BOOL, "BOOL")
		(KeyValueType::DOUBLE, "DOUBLE")
		(KeyValueType::STRING, "STRING");

const boost::unordered_map<uint8_t, KeyValueType> param_msg_2_type = map_list_of
		(0, KeyValueType::INT)
		(1, KeyValueType::BOOL)
		(2, KeyValueType::DOUBLE)
		(3, KeyValueType::STRING);

KeyValueStamped::KeyValueStamped(const brain_box_msgs::msg::KeyValue::SharedPtr msg)
{
	brain_box_msgs::msg::KeyValue p = *msg;
	key_ = p.key;
	type_ = param_msg_2_type.at(p.type);
	if (!setValue(p.value))
	{
		ROS_ERROR("KeyValueStamped::KeyValueStamped invalid format for param[%s] type[%d] value[%s]", key_.c_str(), (int)p.type, p.value.c_str());
	}
	stamp_ = am::ClockNow();
}

KeyValueStamped::KeyValueStamped(std::string key, KeyValueType type) : key_(key), type_(type) {
	// Check to see if this value is in the parameter DB and if so initialize it to that value
	stamp_ = am::ClockNow();

	std::string s;
	int i;
	bool b;
	double d;

	rclcpp::Parameter key_p;
	if (!am::Node::node->get_parameter(key, key_p))
	{
		ROS_WARN("KeyValueStamped::KeyValueStamped no param[%s] in param server", key.c_str());
		return;
	}

	switch (type_)
	{
	case KeyValueType::INT:
		if (!am::Node::node->get_parameter(key, i))
		{
			ROS_ERROR("KeyValueStamped::KeyValueStamped type mismatch for parameter[%s] looking for INT", key_.c_str());
			return;
		}
		setValue(i);
		break;
	case KeyValueType::BOOL:
		if (!am::Node::node->get_parameter(key, b))
		{
			ROS_ERROR("KeyValueStamped::KeyValueStamped type mismatch for parameter[%s] looking for BOOL", key_.c_str());
			return;
		}
		setValue(b);
		break;
	case KeyValueType::DOUBLE:
		if (!am::Node::node->get_parameter(key, d))
		{
			ROS_ERROR("KeyValueStamped::KeyValueStamped type mismatch for parameter[%s] looking for DOUBLE", key_.c_str());
			return;
		}
		setValue(d);
		break;
	case KeyValueType::STRING:
		if (!am::Node::node->get_parameter(key, s))
		{
			ROS_ERROR("KeyValueStamped::KeyValueStamped type mismatch for parameter[%s] looking for DOUBLE", key_.c_str());
			setValue(s);
			return;
		}
		break;
	}
};

bool KeyValueStamped::setValue(const std::string& value) {
	string_value_ = value;

	std::istringstream iss (value);

	switch (type_)
	{
	case KeyValueType::INT:
		iss >> int_value_;
		if (iss.fail())
		{
			return false;
		}
		bool_value_ = int_value_;
		double_value_ = int_value_;
		break;
	case KeyValueType::BOOL:
		bool_value_ = (value == "true") || (value == "TRUE");
		if (!bool_value_)
		{
			if (value != "FALSE" && value != "false")
			{
				return false;
			}
			int_value_ = bool_value_;
			double_value_ = bool_value_;
		}
		break;
	case KeyValueType::DOUBLE:
		iss >> double_value_;
		if (iss.fail())
		{
			return false;
		}
		bool_value_ = double_value_;
		int_value_ = double_value_;
		break;
	}

	return true;
}

bool KeyValueStamped::setValue(bool value) {
	string_value_ = value ? "TRUE" : "FALSE";
	bool_value_ = value;
	int_value_ = value;
	double_value_ = value;
	stamp_ = am::ClockNow();
	updated_ = true;
	return true;
}

bool KeyValueStamped::setValue(int value) {
	string_value_ = std::to_string(value);
	bool_value_ = value != 0;
	int_value_ = value;
	double_value_ = value;
	stamp_ = am::ClockNow();
	updated_ = true;
	return true;
}

bool KeyValueStamped::setValue(double value) {
	string_value_ = std::to_string(value);
	bool_value_ = value != 0.0;
	int_value_ = value;
	double_value_ = value;
	stamp_ = am::ClockNow();
	updated_ = true;
	return true;
}

bool KeyValueStamped::setValue(KeyValueStamped& kv) {
	string_value_ = kv.string_value_;
	bool_value_ = kv.bool_value_;
	int_value_ = kv.int_value_;
	double_value_ = kv.double_value_;
	stamp_ = am::ClockNow();
	updated_ = true;
	return true;
}

void KeyValueStamped::sendParam(rclcpp::Publisher<brain_box_msgs::msg::KeyValue>::SharedPtr pub) {
	// Send the update message out
	brain_box_msgs::msg::KeyValue kvm;
	kvm.key = key_;
	kvm.value = string_value_;
	kvm.type = param_type_2_msg.at(type_);
	ROS_INFO("      KeyValueStamped::sendParam %s", toString().c_str());
	pub->publish(kvm);
}

void KeyValueStamped::print(std::string prefix) {
//	std::cout << preRCLCPP_INFOfix+"param[" << key_ << "]  type[" << param_type_2_string.at(type_) << "]  value[" << string_value_ << "]" << std::endl;
	std::cout << prefix+toString() << std::endl;
}

bool KeyValueStamped::saveDefault(rclcpp::Node::SharedPtr nh) {
	ROS_WARN("    KeyValueStamped::saveDefault saving defaults for %s", toString().c_str());

	int int_val;
	double double_val;
	std::string string_val;
	bool bool_val;
	bool ret_val;
	rclcpp::Parameter param;
	if (!nh->get_parameter(getKey(), param))
	{
		ROS_WARN("KeyValueStamped::saveDefault: NO DEFAULT VALUE FOUND FOR %s", toString().c_str());
		db_init_ = false;
		return false;
	}
	else
	{
		switch (getType())
		{
		case KeyValueType::INT:
			ret_val = nh->get_parameter(getKey(), int_val);
			if (ret_val)
			{
				setValue(int_val);
			}
			break;
		case KeyValueType::STRING:
			ret_val = nh->get_parameter(getKey(), string_val);
			if (ret_val)
			{
				setValue(string_val);
			}
			break;
		case KeyValueType::BOOL:
			ret_val = nh->get_parameter(getKey(), bool_val);
			if (ret_val)
			{
				setValue(bool_val);
			}
			break;
		case KeyValueType::DOUBLE:
			ret_val = nh->get_parameter(getKey(), double_val);
			if (ret_val)
			{
				setValue(double_val);
			}
			break;
		}
	}

	if (!ret_val)
	{
		ROS_WARN("KeyValueStamped::saveDefault: UNABLE TO GET THE PARAM DB VALUE FOR %s", toString().c_str());
		db_init_ = false;
		return false;
	}

	db_init_ = true;
	return true;
}
