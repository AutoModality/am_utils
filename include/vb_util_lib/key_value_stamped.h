/*
 * key_value_stamped.h
 *
 *  Created on: Nov 23, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_KEY_VALUE_STAMPED_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_KEY_VALUE_STAMPED_H_

#include <sstream>
#include <string>
#include <iostream>
#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

#include <am_utils/am_ros2_utility.h>
#include <brain_box_msgs/msg/key_value.hpp>

// Currently limited number of parameter types handled
enum class KeyValueType{INT=0, BOOL, DOUBLE, STRING};

extern const boost::unordered_map<KeyValueType, std::string> param_type_2_string;

class KeyValueStamped {
public:
    KeyValueStamped() {};

	KeyValueStamped(const brain_box_msgs::msg::KeyValue::SharedPtr msg);

	KeyValueStamped(std::string key, KeyValueType type);

	KeyValueStamped(std::string key) : key_(key), type_(KeyValueType::STRING) {
		stamp_ = am::ClockNow();
	};

	KeyValueStamped(std::string key, std::string val) : key_(key), type_(KeyValueType::STRING) {
		setValue(val);
		stamp_ = am::ClockNow();
	};

	KeyValueStamped(std::string key, bool val) : key_(key), type_(KeyValueType::BOOL), bool_value_(val) {
		setValue(val);
		stamp_ = am::ClockNow();
	};

	KeyValueStamped(std::string key, int val) : key_(key), type_(KeyValueType::INT), int_value_(val) {
		setValue(val);
		stamp_ = am::ClockNow();
	};

	KeyValueStamped(std::string key, double val) : key_(key), type_(KeyValueType::DOUBLE), double_value_(val) {
		setValue(val);
		am::ClockNow();
	};

	virtual ~KeyValueStamped() {};

	void getValue(std::string& value) const {
		value = string_value_;
	}
	void getValue(bool& value) const {
		value = bool_value_;
	}
	void getValue(int& value) const {
		value = int_value_;
	}
	void getValue(double& value) const {
		value = double_value_;
	}

	bool setValue(const std::string& value);
	bool setValue(bool value);
	bool setValue(int value);
	bool setValue(double value);
	bool setValue(KeyValueStamped& kv);

	void setKey(const std::string& key) {
		key_ = key;
	}
	std::string getKey() const {
		return key_;
	}

	bool isUpdated() {
		bool ret_val = updated_;
		updated_ = false;
		return ret_val;
	}

	void setKeyValue(const std::string key, const std::string val) {
		key_ = key;
		setValue(val);
	}
	void setKeyValue(const std::string key, bool val) {
		key_ = key;
		setValue(val);
	}
	void setKeyValue(const std::string key, int val) {
		key_ = key;
		setValue(val);
	}
	void setKeyValue(const std::string key, double val) {
		key_ = key;
		setValue(val);
	}

	KeyValueType getType() const {
		return type_;
	}

	void setType(KeyValueType type) {
		type_ = type;
	}

	void sendParam(rclcpp::Publisher<brain_box_msgs::msg::KeyValue>::SharedPtr pub);

    void print(std::string prefix);

    std::string toString() {
    	return "name["+key_+"] type["+param_type_2_string.at(type_)+
    			"] value["+string_value_+", "+std::to_string(bool_value_)+", "+std::to_string(int_value_)+", "+std::to_string(double_value_)+
				"] save_default["+std::to_string(save_default_)+"]";
    }

	bool isSaveDefault() const {
		return save_default_;
	}

	void setSaveDefault(bool saveDefault) {
		save_default_ = saveDefault;
	}

	bool isDbInit() const {
		return db_init_;
	}

	bool saveDefault(rclcpp::Node::SharedPtr node);

private:

	std::string key_ {""};

	std::string string_value_ {""};
	bool bool_value_ {false};
	int int_value_ {0};
	double double_value_ {0.0};

	bool save_default_ {false};

	bool db_init_ {false};

    KeyValueType type_ {KeyValueType::STRING};

	rclcpp::Time stamp_;

	bool updated_ {false};
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_KEY_VALUE_STAMPED_H_ */
