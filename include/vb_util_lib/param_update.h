/*
 * param_update.h
 *
 *  Created on: Jul 9, 2019
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_PARAM_UPDATE_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_PARAM_UPDATE_H_

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <vb_util_lib/key_value_stamped.h>
#include <brain_box_msgs/KeyValue.h>

#define PARAM_UPDATE_TOPIC "/param_update"

class ParamUpdate {
public:
//	ParamUpdate();
	ParamUpdate(ros::NodeHandle& nh, bool publisher=false);
	virtual ~ParamUpdate();

	void addParam(std::string key, KeyValueType type, bool save_default = false) {
		addParam(key, type, save_default, params_);
	}

	void addParam(KeyValueStamped& kv, bool save_default = false) {
		addParam(kv, save_default, params_);
	}

//	void addDefaultParam(std::string key, KeyValueType type) {
//		addParam(key, type, false, default_params_);
//	}
//
    /**
     * Install a callback handler.
     * Use the following sytax to call this function:
     *   setCallback(std::bind(&MyClass::Callback, this, std::placeholders::_1));
     */
    void setCallback(std::function<void(KeyValueStamped&)> callback) {
        callback_ = callback;
    }

    // Return the KeyValue object from the params list
    bool getParam(KeyValueStamped& param) {
    	return getParam(param.getKey(), param.getType(), param);
    }
    bool getParam(std::string key, KeyValueType type, KeyValueStamped& param) {
    	return getParam(key, type, param, params_);
    }
    bool getParam(std::string key, KeyValueStamped& param) {
    	return getParam(key, param, params_);
    }

    // Return the KeyValue object from the default params list
    bool getDefaultParam(KeyValueStamped& param) {
    	return getDefaultParam(param.getKey(), param.getType(), param);
    }
    bool getDefaultParam(std::string key, KeyValueType type, KeyValueStamped& param) {
    	return getParam(key, type, param, default_params_);
    }
    bool getDefaultParam(std::string key, KeyValueStamped& param) {
    	return getParam(key, param, default_params_);
    }

    // Set the value for the specified key value object, but only if the keyValue pair has been registered by addParam
    bool setParam(KeyValueStamped& kvs, bool update_param_db=false);

    // Returns true if a Key Value pair has been updated since the last call to the method.
    // Also returns the KeyValueStamped object of the last parameter updated.
    bool paramUpdated(KeyValueStamped& param);

    // Sends out a Key Value update message regardless of whether it has been registered
    bool sendParam(KeyValueStamped& kvs);

    // Sends all the params in the params list.
    // If save default is true then it also saves the default parameters of that parameter before sending it out
    bool sendAllParams(bool save_defaults = false) {
    	if (save_defaults)
    	{
    		updateDefaultParams();
    	}
    	return sendAllParams(params_);
    }

    bool sendAllDefaultParams();

    void print(std::string prefix);

	const std::vector<KeyValueStamped>& getDefaultParams() const {
		return default_params_;
	}

	const std::vector<KeyValueStamped>& getParams() const {
		return params_;
	}

	bool isPublisher() const {
		return publisher_;
	}

private:
	ros::NodeHandle* nh_;
	bool publisher_ {false};

    void addParam(std::string key, KeyValueType type, bool save_default, std::vector<KeyValueStamped>& param_list) {
    	KeyValueStamped kv(key, type);
    	addParam(kv, save_default, param_list);
    }
    void addParam(KeyValueStamped& kv, bool save_default, std::vector<KeyValueStamped>& param_list);

    bool getParam(std::string key, KeyValueType type, KeyValueStamped& param, std::vector<KeyValueStamped>& param_list);
    bool getParam(std::string key, KeyValueStamped& param, std::vector<KeyValueStamped>& param_list);

    bool sendAllParams(std::vector<KeyValueStamped>& param_list);
    bool updateDefaultParams();
    KeyValueStamped* findParam(KeyValueStamped& kvs);

	ros::Publisher pub_;
	ros::Subscriber sub_;
	bool ros_init_ {false};

	KeyValueStamped latest_update_;

	bool updated_ {false};

	std::vector<KeyValueStamped>params_;

	std::vector<KeyValueStamped>default_params_;

    /**
     * Callback function pointer
     */
    std::function<void(KeyValueStamped&)> callback_ {NULL};

    void paramUpdateCB(const brain_box_msgs::KeyValue::ConstPtr& msg);
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_PARAM_UPDATE_H_ */
