/*
 * param_update.cpp
 *
 *  Created on: Jul 9, 2019
 *      Author: ubuntu
 */

#include <vb_util_lib/param_update.h>

//ParamUpdate::ParamUpdate() {
//	// TODO Auto-generated constructor stub
//
//}

ParamUpdate::ParamUpdate(ros::NodeHandle& nh, bool publisher) : nh_(&nh), publisher_(publisher) {
	// Create ros publishers and subscribers
	if (publisher)
	{
		pub_ = nh.advertise<brain_box_msgs::KeyValue>(PARAM_UPDATE_TOPIC, 10);
	}
	else
	{
		sub_ = nh.subscribe<brain_box_msgs::KeyValue>(PARAM_UPDATE_TOPIC, 10, &ParamUpdate::paramUpdateCB, this);
	}
	ros_init_ = true;
}

ParamUpdate::~ParamUpdate() {
	// TODO Auto-generated destructor stub
}

bool ParamUpdate::sendAllParams(std::vector<KeyValueStamped>& param_list) {
	ROS_INFO("    ParamUpdate::sendAllParams");

	for (std::vector<KeyValueStamped>::iterator i = param_list.begin(); i != param_list.end(); i++)
	{
		i->sendParam(pub_);
	}
}

bool ParamUpdate::sendAllDefaultParams() {
	ROS_INFO("    ParamUpdate::sendAllDefaultParams");

	for (std::vector<KeyValueStamped>::iterator i = default_params_.begin(); i != default_params_.end(); i++)
	{
		if (i->isDbInit())
		{
			i->sendParam(pub_);
		}
	}

	return true;
}

bool ParamUpdate::updateDefaultParams() {
	ROS_INFO("    ParamUpdate::updateDefaultParams");
	for (std::vector<KeyValueStamped>::iterator i = default_params_.begin(); i != default_params_.end(); i++)
	{
		i->saveDefault(*nh_);
	}
}

void ParamUpdate::paramUpdateCB(const brain_box_msgs::KeyValue::ConstPtr& msg) {
	KeyValueStamped kv(msg);

	setParam(kv, true);
}

bool ParamUpdate::paramUpdated(KeyValueStamped& param) {
	if (!updated_)
	{
		return false;
	}

	updated_ = false;

	param = latest_update_;

	return true;
}

void ParamUpdate::addParam(KeyValueStamped& kv, bool save_default,
		std::vector<KeyValueStamped>& param_list) {

	KeyValueStamped kvs = kv;
	if (getParam(kv.getKey(), kvs, param_list))
	{
		// Already there so don't dup it
		return;
	}

	kvs.setSaveDefault(save_default);
	param_list.push_back(kvs);

	if (save_default)
	{
		KeyValueStamped kvd = kvs;
		if (!getDefaultParam(kvd.getKey(), kvd))
		{
			// Doesn't exist in the defaults list so add it
			default_params_.push_back(kvd);
		}
	}
}

bool ParamUpdate::getParam(std::string key, KeyValueStamped& param,
		std::vector<KeyValueStamped>& param_list) {

	for (int i = 0; i < param_list.size(); i++)
	{
		if (param_list.at(i).getKey() == key)
		{
			param = param_list.at(i);
			return true;
		}
	}

	return false;
}

bool ParamUpdate::getParam(std::string key, KeyValueType type, KeyValueStamped& param,
		std::vector<KeyValueStamped>& param_list) {

	for (int i = 0; i < param_list.size(); i++)
	{
		if (param_list.at(i).getKey() == key && param_list.at(i).getType() == type)
		{
			param = param_list.at(i);
			return true;
		}
	}

	return false;
}

bool ParamUpdate::sendParam(KeyValueStamped& kvs) {
	if (!ros_init_)
	{
		return false;
	}

	kvs.sendParam(pub_);
	return true;
}

KeyValueStamped* ParamUpdate::findParam(KeyValueStamped& kvs) {
	for (std::vector<KeyValueStamped>::iterator i = params_.begin(); i != params_.end(); i++)
	{
		if (i->getKey() == kvs.getKey() && i->getType() == kvs.getType())
		{
			return &(*i);
		}
	}

	return NULL;
}

bool ParamUpdate::setParam(KeyValueStamped& kvs, bool update_param_db) {
	KeyValueStamped* kv;

	if ((kv = findParam(kvs)) == NULL)
	{
		ROS_WARN("ParamUpdate::setParam KEY[%s] not found", kvs.getKey().c_str());
		return false;
	}

	kv->setValue(kvs);
	latest_update_ = *kv;
	updated_ = true;

	if (update_param_db)
	{
		// Update the parameter DB
		int int_val;
		double double_val;
		std::string string_val;
		bool bool_val;
		if (!nh_->hasParam(kv->getKey()))
		{
			ROS_WARN("ParamUpdate::setParam: NO PARAM DB ENTRY FOUND FOR %s", kv->getKey().c_str());
		}
		else
		{
			ROS_INFO("ParamUpdate::setParam: UPDATING PARAM DB %s", kv->getKey().c_str());

			switch (kv->getType())
			{
			case KeyValueType::INT:
				kv->getValue(int_val);
				nh_->setParam(kv->getKey(), int_val);
				break;
			case KeyValueType::STRING:
				kv->getValue(string_val);
				nh_->setParam(kv->getKey(), string_val);
				break;
			case KeyValueType::BOOL:
				kv->getValue(bool_val);
				nh_->setParam(kv->getKey(), bool_val);
				break;
			case KeyValueType::DOUBLE:
				kv->getValue(double_val);
				nh_->setParam(kv->getKey(), double_val);
				break;
			}
		}
	}

	if (callback_ != NULL)
	{
		callback_(*kv);
	}

	return true;
}

void ParamUpdate::print(std::string prefix) {
	std::cout << prefix << "PARAMS:";
	if (params_.size() == 0)
	{
		std::cout << "NONE" << std::endl;
	}
	else
	{
		std::cout << std::endl;
		for (std::vector<KeyValueStamped>::iterator i = params_.begin(); i != params_.end(); i++)
		{
			i->print(prefix+"     ");
		}
	}
	std::cout << prefix << "DEFAULTS:";
	if (default_params_.size() == 0)
	{
		std::cout << "NONE" << std::endl;
	}
	else
	{
		std::cout << std::endl;
		for (std::vector<KeyValueStamped>::iterator i = default_params_.begin(); i != default_params_.end(); i++)
		{
			i->print(prefix+"     ");
		}
	}
}
