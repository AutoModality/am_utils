/*
 * location_source.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: ubuntu
 */

#include <vb_util_lib/location_source.h>

LocationSource::LocationSource(std::string frame_id, std::string x, std::string y, std::string z, std::string yaw) :
frame_id_(frame_id), x_id_(x), y_id_(y), z_id_(z), yaw_id_(yaw), initialized_(true) {
}

LocationSource::~LocationSource() {
    // TODO Auto-generated destructor stub
}

void LocationSource::print(std::string prefix) {
    std::cout << prefix << "LOCATION SOURCE: ";
    if (initialized_ || isAltSrcSet() || isPosSrcSet())
    {
    	std::cout << std::endl;
    	if (initialized_)
    	{
    		std::cout << prefix+"     " << "Frame Sources: FRAME_ID[" << frame_id_ << "]  DOF_X[";
    		std::cout << x_id_ << "]  DOF_Y[" << y_id_ << "]  DOF_Z[" << z_id_ << "]  DOF_YAW[" << yaw_id_ << "]" << std::endl;
    	}
    	else
    	{
    		std::cout << prefix+"     " << "Frame Sources: NONE" << std::endl;
    	}

    	std::cout << prefix+"     Sensor Sources: ";
    	if (isAltSrcSet() || isPosSrcSet())
    	{
    		if (isPosSrcSet())
    		{
    			std::cout << prefix+"     " << "position_source[" << PositionSource_to_string.at(position_source_);
    		}
    		if (isAltSrcSet())
    		{
    			std::cout << "]  altitude_source[" << AltitudeSource_to_string.at(altitude_source_) << "]";
    		}
    		std::cout << std::endl;
    	}
    	else
    	{
    		std::cout << "NONE" << std::endl;
    	}

    	std::cout << prefix+"     Features: ";
    	if (feature_ids_.size() == 0)
    	{
    		std::cout << "NONE" << std::endl;
    	}
    	else
    	{
    		std::cout << std::endl;
    		for (std::vector<std::string>::iterator i = feature_ids_.begin(); i != feature_ids_.end(); i++)
    		{
    			std::cout << prefix+"          feature_id[" << *i << "]" << std::endl;
    		}
    	}
    }
    else
    {
		std::cout << "NONE" << std::endl;
    }

}

