/*
 * location_source.h
 *
 *  Created on: Mar 15, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_LOCATION_SOURCE_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_LOCATION_SOURCE_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <brain_box_msgs/LocationSource.h>

//#include <boost/assign/list_of.hpp>
//#include <boost/unordered_map.hpp>
//
//using boost::assign::map_list_of;
//

enum class PositionSource{GPS=0, RTK, FCU_VEL_ODOM, FCU_VO_POS, FCU_VO_ODOM, FEATURE, FEATURE_ODOM, FUSION, INTEL_VO_POS, INTEL_VO_ODOM, BESPOON, TEST};
#define PositionSourceIdx(src) static_cast<int>(src)

enum class OrientationSource{FCU=0, FCU_ODOM, INTEL_VO, INTEL_VO_ODOM, FEATURE, FEATURE_ODOM};
#define OrientationSourceIdx(src) static_cast<int>(src)

enum class AltitudeSource{GPS_ALT=0, GPS_AGL, FCU_LAUNCH, ALTIMETER, ALTIMETER_ODOM, FEATURE, FEATURE_ODOM, FCU_VEL_ODOM, FCU_VO_POS, FCU_VO_ODOM, INTEL_VO_POS, INTEL_VO_ODOM};
#define AltitudeSourceIdx(src) static_cast<int>(src)


class LocationSource
{
public:
	/// Structure used to describe how a feature is used for localization purposes
	struct FeatureSource {
		std::string id;		// id of the feature
		bool fixed;			// true if the feature is fixed in parent FoR, i.e. not moving
		bool critical;		// true if the feature is critical, i.e. error thrown if not being updated
		bool dof_x;			// true if used for X dof
		bool dof_y;			// true is used for Y dof
		bool dof_z;			// true if used for Z dof
		bool dof_yaw;		// true if used for YAW dof
	};

    LocationSource() {}
    LocationSource(std::string frame_id, std::string x, std::string y, std::string z, std::string yaw);
    virtual ~LocationSource();

    void print(std::string prefix);

    const std::string& xId() const {
        return x_id_;
    }

    const std::string& frameId() const {
        return frame_id_;
    }

    void xId(const std::string& id) {
        x_id_ = id;
    }

    const std::string& yId() const {
        return y_id_;
    }

    void yId(const std::string& id) {
        y_id_ = id;
    }

    const std::string& yawId() const {
        return yaw_id_;
    }

    void yawId(const std::string& yaw_id) {
        yaw_id_ = yaw_id;
    }

    const std::string& zId() const {
        return z_id_;
    }

    void zId(const std::string& id) {
        z_id_ = id;
    }

    void setId(std::string frame_id, std::string x, std::string y, std::string z, std::string yaw) {
    	frame_id_ = frame_id;
        x_id_ = x;
        y_id_ = y;
        z_id_ = z;
        yaw_id_ = yaw;
        initialized_ = true;
    }

    bool isSet() const {
    	return initialized_;
    }

    // Mappings between various location sources
    std::map<OrientationSource, std::string> OrientationSource_to_string = {
            {OrientationSource::FCU, "FCU"},
            {OrientationSource::FCU_ODOM, "FCU_ODOM"},
            {OrientationSource::INTEL_VO_ODOM, "INTEL_VO_ODOM"},
            {OrientationSource::INTEL_VO, "INTEL_VO"},
            {OrientationSource::FEATURE_ODOM, "FEATURE_ODOM"},
            {OrientationSource::FEATURE, "FEATURE"}};

    std::map<std::string, OrientationSource> string_to_OrientationSource = {
            {"FCU", OrientationSource::FCU},
            {"FCU_ODOM", OrientationSource::FCU_ODOM},
            {"INTEL_VO", OrientationSource::INTEL_VO},
            {"INTEL_VO_ODOM", OrientationSource::INTEL_VO_ODOM},
            {"FEATURE_ODOM", OrientationSource::FEATURE_ODOM},
            {"FEATURE", OrientationSource::FEATURE}};

    std::map<PositionSource, std::string> PositionSource_to_string = {
            {PositionSource::GPS, "GPS"},
            {PositionSource::RTK, "RTK"},
            {PositionSource::FCU_VEL_ODOM, "FCU_VEL_ODOM"},
            {PositionSource::FCU_VO_POS, "FCU_VO_POS"},
            {PositionSource::FCU_VO_ODOM, "FCU_VO_ODOM"},
            {PositionSource::FEATURE, "FEATURE"},
            {PositionSource::FEATURE_ODOM, "FEATURE_ODOM"},
            {PositionSource::FUSION, "FUSION"},
            {PositionSource::INTEL_VO_POS, "INTEL_VO_POS"},
    		{PositionSource::INTEL_VO_ODOM, "INTEL_VO_ODOM"}};

    std::map<std::string, PositionSource> string_to_PositionSource = {
            {"GPS", PositionSource::GPS},
            {"RTK", PositionSource::RTK},
            {"FCU_VEL_ODOM", PositionSource::FCU_VEL_ODOM},
            {"FCU_VO_POS", PositionSource::FCU_VO_POS},
            {"FCU_VO_ODOM", PositionSource::FCU_VO_ODOM},
            {"FEATURE", PositionSource::FEATURE},
            {"FEATURE_ODOM", PositionSource::FEATURE_ODOM},
            {"FUSION", PositionSource::FUSION},
            {"INTEL_VO_POS", PositionSource::INTEL_VO_POS},
    		{"INTEL_VO_ODOM", PositionSource::INTEL_VO_ODOM}};

    std::map<AltitudeSource, std::string> AltitudeSource_to_string = {
            {AltitudeSource::ALTIMETER, "ALTIMETER"},
            {AltitudeSource::ALTIMETER_ODOM, "ALTIMETER_ODOM"},
            {AltitudeSource::FCU_LAUNCH, "FCU_LAUNCH"},
            {AltitudeSource::FEATURE, "FEATURE"},
            {AltitudeSource::FEATURE_ODOM, "FEATURE_ODOM"},
            {AltitudeSource::FCU_VEL_ODOM, "FCU_VEL_ODOM"},
            {AltitudeSource::FCU_VO_POS, "FCU_VO_POS"},
            {AltitudeSource::FCU_VO_ODOM, "FCU_VO_ODOM"},
            {AltitudeSource::GPS_AGL, "GPS_AGL"},
            {AltitudeSource::GPS_ALT, "GPS_ALT"},
            {AltitudeSource::INTEL_VO_POS, "INTEL_VO_POS"},
            {AltitudeSource::INTEL_VO_ODOM, "INTEL_VO_ODOM"}};

    std::map<std::string, AltitudeSource> string_to_AltitudeSource = {
            {"ALTIMETER", AltitudeSource::ALTIMETER},
            {"ALTIMETER_ODOM", AltitudeSource::ALTIMETER_ODOM},
            {"FCU_LAUNCH", AltitudeSource::FCU_LAUNCH},
            {"FEATURE", AltitudeSource::FEATURE},
            {"FEATURE_ODOM", AltitudeSource::FEATURE_ODOM},
            {"FCU_VEL_ODOM", AltitudeSource::FCU_VEL_ODOM},
            {"FCU_VO_POS", AltitudeSource::FCU_VO_POS},
            {"FCU_VO_ODOM", AltitudeSource::FCU_VO_ODOM},
            {"GPS_AGL", AltitudeSource::GPS_AGL},
            {"GPS_ALT", AltitudeSource::GPS_ALT},
            {"INTEL_VO_POS", AltitudeSource::INTEL_VO_POS},
            {"INTEL_VO_ODOM", AltitudeSource::INTEL_VO_ODOM}};


    void setAltSrc(const std::string value) {
    	altitude_source_ = string_to_AltitudeSource[value];
    	altitude_source_set_ = true;
    }
    void setAltSrc(const AltitudeSource altitudeSource ) {
    	altitude_source_ = altitudeSource;
    	altitude_source_set_ = true;
    }
    AltitudeSource getAltSrc() const {
    	return altitude_source_;
    }
    bool isAltSrcSet() const {
    	return altitude_source_set_;
    }

    void setPosSrc(const std::string value) {
    	position_source_ = string_to_PositionSource[value];
    	position_source_set_ = true;
    }
    void setPosSrc(const PositionSource positionSource) {
    	position_source_ = positionSource;
    	position_source_set_ = true;
    }
    PositionSource getPosSrc() const {
    	return position_source_;
    }
    bool isPosSrcSet() {
    	return position_source_set_;
    }

    void setAttSrc(const std::string value) {
    	orientation_source_ = string_to_OrientationSource[value];
    	orientation_source_set_ = true;
    }
    void setAttSrc(const OrientationSource orientationSource) {
    	orientation_source_ = orientationSource;
    	orientation_source_set_ = true;
    }
    OrientationSource getAttSrc() const {
    	return orientation_source_;
    }
    bool isAttSrcSet() const {
    	return orientation_source_set_;
    }

	std::vector<FeatureSource>& getFeatures() {
		return features_;
	}

private:
    bool initialized_ {false};
    std::string frame_id_ {""};
    std::string x_id_ {""};;
    std::string y_id_ {""};
    std::string z_id_ {""};
    std::string yaw_id_ {""};

    PositionSource position_source_ {PositionSource::FCU_VO_ODOM};
    bool position_source_set_ {false};
    AltitudeSource altitude_source_ {AltitudeSource::ALTIMETER};
    bool altitude_source_set_ {false};
    OrientationSource orientation_source_ {OrientationSource::FCU};
    bool orientation_source_set_ {false};

    /// Used to receive location source messages
    ros::Subscriber location_source_sub_;

    /// Used to receive location source messages
    ros::Subscriber location_source_pub_;
    void locationSourceCB(const brain_box_msgs::LocationSource::ConstPtr& msg);

    /// Update the vector of control states with the message just received
    void updateLocationSource(const brain_box_msgs::LocationSource::ConstPtr& msg);

    /// List of all the features being used for localization purposes
    std::vector<FeatureSource>features_;
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_LOCATION_SOURCE_H_ */
