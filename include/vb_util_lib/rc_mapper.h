/* rm_mapper.h
 *
 * Created on: 6 March 2020
 *     Author: tkeitel
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_RC_MAPPER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_RC_MAPPER_H_

#include <stdio.h>
#include <string>
#include <ros/ros.h>
//#include <sensor_msgs/Joy.h>
//#include <mavros_msgs/RCIn.h>


class rc_mapper
 {
    public:
    // DJI
    #define  DJI_RC_THROTTLE 3 
    #define  DJI_RC_YAW 2 
    #define  DJI_RC_ROLL 0 
    #define  DJI_RC_PITCH 1 
    #define  DJI_RC_GEAR 5 
    #define  DJI_RC_MODE 4 
    #define  DJI_RC_ARM 6 

    #define  DJI_RC_THROTTLE_SIGN 1 
    #define  DJI_RC_YAW_SIGN 1 
    #define  DJI_RC_ROLL_SIGN 1 
    #define  DJI_RC_PITCH_SIGN 1 


    #define  DJI_RC_MAX_VALUE 1987 
    #define  DJI_RC_MIN_VALUE 998 

    // PX4
    #define  PX4_RC_THROTTLE 2 
    #define  PX4_RC_YAW 3 
    #define  PX4_RC_ROLL 0 
    #define  PX4_RC_PITCH 1 
    #define  PX4_RC_GEAR 7 
    #define  PX4_RC_MODE 5 
    #define  PX4_RC_ARM 6 

    #define  PX4_RC_THROTTLE_SIGN 1 
    #define  PX4_RC_YAW_SIGN 1 
    #define  PX4_RC_ROLL_SIGN 1 
    #define  PX4_RC_PITCH_SIGN 1 

    #define  PX4_RC_MAX_VALUE 1987 
    #define  PX4_RC_MIN_VALUE 998 

    // ACSL
    #define  ACSL_RC_THROTTLE 2 
    #define  ACSL_RC_YAW 3 
    #define  ACSL_RC_ROLL 0 
    #define  ACSL_RC_PITCH 1 
    #define  ACSL_RC_GEAR 8 
    #define  ACSL_RC_MODE 6 
    #define  ACSL_RC_ARM 9 

    #define  ACSL_RC_THROTTLE_SIGN -1 
    #define  ACSL_RC_YAW_SIGN 1 
    #define  ACSL_RC_ROLL_SIGN 1 
    #define  ACSL_RC_PITCH_SIGN -1      

    #define  ACSL_RC_MAX_VALUE 2020 
    #define  ACSL_RC_MIN_VALUE 1020 

    // LOGITECH
    #define  LOGITECH_RC_THROTTLE 1
    #define  LOGITECH_RC_YAW 0
    #define  LOGITECH_RC_ROLL 2
    #define  LOGITECH_RC_PITCH 3
    #define  LOGITECH_RC_GEAR 0
    #define  LOGITECH_RC_MODE 0
    #define  LOGITECH_RC_ARM 0

    #define  LOGITECH_RC_THROTTLE_SIGN 1
    #define  LOGITECH_RC_YAW_SIGN 1
    #define  LOGITECH_RC_ROLL_SIGN 1
    #define  LOGITECH_RC_PITCH_SIGN 1

    #define  LOGITECH_RC_MAX_VALUE 1
    #define  LOGITECH_RC_MIN_VALUE -1

    // Default
    #define DEFAULT_MIN_VALUE 1020 
    #define DEFAULT_MAX_VALUE 2020 

    enum RC_Definitions
    {
            eRCThrottle = 0,
            eRCYaw,
            eRCRoll,
            eRCPitch,
            eRCGear,
            eRCMode,
            eRCArm,
            eEndOfRCDefinitions
    };

    enum RC_Signs
    {
            eThrottleSign = 0,
            eYawSign,
            eRollSign,
            ePitchSign,
            eEndOfRCSigns
     };

    enum RC_TYPES
    {
        eRCT_PX4 = 0,
        eRCT_DJI,
        eRCT_ACSL,
        eRCT_LOGITECH,
        eRCT_Unknown,
        eEndOfRCTypes
    };

    rc_mapper(ros::NodeHandle n);
    
    virtual ~rc_mapper();

    int GetChannel(int _chan);
    
    bool SetChannel(int _chan, int _signal);
    

    int GetMax(void);
    int GetMin(void);

    void SetMax(int _max);
    void SetMin(int _min);

    int GetSign(int _chan);

    bool SetSign(int _chan, int _sign);

    std::string GetRCType(void);
    std::string GetRCTypeName(void);
    RC_TYPES GetSetRC(void);

    private:
        ros::NodeHandle m_Handle;
        RC_TYPES m_SetRC; 
        std::string m_RCType;
        std::string m_RCTypeNames[eEndOfRCTypes];
        int m_Channels[eEndOfRCDefinitions];
        int m_Signs[eEndOfRCSigns];
        int m_MinValue;
        int m_MaxValue;

        static bool Valid(int _chan);

        void SetFromDefaults(void);

        RC_TYPES DetermineRCSetting(std::string* _rctype);

        void SetFromParameters(void);
    };

#endif

