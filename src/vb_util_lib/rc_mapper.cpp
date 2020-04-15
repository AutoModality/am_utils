/*
* rc_mapper.cpp
*
* Created on: March 5th, 2020
*     Author: Todd Keitel
*
*/

#include <rc_mapper.h>
#include <vb_util_lib/topics.h>

rc_mapper::rc_mapper(ros::NodeHandle n)
: m_Handle(n)
, m_SetRC(eRCT_DJI)
, m_MinValue(1)
, m_MaxValue(1)
{
    m_RCTypeNames[eRCT_PX4].assign("PX4");
    m_RCTypeNames[eRCT_DJI].assign("DJI");
    m_RCTypeNames[eRCT_ACSL].assign("ACSL");
    m_RCTypeNames[eRCT_Unknown].assign("Unknown");

    //default settings
    m_RCType.assign(m_RCTypeNames[m_SetRC].c_str());
    SetFromDefaults();

    //set parameters form the input parameters (if present)
    SetFromParameters();
}

rc_mapper::~rc_mapper() {}

int rc_mapper::GetChannel(int _chan)
{
    int retvalue = 0;
    if(Valid(_chan))
    {
        retvalue = m_Channels[_chan];
    }
    return retvalue;
}

bool rc_mapper::SetChannel(int _chan, int _signal)
{
    bool success = false;
    if(Valid(_chan))
    {
        success = true;
        m_Channels[_chan] = _signal;
    }
        return success;
}

int rc_mapper::GetMax(void) { return m_MaxValue; }
int rc_mapper::GetMin(void) { return m_MinValue; }

void rc_mapper::SetMax(int _max) { m_MaxValue = _max;}
void rc_mapper::SetMin(int _min) { m_MinValue = _min;}

int rc_mapper::GetSign(int _chan)
{
    int retvalue = 0;
    if(Valid(_chan))
    {
        retvalue = m_Signs[_chan];
    }
    return retvalue;
}

bool rc_mapper::SetSign(int _chan, int _sign)
{
    bool success = false;
    if(Valid(_chan))
    {
        m_Signs[_chan] = _sign;
        success = true;
    }
    return success;
}

std::string rc_mapper::GetRCType(void) { return m_RCType; }
std::string rc_mapper::GetRCTypeName(void) { return m_RCType; }
rc_mapper::RC_TYPES rc_mapper::GetSetRC(void) { return m_SetRC; }

bool rc_mapper::Valid(int _chan) { return _chan >= 0 && _chan < eEndOfRCDefinitions; }

void rc_mapper::SetFromDefaults(void)
{
        //Set default type to DJI
    m_SetRC = eRCT_DJI;
    m_RCType.assign(m_RCTypeNames[m_SetRC].c_str());
    //Values [to be] obtained from testing with DJI RC controller

    ///TODO Find the real DJI values for m_Channels and m_Signs
    m_Channels[eRCThrottle] = DJI_RC_THROTTLE;
    m_Channels[eRCYaw] = DJI_RC_YAW;
    m_Channels[eRCRoll] = DJI_RC_ROLL;
    m_Channels[eRCPitch] = DJI_RC_PITCH;
    m_Channels[eRCGear] = DJI_RC_GEAR;
    m_Channels[eRCMode] = DJI_RC_MODE;
    m_Channels[eRCArm] = DJI_RC_ARM;

    m_Signs[eThrottleSign] = DJI_RC_THROTTLE_SIGN;
    m_Signs[eYawSign] = DJI_RC_YAW_SIGN;
    m_Signs[eRollSign] = DJI_RC_ROLL_SIGN;
    m_Signs[ePitchSign] = DJI_RC_PITCH_SIGN;
}

rc_mapper::RC_TYPES rc_mapper::DetermineRCSetting(std::string* _rctype)
{
    rc_mapper::RC_TYPES retvalue = eRCT_DJI;

    for(int iI = 0; iI < eEndOfRCTypes; ++iI)
    {
        if(0 == _rctype->compare(m_RCTypeNames[iI]))
        {
            retvalue = static_cast<RC_TYPES>(iI);
            break;
        }
    }
    return retvalue;
}

void rc_mapper::SetFromParameters(void)
{
    m_Handle.param<std::string>("/RC/rc_type", m_RCType, m_RCTypeNames[eRCT_DJI].c_str());
    m_SetRC = DetermineRCSetting(&m_RCType);

    ROS_INFO("RC: SetRC string[%s] and enum[%d]", m_RCType.c_str(), m_SetRC);

    switch(m_SetRC)
    {
    case eRCT_DJI:
    default:
    {
    m_Handle.param("/RC/throttle", m_Channels[eRCThrottle], DJI_RC_THROTTLE);
    m_Handle.param("/RC/yaw", m_Channels[eRCYaw], DJI_RC_YAW);
    m_Handle.param("/RC/roll", m_Channels[eRCRoll], DJI_RC_ROLL);
    m_Handle.param("/RC/pitch", m_Channels[eRCPitch], DJI_RC_PITCH);
    m_Handle.param("/RC/gear", m_Channels[eRCGear], DJI_RC_GEAR);
    m_Handle.param("/RC/mode", m_Channels[eRCMode], DJI_RC_MODE);
    m_Handle.param("/RC/arm", m_Channels[eRCArm], DJI_RC_ARM);
    m_Handle.param("/RC/throttle_sign", m_Signs[eThrottleSign], DJI_RC_THROTTLE_SIGN);
    m_Handle.param("/RC/yaw_sign", m_Signs[eYawSign], DJI_RC_YAW_SIGN);
    m_Handle.param("/RC/roll_sign", m_Signs[eRollSign], DJI_RC_ROLL_SIGN);
    m_Handle.param("/RC/pitch_sign", m_Signs[ePitchSign], DJI_RC_PITCH_SIGN);
    m_Handle.param("/RC/minvalue", m_MinValue, DJI_RC_MIN_VALUE);
    m_Handle.param("/RC/maxvalue", m_MaxValue, DJI_RC_MAX_VALUE);

    if(m_SetRC != eRCT_DJI)
    {
        m_SetRC = eRCT_DJI;
        ROS_ERROR("RC: Illegal RC type parameter[%s], defaulting to DJI", m_RCType.c_str());
        m_RCType = m_RCTypeNames[m_SetRC];
    }
        break;
    }

    case eRCT_PX4:
    {
        m_Handle.param("/RC/throttle", m_Channels[eRCThrottle], PX4_RC_THROTTLE);
        m_Handle.param("/RC/yaw", m_Channels[eRCYaw], PX4_RC_YAW);
        m_Handle.param("/RC/roll", m_Channels[eRCRoll], PX4_RC_ROLL);
        m_Handle.param("/RC/pitch", m_Channels[eRCPitch], PX4_RC_PITCH);
        m_Handle.param("/RC/gear", m_Channels[eRCGear], PX4_RC_GEAR);
        m_Handle.param("/RC/mode", m_Channels[eRCMode], PX4_RC_MODE);
        m_Handle.param("/RC/arm", m_Channels[eRCArm], PX4_RC_ARM);
        m_Handle.param("/RC/throttle_sign", m_Signs[eThrottleSign], PX4_RC_THROTTLE_SIGN);
        m_Handle.param("/RC/yaw_sign", m_Signs[eYawSign], PX4_RC_YAW_SIGN);
        m_Handle.param("/RC/roll_sign", m_Signs[eRollSign], PX4_RC_ROLL_SIGN);
        m_Handle.param("/RC/pitch_sign", m_Signs[ePitchSign], PX4_RC_PITCH_SIGN);
        m_Handle.param("/RC/minvalue", m_MinValue, PX4_RC_MIN_VALUE);
        m_Handle.param("/RC/maxvalue", m_MaxValue, PX4_RC_MAX_VALUE);
        break;
    }
    case eRCT_ACSL:
    {
        m_Handle.param("/RC/throttle", m_Channels[eRCThrottle], ACSL_RC_THROTTLE);
        m_Handle.param("/RC/yaw", m_Channels[eRCYaw], ACSL_RC_YAW);
        m_Handle.param("/RC/roll", m_Channels[eRCRoll], ACSL_RC_ROLL);
        m_Handle.param("/RC/pitch", m_Channels[eRCPitch], ACSL_RC_PITCH);
        m_Handle.param("/RC/gear", m_Channels[eRCGear], ACSL_RC_GEAR);
        m_Handle.param("/RC/mode", m_Channels[eRCMode], ACSL_RC_MODE);
        m_Handle.param("/RC/arm", m_Channels[eRCArm], ACSL_RC_ARM);
        m_Handle.param("/RC/throttle_sign", m_Signs[eThrottleSign], ACSL_RC_THROTTLE_SIGN);
        m_Handle.param("/RC/yaw_sign", m_Signs[eYawSign], ACSL_RC_YAW_SIGN);
        m_Handle.param("/RC/roll_sign", m_Signs[eRollSign], ACSL_RC_ROLL_SIGN);
        m_Handle.param("/RC/pitch_sign", m_Signs[ePitchSign], ACSL_RC_PITCH_SIGN);
        m_Handle.param("/RC/minvalue", m_MinValue, ACSL_RC_MIN_VALUE);
        m_Handle.param("/RC/maxvalue", m_MaxValue, ACSL_RC_MAX_VALUE);
        break;
    }
    }
    ROS_INFO("RC: Parameter driven values: throttle[%d], yaw[%d], roll[%d], pitch[%d], gear[%d], mode[%d], arm[%d], min[%d], max[%d]",
        m_Channels[eRCThrottle], m_Channels[eRCYaw], m_Channels[eRCRoll], m_Channels[eRCPitch],
        m_Channels[eRCGear], m_Channels[eRCMode], m_Channels[eRCArm], m_MinValue, m_MaxValue);
}
