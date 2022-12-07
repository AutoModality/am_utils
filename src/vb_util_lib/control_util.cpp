/*
 * ControlUtil.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: ubuntu
 */


#include <vb_util_lib/control_util.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const char* axisLabel[] = {"X", "Y", "Z"};
const char* paramLabel[] = {"P", "I", "D"};

///*
//** Pi wrapping utility
//*/
//double wrap_pi(double bearing)
//{
//  while (bearing >= _PI_R)
//    {
//      bearing -= _TWOPI_R;
//    }
//
//  while (bearing <= -_PI_R)
//    {
//      bearing += _TWOPI_R;
//    }
//
//  return bearing;
//}
//
//double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
//	double xd = p2.x - p1.x;
//	double yd = p2.y - p1.y;
//	double zd = p2.z - p1.z;
//	return sqrt((xd*xd) + (yd*yd) + (zd*zd));
//}

VehicleAttitude quaternionMsg2Att(const geometry_msgs::msg::Quaternion q) {
    tf2::Quaternion quat;
    //tf2::quaternionMsgToTF(q, quat);
    tf2::fromMsg(q, quat);
    VehicleAttitude va {0.0,0.0,0.0,0.0};
    tf2::Matrix3x3(quat).getRPY(va.roll, va.pitch, va.yaw);
    return va;
}

void testQuats() {
    double pi = 3.14159;
    tf2::Quaternion qp, qn;
    tf2::Quaternion quat1;
    tf2::Quaternion quat2;
    tf2::Quaternion quat2_inv;
    tf2::Quaternion quat_90;
    tf2::Quaternion quat_90_inv;
    tf2::Quaternion quat_90m;
    tf2::Quaternion quat_90m_inv;
    tf2::Quaternion quat;
    tf2::Quaternion quat_inv;
    double roll, pitch, yaw;
    tf2::Vector3 axis;

    printf("XXXXXXXXXXXXXXXXXXXXX\n");
    qp.setValue(1.0, 2.0, 3.0, 0.0);
    printf("XXXX point to rotate quat[%0.4f, %0.4f, %0.4f %0.4f]\n",
            qp.getW(), qp.getX(), qp.getY(), qp.getZ());

    axis = tf2::Vector3(0.0, 0.0, 1.0);
    quat1.setRotation(axis, pi);
    printf("XXXX yaw 180 degrees quat[%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat1.getW(), quat1.getX(), quat1.getY(), quat1.getZ());

    quat.setRPY(0.0, 0.0, 0.0);
    quat_inv = quat.inverse();
    printf("XXXX no rotation quat[%0.4f, %0.4f, %0.4f %0.4f] quat_inv[%0.4f, %0.4f, %0.4f %0.4f] \n",
            quat.getW(), quat.getX(), quat.getY(), quat.getZ(),
            quat_inv.getW(), quat_inv.getX(), quat_inv.getY(), quat_inv.getZ());

    qn = qp * quat_inv;
    qn = quat * qn;
    printf("XXXX point not rotated [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());

    quat_90.setRPY(0.0, pi / 2.0, 0.0);
    quat_90_inv = quat_90.inverse();
    printf("XXXX pitch 90 quat[%0.4f, %0.4f, %0.4f %0.4f] quat_inv[%0.4f, %0.4f, %0.4f %0.4f] \n",
            quat_90.getW(), quat_90.getX(), quat_90.getY(), quat_90.getZ(),
            quat_90_inv.getW(), quat_90_inv.getX(), quat_90_inv.getY(), quat_90_inv.getZ());

    qn = qp * quat_90_inv;
    qn = quat_90 * qn;
    printf("XXXX point pitched 90 degrees [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());

//    tf::Quaternion quat3 = quat * quat1;
//    printf("XXXX yaw 180 degrees then pitch -90 quat[%0.4f, %0.4f, %0.4f %0.4f]\n",
//            quat3.getW(), quat3.getX(), quat3.getY(), quat3.getZ());
//
//    quat3 = quat1 * quat;
//    printf("XXXX pitch -90 then yaw 180 quat[%0.4f, %0.4f, %0.4f %0.4f]\n",
//            quat3.getW(), quat3.getX(), quat3.getY(), quat3.getZ());

    quat.setRPY(pi, 0.0, 0.0);
    quat_inv = quat.inverse();
    printf("XXXX roll 180 quat[%0.4f, %0.4f, %0.4f %0.4f] quat_inv[%0.4f, %0.4f, %0.4f %0.4f] \n",
            quat.getW(), quat.getX(), quat.getY(), quat.getZ(),
            quat_inv.getW(), quat_inv.getX(), quat_inv.getY(), quat_inv.getZ());

    qn = qp * quat_inv;
    qn = quat * qn;
    printf("XXXX point rolled 180 degrees (NED) [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());

    qn = qn * quat_90_inv;
    qn = quat_90 * qn;
    printf("XXXX point rolled 180 degrees and then pitched 90 (two ops) [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());

    quat2 = quat_90 * quat;
    quat2_inv = quat2.inverse();
    printf("XXXX combined quat (roll 180, pitch +90) for downward facing sensor in NED [%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat2.getW(), quat2.getX(), quat2.getY(), quat2.getZ());
    qn = qp * quat2_inv;
    qn = quat2 * qn;
    printf("XXXX point rolled 180 degrees and then pitched 90 (one op) [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());
    double ang = (29.0 / 180.0) * pi;
//    quat_90m.setRPY(0.0, -pi / 2.0, 0.0);
    quat_90m.setRPY(0.0, -ang, 0.0);
    quat_90m_inv = quat_90m.inverse();
    printf("XXXX pitch 90 quat[%0.4f, %0.4f, %0.4f %0.4f] quat_inv[%0.4f, %0.4f, %0.4f %0.4f] \n",
            quat_90m.getW(), quat_90m.getX(), quat_90m.getY(), quat_90m.getZ(),
            quat_90m_inv.getW(), quat_90m_inv.getX(), quat_90m_inv.getY(), quat_90m_inv.getZ());

    quat2 = quat_90m * quat;
    quat2_inv = quat2.inverse();
    printf("XXXX combined quat (roll 180, pitch -90) for downward facing sensor in NED [%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat2.getW(), quat2.getX(), quat2.getY(), quat2.getZ());
    qn = qp * quat2_inv;
    qn = quat2 * qn;
    printf("XXXX point rolled 180 degrees and then pitched -90 (one op) [%0.4f, %0.4f, %0.4f %0.4f]\n",
            qn.getW(), qn.getX(), qn.getY(), qn.getZ());

    // To rotate a quaternion q1 by another quaternion q2 -> q2 * q1
    // Note that q2 * q1 DOES NOT EQUAL q1 * q2

    // Test of rotating quaternions
    // First define a quaternion with -90 pitch
    tf2::Quaternion quat_vec;
    quat_vec.setRPY(0.0, -pi / 2.0, 0.0);
    printf("XXXX -90 pitch to be rotated[%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat_vec.getW(), quat_vec.getX(), quat_vec.getY(), quat_vec.getZ());
    // Define a rotation of +90 pitch
    quat.setRPY(0.0, pi / 2.0, 0.0);
    printf("XXXX +90 pitch amount to rotate[%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat.getW(), quat.getX(), quat.getY(), quat.getZ());
    // Now rotate the quaternion vector by the 90 degree pitch quaternion
    // They should cancel out
    tf2::Quaternion quat_rot = quat * quat_vec;
//    quat_rot *= quat_vec;
    printf("XXXX -90 pitch rotated by +90 pitch [%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat_rot.getW(), quat_rot.getX(), quat_rot.getY(), quat_rot.getZ());

    // Now try rotating by a roll of 90
    quat.setRPY(pi / 2.0, 0.0, 0.0);
    printf("XXXX +90 roll amount to rotate[%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat.getW(), quat.getX(), quat.getY(), quat.getZ());
    // Now rotate the quaternion vector by the 90 degree pitch quaternion
    // They should cancel out
    quat_rot = quat_vec * quat;
//    quat_rot *= quat_vec;
    printf("XXXX -90 pitch rotated by +90 roll [%0.4f, %0.4f, %0.4f %0.4f]\n",
            quat_rot.getW(), quat_rot.getX(), quat_rot.getY(), quat_rot.getZ());

    return;
}


