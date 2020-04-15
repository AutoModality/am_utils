/*
 * task_trigger.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: ubuntu
 */

#include <vb_util_lib/task_trigger.h>
#include <vb_util_lib/gps_util.h>

TaskTrigger::TaskTrigger() {
    conditions_.location_.error = -1.0;
}

TaskTrigger::~TaskTrigger() {
}

bool TaskTrigger::loadJSON(Json::Value& fp) {
//    fc->uuid = fp.get("task_time", "").asString();
    conditions_.task_time_ = fp.get("task_time", -1.0).asDouble();
    conditions_.mission_time_ = fp.get("mission_time", -1.0).asDouble();
    conditions_.at_milestone_id_ = fp.get("at_milestone", "XXX").asString();
    conditions_.past_milestone_id_ = fp.get("past_milestone", "XXX").asString();
    conditions_.total_flight_time_ = fp.get("total_flight_time", -1.0).asDouble();
    conditions_.total_flight_distance_ = fp.get("total_flight_distance", -1.0).asDouble();
    conditions_.task_flight_distance_ = fp.get("task_flight_distance_", -1.0).asDouble();
    conditions_.flight_plan_complete_ = fp.get("flight_plan_complete", false).asBool();
    conditions_.FCU_connected_ = fp.get("FCU_connected", false).asBool();
    Json::Value& fpl = fp["Location"];
    if (fpl.isObject())
    {
        if (!GISEntity::loadGISCoordinate(fpl, &conditions_.location_))
        {
            return false;
        }
    }

    if (conditions_.location_.error < 0.0 &&
            conditions_.task_time_ < 0.0 &&
            conditions_.mission_time_ < 0.0 &&
            conditions_.at_milestone_id_ == "" &&
            conditions_.past_milestone_id_ == "" &&
            conditions_.total_flight_time_ < 0.0 &&
            conditions_.total_flight_distance_ < 0.0 &&
            conditions_.task_flight_distance_ < 0.0 &&
            !conditions_.flight_plan_complete_ &&
            !conditions_.FCU_connected_)
    {
        // no condition for trigger
        ROS_WARN("    No condition for trigger, always true");
    }

//    printTriggers("", trigger_);

    return true;
}

void TaskTrigger::printTriggers(std::string pre, TriggerData& t) {
    printf("%s  TRIGGERS mt:%0.3f  tt:%0.3f tft:%0.3f  tfd:%0.3f  tskd:%0.3f\n", pre.c_str(),
            t.mission_time_, t.task_time_, t.total_flight_time_,
            t.total_flight_distance_, t.task_flight_distance_);
    printf("%s    MILESTONE AT[%s:%d] PAST[%s:%d]\n", pre.c_str(),
            t.at_milestone_id_.c_str(), t.at_milestone_index_,
            t.past_milestone_id_.c_str(), t.past_milestone_index_);
    printf("%s    FLIGHT PLAN DONE [%s]   FCU CONNECTED [%s]\n", pre.c_str(),
            t.flight_plan_complete_ ? "TRUE" : "FALSE",
            t.FCU_connected_ ? "TRUE" : "FALSE");

    std::string loc_st = "LOCATION: NONE";
    if (t.location_.error >= 0.0)
    {
        loc_st = "LOCATION: " +
                std::to_string(t.location_.coord.gps.latitude)+"," +
                std::to_string(t.location_.coord.gps.longitude)+","+
                std::to_string(t.location_.coord.gps.altitude);
    }
    printf("%s    %s\n",  pre.c_str(), loc_st.c_str());

    return;
}

bool TaskTrigger::isTriggered(TriggerData& t) {
    bool milestone_at = conditions_.at_milestone_index_ < 0 || conditions_.at_milestone_index_ == t.at_milestone_index_;
    bool milestone_past = conditions_.past_milestone_index_ < 0 || conditions_.past_milestone_index_ < t.past_milestone_index_;
    bool mission_t = conditions_.mission_time_ <= t.mission_time_;
    bool task_t = conditions_.task_time_ <= t.task_time_;
    bool fcu_connected = !conditions_.FCU_connected_ || t.FCU_connected_;
    bool flight_plan_done = !conditions_.flight_plan_complete_ || t.flight_plan_complete_;
//    printTriggers("C  ", getConditions());
//    printTriggers("S  ", t);
//    ROS_INFO_THROTTLE(1, "IS TRIGGERED: at:%d, past:%d, miss_t:%d, task_t:%d, FCU:%d, FPD:%d",
//            milestone_at, milestone_past, mission_t, task_t, fcu_connected, flight_plan_done);
//    bool lf = latLonDeg_to_Dist(gis_loc.coord.gps.latitude,
//            gis_loc.coord.gps.longitude,
//            location_.coord.gps.latitude,
//            location_.coord.gps.longitude) < location_.error;
    return (milestone_at && milestone_past &&
            mission_t && task_t &&
            fcu_connected && flight_plan_done);
}
