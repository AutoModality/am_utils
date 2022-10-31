/*
 * mission_spec.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: ubuntu
 */

#include <flight_lib/mission_spec.h>
#include <vb_util_lib/am_util.h>

MissionSpec::MissionSpec(ros::NodeHandle& nh) : nh_(nh) {
    tasks_.clear();

}

MissionSpec::~MissionSpec() {
    // TODO Auto-generated destructor stub
}

bool MissionSpec::reset() {
    for (auto it = tasks_.begin(); it != tasks_.end(); it++)
    {
        it->reset();
    }
    return true;
}


bool MissionSpec::loadJSON(std::string root_dir, Json::Value& fp) {
    root_dir_ = root_dir;
    name_ = fp.get("name", "").asString();
    if (name_ == "")
    {
        ROS_ERROR("MS: Mission spec has no name");
        return false;
    }

    version_ = fp.get("version", "0.0").asString();
    if (version_ == "")
    {
        ROS_WARN("MS: Mission spec has no version string");
    }

    script_ = fp.get("script", "").asString();
    if (script_ == "")
    {
        ROS_ERROR("MS: Mission spec has no script file");
        return false;
    }
    script_file_ = root_dir + script_;

    flight_plan_name_ = fp.get("flight_plans", "").asString();
    if (flight_plan_name_ == "")
    {
        ROS_WARN("MS: Mission spec has no flight plan file");
    }
    flight_plans_file_ = root_dir + flight_plan_name_;

    ROS_INFO("MS: Loading mission spec name[%s], version[%s], script[%s], flight_plans[%s]",
            name_.c_str(), version_.c_str(), script_file_.c_str(), flight_plans_file_.c_str());
    if (!fileExists(script_file_))
    {
        ROS_ERROR("MS: MISSION SCRIPT FILE %s DOES NOT EXIST!", script_file_.c_str());
        return false;
    }

    // Parse out all the tasks
    for (Json::Value::iterator it = fp["Tasks"].begin(); it != fp["Tasks"].end(); ++it)
    {
        MissionTask fc;
        if (!fc.loadJSON(*it))
        {
            return false;
        }
        tasks_.push_back(fc);
    }

    // Load flight plan and initialize all triggers
    if (!loadFlightPlanFile(flight_plans_file_))
    {
        return false;
    }

    return true;

}

bool MissionSpec::assignMilestoneIndex(TriggerData& td) {
//    ROS_INFO("IDX SEARCHING FOR AT[%s]  PAST[%s]",
//            td.at_milestone_id_.c_str(), td.past_milestone_id_.c_str());
    if (td.at_milestone_id_ != "XXX")
    {
//        ROS_INFO("IDX_AT[%s]", td.at_milestone_id_.c_str());
        td.at_milestone_index_ =
                VxTrajectory::findMilestoneIdx(td.at_milestone_id_, trajectories_);
        if (td.at_milestone_index_ < 0)
        {
            ROS_ERROR("ERROR LOCATING AT MILESTONE ID %s\n", td.at_milestone_id_.c_str());
            return false;
        }
//        ROS_INFO("FOUND AT INDEX %d", td.at_milestone_index_);
    }
    if (td.past_milestone_id_ != "XXX")
    {
//        ROS_INFO("IDX_PAST[%s]", td.past_milestone_id_.c_str());
        td.past_milestone_index_ =
                VxTrajectory::findMilestoneIdx(td.past_milestone_id_, trajectories_);
        if (td.past_milestone_index_ < 0)
        {
            ROS_ERROR("ERROR LOCATING PAST MILESTONE ID %s\n", td.past_milestone_id_.c_str());
            return false;
        }
//        ROS_INFO("FOUND PAST INDEX %d", td.past_milestone_index_);
    }
    return true;
}

bool MissionSpec::loadFlightPlanFile(std::string fn) {
    ROS_INFO("MS: LOADING FLIGHT PLAN %s", fn.c_str());
    if (!fileExists(fn))
    {
        ROS_ERROR("MS: FLIGHT PLAN FILE %s DOES NOT EXIST!", fn.c_str());
        return false;
    }

    if (!FlightPlan::loadFlightPlanFile(nh_, fn, flight_plans_, trajectories_))
    {
        return false;
    }

    for (int i = 0; i < tasks_.size(); i++)
    {
        for (int j = 0; j < tasks_.at(i).getOperations().size(); j++)
        {
            if (!assignMilestoneIndex(
                    tasks_.at(i).getOperations().at(j)->getTrigger().getConditions()))
            {
                return false;
            }
        }
    }

    return true;
}

void MissionSpec::print(const std::string &pre) {
    printf("%s  MISSION_SPEC[%s], version[%s]\n", pre.c_str(), name_.c_str(), version_.c_str());
    printf("%s      script file[%s]\n", pre.c_str(), script_file_.c_str());
    printf("%s      flight plan file[%s]\n", pre.c_str(), flight_plans_file_.c_str());

    printf("%s    TASKS:\n", pre.c_str());
    for (auto it = tasks_.begin(); it != tasks_.end(); it++)
    {
        it->print(pre+"      ");
    }

    printf("%s    FLIGHT PLANS:\n", pre.c_str());
    for (auto it = flight_plans_.begin(); it != flight_plans_.end(); it++)
    {
        it->print(pre+"      ");
    }

    printf("%s    TRAJECTORIES:\n", pre.c_str());
    for (auto it = trajectories_.begin(); it != trajectories_.end(); it++)
    {
        it->print(pre+"      ");
    }

    return;
}





