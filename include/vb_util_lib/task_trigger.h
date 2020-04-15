/*
 * task_trigger.h
 *
 *  Created on: Feb 25, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TASK_TRIGGER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TASK_TRIGGER_H_

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <vb_util_lib/json.h>
#include <vb_util_lib/gis_util.h>
#include <vb_util_lib/gis_entity.h>

struct TriggerData {
    /**
     * At milestone id to currently executing flight plan.
     * If "" then not used
     */
    std::string at_milestone_id_ {""};
    int at_milestone_index_ {-1};

    /**
     * Past milestone id to currently executing flight plan.
     * If "" then not used
     */
    std::string past_milestone_id_ {""};
    int past_milestone_index_ {-1};

    /**
     * Time to trigger in sec from start of mission.
     * If < 0 then not used.
     */
    double mission_time_ {-1.0};

    /**
     * Time to trigger in sec from start of task.
     * If < 0 then not used.
     */
    double task_time_ {-1.0};

    /**
     * Time to trigger in sec based on flight time.
     * If < 0 then not used.
     */
    double total_flight_time_ {-1.0};

    /**
     * Total distance traveled since start of flight plan.
     * If < 0 then not used.
     */
    double total_flight_distance_ {-1.0};

    /**
     * Distance traveled since start of task
     * If < 0 then not used.
     */
    double task_flight_distance_ {-1.0};

    /**
     * Wait for flight plan to complete
     */
    bool flight_plan_complete_ {false};

    /**
     * Wait for FCU to connect
     */
    bool FCU_connected_ {false};

    /**
     * Location trigger.
     * If error < 0 then not used.
     */
    GISCoordinate location_;
};

class TaskTrigger {
public:
    TaskTrigger();
    virtual ~TaskTrigger();

    /**
     * Load JSON representation
     */
    bool loadJSON(Json::Value& fp);

    bool isTriggered(TriggerData& trigger);

    TriggerData& getConditions() {
        return conditions_;
    }

    static void printTriggers(std::string pre, TriggerData& t);
    void print(std::string pre) {
        printTriggers(pre, conditions_);
    }

    static void initTriggerData(TriggerData& td) {
        td.at_milestone_id_ = "";
        td.at_milestone_index_ = -1;
        td.past_milestone_id_ = "";
        td.past_milestone_index_ = -1;
        td.mission_time_ = -1.0;
        td.task_time_ = -1.0;
        td.total_flight_time_ = -1.0;
        td.total_flight_distance_ = -1.0;
        td.task_flight_distance_ = -1.0;
        td.flight_plan_complete_ = false;
        td.FCU_connected_ = false;
    }

private:
    TriggerData conditions_;

};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TASK_TRIGGER_H_ */
