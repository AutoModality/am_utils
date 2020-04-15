#ifndef VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_H_
#define VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_H_

#include <map>
#include <string>
#include <iterator>
#include <ros/ros.h>
#include <std_msgs/Header.h>

class CircularList
{
public:
	//CircularList();
	CircularList(int no = 50);
    virtual ~CircularList();
	
	template<class T>
	void addObject(const T &cs)
	{
		obj_list_[obj_list_end_] = cs.header;
		obj_list_cnt_ =obj_list_cnt_ < num_objects_ ? obj_list_cnt_ + 1 : num_objects_;
		obj_list_end_ =obj_list_end_ < (num_objects_- 1) ? obj_list_end_+1 : 0;
	}
	void addObject(const std_msgs::Header &cs);
    std_msgs::Header& getObjectAt(unsigned int idx) {
        return obj_list_[getObjectListIndex(idx)];
    }

    void getObjectTimeIdx(ros::Time time, unsigned int& start_idx, unsigned int& number);

    void clearObjList() {
        obj_list_end_ = 0;
        obj_list_cnt_ = 0;
    }

    void printObjList(std::string prefix);

    void testObjList();

private:

    // List of body_FLU odometry locations implemented as a circular array
	//#define NUM_OBJECTS 8
	int num_objects_;
    std_msgs::Header *obj_list_;
    unsigned int obj_list_end_ {0};
    unsigned int obj_list_cnt_ {0};
    unsigned int getObjectListIndex(unsigned int idx);

};

#endif /* VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_H_ */
