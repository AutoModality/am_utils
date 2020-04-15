#ifndef VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_TEMPLATE_H_
#define VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_TEMPLATE_H_

#include <map>
#include <string>
#include <iterator>
#include <ros/ros.h>

template<typename T>
class CircularList : public T
{
public:
	/*
	 *	This is Constructor with number of objects preset to 50 
	*/
	CircularList(int no = 50)
	{
		num_objects_ = no;
		obj_list_ = new T[num_objects_];
		clearObjList();
	}
	/*
	 *	This is destructor releasing all the resources array
	*/
    virtual ~CircularList()
    {
    	delete[] obj_list_;
    }
	/*
	 *	This is a function to add objects
	*/
	void addObject(const T &cs)
	{
		obj_list_[obj_list_end_] = cs;
		obj_list_cnt_ =obj_list_cnt_ < num_objects_ ? obj_list_cnt_ + 1 : num_objects_;
		obj_list_end_ =obj_list_end_ < (num_objects_- 1) ? obj_list_end_+1 : 0;
	}
	/*
	 *	This is a function to get objects at particular index
	*/
    T& getObjectAt(unsigned int idx) 
    {
	if(idx >= 0)
	{
		return obj_list_[getObjectListIndex(idx)];
	}
    }
    int getObjectAt(unsigned int idx, T& obj) 
    {
	if(obj_list_cnt_ == 0)
	{
		return -1;
	}
	if(idx >= 0)
	{
		obj = obj_list_[getObjectListIndex(idx)];
		return 1;
	}
	return -1;
    }
    /*
	 *	This is a function to search for objects which are close to the passed index
	*/
    void getObjectTimeIdx(ros::Time time, unsigned int& start_idx, unsigned int& number)
    {
    	number = 0;
		int idx = (int)obj_list_cnt_ - 1;

		for (;idx >= 0; idx--, number++)
		{
			T s = getObjectAt(idx);
		    if (time > s.header.stamp)
		    {
		        start_idx = idx;
		        number++;
		        return;
		    }
  	  	}
    }
    /*
	 *	A function to clear the object list
	*/
    void clearObjList() 
    {
        obj_list_end_ = 0;
        obj_list_cnt_ = 0;
    }
	/*
	 *	This is a function for test routines
	*/
    void printObjList(std::string prefix)
    {
    	printf("\n%s OBJECT LIST cnt[%u] end[%u] ACTUAL ARRAY:\n", prefix.c_str(),obj_list_cnt_,obj_list_end_);

		for (int i = 0; i <obj_list_cnt_; i++ )
		{
			T& s =obj_list_[i];
		    printf("%s       STATE[%d] frame[%s] time[%u:%u]\n", prefix.c_str(), i, s.frame_id.c_str(), s.stamp.sec, s.stamp.nsec);
		}

		printf("%s OBJECT LIST cnt[%u] end[%u] VIRTUAL ARRAY:\n", prefix.c_str(),obj_list_cnt_,obj_list_end_);
		for (int i = 0; i <obj_list_cnt_; i++ )
		{
			T& s = getObjectAt(i);
		    printf("%s       STATE[%d] frame[%s] time[%u:%u]\n", prefix.c_str(), i, s.frame_id.c_str(), s.stamp.sec, s.stamp.nsec);
		}
    }
	/*
	 *	 This is for testing
	*/
    void testObjList()
    {
    	int i = 0;
		ros::Time base = ros::Time::now();
		ros::Time stamp = base;
		ros::Duration d(1.0);
		for (;i < 5; i++)
		{
			std_msgs::Header s;
		    s.frame_id = "id"+std::to_string(i);
		    s.stamp = stamp;
		    addObject(s);
		    stamp = stamp+d;
		}
		printObjList("     ");

		for (;i < 12; i++)
		{
			std_msgs::Header s;
		    s.frame_id = "id"+std::to_string(i);
		    s.stamp = stamp;
		    addObject(s);
		    stamp = stamp+d;
		}
		printObjList("     ");

		unsigned start_idx = 0;
		unsigned cnt = 0;

		stamp = base + ros::Duration(5.5);
		getObjectTimeIdx(stamp, start_idx, cnt);
		unsigned k = start_idx;
		printf(">>> SEARCHING FOR ALL OBJECTS WITHIN STAMP[%u:%u] start_idx[%u]  cnt[%u]\n", stamp.sec, stamp.nsec, start_idx, cnt);
		for (int j = 0; j < cnt; j++, k++)
		{
			std_msgs::Header& s = getObjectAt(k);
		    printf("       num[%d] idx[%u] feature[%s] time[%u:%u]\n", j, k, s.frame_id.c_str(), s.stamp.sec, s.stamp.nsec);
		}
    }

private:

    // List of body_FLU odometry locations implemented as a circular array
	//#define NUM_OBJECTS 8
	int num_objects_;
    T *obj_list_;
    unsigned int obj_list_end_ {0};
    unsigned int obj_list_cnt_ {0};
    /*
	 *	This is a private method to get object list index
	*/
    unsigned int getObjectListIndex(unsigned int idx){
    	// range check the index
		if (idx >=obj_list_cnt_) {
		    idx = obj_list_cnt_ == 0 ? 0 : obj_list_cnt_ - 1;
		    ROS_ERROR("circ_list_template: object array index out of range = %u", idx);
		}

		if (obj_list_cnt_ >= num_objects_)
		{
		    if ((idx +=obj_list_end_) >= num_objects_)
		    {
		        idx -= num_objects_;
		    }
		}

		return idx;
	}
};

#endif /* VISBOX_PACKAGES_VB_UTILS_INCLUDE_CIRCULAR_LIST_TEMPLATE_H_ */
