/*
 * frame_states.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: ubuntu
 */

#include <vb_util_lib/circ_list.h>

//CircularList::CircularList()  {

//    clearObjList();
//}
void CircularList::addObject(const std_msgs::msg::Header &cs)
{
	obj_list_[obj_list_end_] = cs;
	obj_list_cnt_ =obj_list_cnt_ < num_objects_ ? obj_list_cnt_ + 1 : num_objects_;
	obj_list_end_ =obj_list_end_ < (num_objects_- 1) ? obj_list_end_+1 : 0;
}

CircularList::CircularList(int no)
{
	num_objects_ = no;
	obj_list_ = new std_msgs::msg::Header[num_objects_];
	clearObjList();
}

CircularList::~CircularList() {
    // TODO Auto-generated destructor stub
    delete[] obj_list_;
}


void CircularList::getObjectTimeIdx(rclcpp::Time time, unsigned int& start_idx, unsigned int& number) {
    number = 0;
    int idx = (int)obj_list_cnt_ - 1;

    for (;idx >= 0; idx--, number++)
    {
    	std_msgs::msg::Header s = getObjectAt(idx);
        if (time > s.stamp)
        {
            start_idx = idx;
            number++;
            return;
        }
    }
}

unsigned int CircularList::getObjectListIndex(unsigned int idx) {
    // range check the index
    if (idx >=obj_list_cnt_) {
        perror("object array index out of range\n");
        idx =obj_list_cnt_ - 1;
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

void CircularList::printObjList(std::string prefix) {
    printf("\n%s OBJECT LIST cnt[%u] end[%u] ACTUAL ARRAY:\n", prefix.c_str(),obj_list_cnt_,obj_list_end_);

    for (int i = 0; i <obj_list_cnt_; i++ )
    {
    	std_msgs::msg::Header& s =obj_list_[i];
        printf("%s       STATE[%d] frame[%s] time[%u:%u]\n", prefix.c_str(), i, s.frame_id.c_str(), s.stamp.sec, s.stamp.nanosec);
    }

    printf("%s OBJECT LIST cnt[%u] end[%u] VIRTUAL ARRAY:\n", prefix.c_str(),obj_list_cnt_,obj_list_end_);
    for (int i = 0; i <obj_list_cnt_; i++ )
    {
    	std_msgs::msg::Header& s = getObjectAt(i);
        printf("%s       STATE[%d] frame[%s] time[%u:%u]\n", prefix.c_str(), i, s.frame_id.c_str(), s.stamp.sec, s.stamp.nanosec);
    }
}

void CircularList::testObjList() {
    int i = 0;
    rclcpp::Time base = rclcpp::Clock().now();
    rclcpp::Time stamp = base;
    rclcpp::Duration d(1.0);
    for (;i < 5; i++)
    {
    	std_msgs::msg::Header s;
        s.frame_id = "id"+std::to_string(i);
        s.stamp = stamp;
        addObject(s);
        stamp = stamp+d;
    }
    printObjList("     ");

    for (;i < 12; i++)
    {
    	std_msgs::msg::Header s;
        s.frame_id = "id"+std::to_string(i);
        s.stamp = stamp;
        addObject(s);
        stamp = stamp+d;
    }
    printObjList("     ");

    unsigned start_idx = 0;
    unsigned cnt = 0;

    stamp = base + rclcpp::Duration(5.5);
    getObjectTimeIdx(stamp, start_idx, cnt);
    unsigned k = start_idx;
    printf(">>> SEARCHING FOR ALL OBJECTS WITHIN STAMP[%f:%ld] start_idx[%u]  cnt[%u]\n", stamp.seconds(), stamp.nanoseconds(), start_idx, cnt);
    for (int j = 0; j < cnt; j++, k++)
    {
    	std_msgs::msg::Header& s = getObjectAt(k);
        printf("       num[%d] idx[%u] feature[%s] time[%u:%u]\n", j, k, s.frame_id.c_str(), s.stamp.sec, s.stamp.nanosec);
    }


}
