/*
 * AMMsgHead.h
 *
 *  Created on: May 3, 2015
 *      Author: dan
 */

#ifndef BRAIN_BOX_SRC_AM_MSG_HEAD_H_
#define BRAIN_BOX_SRC_AM_MSG_HEAD_H_

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

class AMMsgHead {
public:
	AMMsgHead();
	virtual ~AMMsgHead();

	inline std::string *getVeh() {
		return &veh_;
	}

	inline void setVeh(std::string veh) {
		veh_ = veh;
	}

	inline int getSeq() {
		return seq_;
	}

	inline void setSeq(int seq) {
		seq_ = seq;
	}

	inline boost::posix_time::ptime *getTime() {
		return &time_;
	}

	inline void setTime(boost::posix_time::ptime time) {
		time_ = time;
	}

	inline std::string *getName() {
		return &name_;
	}

	inline void setName(std::string name) {
		name_ = name;
	}

	inline std::string *getFrom() {
		return &from_;
	}

	inline void setFrom(std::string from) {
		from_ = from;
	}

	inline std::string *getType() {
		return &type_;
	}

	inline void setType(std::string type) {
		type_ = type;
	}

private:
	std::string veh_;
	int seq_ = 0;
	boost::posix_time::ptime time_;
	std::string name_;
	std::string from_;
	std::string type_;
};

#endif /* BRAIN_BOX_SRC_AM_MSG_HEAD_H_ */
