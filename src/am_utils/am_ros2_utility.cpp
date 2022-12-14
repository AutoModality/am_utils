/*
 * am_ros2_utility.cpp
 *
 *  Created on: Dec 7, 2022
 *      Author: alireza
 */

#include <am_utils/am_ros2_utility.h>
#include <iostream>


namespace am
{
std::vector<std::string> toVector(const std::string &raw_str, char del)
{
	std::string line;
	std::vector<std::string> vec;
	std::stringstream ss(raw_str);
	while(std::getline(ss, line, del)) {
		vec.push_back(line);
	}
	return vec;
}

std::string vectToStr(const std::vector<std::string> &vect)
{
	std::string res = "";
	for(int i = 0; i < vect.size(); i++)
	{
		res += vect[i];
		if(i < vect.size() - 1)
		{
			res += ", ";
		}
	}

	return res;
}
}
