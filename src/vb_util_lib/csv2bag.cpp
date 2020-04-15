#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int main()
{
	std::ifstream ifile("test.csv");

    std::string line; // we read the full line here
    for (int i = 0; i < 7; i++) std::getline(ifile, line);
    while (std::getline(ifile, line)) // read the current line
    {
        std::istringstream iss{line}; // construct a string stream from line

        // read the tokens from current line separated by comma
        std::vector<std::string> tokens; // here we store the tokens
        std::string token; // current token
        for (int i = 0; i < 10; i++) 
        {
			std::getline(iss, token, ',');
            tokens.push_back(token); // add the token to the vector
        }

        // we can now process the tokens
        // first display them
        std::cout << "Tokenized line: ";
        for (const auto& elem : tokens)
            std::cout << "[" << elem << "]";
        std::cout << std::endl;

		rosbag::Bag bag;
		bag.open("test.bag", rosbag::bagmode::Write);

		std_msgs::String str;
		str.data = std::string("foo");

		std_msgs::Int32 i;
		i.data = 42;

		bag.write("chatter", ros::Time::now(), str);
		bag.write("numbers", ros::Time::now(), i);

		bag.close();
}
