/*
 * csv_logger.h
 *
 *  Created on: May 5, 2020
 *      Author: ubuntu
 */

#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <vector>
#include <utility>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace std;

namespace am
{
class CSV_LOGGER
{
private:
	std::string file_location_;
public:
	CSV_LOGGER(const std::string &fl, const vector<std::string> &col_header);

	CSV_LOGGER(const std::string &fl, const std::string &header, char deliminator = ',');

	CSV_LOGGER(const std::string &fl);

	CSV_LOGGER();

	virtual ~CSV_LOGGER();

	void splitToVector(std::vector<std::string> &result, const std::string &header, char deliminator = ',');

	//Insert Functions
	void insert(const std::vector<std::string> &content);

	void insert(const std::string &content, char deliminator = ',');


	template <class T>
	void insert(const T &t, char deliminator = ',', bool endOfLine = false)
	{
		if(!fileExists(file_location_))
		{
			std::cout << __func__ << ": File does not exist." << std::endl;
			return;
		}
		std::ofstream of;
		of.open(file_location_.c_str(), std::ofstream::out | std::ofstream::app);

		std::stringstream ss;
		ss << t;

		if(endOfLine)
		{
			ss << "\n";
		}
		else
		{
			ss << ',';
		}

		of << ss.str();
		of.close();

	}

	bool fileExists(const std::string &csv_file_name);

	bool createFile(const std::string &csv_file_location);

	bool createPath(const std::string &path);

};
}


#endif /* CSV_LOGGER_H_ */
