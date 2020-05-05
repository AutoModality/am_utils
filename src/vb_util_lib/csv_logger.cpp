/*
 * csv_logger.cpp
 *
 *  Created on: May 5, 2020
 *      Author: ubuntu
 */


#include<vb_util_lib/csv_logger.h>
#include<fstream>
#include<iostream>
#include<string>
#include<boost/filesystem.hpp>

namespace am
{
CSV_LOGGER::CSV_LOGGER()
{
}

CSV_LOGGER::CSV_LOGGER(const std::string &fl, const vector<std::string> &col_header)
{
	file_location_ = fl;
	bool fileStatus = fileExists(file_location_);
	std::ofstream of;
	of.open(file_location_.c_str(), std::ofstream::out | std::ofstream::app);
	if(!fileStatus)
	{
		stringstream ss;
		for(int i =0; i < col_header.size(); i++)
		{
			ss << col_header[i];
			if(i < col_header.size() - 1) ss << ",";
		}
		ss << "\n";
		of << ss.str();
	}


	of.close();
}
CSV_LOGGER::CSV_LOGGER(const std::string &fl)
{
	file_location_ = fl;
	if(!fileExists(file_location_))
	{
		if(!createFile(fl))
		{
			std::cout << __func__ << ": File Creation Failed." << std::endl;
		}
	}
}

CSV_LOGGER::CSV_LOGGER(const std::string &fl, const std::string &header, char deliminator)
{
	//Parse the header with deliminator
	std::vector<std::string> local_header;
	splitToVector(local_header, header, deliminator);

	file_location_ = fl;
	bool fileStatus = fileExists(file_location_);
	std::ofstream of;
	of.open(file_location_.c_str(), std::ofstream::out | std::ofstream::app);
	if(!fileStatus)
	{
		stringstream ss;
		for(int i =0; i < local_header.size(); i++)
		{
			ss << local_header[i];
			if(i < local_header.size() - 1) ss << ",";
		}
		ss << "\n";
		of << ss.str();
	}
}

CSV_LOGGER::~CSV_LOGGER()
{

}

void CSV_LOGGER::splitToVector(std::vector<std::string> &result, const std::string &header, char deliminator)
{
	std::stringstream ss(header);
	std::string token;
	while (std::getline(ss, token, deliminator)) {
		result.push_back(token);
	}
}

bool CSV_LOGGER::fileExists(const std::string &csv_file_location)
{
	if (FILE *file = fopen(csv_file_location.c_str(), "r"))
	{
		fclose(file);
		return true;
	}
	else
	{
		return false;
	}
}

void CSV_LOGGER::insert(const std::string &content, char deliminator)
{
	//Parse the header with deliminator
	std::vector<std::string> content_vect;
	splitToVector(content_vect, content, deliminator);
	insert(content_vect);
}

/*
 *  insert function inserts a row into the given file
 */
void CSV_LOGGER::insert(const std::vector<std::string> &content)
{
	if(!fileExists(file_location_))
	{
		std::cout << __func__ << ": File does not exist." << std::endl;
		return;
	}

	std::ofstream of;
	of.open(file_location_.c_str(), std::ofstream::out | std::ofstream::app);

	if(content.size() > 0)
	{
		std::stringstream ss;
		for(int i =0; i < content.size(); i++)
		{
			ss << content[i];
			if(i < content.size() - 1)
			{
				ss << ",";
			}
		}
		ss << "\n";
		of << ss.str();
	}

	of.close();

}

bool CSV_LOGGER::createFile(const std::string &csv_file_location)
{
	if(fileExists(csv_file_location))
	{
		std::cout << __func__ << ": File creation failed: File Exists Already." << std::endl;
		return false;
	}

	std::ofstream of;
	of.open(csv_file_location.c_str(), std::ios::out | std::ios::app);
	if(!fileExists(csv_file_location))
	{
		of.close();
		std::cout << __func__ << ": File creation failed: Something went wrong : " << csv_file_location << std::endl;
		return false;
	}

	of.close();
	return true;
}


bool CSV_LOGGER::createPath(const std::string &path)
{
	return boost::filesystem::create_directories(path);
}

}


