/*
 * am_util.cpp
 *
 *  Created on: Nov 7, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/am_util.h>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <sys/stat.h>

/**
 * Simple check to see if file exists
 */
bool fileExists(const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

std::string get_file_contents(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return(contents);
  }
  throw(errno);
}

std::vector<std::string> getDirNameList(std::string root) {
    using namespace std;
    using namespace boost::filesystem;

    vector<string> list;
    path p(root);
    for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (is_directory(i->path()))
        {
            list.push_back(i->path().filename().string());
        }
        else
            continue;
    }

    return list;
}

std::vector<std::string> getDirPathList(std::string root) {
    using namespace std;
    using namespace boost::filesystem;

    vector<string> list;
    path p(root);
    for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (is_directory(i->path()))
        {
            list.push_back(i->path().string());
        }
        else
            continue;
    }

    return list;
}


