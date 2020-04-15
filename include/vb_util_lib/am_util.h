/*
 * Utility functions
 */

#include <fstream>
#include <string>
#include <cerrno>
#include <vector>

extern std::string get_file_contents(const char *filename);
extern std::string getMediaFileDir(std::string prefix);
extern std::vector<std::string> getDirNameList(std::string root);
extern std::vector<std::string> getDirPathList(std::string root);
extern bool fileExists(const std::string& name);
