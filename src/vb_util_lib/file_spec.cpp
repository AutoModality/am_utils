#include <vb_util_lib/file_spec.h>
#include <cstdlib>

const boost::posix_time::ptime& FileSpec::getDate() const {
	return date_;
}

void FileSpec::setDate(const boost::posix_time::ptime& date) {
	this->date_ = date;
}

const std::string& FileSpec::getMd5() const {
	return md5_;
}

void FileSpec::setMd5(const std::string& md5) {
	md5_ = md5;
}

const std::string& FileSpec::getName() const {
	return name_;
}

void FileSpec::setName(const std::string& name) {
	name_ = name;
}

int FileSpec::getSize() const {
	return size_;
}

void FileSpec::setSize(int size) {
	size_ = size;
}

FileSpec::FileType FileSpec::getType() const {
	return type_;
}

void FileSpec::setType(FileSpec::FileType type) {
	type_ = type;
}

FileSpec::Directory FileSpec::getDir() const {
	return dir_;
}

void FileSpec::setDir(FileSpec::Directory dir) {
	dir_ = dir;
}

void FileSpec::setPath(std::string& base, std::string param, std::string env_name) {
	char sep = '/';
	std::string base_dir = "";
	prefix_ = "";
	name_ = "";

	size_t i = base.rfind(sep, base.length());
	if (i != std::string::npos)
	{
		name_ = base.substr(i+1, base.length() - i);
		base_dir = base.substr(0, i+1);
	}
	else
	{
		name_= base;
		base_dir = "";
	}

	if (base_dir.length() != 0 && base_dir.at(0) == sep)
	{
		// Absolute path name
		prefix_ = base_dir;
		return;
	}

	// Append the separator to the base_dir
	if (base_dir.length() > 0 && base_dir.at(base_dir.length()-1) != sep)
	{
		base_dir += "/";
	}

	if (param.length() > 0)
	{
		// use the parameter as the directory name.
		prefix_ = param;
	} else if (env_name.length() > 0)
	{
		// use the environment variable
	    if(const char* env_p = std::getenv(env_name.c_str()))
	    {
	    	prefix_ = env_p;
	    }
	}

	// Append the separator to the prefix
	if (prefix_.length() > 0 && prefix_.at(prefix_.length()-1) != sep)
	{
		prefix_ += "/";
	}
	prefix_ += base_dir;

	return;
}

void FileSpec::getPath(std::string& full_name) {
	full_name = prefix_ + name_;
}
