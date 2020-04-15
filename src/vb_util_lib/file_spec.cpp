#include <vb_util_lib/file_spec.h>

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
