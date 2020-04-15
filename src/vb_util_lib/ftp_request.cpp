#include <vb_util_lib/ftp_request.h>

const std::string& FTPRequest::getUuid() const {
	return uuid_;
}

void FTPRequest::setUuid(const std::string& uuid) {
	uuid_ = uuid;
}

const boost::posix_time::ptime& FTPRequest::getDate() const {
	return date_;
}

void FTPRequest::setDate(const boost::posix_time::ptime& date) {
	this->date_ = date;
}

FTPRequest::Command FTPRequest::getCommand() const {
	return command_;
}

void FTPRequest::setCommand(FTPRequest::Command command) {
	command_ = command;
}

const std::string& FTPRequest::getVxId() const {
	return vxID_;
}

void FTPRequest::setVxId(const std::string& vxId) {
	vxID_ = vxId;
}

const FileSpec& FTPRequest::getFileSpec() const {
	return fileSpec_;
}

void FTPRequest::setFileSpec(const FileSpec& fileSpec) {
	fileSpec_ = fileSpec;
}

