#include <vb_util_lib/ftp_response.h>

const std::string& FTPResponse::getUuid() const {
	return uuid_;
}

void FTPResponse::setUuid(const std::string& uuid) {
	uuid_ = uuid;
}

const boost::posix_time::ptime& FTPResponse::getDate() const {
	return date_;
}

void FTPResponse::setDate(const boost::posix_time::ptime& date) {
	this->date_ = date;
}

FTPResponse::MessageType FTPResponse::getType() const {
	return type_;
}

void FTPResponse::setType(FTPResponse::MessageType type) {
	type_ = type;
}

const std::string& FTPResponse::getVxId() const {
	return vxID_;
}

void FTPResponse::setVxId(const std::string& vxId) {
	vxID_ = vxId;
}

const std::vector<FileSpec>& FTPResponse::getFileSpecs() const {
	return fileSpecs_;
}

void FTPResponse::setFileSpecs(const std::vector<FileSpec>& fileSpecs) {
	fileSpecs_ = fileSpecs;
}

bool FTPResponse::isSuccess() const {
	return success_;
}

void FTPResponse::setSuccess(bool success) {
	this->success_ = success;
}
