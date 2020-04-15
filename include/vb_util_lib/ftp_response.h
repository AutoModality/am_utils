/*
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_RESPSONSE_H_
#define VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_RESPSONSE_H_

#include <vb_util_lib/file_spec.h>

class FTPResponse
{
public:
	enum MessageType {VXID, DIRECTORY_LISTING};

	const std::string& getUuid() const;
	void setUuid(const std::string& uuid);
	const boost::posix_time::ptime& getDate() const;
	void setDate(const boost::posix_time::ptime& date);
	MessageType getType() const;
	void setType(MessageType type);
	const std::string& getVxId() const;
	void setVxId(const std::string& vxId);
	const std::vector<FileSpec>& getFileSpecs() const;
	void setFileSpecs(const std::vector<FileSpec>& fileSpecs);
	const bool getSuccess() const;
	bool isSuccess() const;
	void setSuccess(bool success);

private:
	std::string uuid_;
	boost::posix_time::ptime date_;
	MessageType type_;
	std::string vxID_;
	std::vector<FileSpec> fileSpecs_;
	bool success_;
};

#endif /* VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_RESPSONSE_H_ */
