/*
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_REQUEST_H_
#define VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_REQUEST_H_

#include <vb_util_lib/file_spec.h>

class FTPRequest
{
public:
	enum Command {DIRECTORY, SEND, RECEIVE};

	const std::string& getUuid() const;
	void setUuid(const std::string& uuid);
	const boost::posix_time::ptime& getDate() const;
	void setDate(const boost::posix_time::ptime& date);
	Command getCommand() const;
	void setCommand(Command command);
	const std::string& getVxId() const;
	void setVxId(const std::string& vxId);
	const FileSpec& getFileSpec() const;
	void setFileSpec(const FileSpec& fileSpec);

private:
	std::string uuid_;
	boost::posix_time::ptime date_;
	Command command_;
	std::string vxID_;
	FileSpec fileSpec_;
};

#endif /* VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FTP_REQUEST_H_ */
