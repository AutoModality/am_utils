#ifndef VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FILE_SPEC_H_
#define VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FILE_SPEC_H_

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

class FileSpec
{
public:
	enum Directory {PARTS, LOGS, PHOTOS, VIDEOS};

	enum FileType {FILE, DIRECTORY, PART, MISSION_SPEC, SCRIPT, FLIGHT_PLAN, LOG, PHOTO, VIDEO};

	const boost::posix_time::ptime& getDate() const;
	void setDate(const boost::posix_time::ptime& date);
	const std::string& getMd5() const;
	void setMd5(const std::string& md5);
	const std::string& getName() const;
	void setName(const std::string& name);
	int getSize() const;
	void setSize(int size);
	FileType getType() const;
	void setType(FileType type);
	Directory getDir() const;
	void setDir(Directory dir);

private:
	Directory dir_;
	FileType type_;
	std::string name_;
	std::string md5_;
	boost::posix_time::ptime date_;
	int size_;
};

#endif /* VISBOX_PACKAGES_SYSTEM_COMMS_INCLUDE_COMMS_FILE_SPEC_H_ */
