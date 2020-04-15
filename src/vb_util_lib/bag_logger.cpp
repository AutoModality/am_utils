#include <vb_util_lib/bag_logger.h>

BagLogger *BagLogger::s_instance_ = 0;

std::string BagLogger::getLogFileDir() {
    using namespace std;
    using namespace boost::filesystem;

    path p("/media/ubuntu");
    for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (is_directory(i->path()))
        {
            std::size_t found = i->path().filename().string().find("LOG");
            if (found!=std::string::npos)
            {
                //                cout << "FOUND: "+i->path().filename().string()+"  AT: "+i->path().string() << endl;
                return i->path().string()+"/";
            }
        }
        else
            continue;
    }
    ROS_INFO("LG: NO LOG_XXX SD CARD FOUND");

    return string(DEFAULT_BAG_DIR);
}

std::string BagLogger::getSequence(std::string dir, std::string prefix) {
    using namespace std;
    using namespace boost::filesystem;
    using namespace boost;

    unsigned int seq_num = 0;

    path p(dir);
    for (auto i = directory_iterator(p); i != directory_iterator(); i++)
    {
        if (!is_directory(i->path())) //we eliminate directories
        {
            // cout << i->path().filename().string() << endl;
            std::vector<std::string> fields;
            boost::split(fields, i->path().filename().string(), boost::is_any_of("-"));
            if (fields.size() >= 2)
            {
                if (fields[0] == prefix)
                {
                    unsigned long tmp = atol(fields[1].c_str());
                    if (tmp < 9999 && tmp > seq_num)
                    {
                        seq_num = tmp;
                    }
                }
            }
        }
        else
            continue;
    }

    seq_num++;

    std::stringstream s;
    s << std::setfill('0') << std::setw(4) << seq_num;

//    log_name_seq_ = s.str();

    return s.str();
}

std::string BagLogger::getLogFileName(std::string prefix, std::string suffix) {
//    log_name_prefix_ = prefix;
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    std::string dir_str = getLogFileDir();

    std::string seq_str = getSequence(dir_str, prefix);

    std::string format_str = dir_str+prefix+"-"+seq_str+"-%Y-%m-%d-%H-%M-%S."+suffix;

    strftime(buffer, 80, format_str.c_str(), timeinfo);
    std::string str(buffer);

    return str;
}

void BagLogger::dumpParams(std::string prefix) {
    std::string filename = getLogFileName(prefix);
    std::string cmd = "rosparam dump " + filename;

    ROS_INFO("BAG_LOGGER: Dumping params to %s", filename.c_str());

    system(cmd.c_str());

    return;
}

void BagLogger::startLogging(std::string prefix, unsigned log_level) {
    if (is_logging_)
    {
        if ((prefix == prefix_) && (log_level == log_level_))
        {
            ROS_INFO("LG: Already logging at prefix[%s] and log level[%d]", prefix.c_str(), log_level_);
            return;
        }
        else
        {
            ROS_INFO("LG: Closing bag file %s", file_name_.c_str());
            bag.close();
            log_level_ = 0;
            prefix_ = "";
            is_logging_ = false;
        }
    }

    if (log_level > 0)
    {
        prefix_ = prefix;
        file_name_ = getLogFileName(prefix);
        bag.open(file_name_, rosbag::bagmode::Write);
        ROS_INFO("Opening bag file %s", file_name_.c_str());
        log_level_ = log_level;
        is_logging_ = true;
    }
}

void BagLogger::stopLogging() {
    if (is_logging_)
    {
        log_level_ = 0;
        is_logging_ = false;
        ROS_INFO("LG: Closing bag file %s", file_name_.c_str());
        prefix_ = "";
        bag.close();
    }
    return;
}

void BagLogger::open_log(std::string prefix, FILE **debug_file, int log_level) {
  return;
    if (*debug_file != NULL)
    {
        ROS_WARN("LOGGER: Tried to reopen test logger with prefix %s", prefix.c_str());
        return;
    }
    if (log_level > 0)
    {
        std::string log_name = BagLogger::getLogFileName(prefix, "txt");

        if (log_name != "")
        {
            ROS_INFO("LOGGER: OPENING GENERAL TEXT LOG FILE %s", log_name.c_str());
            *debug_file = fopen (log_name.c_str(),"w");
        }
    }
}

void BagLogger::close_log(FILE **debug_file) {
  return;
    if (*debug_file != NULL)
    {
        fclose(*debug_file);
        *debug_file = NULL;
    }
}

void BagLogger::debug_log(FILE **debug_file, const char* format, ...) {
    va_list args;
    return;
    if (*debug_file == NULL)
    {
        return;
    }

    va_start( args, format );
    vfprintf( *debug_file, format, args );
    va_end( args );

    fflush(*debug_file);
}



