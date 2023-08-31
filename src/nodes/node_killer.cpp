#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <signal.h>
#include <csignal>

#include <am_utils/am_ros2_utility.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <unistd.h>
#include <fstream>

// Function to get a list of process IDs by their name
std::vector<int> getProcessIDsByName(const std::string& name) {
    std::vector<int> processIDs;

    DIR* dir = opendir("/proc");
    if (dir == nullptr) {
        std::cerr << "Error opening /proc directory" << std::endl;
        return processIDs;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_DIR) {
            // Check if the entry is a directory and its name is a numeric PID
            int pid = atoi(entry->d_name);
            if (pid > 0) {
                std::string cmdLinePath = "/proc/" + std::to_string(pid) + "/cmdline";
                std::ifstream cmdLineFile(cmdLinePath);
                std::string cmdLine;
                std::getline(cmdLineFile, cmdLine);

                // Check if the process name matches and is not an ".sh" process or the current process
                if (!cmdLine.empty() && cmdLine.find(name) != std::string::npos &&
                    cmdLine.find(".sh") == std::string::npos &&
                    pid != getpid()) {
                    processIDs.push_back(pid);
                }
            }
        }
    }

    closedir(dir);
    return processIDs;
}

// Function to kill processes by their IDs
void killProcesses(const std::vector<int>& pids) {
    for (int pid : pids) {
		if(pid == getpid())
		{
			continue;
		}

        if (kill(pid, SIGTERM) == 0) {
            std::cout << "Killed process with PID: " << pid << " vs " << getpid() << std::endl;
        } else {
            std::cerr << "Error killing process with PID: " << pid << std::endl;
        }
    }
}

void kill_process_with_name(const std::string p_name)
{
	killProcesses(getProcessIDsByName(p_name));
}

void signalHandler( int signum )
{

	kill_process_with_name("gzclient");

	kill_process_with_name("gzserver");

	kill_process_with_name("ros");

	kill_process_with_name("python3");

	system("ros2 daemon stop");

	printf("\n");

	exit(signum);

}


int main()
{

	signal(SIGINT, signalHandler);
	while(1)
	{
	      //std::cout << "Going to sleep...." << std::endl;
	      sleep(1);
	}
    return 0;
}
