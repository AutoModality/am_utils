#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <signal.h>
#include <csignal>

#include <am_utils/am_ros2_utility.h>

void kill_process_with_name(const char* partialName) {
    FILE* fp;
    char buffer[1024];

    __pid_t current_pid= getpid();

    fp = popen((std::string("ps -ef | grep ") + std::string(partialName)).c_str(), "r");
    if (fp == NULL) {
        printf("Failed to run command\n");
        return;
    }

    //printf("PID\tUSER\tCPU\tMEM\tCOMMAND\n");

    while (fgets(buffer, sizeof(buffer), fp) != NULL)
    {
    	if (strstr(buffer, partialName) != NULL)
    	{
    		std::vector<std::string> params = am::toVector<std::string>(std::string(buffer), ' ');
    		/*for(int i = 0; i < params.size(); i++)
    		{
    			printf(" %d: %s", i, params[i].c_str());
    		}
    		printf("\n");*/
    		__pid_t pid = std::stoi(params[3]);
    		printf("pid: %d \n", (int)pid);
    		if((int) current_pid != (int)pid)
    		{
    			kill(pid, SIGINT);

    			printf("Killed %s \n", partialName);
    		}
    	}
    	usleep(100000);
    }

    pclose(fp);
}

void signalHandler( int signum )
{

	kill_process_with_name("gzclient");

	kill_process_with_name("gzserver");

	kill_process_with_name("ros2");

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
