
#include <stdio.h>
#include <iostream>

extern "C" {
#include "remoteAPI/extApi.h"
}

int main(int argc,char* argv[])
{
	char teamID[5], i;
	char msg[] = "Congratulations NS#@@@@ on completing Task 0 of Nutty Squirrel!";
	simxInt winPos[2] = {800,100};
	printf("Enter your Team's ID: NS#");
	fgets(teamID, 5, stdin);
	for (i = 0; i < 4; i++)
		msg[19 + i] = teamID[i];
	simxInt portNb = 19997; //Change this number to the number in your "remoteApiConnections.txt" file, alternatively, you can put this number there.
	simxInt clientID = -1;  //Intialize the ClientID 
	simxInt* consoleHandle = new simxInt[1]();
	simxFinish(-1); ///Close all connections if any are open.
	clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);
	if (clientID > -1)
		printf("\n\nConnection Success ... \n");
	else
		printf("\n\nConnection Fail ... \n");
	simxAuxiliaryConsoleOpen(clientID, "Task 0 Nutty Squirrel", 5, 7, winPos, NULL, NULL, NULL, consoleHandle, simx_opmode_oneshot_wait); //Create a new console window in V-REP
	simxAuxiliaryConsolePrint(clientID, *consoleHandle, msg, simx_opmode_oneshot_wait); //Print our message in that window
	simxFinish(clientID);	//Close the connection
	return 0;
}