#include "NS_Task_1_Predef.h"

simxInt ID = 0;
simxUChar *lineSensorOutput = new simxUChar[3](), *colorSensorOutput = new simxUChar[3](), detection_state=0;
simxFloat detected_point[3] = { 0,0,0 }; 
unsigned int color_sensor_pulse_count = 0;
simxInt dir_left = 0, dir_right = 0, vis_error, prox_error, place_handle = -3;
simxInt *res = new simxInt[2](), *obs1 = new simxInt[1](), *obs2 = new simxInt[1](), *obs3 = new simxInt[1](), *leftJoint = new simxInt[1](), *rightJoint = new simxInt[1](), *lineSensor = new simxInt[1](), *colorSensor = new simxInt[1](), *eBot = new simxInt[1](), *cuboid0 = new simxInt[1](), *cuboid = new simxInt[1](), *cuboid3 = new simxInt[1](), *cuboid4 = new simxInt[1](), *proxSensor = new simxInt[1]();
simxFloat linear_velocity_left = 0, linear_velocity_right = 0;
const simxFloat posObv[3] = { 50,50,50 }, posPlaceRel[3] = { 0.125, 0, 0 };

void getObjectHandles(void)
{
	simxGetObjectHandle(ID, "LeftJoint", leftJoint, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "RightJoint", rightJoint, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "LineSensor", lineSensor, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "ColorSensor", colorSensor, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "eBot", eBot, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "Obs1", obs1, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "Obs2", obs2, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "Obs3", obs3, simx_opmode_oneshot_wait);
	simxGetObjectHandle(ID, "ProximitySensor", proxSensor, simx_opmode_oneshot_wait);
}

void setJointVelocities(void)
{
		simxSetJointTargetVelocity(ID, *rightJoint, linear_velocity_right, simx_opmode_oneshot);
		simxSetJointTargetVelocity(ID, *leftJoint, linear_velocity_left, simx_opmode_oneshot);
}

void forward(void)
{
	dir_left = dir_right = 1;
	linear_velocity_left = linear_velocity_right = LIN_MAX;
}

void back(void)
{
	dir_left = dir_right = -1;
	linear_velocity_left = linear_velocity_right = -LIN_MAX;
}

void soft_right(void)
{
	dir_left = 1;
	dir_right = 0;
	linear_velocity_left = LIN_MAX;
	linear_velocity_right = 0;
}

void right(void)
{
	dir_left = 1;
	dir_right = -1;
	linear_velocity_left = LIN_MAX;
	linear_velocity_right = -LIN_MAX;
}

void soft_left(void)
{
	dir_left = 0;
	dir_right = 1;
	linear_velocity_left = 0;
	linear_velocity_right = LIN_MAX;
}

void left(void)
{
	dir_left = -1;
	dir_right = 1;
	linear_velocity_left = -LIN_MAX;
	linear_velocity_right = LIN_MAX;
}

void stop(void)
{
	dir_left = 0;
	dir_right = 0;
	linear_velocity_left = 0;
	linear_velocity_right = 0;
}

void velocity(int left_motor_velocity, int right_motor_velocity)
{
	if (left_motor_velocity > 255)
		left_motor_velocity = 255;
	else if (left_motor_velocity < 0)
		left_motor_velocity = 0;
	if (right_motor_velocity > 255)
		right_motor_velocity = 255;
	else if (right_motor_velocity < 0)
		right_motor_velocity = 0;

	linear_velocity_left = dir_left * LIN_MAX*left_motor_velocity / float(255.0);
	linear_velocity_right = dir_right * LIN_MAX*right_motor_velocity / float(255.0);
	

}


void getLineSensorData(void)
{
	simxGetVisionSensorImage(ID, *lineSensor, res, &lineSensorOutput, 1, simx_opmode_buffer);

}

void getColorSensorData(void)
{
	simxGetVisionSensorImage(ID, *colorSensor, res, &colorSensorOutput, 0, simx_opmode_buffer);
}

unsigned char getProxSensorDistance(void)
{
	unsigned char retval = 140;
	prox_error = simxReadProximitySensor(ID, *proxSensor, &detection_state, detected_point, NULL, NULL, simx_opmode_buffer);
	if (detection_state != 0)
	{
		detection_state = 0;
		retval = unsigned char(detected_point[2] * 1000);
	}
	else
		retval -= rand() % 40;
	return retval;
}


void filter_red(void)
{
	getColorSensorData();
	color_sensor_pulse_count = unsigned int(colorSensorOutput[0] * 5000 / 255.0)+(rand()%1001);
}

void filter_green(void)
{
	getColorSensorData();
	color_sensor_pulse_count = unsigned int(colorSensorOutput[1] * 5000 / 255.0)+(rand() % 1001);
}

void filter_blue(void)
{
	getColorSensorData();
	color_sensor_pulse_count = unsigned int(colorSensorOutput[2] * 5000 / 255.0)+ (rand() % 1001);
}

void filter_clear(void)
{
	getColorSensorData();
	color_sensor_pulse_count = unsigned int((colorSensorOutput[0]+ colorSensorOutput[1]+ colorSensorOutput[2]) * 5000 / 255.0)+(rand() % 1001);
}


int initial(void)
{
	int portNb = 19997;
	int clientID = -1;
	simxFinish(-1);

	clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);

	if (clientID > -1)
		printf("Connection Success ... \n");
	else
		printf("Connection Fail");

	return clientID;
}

int simulatorStart(int ID)
{
	printf("\n Please Enter Y to Start Simulation:		");
	char checkStartCmd = getchar();
	simxInt start = -1;
	if (checkStartCmd == 'Y' || checkStartCmd == 'y')
	{
		printf(" Starting ...");
		do
		{
			start = simxStartSimulation(ID, simx_opmode_oneshot_wait);
		} while (start != simx_return_ok);
	}
	return start;
}

void initVisionSensors(void)
{
	do 
		vis_error = simxGetVisionSensorImage(ID, *lineSensor, res, &lineSensorOutput, 1, simx_opmode_streaming);
	while (vis_error != simx_return_ok || vis_error == simx_error_novalue_flag);
	do
		vis_error = simxGetVisionSensorImage(ID, *colorSensor, res, &colorSensorOutput, 0, simx_opmode_streaming);
	while (vis_error != simx_return_ok || vis_error == simx_error_novalue_flag);
	for(int i = 0; i < VIS_SEN_INIT_VAL; i++)
	{
		getLineSensorData();
		getColorSensorData();
	}
}

void initProxSensor(void)
{
	do
		prox_error = simxReadProximitySensor(ID, *proxSensor, NULL, NULL, NULL, NULL, simx_opmode_streaming);
	while (prox_error != simx_return_ok || prox_error == simx_error_novalue_flag);
}

void initSensors(void)
{
	initVisionSensors();
	initProxSensor();
}

void pick(void)
{

	//Set OBj position to oblvion
	simxInt pickDetectObjectHandle = -2;
	do
		prox_error = simxReadProximitySensor(ID, *proxSensor, NULL, NULL, &pickDetectObjectHandle, NULL, simx_opmode_buffer);
	while (prox_error != simx_return_ok || prox_error == simx_error_novalue_flag);
	if (pickDetectObjectHandle != -2)
	{
		if (pickDetectObjectHandle == *obs1 || pickDetectObjectHandle == *obs2 || pickDetectObjectHandle == *obs3)
		{
			printf("\nCan't pick an obstacle!");
		}
		else
		{
			if (place_handle == -3)
				simxSetObjectPosition(ID, pickDetectObjectHandle, -1, posObv, simx_opmode_oneshot);
			else
				printf("\nCan't pick, another block already picked up.");
		}

	}
	else
		printf("\nNo object to pick.");
	place_handle = pickDetectObjectHandle;
}

void place(void)
{
	//setobjpos relative to Robot
	if (place_handle != -3)
	{	
		simxSetObjectPosition(ID, place_handle, *eBot, posPlaceRel, simx_opmode_oneshot);
		place_handle = -3;
	}
	else
		printf("Nothing has been picked");

}


unsigned char ADC_Conversion(unsigned char ch_no)
{
	if (ch_no == 1)	//Left Line Sensor
		return ~lineSensorOutput[0];
	else if (ch_no == 2)//Middle Line Sensor
		return ~lineSensorOutput[1];
	else if (ch_no == 3)//Right Line Sensor
		return ~lineSensorOutput[2];
	else if (ch_no == FRONT_IR_ADC_CHANNEL) //Channel for Proximity sensor
		return getProxSensorDistance();
	return 255;
}


void _delay_ms(unsigned int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


void init(void)
{
	ID = initial();
	getObjectHandles();
	int check = simulatorStart(ID);
	initSensors();
	
}

void cleanUp(void)
{
	printf("That's it folks!");
	int check;
	do
	{
		check = simxStopSimulation(ID, simx_opmode_oneshot_wait);
	} while (check != simx_return_ok);
	simxFinish(-1);
	ID = -1;
}

void threadCalls(void)
{
	while (ID != -1)
	{
		setJointVelocities();
		getLineSensorData();
	}
}
