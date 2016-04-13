#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h"
#define LASER_SCAN_DIVISION 9
#define OBSTACLE_DISTANCE	0.2f
#define MAX_POLE_DISTANCE	0.5f
#define EDGE_DISTANCE		0.3f
#define MAX_LINEAR_SPEED	0.15f
#define MAX_ANGULAR_SPEED	0.35f

geometry_msgs::Twist velocityCommand;

bool turn_enable = false; //flag for enabling 180 turn
bool turn_complete = true;

int checkprint = 0;
int pole_print = 0;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData) 
{ 
	int print_val;
	int pole_print_val;

	bool obstacle_array[LASER_SCAN_DIVISION];
	bool pole_detected[2];

	// Example of using some of the non-range data-types
	float rangeDataNum =  1 + (laserScanData->angle_max - laserScanData->angle_min) /
					  	(laserScanData->angle_increment);

	float scanSector[LASER_SCAN_DIVISION];
	for (int i = 0; i < LASER_SCAN_DIVISION; i++)
	{
		scanSector[i] = i * rangeDataNum/(LASER_SCAN_DIVISION - 1);
	}

	/**Initialising the obstacle array**/
	if(laserScanData->ranges[scanSector[0]] < EDGE_DISTANCE)
	{
		obstacle_array[0] = true;
	}
	else
	{
		obstacle_array[0] = false;
	}
	printf("%d, ", obstacle_array[0] );
	// Go through the laser data 
	for(int j = 1; j < LASER_SCAN_DIVISION - 1; ++j)
	{
		if(laserScanData->ranges[scanSector[j]] < OBSTACLE_DISTANCE) //obstacle present
		{
			obstacle_array[j] = true;
		}
		else
		{
			obstacle_array[j] = false;
		}
		printf("%d, ", obstacle_array[j] );
	}
	if(laserScanData->ranges[scanSector[8]] < EDGE_DISTANCE)
	{
		obstacle_array[8] = true;
	}
	else
	{
		obstacle_array[8] = false;
	}
	printf("%d, ", obstacle_array[8] );
	printf("\n");
	/**OBSTACLE ARRAY SETUP END**/

	/*Pole Detection*/
	if((laserScanData->ranges[scanSector[1]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[1]] <= MAX_POLE_DISTANCE)
		|| (laserScanData->ranges[scanSector[2]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[2]] <= MAX_POLE_DISTANCE)
		|| (laserScanData->ranges[scanSector[3]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[3]] <= MAX_POLE_DISTANCE))
	{
		pole_detected[0] = true; 
	}
	else
	{
		pole_detected[0] = false;
	}

	if((laserScanData->ranges[scanSector[5]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[5]] <= MAX_POLE_DISTANCE)
		|| (laserScanData->ranges[scanSector[6]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[6]] <= MAX_POLE_DISTANCE)
		|| (laserScanData->ranges[scanSector[7]] > OBSTACLE_DISTANCE && laserScanData->ranges[scanSector[7]] <= MAX_POLE_DISTANCE))
	{
		pole_detected[1] = true; 
	}
	else
	{
		pole_detected[1] = false;
	}

	if(pole_detected[1] && pole_detected[0])
	{
		pole_print_val = 1;
		if(pole_print != pole_print_val)
		{
			pole_print = pole_print_val;
			printf("LEFT and RIGHT pole detected");
		}
	}
	else if(pole_detected[0])
	{
		pole_print_val = 2;
		if(pole_print != pole_print_val)
		{
			pole_print = pole_print_val;
			printf("RIGHT ONLY pole detected");
		}
	}
	else if (pole_detected[1])
	{
		pole_print_val = 3;
		if(pole_print != pole_print_val)
		{
			pole_print = pole_print_val;
			printf("LEFT ONLY pole detected");
		}
	}
	else
	{
		pole_print_val = 4;
		if(pole_print != pole_print_val)
		{
			pole_print = pole_print_val;
			printf("NO pole detected");
		}
		//turn_enable = true;
	}


	if (!turn_enable)
	{
		print_val = 1;
		if(checkprint != print_val)
		{
			checkprint = print_val;
			printf("GOING STRAIGHT\n");
		}

		velocityCommand.linear.x = MAX_LINEAR_SPEED;
		velocityCommand.angular.z = 0;

		if((obstacle_array[1] || obstacle_array[2]) || (obstacle_array[3] || obstacle_array[0]))
		{
			print_val = 2;
			if(checkprint != print_val)
			{
				checkprint = print_val;
				printf("SLIGHT LEFT\n");
			}

			velocityCommand.linear.x = (2 / 3) * MAX_LINEAR_SPEED;
			velocityCommand.angular.z = MAX_ANGULAR_SPEED;		
		}
		else if ((obstacle_array[5] || obstacle_array[6]) || (obstacle_array[8] || obstacle_array[7]))
		{
			print_val = 3;
			if(checkprint != print_val)
			{
				checkprint = print_val;
				printf("SLIGHT RIGHT\n");
			}
			
			velocityCommand.linear.x = (2 / 3) * MAX_LINEAR_SPEED;
			velocityCommand.angular.z = -MAX_ANGULAR_SPEED;
		}
		//else if(obstacle_array[4])
		//{
		//	if(laserScanData->ranges[obstacle_array[2]] > laserScanData->ranges[obstacle_array[]]
		//}
	} 
	else //turn enable
	{
		printf("TURNING \n");
		if (laserScanData->ranges[obstacle_array[1]] > laserScanData->ranges[obstacle_array[7]])
		{
			velocityCommand.linear.x = MAX_LINEAR_SPEED;
			velocityCommand.angular.z = -MAX_ANGULAR_SPEED;
		}
		else if(laserScanData->ranges[obstacle_array[7]] > laserScanData->ranges[obstacle_array[1]])
		{
			velocityCommand.linear.x = MAX_LINEAR_SPEED;
			velocityCommand.angular.z = MAX_ANGULAR_SPEED;
		}
		else
		{
			turn_enable = false;
			printf("turn done \n");
		}
	}
}
int main (int argc, char **argv) 
{ 
	// command line ROS arguments
	ros::init(argc, argv, "pioneer_laser_node"); 
	// ROS comms access point 
	ros::NodeHandle my_handle;  
	// loop 10 Hz 
	ros::Rate loop_rate(10);
	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",1); 
	// subscribe to the scan topic and define a callback function to process the data
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	// publish the velocity set in the call back
	while(ros::ok())
	{ 
		ros::spinOnce();
		loop_rate.sleep(); 
		// publish
		vel_pub_object.publish(velocityCommand); 
	} 
	return 0; 
}