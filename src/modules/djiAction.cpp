#include "djiAction.h"


namespace action_handler
{
	task_action_type* task_action_ptr;
   local_navigation_action_type* local_navigation_action_ptr;
   gps_navigation_action_type* gps_navigation_action_ptr;
	waypoint_navigation_action_type* waypoint_navigation_action_ptr; 
	

	dji_ros::taskFeedback  task_action_feedback;
	dji_ros::taskResult  task_action_result; 

	dji_ros::local_navigationFeedback local_navigation_feedback;
	dji_ros::local_navigationResult local_navigation_result;

	dji_ros::gps_navigationFeedback gps_navigation_feedback;
	dji_ros::gps_navigationResult gps_navigation_result;

	dji_ros::waypoint_navigationFeedback waypoint_navigation_feedback;
	dji_ros::waypoint_navigationResult waypoint_navigation_result;


	bool task_action_callback(const dji_ros::taskGoalConstPtr& goal, task_action_type* task_action)
	{
		uint8_t request_action = goal->task;

		if(request_action == 1)
		{
			//takeoff
			DJI_Pro_Status_Ctrl(4,0);
		}
		else if(request_action == 2)
		{
			//landing
			DJI_Pro_Status_Ctrl(6,0);
		}
		else if(request_action == 3)
		{
			//gohome
			DJI_Pro_Status_Ctrl(1,0);
		}

		task_action_feedback.progress = 1;
		task_action->publishFeedback(task_action_feedback);
		task_action->setSucceeded();
		

		return true;

	}

	bool local_navigation_action_callback(const dji_ros::local_navigationGoalConstPtr& goal, local_navigation_action_type* local_navigation_action)
	{
		printf("in");
		float dst_x = goal->x;
		float dst_y = goal->y;
		float dst_z = goal->z;

		float org_x = dji_variable::local_position.x;
		float org_y = dji_variable::local_position.y;
		float org_z = dji_variable::local_position.z;

		float dis_x = dst_x - org_x;
		float dis_y = dst_y - org_y;
		float dis_z = dst_z - org_z;

		float det_x,det_y,det_z;

		attitude_data_t user_ctrl_data;
      user_ctrl_data.ctrl_flag = 0x90;
      user_ctrl_data.thr_z = dst_z;
      user_ctrl_data.yaw = 0;

		int process = 0;

		while (process < 100) {

         user_ctrl_data.roll_or_x = dst_x - dji_variable::local_position.x;
         user_ctrl_data.pitch_or_y = dst_y - dji_variable::local_position.y;

         DJI_Pro_Attitude_Control(&user_ctrl_data);

			det_x = 100* (dst_x - dji_variable::local_position.x)/dis_x;
			det_y = 100* (dst_y - dji_variable::local_position.y)/dis_y;
			det_z = 100* (dst_z - dji_variable::local_position.z)/dis_z;

			process = 100 - (int)(det_x + det_y + det_z) /3;

			local_navigation_feedback.progress = process;
			local_navigation_action->publishFeedback(local_navigation_feedback);

         usleep(20000);

      }

		local_navigation_action->setSucceeded();

	return true;
	}

	bool gps_navigation_action_callback(const dji_ros::gps_navigationGoalConstPtr& goal, gps_navigation_action_type* gps_navigation_action)
	{

	return true;
	}
	bool waypoint_navigation_action_callback(const dji_ros::waypoint_navigationGoalConstPtr& goal, waypoint_navigation_action_type* waypoint_navigation_action)
	{

	return true;
	}

	int init_actions(ros::NodeHandle &n)
	{
		task_action_type task_action(n, "DJI_ROS/task_action",boost::bind(&task_action_callback, _1, &task_action), false);
		

		local_navigation_action_type local_navigation_action(n,"DJI_ROS/local_navigation_action", boost::bind(&local_navigation_action_callback, _1, &local_navigation_action), false );

		local_navigation_action_ptr = (local_navigation_action_type*)malloc(sizeof(local_navigation_action_type));
		memcpy(local_navigation_action_ptr, &(local_navigation_action), sizeof(local_navigation_action_type));

		local_navigation_action_ptr->start();

		gps_navigation_action_type gps_navigation_action(n,"DJI_ROS/gps_navigation_action", boost::bind(&gps_navigation_action_callback, _1, &gps_navigation_action), false );

		waypoint_navigation_action_type waypoint_navigation_action(n,"DJI_ROS/waypoint_navigation_action", boost::bind(&waypoint_navigation_action_callback, _1, &waypoint_navigation_action), false );

		return 0;
	}
}
