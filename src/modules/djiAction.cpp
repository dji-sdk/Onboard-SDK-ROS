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
		task_action_ptr->publishFeedback(task_action_feedback);
		task_action_ptr->setSucceeded();
		

		return true;

	}

	bool local_navigation_action_callback(const dji_ros::local_navigationGoalConstPtr& goal, local_navigation_action_type* local_navigation_action)
	{
		/*IMPORTAN*/
		/*
			Although there has declared a pointer local_navigation_action as function parameter,
			it is the local_navigation_action_ptr that we should use.
			If local_navigation_action is used instead, there will be a runtime sengmentation fault.
			I tried to find a way to fix it / make it better,
			but I can neither remove the function parameter declaration nor use it.

			so interesting
		*/

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

		int x_progress = 0; 
		int y_progress = 0; 
		int z_progress = 0; 
		while (x_progress < 100 || y_progress < 100 || z_progress <100) {

         user_ctrl_data.roll_or_x = dst_x - dji_variable::local_position.x;
         user_ctrl_data.pitch_or_y = dst_y - dji_variable::local_position.y;

         DJI_Pro_Attitude_Control(&user_ctrl_data);

			det_x = (100* (dst_x - dji_variable::local_position.x))/dis_x;
			det_y = (100* (dst_y - dji_variable::local_position.y))/dis_y;
			det_z = (100* (dst_z - dji_variable::local_position.z))/dis_z;

			x_progress = 100 - (int)det_x;
			y_progress = 100 - (int)det_y;
			z_progress = 100 - (int)det_z;

			local_navigation_feedback.x_progress = x_progress;
			local_navigation_feedback.y_progress = y_progress;
			local_navigation_feedback.z_progress = z_progress;
			local_navigation_action_ptr->publishFeedback(local_navigation_feedback);

         usleep(20000);

      }

		local_navigation_action_ptr->setSucceeded();

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
		task_action_ptr = new task_action_type(n, "DJI_ROS/task_action",boost::bind(&task_action_callback, _1, task_action_ptr), false);
		//task_action_ptr->start();

		local_navigation_action_ptr = new local_navigation_action_type(n,"DJI_ROS/local_navigation_action", boost::bind(&local_navigation_action_callback, _1, local_navigation_action_ptr), false );
		local_navigation_action_ptr->start();

		gps_navigation_action_ptr = new gps_navigation_action_type(n,"DJI_ROS/gps_navigation_action", boost::bind(&gps_navigation_action_callback, _1, gps_navigation_action_ptr), false );
		gps_navigation_action_ptr->start();

		waypoint_navigation_action_ptr = new waypoint_navigation_action_type(n,"DJI_ROS/waypoint_navigation_action", boost::bind(&waypoint_navigation_action_callback, _1, waypoint_navigation_action_ptr), false );
		waypoint_navigation_action_ptr->start();

		return 0;
	}
}
