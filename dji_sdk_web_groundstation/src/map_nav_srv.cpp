#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk_web_groundstation/WebWaypointReceiveAction.h>
#include <dji_sdk_web_groundstation/MapNavSrvCmd.h>
#include <dji_sdk/WaypointNavigationAction.h>

using namespace actionlib;

typedef dji_sdk_web_groundstation::WebWaypointReceiveAction Action_t;
typedef dji_sdk_web_groundstation::WebWaypointReceiveGoal Goal_t;
typedef dji_sdk_web_groundstation::WebWaypointReceiveGoalConstPtr GoalConstPtr_t;
typedef dji_sdk_web_groundstation::WebWaypointReceiveFeedback Feedback_t;
typedef dji_sdk_web_groundstation::WebWaypointReceiveResult Result_t;

typedef dji_sdk::WaypointNavigationAction WPAction_t;

SimpleActionServer<Action_t>* asPtr_;

DJIDrone* drone;

uint8_t cmdCode_ = 0;
uint8_t stage_ = 0;
uint64_t tid_ = 0;
uint64_t cmdTid_ = 1;

uint8_t lat_p_; //latitude_progress
uint8_t lon_p_; //longitude_progress
uint8_t alt_p_; //altitude_progress
uint8_t idx_p_; //index_progress


void wp_feedbackCB(const dji_sdk::WaypointNavigationFeedbackConstPtr& fb) {
    lat_p_ = fb->latitude_progress;
    lon_p_ = fb->longitude_progress;
    alt_p_ = fb->altitude_progress;
    idx_p_ = fb->index_progress;
}

void goalCB() {
    Feedback_t fb;
    Result_t rslt;

    cmdCode_ = 0; //eliminate effect of last task
    Goal_t newGoal = *( asPtr_->acceptNewGoal() );
    ROS_INFO_STREAM( "Received goal: \n" << newGoal << "\n" );

    //********** stage code **********
    //*  0: waiting for waypointList
    //*  1: waiting for start
    //*  2: in progress
    //*  3: paused
    //*  4: canceled
    //********************************

    //check last task stage
    if(stage_ == 2) {
        rslt.result = false;
        asPtr_->setAborted(rslt, "Last task is in progress!");
        stage_ = 0;
        tid_ = 0;
        return;
    }
    if(stage_ == 3) {
        rslt.result = false;
        asPtr_->setAborted(rslt, "Last task is paused!");
        stage_ = 0;
        tid_ = 0;
        return;
    }

    tid_ = newGoal.tid;
    dji_sdk::WaypointList wpl = newGoal.waypoint_list;
    stage_ = 1;

    while(ros::ok()) {
        if(cmdCode_ == 'c') { //"c" for cancel
            //cmdCode_ = 0; //eliminate effect of next task
            ROS_INFO("Cancel task with tid %llu", tid_);
            stage_ = 4;
            fb.stage = stage_;
            asPtr_->publishFeedback(fb);
        } else if(tid_ == cmdTid_){
            if(cmdCode_ == 's' && stage_ == 1) { //"s" for start
                ROS_INFO("Start task with tid %llu", tid_);
                stage_ = 2;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
            if(cmdCode_ == 'p' && stage_ == 2) { //"p" for pause
                ROS_INFO("Pause task with tid %llu", tid_);
                stage_ = 3;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
            if(cmdCode_ == 'r' && stage_ == 3) { //"r" for resume
                ROS_INFO("Resume task with tid %llu", tid_);
                stage_ = 2;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
        } else {
            if(cmdCode_ == 'n') { //"n" for newer waypointLine arrived
                ROS_INFO("A latest task arrived.");
                ROS_INFO("Set task(if any) with tid %llu preemted", newGoal.tid);
                stage_ = 4;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
            }
        }

        /*ROS_INFO_ONCE("Stage: %d", stage_);
        ROS_INFO_ONCE("CmdCode: %d", cmdCode_);
        ROS_INFO_ONCE("tid: %llu", tid_);*/

        bool isFinished; //flag for task result
        int cnt; //feedback count
        switch(stage_) {
            case 0: //"0" for waiting for waypointList
                rslt.result = false;
                asPtr_->setAborted(rslt);
                ROS_INFO("The task is aborted for no waypointList received.");
                stage_ = 0;
                tid_ = 0;
                return;
            case 1: //"1" for waiting for start
            case 3: //"3" for paused
                continue;
            case 2: //"2" for in progress
                //wpClientPtr_->waitForServer();
                drone->takeoff();
                drone->waypoint_navigation_wait_server();
                
                //wpClientPtr_->sendGoal(wpGoal, 
                drone->waypoint_navigation_send_request(wpl,
                    SimpleActionClient<WPAction_t>::SimpleDoneCallback(), 
                    SimpleActionClient<WPAction_t>::SimpleActiveCallback(), 
                    &wp_feedbackCB
                );
                ROS_DEBUG("[DEBUG] The goal is sent!");

                //wait 1200*0.25s=300s until the task finishes
                cnt = 1200;
                while(ros::ok() && cnt--) {
                    fb.latitude_progress = lat_p_;
                    fb.longitude_progress = lon_p_;
                    fb.altitude_progress = alt_p_;
                    fb.index_progress = idx_p_;
                    asPtr_->publishFeedback(fb);
                    ROS_DEBUG_COND(cnt==1199, "[DEBUG] cmdCode_: %c", cmdCode_);
                    if(cmdCode_ == 'c') {
                        //cmdCode_ = 0; //eliminate effect of next task
                        rslt.result = false;
                        asPtr_->setAborted(rslt);
                        ROS_INFO("The task is canceled while executing.");
                        stage_ = 0;
                        tid_ = 0;
                        //wpClientPtr_->cancelGoal();
                        drone->waypoint_navigation_cancel_current_goal();
                        return;
                    }
                    isFinished = drone->waypoint_navigation_wait_for_result(); 
                }
                if(isFinished) {
                    ROS_INFO("Action finished: %s", 
                        drone->waypoint_navigation_get_state().toString().c_str()
                    );
                    rslt.result = true;
                    asPtr_->setSucceeded(rslt);
                } else {
                    ROS_INFO("The task cannot finish before the time out.");
                    rslt.result = false;
                    asPtr_->setAborted(rslt);
                }
                stage_ = 0;
                tid_ = 0;
                return;
            case 4: //"4" for canceled
                ROS_INFO("The task is canceled.");
                stage_ = 0;
                tid_ = 0;
                rslt.result = false;
                asPtr_->setPreempted(rslt);
                drone->waypoint_navigation_cancel_all_goals();
                return;
        }
    }
}

//TODO: preemptCB
void preemptCB() {
    ROS_INFO("Hey! I got preempt!");
}

void cmdCB(const dji_sdk_web_groundstation::MapNavSrvCmdConstPtr& msg) {
    ROS_INFO("Received command \"%c\" of tid %llu", msg->cmdCode, msg->tid);
    cmdCode_ = msg->cmdCode;
    cmdTid_ = msg->tid;
}

//TRUE for request control and FALSE for release control
void ctrlCB(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data)
        ROS_INFO("Request to obtain control");
    else
        ROS_INFO("Release control");

    drone->sdk_permission_control(msg->data);
    
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_nav_srv");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    
    //web_waypoint_receive action server
    asPtr_ = new SimpleActionServer<Action_t>(
        nh, 
        "dji_sdk_web_groundstation/web_waypoint_receive_action", 
        false
    );

    //command subscribers
    ros::Subscriber sub1 = nh.subscribe("dji_sdk_web_groundstation/map_nav_srv/cmd", 1, cmdCB);
    ros::Subscriber sub2 = nh.subscribe("dji_sdk_web_groundstation/map_nav_srv/ctrl", 1, ctrlCB);

    asPtr_->registerGoalCallback(&goalCB);
    asPtr_->registerPreemptCallback(&preemptCB);
    asPtr_->start();

    //use multi thread to handle the subscribers.
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
