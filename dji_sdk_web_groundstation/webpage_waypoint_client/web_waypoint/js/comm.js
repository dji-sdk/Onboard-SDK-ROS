var home_lat = 0;
var home_lon = 0;
var home_hei = 0;

var isExecuted = false;


function Communicator(socket) {

    //rosbridge config
    this.ros = new ROSLIB.Ros({
        url : socket.url
    });

    this.ros.on('connection', function(){
        console.log('Connected to websocket server.');
        $( '<div>Connected to websocket server.</div>' ).appendTo("#monitor");
    });
    this.ros.on('error', function(){
        console.log('Error connecting to websocket server: ', error);
        $( '<div>Error connecting to websocket server: '+error+'</div>' ).appendTo("#monitor");
    });
    this.ros.on('close', function(){
        console.log('Connection is closed!.');
        $( '<div>Connection is closed!</div>' ).appendTo("#monitor");
    });

    this.tid = 0;

    //action client
    this.web_wp_client = new ROSLIB.ActionClient({
        ros : this.ros,
        serverName : 'dji_sdk_web_groundstation/web_waypoint_receive_action',
        actionName : 'dji_sdk_web_groundstation/WebWaypointReceiveAction'
    });;

    //publisher
    this.ctrlTopic = new ROSLIB.Topic({
        ros : this.ros,
        name : 'dji_sdk_web_groundstation/map_nav_srv/ctrl',
        messageType : 'std_msgs/Bool'
    });
    this.cmdTopic = new ROSLIB.Topic({
        ros : this.ros,
        name : 'dji_sdk_web_groundstation/map_nav_srv/cmd',
        messageType : 'dji_sdk_web_groundstation/MapNavSrvCmd'
    });

    //subscriber
    this.rosListener = new ROSLIB.Topic({
        ros : this.ros,
        name : 'dji_sdk/global_position',
        messageType : 'dji_sdk/GlobalPosition'
    });
    this.rosListener.subscribe(function(msg) {
        home_lat = msg.latitude;
        home_lon = msg.longitude;
        home_hei = msg.height;
    });
}
Communicator.prototype.setNavigationMode = function() {

    var _msg = new ROSLIB.Message({
        data : true
    });

    console.log('Request to obtain control');
    this.ctrlTopic.publish(_msg);

};

Communicator.prototype.stopNavigationMode = function() {

    var _msg = new ROSLIB.Message({
        data : false
    });

    console.log('Release control');
    this.ctrlTopic.publish(_msg);

};

Communicator.prototype.uploadWayline = function() {

    isExecuted = false;

    if (my_planner.markerList.length < 2){
        alert("Please set 2 waypoints at least!");
        return;
    }

    // rosbridge config
    this.tid = new Date().getTime();
    var _msg = new ROSLIB.Message({
        cmdCode : "n".charCodeAt(0),
        tid : this.tid
    });
    console.log('Upload new waypointLine');
    this.cmdTopic.publish(_msg);

    var _waypointList = new Array();
    for(i = 0; i < my_planner.markerList.length; i++) {
        var _wp = new ROSLIB.Message({
            latitude : my_planner.markerList[i][1].point.lat,
            longitude : my_planner.markerList[i][1].point.lng,
            altitude : my_planner.markerList[i][2].alti,
            heading : 0,
            staytime : 0
        });
        _waypointList.push(_wp);
    }
    var goal_waypointList = {
        waypoint_list : _waypointList
    };
    var goal = new ROSLIB.Goal({
        actionClient : this.web_wp_client,
        goalMessage : {
            waypoint_list : goal_waypointList,
            tid : this.tid
        }
    });

    goal.on('feedback', function(feedback) {
        //console.log('Feedback: current stage ' + feedback.stage);
        var stageMsg = '';
        switch(feedback.stage) {
            case 0:
                stageMsg = 'waiting for waypointList'; break;
            case 1:
                stageMsg = 'waiting for start'; break;
            case 2:
                stageMsg = 'in progress'; break;
            case 3:
                stageMsg = 'paused'; break;
            case 4:
                stageMsg = 'canceled'; break;
        }

        var str = '<div>Feedback: current stage ' + feedback.stage + ' - '
            + stageMsg + '</div>'
            + '<div>Latitude progress: ' + feedback.latitude_progress + '%</div>'
            + '<div>Longitude progress: ' + feedback.longitude_progress + '%</div>'
            + '<div>Altitude progress: ' + feedback.altitude_progress + '%</div>'
            + '<div>Index progress: ' + feedback.index_progress + '</div>';
        $("#state-update").empty();
        $( str ).appendTo("#state-update");
    });

    goal.on('result', function(result) {
        if(result.result) {
            console.log('Execution Succeed!');
            alert('Execution Succeed!');
        } else {
            console.log('Last Execution Fail!');
            if(isExecuted) alert('Last Execution Fail!');
        }
    });

    goal.send();
};


Communicator.prototype.startWayline = function() {

    isExecuted = true;

    var _msg = new ROSLIB.Message({
        cmdCode : "s".charCodeAt(0),
        tid : this.tid
    });

    console.log('Start task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};

Communicator.prototype.cancelWayline = function() {

    //The function is not defined
    //this.web_wp_client.cancel();

    var _msg = new ROSLIB.Message({
        cmdCode : "c".charCodeAt(0),
        tid : this.tid
    });

    console.log('Cancel current task');
    this.cmdTopic.publish(_msg);

};

//TODO: have't implemented this function in ROS yet
Communicator.prototype.pauseWayline = function() {

    var _msg = new ROSLIB.Message({
        cmdCode : "p".charCodeAt(0),
        tid : this.tid
    });

    console.log('Pause task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};

//TODO: have't implemented this function in ROS yet
Communicator.prototype.continueWayline = function() {

    var _msg = new ROSLIB.Message({
        cmdCode : "r".charCodeAt(0),
        tid : this.tid
    });

    console.log('Resume task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};
