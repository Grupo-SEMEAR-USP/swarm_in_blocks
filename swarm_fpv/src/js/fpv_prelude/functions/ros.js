var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    document.getElementById("paragraph").innerHTML = "Connected";
  });

  ros.on('error', function(error) {
    document.getElementById("paragraph").innerHTML = "Error";
  });

  ros.on('close', function() {
    document.getElementById("paragraph").innerHTML = "Closed";
  });

// topics

var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '',
    messageType : 'geometry_msgs/TwistStamped'
});

var cmdPos = new ROSLIB.Topic({
    ros : ros,
    name : '',
    messageType : 'geometry_msgs/PoseStamped'
});

// messages

var twistStamped = new ROSLIB.Message ({
    header : {
        seq : 0,
        stamp :{
            secs : 0,
            nsecs : 0,
        },
        frame_id : '',
    },
    
    twist : {
        linear : {
            x : 0,
            y : 0,
            z : 0
    },
        angular : {
            x : 0,
            y : 0,
            z : 0
    }}
});

var poseStamped = new ROSLIB.Message ({
    header : {
        seq : 0,
        stamp :{
            secs : 0,
            nsecs : 0,
        },
        frame_id : '',
    },

    pose : {
        position: {
            x : 0,
            y : 0,
            z : 0
        },
        orientation : {
            x : 0,
            y : 0,
            z : 0,
            w : 0 
        }
    }
})

/////////////////////// INFO
/*
LOCAL POSITION:
/clover0/mavros/local_position/pose 
Type: geometry_msgs/PoseStamped

header : 
    seq :
    stamp :
        secs :
        nsecs :
    frame_id :

pose : 
    position :
        x
        y
        z
    orientation :
        x
        y
        z
        w

================
SET POINT POSITION
/clover0/mavros/setpoint_position/local 
Type: geometry_msgs/PoseStamped

================
SET POINT VELOCITYs
/clover0/mavros/setpoint_velocity/cmd_vel
Type: geometry_msgs/TwistStamped

header : 
    seq :
    stamp :
        secs :
        nsecs :
    frame_id :

twist : 
    linear :
        x
        y
        z
    angular : 
        x
        y
        z
*/
