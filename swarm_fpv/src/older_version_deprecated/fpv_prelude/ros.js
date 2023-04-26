// setting up connection
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log("Connected");
});

ros.on('error', () => {
    console.log("Error");
});

ros.on('close', () => {
    console.log("Closed");
});

// communication
// var topic = new ROSLIB.Topic({
//     ros : ros,
//     name : '/listener',
//     messageType : 'std_msgs/String'
// });


// topic.subscribe( (message) => {
//     console.log("Received on " + topic.name + " the message: " + message.data);
//     //listener.unsubscribe();
// })


// function pubMsg(message) {
//     var msg = new ROSLIB.Message({
//         data : message
//     });

//     topic.publish(msg);
// }


