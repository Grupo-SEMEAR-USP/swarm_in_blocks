let element = document.getElementById("output")
let acceptedValues = ["KeyW", "KeyS", "KeyA", "KeyD",
                      "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"]

// deals with the text updating (for debugging purposes)
function updateContent(text) {
    element.innerText = text

    // do ROS stuff here
}
var vel = 2;
// objetct
function Drone(id, name) {
        // ROS parts
        this.id = id;
        this.name = name;
        this.body = `body${id}`;

        // this.setParamRos = () => {
            // services set velocity
            this.setVelocity = new ROSLIB.Service({
                ros : ros,
                name : `/clover${id}/set_velocity`,
                serviceType : 'clover/SetVelocity'
            });


        // };

        // protopipo de request
        this.setRequest = (vz=0, vy=0, vx=0, yr=0, yaw=0) => {
            let request = new ROSLIB.ServiceRequest({
                vx : vx,
                vy : vy,
                vz : vz,
                yaw : yaw,
                yaw_rate : yr,
                frame_id : this.body,
                auto_arm : true
                // --- response:
                // bool success
                // string message

            });

            return request
        };
}


// rectangle.addEventListener("keydown",
// (event) => {
//     let key = event.code;
//     console.log("RECTANGLE")
// })


// $("#listener").hover(function() {
//     this.focus();
// }, function() {
//     this.blur();
// }).keydown(function(e) {
//     alert(e.keyCode);
// });

