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
    
    // movement methods
    // left joystick
        this.KeyW =  function () {
            updateContent("up");
            // pubMsg("W FOI PRESSIONADO")
            this.setVelocity.callService(this.setRequest(vz=vel, vy=0, vx=0, yr=0), function (result) {
                console.log(result)
            });
        };
        this.KeyA = () => {
            updateContent("yaw+");
            this.setVelocity.callService(this.setRequest(vz=0, vy=0, vx=0, yr=vel, yaw=NaN), function (result) {
                console.log(result)
            });
            // pubMsg("A FOI PRESSIONADO")
        },
        this.KeyS = () => {
            updateContent("down");
            this.setVelocity.callService(this.setRequest(vz=-vel, vy=0, vx=0, yr=0), function (result) {
                console.log(result)
            });
            // pubMsg("S FOI PRESSIONADO")
        },
        this.KeyD = () => {
            updateContent("yaw-");
            this.setVelocity.callService(this.setRequest(vz=0, vy=0, vx=0, yr=-vel, yaw=NaN), function (result) {
                console.log(result)
            });
            // pubMsg("D FOI PRESSIONADO")
        },

        
        // right joystick
        this.ArrowUp = () => {
            updateContent("foward")
            this.setVelocity.callService(this.setRequest(vz=0, vy=0, vx=vel, yr=0), function (result) {
                console.log(result)
            });
        },
        this.ArrowDown = () => {
            updateContent("backward")
            this.setVelocity.callService(this.setRequest(vz=0, vy=0, vx=-vel, yr=0), function (result) {
                console.log(result)
            });
        },
        this.ArrowLeft = () => {
            updateContent("left")
            this.setVelocity.callService(this.setRequest(vz=0, vy=vel, vx=0, yr=0), function (result) {
                console.log(result)
            });
        },
        this.ArrowRight = () => {
            updateContent("right")
            this.setVelocity.callService(this.setRequest(vz=0, vy=-vel, vx=0, yr=0), function (result) {
                console.log(result)
            });
        },
        this.Stop = () => {
            updateContent("stop")
            this.setVelocity.callService(this.setRequest(vx=0, vy=0, vz=0), function (result) {
                console.log(result)
            });
        }
}

// var drone = new Drone(0, "clover0");
//drone['KeyW']()

window.addEventListener("keydown",  // TO DO => Array of elements that accepts multiple keys at the sime time
(event) => {
    let key = event.code;
    if (event.repeat == true) {
        return;
    } else if (acceptedValues.includes(key)) {
    
    console.log(key)
    drone[String(key)]();
    
}
}
,true)

window.addEventListener("keyup", (event) => {
    drone['Stop']();
})