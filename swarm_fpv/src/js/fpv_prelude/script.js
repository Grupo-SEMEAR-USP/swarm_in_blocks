let element = document.getElementById("output")
let acceptedValues = ["KeyW", "KeyS", "KeyA", "KeyD",
                      "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"]

// deals with the text updating (for debugging purposes)
function updateContent(text) {
    element.innerText = text

    // do ROS stuff here
}

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
        this.setRequest = (vx=0, vy=0, vz=0, ) => {
            let request = new ROSLIB.ServiceRequest({
                vx : vx,
                vy : vy,
                vz : vz,
                yaw : 0,
                yaw_rate : 0,
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
            this.setVelocity.callService(this.setRequest(vz=5), function (result) {
                console.log(result)
            });
        };
        this.KeyA = () => {
            updateContent("yaw+");
            // pubMsg("A FOI PRESSIONADO")
        },
        this.KeyS = () => {
            updateContent("down");
            // pubMsg("S FOI PRESSIONADO")
        },
        this.KeyD = () => {
            updateContent("yaw-");
            // pubMsg("D FOI PRESSIONADO")
        },

        // right joystick
        this.ArrowUp = () => {
            updateContent("foward")
            this.setVelocity.callService(this.setRequest(vx=2), function (result) {
                console.log(result)
            });
        },
        this.ArrowDown = () => {
            updateContent("backward")
            this.setVelocity.callService(this.setRequest(vx=-2), function (result) {
                console.log(result)
            });
        },
        this.ArrowLeft = () => {
            updateContent("left")
            this.setVelocity.callService(this.setRequest(vy=-2), function (result) {
                console.log(result)
            });
        },
        this.ArrowRight = () => {
            updateContent("right")
            this.setVelocity.callService(this.setRequest(vy=2), function (result) {
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

var drone = new Drone(0, "clover0");
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