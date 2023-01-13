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
    
    // movement methods
    // left joystick
        this.KeyW =  function () {
            updateContent("up")
            console.log("aa");
            pubMsg("W FOI PRESSIONADO")
        };
        this.KeyA = () => {
            updateContent("yaw+")
        },
        this.KeyS = () => {
            updateContent("down")
        },
        this.KeyD = () => {
            updateContent("yaw-")
        },

        // right joystick
        this.ArrowUp = () => {
            updateContent("foward")
        },
        this.ArrowDown = () => {
            updateContent("backward")
        },
        this.ArrowLeft = () => {
            updateContent("left")
        },
        this.ArrowRight = () => {
            updateContent("right")
        },
        this.Stop = () => {
            updateContent("stop")
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