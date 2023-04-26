var battery = document.getElementById('battery')
var telemetry_x = document.getElementById('telemetry_x')
var telemetry_y = document.getElementById('telemetry_y')
var telemetry_z = document.getElementById('telemetry_z')
var cpu = document.getElementById('cpu')
var state = document.getElementById('state')
var disconnected = document.querySelector('#disconnected');

const pedal = document.querySelector('#pedal');
const pedal_feedback = document.querySelector('#pedal_feedback');
const fpv_disconnected = document.querySelector('#fpv_disconnected');
const fpv_light = document.querySelector('#fpv_light');

var listenerTelemetry ;

function update_Telemetry(id) {
    if (listenerTelemetry) {
        listenerTelemetry.unsubscribe();
    }

    console.log(typeof id)
    if (id != "null") {
        listenerTelemetry = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/local_position/pose`,
            messageType : 'geometry_msgs/PoseStamped'
        })

        listenerTelemetry.subscribe((message) =>{
            x = message.pose.position.x.toFixed(2);
            y = message.pose.position.y.toFixed(2);
            z = message.pose.position.z.toFixed(2);
            // console.log('x: ', x, 'y: ', y, 'z: ', z)
            telemetry_x.innerText = `${x}`
            telemetry_y.innerText = `${y}`
            telemetry_z.innerText = `${z}`            
            // listenerTelemetry.unsubscribe();
        });
        

        var listenerState = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/state`,
            messageType : 'mavros_msgs/State'

        })

        listenerState.subscribe((message) =>{
            mode = message.mode;
            isConnected = message.connected;
            state.innerText = `${isConnected}`
            // visual feedback for streaming header
            if (isConnected == true) {
                console.log('connected:', isConnected)
                fpv_disconnected.innerText = `Connected`
                fpv_disconnected.classList.add('fpv__connected')
                fpv_light.classList.add('fpv__connected-light')
                fpv_disconnected.classList.remove('fpv__disconnected')
                fpv_light.classList.remove('fpv__disconnected-light')
            }
            else {
                // if the class online it is there, remove it
                fpv_disconnected.innerText = `Disconnected`
                fpv_disconnected.classList.remove('fpv__connected')
                fpv_light.classList.remove('fpv__connected-light')
                fpv_disconnected.classList.add('fpv__disconnected')
                fpv_light.classList.add('fpv__disconnected-light')
            }
            listenerState.unsubscribe()
            // telemetry.innerText = `Telemetry:\nx: ${x};\ny: ${y};\nz: ${z};`
        });

    // telemetry.innerText = `TELEMETRY DATA FROM CLOVER ${id}`
    // code here
    }
}

update_Streaming => (id) => {    
    if (id != "null") {
        var listenerState = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/state`,
            messageType : 'mavros_msgs/State'

        })

        listenerState.subscribe((message) =>{
            mode = message.mode;
            isConnected = message.connected;
            if (isConnected == true) {
                isConnected = "Connected"
            }
            else {
                isConnected = "Disconnected"
            }    
            disconnected.innerText = `${isConnected}`
        });
    }
}

// listeners
function update_BatteryStatus(id) {
    if (id != "null") {
        var listenerBattery = new ROSLIB.Topic({
            ros:ros,
            name: `/clover${id}/mavros/battery`,
            messageType: 'sensor_msgs/BatteryState'
        })

        listenerBattery.subscribe(function(message){
            // list1 = message.cell_voltage
            // console.log(message.voltage)
            battery.innerText = `${message.voltage.toFixed(2)} V`
            
        });
    }

}

function update_CPU(id) {
    if (id != "null") {
    cpu.innerText = `CLOVER ${id}`
     // code here
    }
}

// When the user clicks on the div, add the "clicked" class to change the filter brightness
pedal.addEventListener('click', function() {
    pedal.classList.add('clicked');
    pedal_feedback.classList.add('clicked');
});

  // When the user clicks anywhere outside the div, remove the "clicked" class to restore the original filter brightness
document.addEventListener('click', function(event) {
    if (!pedal.contains(event.target)) {
        pedal.classList.remove('clicked');
        pedal_feedback.classList.remove('clicked');
    }
});