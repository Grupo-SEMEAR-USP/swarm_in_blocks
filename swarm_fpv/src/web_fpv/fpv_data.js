var battery = document.getElementById('battery')
var telemetry = document.getElementById('telemetry')
var cpu = document.getElementById('cpu')
var state = document.getElementById('state')

var disconnected = document.querySelector('#disconnected');
const pedal = document.querySelector('#pedal');
const pedal_feedback = document.querySelector('#pedal_feedback');


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
            battery.innerText = `Voltage: ${message.voltage} V`
            
        });
    }

}

function update_Telemetry(id) {
    console.log(typeof id)
    if (id != "null") {
        var listenerTelemetry = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/local_position/pose`,
            messageType : 'geometry_msgs/PoseStamped'
        })

        listenerTelemetry.subscribe((message) =>{
            x = message.pose.position.x.toFixed(2);
            y = message.pose.position.y.toFixed(2);
            z = message.pose.position.z.toFixed(2);
            telemetry.innerText = `Telemetry:\nx: ${x};\ny: ${y};\nz: ${z};`
        });


        var listenerState = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/state`,
            messageType : 'mavros_msgs/State'

        })

        listenerState.subscribe((message) =>{
            mode = message.mode;
            isConnected = message.connected;
            state.innerText = `Mode: ${mode}
                                Connected: ${isConnected}`
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
            disconnected.innerText = `Mode: ${mode}
                                ${isConnected}`
        });
    }
}

function update_CPU(id) {
    if (id != "null") {
    cpu.innerText = `CPU DATA FROM CLOVER ${id}`
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