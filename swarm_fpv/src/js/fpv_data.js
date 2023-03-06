var battery = document.getElementById('battery')
var telemetry = document.getElementById('telemetry')
var cpu = document.getElementById('cpu')
var state = document.getElementById('state')



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
            battery.innerText = `Voltage: ${message.voltage.toFixed(2)} V`
            
        });
    }

}

var listenerTelemetry;

function update_Telemetry(id) {
    if(listenerTelemetry){
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
            telemetry.innerText = `Telemetry:\nx: ${x};\ny: ${y};\nz: ${z};`
            //setInterval(listenerTelemetry.subscribe, 10);
            //listenerTelemetry.unsubscribe()

            
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
                                listenerState.unsubscribe()
            // telemetry.innerText = `Telemetry:\nx: ${x};\ny: ${y};\nz: ${z};`
        });

    // telemetry.innerText = `TELEMETRY DATA FROM CLOVER ${id}`
    // code here
    }
}

function update_CPU(id) {
    if (id != "null") {
    cpu.innerText = `CPU DATA FROM CLOVER ${id}`
     // code here
    }
}