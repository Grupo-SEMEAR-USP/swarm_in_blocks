var battery = document.getElementById('battery')
var telemetry_x = document.getElementById('telemetry_x')
var telemetry_y = document.getElementById('telemetry_y')
var telemetry_z = document.getElementById('telemetry_z')
var cpu = document.getElementById('cpu')
const fpv_disconnected = document.querySelector('#head > #fpv_disconnected');
const fpv_light = document.querySelector('#head > #fpv_light');

// set sleep function 
function sleep(milliseconds) {
    var start = new Date().getTime();
    for (var i = 0; i < 1e7; i++) {
        if ((new Date().getTime() - start) > milliseconds){
            break;
        }
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
            // sleep(5) //not working
            // console.log('x: ', x, 'y: ', y, 'z: ', z)
            telemetry_x.innerText = `${x}`
            telemetry_y.innerText = `${y}`
            telemetry_z.innerText = `${z}`
            listenerTelemetry.unsubscribe();
        });

        var listenerState = new ROSLIB.Topic({
            ros : ros,
            name : `/clover${id}/mavros/state`,
            messageType : 'mavros_msgs/State'
        })

        listenerState.subscribe((message) =>{
            mode = message.mode;
            isConnected = message.connected;
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
                document.getElementById('fpv__disconnected-light').classList.toggle('active');
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

function update_CPU(id) {
    if (id != "null") {
    cpu.innerText = `CLOVER ${id}`
     // code here
    }
}