var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });
  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });
  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

var disconnected = document.querySelector('#disconnected');
const fpv_disconnected = document.querySelector('#fpv_disconnected');
const fpv_light = document.querySelector('#fpv_light');


const openModalLand = document.querySelector("#open-land");
const closeModalLand = document.querySelector(".close-land");
const fade_land = document.querySelector("#fade_land");

const land = document.querySelector("#land");

const toggleModalLand = () => {
	land.classList.toggle("hide");
	fade_land.classList.toggle("hide");
};

[openModalLand, closeModalLand, fade_land].forEach((el) => {
	el.addEventListener("click", () => toggleModalLand());
});

const openModalSeg = document.querySelector(".open-seg");
const closeModalSeg = document.querySelector(".close-seg");
const closeInputModalSeg = document.querySelector(".close-input-seg");
const fade_seg = document.querySelector("#fade_seg");

const seg = document.querySelector("#seg");

const toggleModalSeg = () => {
	seg.classList.toggle("hide");
	fade_seg.classList.toggle("hide");
};

[openModalSeg, closeModalSeg, fade_seg, closeInputModalSeg].forEach((el) => {
	el.addEventListener("click", () => toggleModalSeg());
});


$(document).ready(function() {
	$(".desc").hide();
	$('input[type="radio"]').click(function() {
		var test = $(this).val();
		$(".desc").hide();
		$("#"+test).show();
	});
});

function show(){
	document.getElementById('clover').classList.toggle('active');
	document.getElementById('open-land').classList.toggle('active');
}


const openModalTopic = document.querySelector(".open-topic");
const closeModalTopic = document.querySelector(".close-topic");
const fade_topic = document.querySelector("#fade_topic");

const topic = document.querySelector("#topic_list");

const toggleModalTopic = () => {
	topic.classList.toggle("hide");
	fade_topic.classList.toggle("hide");
};

[openModalTopic, closeModalTopic, fade_topic].forEach((el) => {
	el.addEventListener("click", () => toggleModalTopic());
});

//clovers_info

var listSwarm;
var Connected = [];
  
listSwarm = new ROSLIB.Topic({
  ros: ros,
  name: `swarm_checker/state`,
  messageType: 'swarm_checker/SwarmState'

});

listSwarm.subscribe(function(message) {
  // document.getElementById('connected_ids')
  list = message.all_clovers_ids
  listSwarm.unsubscribe()
});

function show_info(){
	var elemento = document.querySelector("#clovers_info > .cards");

	function duplicaElemento(){
		var clonado = elemento.cloneNode(true);
		document.querySelector("#clovers_info").appendChild(clonado);
	}

	document.querySelector("#clovers_info").innerHTML = "";

	//for(let id in Connected)
	for(let id in list){
		duplicaElemento();
		update_cards(id);
	}
	document.getElementById('clovers_info').classList.toggle('active');
	document.getElementById('mic').classList.toggle('active');
}

//puxa os dados para o card - recebe o id do card
function update_cards(id) {
    if (id != "null") {
      listRasp = new ROSLIB.Topic({
        ros: ros,
        name: `/clover_${id}/cpu_usage`,
        messageType: 'rasp_pkg/raspData'
      });

      listRasp.subscribe((message) => {

        console.log(message)

        var cpu_usage = message.cpu_usage_percent;
        var cpu_freq_current = message.cpu_freq_current;
        var cpu_freq_max = message.cpu_freq_min;
        var cpu_freq_min = message.cpu_freq_max;
        var virtualMemory_percentage = message.virtualMemory_percent;
        var process_usage_list = message.process_usage_list;
        var process_name_list = message.process_name_list;
        var cpu_temperature = message.cpu_temperature;
        var bytes_sent = message.bytes_sent;
        var bytes_recv = message.bytes_recv;
        var packets_sent = message.packets_sent;
        var packets_recv = message.packets_recv;
        var net_data_adress = message.net_data_adress;
      
        var process_list_string = '';
        for (var i = 0; i < process_usage_list.length; i++) {
          process_list_string += 'Process: ' + process_name_list[i] + ' Cpu usage: ' + process_usage_list[i].toFixed(2) + '% <br>';
        }

        // Update the HTML elements with the new values
        document.getElementById("cpu_usage_percent").innerHTML = cpu_usage.toFixed(2) + '%';
        document.getElementById("cpu_freq_current").innerHTML = cpu_freq_current.toFixed(2) + 'Hz';
        document.getElementById("cpu_freq_min").innerHTML = cpu_freq_min.toFixed(2) + 'Hz';
        document.getElementById("cpu_freq_max").innerHTML = cpu_freq_max.toFixed(2) + 'Hz';
        document.getElementById("virtualMemory_percent").innerHTML = virtualMemory_percentage.toFixed(2) + '%';
        document.getElementById("cpu_temperature").innerHTML = cpu_temperature.toFixed(2) + '°C';
		document.getElementById("net_data_adress").innerHTML = net_data_adress;



        document.getElementById("bytes").innerHTML = 'Bytes sent: '+ bytes_sent + ' Bytes received: ' + bytes_recv;
        document.getElementById("packets").innerHTML = 
        document.getElementById("process_list").innerHTML = 'Most used processes: ' + '<br>' +  process_list_string;
      });

	  var listState = new ROSLIB.Topic({
		ros : ros,
		name : `/clover${id}/mavros/state`,
		messageType : 'mavros_msgs/State'

		})

		listState.subscribe((message) =>{
		mode = message.mode;
		Connected = message.connected;
		state.innerText = `${Connected}`
		// visual feedback for streaming header
		if (Connected == true) {
			console.log('connected:', Connected)
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
		listState.unsubscribe()
		// telemetry.innerText = `Telemetry:\nx: ${x};\ny: ${y};\nz: ${z};`
	});


    }
  }


  


