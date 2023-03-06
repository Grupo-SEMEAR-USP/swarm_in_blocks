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
  Connected = message.connected_ids
  // console.log(cloversConnected)
  listSwarm.unsubscribe()
});

function show_info(){
	var elemento = document.querySelector("#clovers_info > .cards");

	function duplicaElemento(){
		var clonado = elemento.cloneNode(true);
		document.querySelector("#clovers_info").appendChild(clonado);
	}

	document.querySelector("#clovers_info").innerHTML = "";

	for(let id in Connected){
		duplicaElemento();
		update_cards(id)
	}
	document.getElementById('clovers_info').classList.toggle('active');
	document.getElementById('mic').classList.toggle('active');
}


var listRasp;
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
        var net_data_max = message.net_data_max;
      
        var process_list_string = '';
        for (var i = 0; i < process_usage_list.length; i++) {
          process_list_string += 'Process: ' + process_name_list[i] + ' Cpu usage: ' + process_usage_list[i].toFixed(2) + '% <br>';
        }

        // Update the HTML elements with the new values
        document.getElementById("cpu_usage_percent").innerHTML = 'Cpu usage: ' + cpu_usage.toFixed(2) + '%';
        document.getElementById("cpu_freq_current").innerHTML = 'Cpu current frequency: ' + cpu_freq_current.toFixed(2) + 'Hz';
        document.getElementById("cpu_freq_min").innerHTML = 'Cpu min frequency: ' + cpu_freq_min.toFixed(2) + 'Hz';
        document.getElementById("cpu_freq_max").innerHTML = 'Cpu max frequency: ' + cpu_freq_max.toFixed(2) + 'Hz';
        document.getElementById("virtualMemory_percent").innerHTML = 'Virtual memory usage: ' + virtualMemory_percentage.toFixed(2) + '%';
        document.getElementById("cpu_temperature").innerHTML = 'Cpu temperature: ' + cpu_temperature.toFixed(2) + ' celsius';
        document.getElementById("bytes").innerHTML = 'Bytes sent: '+ bytes_sent + ' Bytes received: ' + bytes_recv;
        document.getElementById("packets").innerHTML = 
        document.getElementById("process_list").innerHTML = 'Most used processes: ' + '<br>' +  process_list_string;
      });
    }
  }


  


