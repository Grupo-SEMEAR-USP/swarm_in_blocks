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
const closeInputModalLand = document.querySelector(".close-input-land");
const fade_land = document.querySelector("#fade_land");

const land = document.querySelector("#land");

const toggleModalLand = () => {
	land.classList.toggle("hide");
	fade_land.classList.toggle("hide");
};

[openModalLand, closeModalLand, fade_land, closeInputModalLand].forEach((el) => {
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
	document.querySelector("#clovers_info").innerHTML = "";

	for(let id in list){
		new_cards(id);
	}
	document.getElementById('clovers_info').classList.toggle('active');
	document.getElementById('mic').classList.toggle('active');
}


var elemento = document.querySelector("#cards"); //card

//puxa os dados para o card - recebe o id do card
function new_cards(id) {
    if (id != "null") {

      var clonado = elemento.cloneNode(true); //card - igual sempre então é clonado
      document.getElementById("clovers_info").appendChild(clonado);
      document.querySelector(".cards > #card_content > #card_title > .clover_name > #id").innerHTML = "Clover "+id;

      var listRasp = new ROSLIB.Topic({
        ros: ros,
        name: `/clover_${id}/cpu_usage`,
        messageType: 'rasp_pkg/raspData'
      });

      listRasp.subscribe((message) => {
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
        document.querySelector(".cards > #card_content > #card_title > .clover_name > #net_data_adress").innerHTML = net_data_adress.toFixed(2);
        document.querySelector(".cards > #card_content > #card_title > .clover_name > #cpu_usage_percent").innerHTML = cpu_usage.toFixed(2) + '%';
        document.querySelector(".cards > #card_content > #card_title > .clover_name > #virtualMemory_percent").innerHTML = virtualMemory_percentage.toFixed(2) + '%';
        document.querySelector(".cards > #card_content > #card_title > .clover_name > #cpu_temperature").innerHTML = cpu_temperature.toFixed(2) + '°C';
      });
      clonado.className = ".cards_"+id;
    }
  }


// Topiclist

function getTopics() {
    var topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });

    var request = new ROSLIB.ServiceRequest();

    topicsClient.callService(request, function(result) {
		console.log("Getting topics...");
		
		teste = result.topics.join();
		

		var list_topics = teste.replace(/,/g, "<br>");
		console.log(list_topics);

		document.getElementById("desc_topics").innerHTML = list_topics;sss
    });
};




// --------------------------Terminal

function open_terminal(){
	document.getElementById('terminal').classList.toggle('active');
  document.getElementById('open-terminal').classList.toggle('active');
  document.getElementById('open-button-terminal').classList.toggle('active');
}


  


