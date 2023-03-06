// const rosnodejs = require('rosnodejs');

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


// rosnodejs.initNode('rasp_subscriber').then(() => {

//   rosnodejs.getTopics((topics) => {
//     let cloversConnecteds = topics.filter(topic => {
//       return topic.indexOf('/clover') === 0 && topic.indexOf('/rasp_usage') !== -1;
//     }).length;  
//   })

//   //
//   // const cloversConnecteds = rosnodejs.getParam('');
//   console.log("Número de clovers publicando: ", cloversConnecteds)
  
//   for (let id = 1; id <= cloversConnecteds; id++) {

//     update_RaspData(id);
//   }

//   var listenerRasp;
// });

var listenerSwarm;
var cloversConnected = [];
  
listenerSwarm = new ROSLIB.Topic({
  ros: ros,
  name: `swarm_checker/state`,
  messageType: 'swarm_checker/SwarmState'

});

listenerSwarm.subscribe(function(message) {
  // document.getElementById('connected_ids')
  cloversConnected = message.connected_ids
  // console.log(cloversConnected)
  addIds()
  listenerSwarm.unsubscribe()
});


var listenerRasp;

const selectCloverId = document.getElementById("selectCloverId");

selectCloverId.addEventListener("change", function() {
  let id = this.value;
  update_RaspData(id);
});



  function update_RaspData(id) {

    if (id != "null") {

      listenerRasp = new ROSLIB.Topic({
        ros: ros,
        name: `/clover_${id}/cpu_usage`,
        messageType: 'rasp_pkg/raspData'
      });

      // console.log("Tópico criado");

      listenerRasp.subscribe((message) => {

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
  
  function addIds() {
    var element = document.getElementById('selectCloverId')
  
    for (let id in cloversConnected) {
      var child = document.createElement('option')
      child.textContent = `Clover ${list[id]}`
      child.setAttribute('value', `${list[id]}`)
      element.appendChild(child)
    }
  }

  