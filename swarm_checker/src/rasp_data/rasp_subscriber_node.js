
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


  var cloversConnected = [];

  update_RaspData()

  // setTimeout(function() {
  //       update_RaspData()
  //     }, 200);
  

  var listenerRasp;

  function update_RaspData() {

    console.log("rodando");

    listenerRasp = new ROSLIB.Topic({
      ros: ros,
      name: `/clover_1/cpu_usage`,
      messageType: 'rasp_pkg/raspData'
    });

    console.log("Tópico criado");

    listenerRasp.subscribe((message) => {

      console.log(message)

      var cpu_usage = message.cpu_usage_percent;
      var cpu_freq_current = message.cpu_freq_current;
      var cpu_freq_max = message.cpu_freq_min;
      var cpu_freq_min = message.cpu_freq_max;
      var virtualMemory_percentage = message.virtualMemory_percent;
      var process_usage_list = message.process_usage_list;
      var process_name_list = message.process_name_list;
      
      
    
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
      document.getElementById("process_list").innerHTML = 'Most used processes: ' + '<br>' +  process_list_string;


    });


    
  }
  