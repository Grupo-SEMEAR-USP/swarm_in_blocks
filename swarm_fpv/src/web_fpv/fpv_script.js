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
  // ----------------------
  // var lista = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
  var list = [];
  // proportion 640x480
  var width, height;
  width = 510;
  height = 360; // standart sizes (may change)
  var drone;
  
  function define_id(id_c) {
    id = id_c;
  }
  
  function reload_subscriber(element) {
    update_ImageListener(element.value); // updates the visualization of the camera and the control services

    // update_BatteryStatus(element.value); // battery status
    // update_Telemetry(element.value); // position and velocity
    // update_CPU(element.value)

  }
  
  function increaseImageSize() {
    let img = document.getElementById("image_subscriber")
    
    width *= 1.1
    height *= 1.1

    img.setAttribute('width', `${width}`)
    img.setAttribute('height', `${height}`)

    // nome.width = nome.width + 850;
    // nome.height = nome.height + 850;
  }
  function decreaseImageSize() {
    let img = document.getElementById("image_subscriber")
    
    width *= 0.9
    height *= 0.9

    img.setAttribute('width', `${width}`)
    img.setAttribute('height', `${height}`)

    
    // nome.width = 850
    // nome.height = 850;
  }
  
  
  var listener;
  function update_ImageListener(id) {
    if(id == null){
        return
    }
    if (listener) {
      listener.unsubscribe();
    }
    
    drone = new Drone(id, `clover${id}`); // drone obj that initializes navigate services

    listener = new ROSLIB.Topic({
      ros: ros,
      name: `clover${id}/main_camera/image_raw/compressed`,
      messageType: 'sensor_msgs/CompressedImage'
    });
    listener.subscribe(function(message) {
      // console.log('Received message on ' + listener.name);
      document.getElementById('image_subscriber').src = "data:image/jpeg;base64," + message.data;
    
    });

    update_BatteryStatus(id)
    update_Telemetry(id)
    update_CPU(id)
  }
  // update_Streaming(id)
  
  //update_listener(0);
  
  // atualizaçao de lista de ids corretas
  var listenerSwarm;
  
  listenerSwarm = new ROSLIB.Topic({
    ros: ros,
    name: `swarm_checker/state`,
    messageType: 'swarm_checker/SwarmState'
  
  });
  
  listenerSwarm.subscribe(function(message) {
    // document.getElementById('connected_ids')
    list = message.all_clovers_ids // [0, 1, 3, 5] TO DO -> Na verdade é with_camera_ids, mas para simulaçao 
    addIds()
    listenerSwarm.unsubscribe()
  });
  
  function addIds() {
    var element = document.getElementById('streaming-dropdown')
  
    for (let id in list) {
      // create_element(lista[x])
      var child = document.createElement('option')
      console.log(id)
      child.textContent = `Clover ${list[id]}`
      child.setAttribute('value', `${list[id]}`)
      element.appendChild(child)
    }
  }
  
  
  // function create_element(id) {
  //   var element = document.getElementById('dropdown')
  //   var child = document.createElement('option')
  //   child.textContent = `Clover ${id}`
  //   child.setAttribute('value', `${id}`)
  //   element.appendChild(child)
  // }
  
  // cameraId = new ROSLIB.Topic({
  //   ros: ros,
  //   name: 'swarm_checker/state'`
  // })
  

// function update_BatteryStatus(id) {
//   return 
// } 

// function update_Telemetry(id){
//   return
// }

// function update_CPU(id) {
//   return
// }