var ros = new ROSLIB.Ros({
	url : 'ws://' + location.hostname + ':9090'
});

var x = new Boolean(true);

ros.on('error', function(error) {
	err = error;
	alert('Could not connect to ros: please make sure \'swarm_station.launch\' was launched!');
});

ros.on('connection', function() {
	x = true;
	console.log(x);
});

ros.on('close', function() {
	x = false;
	console.log('disconnected');
});


// --------------------------Terminal---------------------

function open_terminal(){
	if(x) {
		document.getElementById('terminal').classList.toggle('active');
		document.getElementById('open-terminal').classList.toggle('active');
		document.getElementById('open-button-terminal').classList.toggle('active');
	}
}

// defining publisher
var marker_state = new ROSLIB.Topic({
	ros: ros,
	name: "/marker_state",
	messageType: 'swarm_visualizer/SwarmStationCommands'
	
})

var btnSalvar = document.querySelector("#ok")

btnSalvar.addEventListener("click", function(event){
	event.preventDefault();

	var frmSafe = document.querySelector("#seg_body");
	var command = frmSafe.rad.value;
	var length = frmSafe.length.length;
	var radius = frmSafe.radius.radius;

	var points = []
	
	if (command == "circle"){
		points[0] = [parseFloat(frmSafe.circle_x.value), parseFloat(frmSafe.circle_y.value), parseFloat(frmSafe.circle_z.value)]
	}
	if (command == "square"){
		points[0] = [parseFloat(frmSafe.square_x.value), parseFloat(frmSafe.square_y.value), parseFloat(frmSafe.square_z.value)]
	}
	if (command == "rectangle"){
		points[0] = [parseFloat(frmSafe.r1_x.value), parseFloat(frmSafe.r1_y.value), parseFloat(frmSafe.r1_z.value)]
		points[1] = [parseFloat(frmSafe.r2_x.value), parseFloat(frmSafe.r2_y.value), parseFloat(frmSafe.r2_z.value)]
	}

	console.log(points)
	point_list = []
	
	for (let index = 0; index < points.length; index++) {
		// const element = array[index];
		// let point = new ROSLIB.Message ({
		point = {
			x: points[index][0],
			y: points[index][1],
			z: points[index][2]
		};
		// console.log(point)
		point_list.push(point)
		
	}

	// console.log(point_list)
	pubMarkerState(command, point_list, length, radius);

});

var point_template = []
	
	for (let index = 0; index < 2; index++) {
		// const element = array[index];
		// let point = new ROSLIB.Message ({
		point = {
			x: 0,
			y: 0,
			z: 0
		};
		// console.log(point)
		point_template.push(point)
		
	}




var geometry_msgsPoint = function() {};

function pubMarkerState(command='reload', points= point_template, length=0, radius=0) {
	

	
	var msg = new ROSLIB.Message({
		command : command, //cirlce, rectangle, square
		points: points, //[[x,y,z],[x,y,z],...]
		length: length, //square
		radius: radius //circle
	});

	console.log(`Republishing on /marker_state command ${command}`)
	console.log(msg)
	marker_state.publish(msg)
}

// string command
// geometry_msgs/Point[] points
// float32 length
// float32 radio

// defining ros3d  

var largura = window.innerWidth;
var altura = window.innerHeight;

var viewer, tfClient;

setInterval(function() {
	var width = window.innerWidth;
	var height = window.innerHeight;
}, 1000);

function setScene(fixedFrame) {
	viewer = new ROS3D.Viewer({
		divID: 'viz',
		width: largura,
		height: altura,
		antialias: true
	});

	tfClient = new ROSLIB.TFClient({
		ros: ros,
		angularThres: 0.01,
		transThres: 0.01,
		rate: 10.0,
		fixedFrame : fixedFrame
	});

	var map = new ROS3D.Grid({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});

	viewer.scene.add(map);
}

function addAxes() {
	var axes = new ROS3D.Axes({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});
	viewer.scene.add(axes);
}

function addVehicle() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/vehicle_marker',
		rootObject: viewer.scene
		
	});

	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/safe_marker',
		rootObject: viewer.scene
		
	});
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/text_marker',
		rootObject: viewer.scene
		
	});

	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/base_vehicle_marker',
		rootObject: viewer.scene
	})

	for (let i = 0; i < 3; i++) {
		pubMarkerState(command='reload');
		console.log("reloading vehicles")
	}
}


function addCamera() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/main_camera/camera_markers',
		rootObject: viewer.scene
	});
}

function addAruco() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_detect/visualization',
		rootObject: viewer.scene
	});
}

function addArucoMap() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_map/visualization',
		rootObject: viewer.scene
	});
}
setScene('map')
addAxes()
addVehicle()

