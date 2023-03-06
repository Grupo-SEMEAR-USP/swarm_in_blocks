var ros = new ROSLIB.Ros({
	url : 'ws://' + location.hostname + ':9090'
});

var titleEl = document.querySelector('title');

ros.on('error', function(error) {
	titleEl.innerText = 'Disconnected';
	err = error;
	alert('Connection error: please enable \'rosbridge\' in clover.launch!');
});

ros.on('connection', function() {
	console.log('connected');
	titleEl.innerText = 'Connected';
	// pubMarkerState('reload')
});

ros.on('close', function() {
	console.log('disconnected');
	titleEl.innerText = 'Disconnected';
});

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
	var lenght = frmSafe.lenght.lenght;
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
	// point_list = [
	// {
	// 	x: points[0][0],
	// 	y: points[0][1],
	// 	z: points[0][2]
	// },
	// {
	// 	x: points[1][0],
	// 	y: points[1][1],
	// 	z: points[1][2]
	// }];

	for (let index = 0; index < points.length; index++) {
		// const element = array[index];
		let point = new ROSLIB.Message ({
			x: points[index][0],
			y: points[index][1],
			z: points[index][2]
		});

		point_list.push(point)
		
	}

	console.log(point_list)
	pubMarkerState(command, point_list, lenght, radius);

})

var geometry_msgsPoint = function() {};

function pubMarkerState(command='reload', points={}, lenght=0, radius=0) {
	
	
	var msg = new ROSLIB.Message({
		command : command, //cirlce, rectangle, square
		points: points, //[[x,y,z],[x,y,z],...]
		lenght: lenght, //square
		radius: radius //circle
	});

	console.log(`Republishing on /marker_state command ${command}`)
	console.log(msg)
	marker_state.publish(msg)
}

// string command
// geometry_msgs/Point[] points
// float32 lenght
// float32 radio

// defining ros3d  

var largura = window.innerWidth
|| document.documentElement.clientWidth
|| document.body.clientWidth;

var altura = window.innerHeight
|| document.documentElement.clientHeight
|| document.body.clientHeight;

var viewer, tfClient;

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

