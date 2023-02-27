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
	messageType: 'std_msgs/String'
	
})

function pubMarkerState(message) {

	var msg = new ROSLIB.Message({
		data : message,
	});

	marker_state.publish(msg)
}


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

	for (let i = 0; i < 3; i++) {
		pubMarkerState('reload');
		console.log(i)
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

const openModalLand = document.querySelector(".open-land");
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
const fade_seg = document.querySelector("#fade_seg");

const seg = document.querySelector("#seg");

const toggleModalSeg = () => {
	seg.classList.toggle("hide");
	fade_seg.classList.toggle("hide");
};

[openModalSeg, closeModalSeg, fade_seg].forEach((el) => {
	el.addEventListener("click", () => toggleModalSeg());
});


