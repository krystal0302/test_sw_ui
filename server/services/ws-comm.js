// --- Front-End WebSocket Communication ---
const WebSocket = require('ws');
const hostIP = require('../util').getHostIP();
const ROSLIB = require('roslib');
const { createCanvas, loadImage } = require('canvas')
const canvas = createCanvas(200, 200)
const ctx = canvas.getContext('2d')

// =============================
//   ROS Web Bridge Connection
// =============================
let ros = new ROSLIB.Ros();

setInterval(function () {
	if (ros === undefined) { return; }
	if (!ros.isConnected) {
		ros.connect(`ws://${hostIP}:9090`);
		return;
	}
}, 1200)

ros.on('connection', function () {
	topicMessages_['ros_connect_status'] = "on";
	console.log('rosbridge Connection MADE!');
});

ros.on('close', function () {
	topicMessages_['ros_connect_status'] = "closed";
	console.log('rosbridge Connection CLOSED.');
});

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function (_error) {
	topicMessages_['ros_connect_status'] = "error";
	console.log(_error);
});

// ==============================
//    ROS TOPICS SUBSCRIPTIONS 
// ==============================

// ------ task_state topic ------
let topicMessages_ = {};

// ------ fleet_state topic ------
let fleet_fleet_obj_ = {};

let fleet_state_sub = new ROSLIB.Topic({
	ros: ros,
	name: '/fleet_state',
	messageType: 'far_fleet_msgs/msg/FleetState'
});

fleet_state_sub.subscribe(function (_message) {
	// --- DATA ADAPTION --- 

	// --- update fleet messages ---
	fleet_fleet_obj_[_message.fleet_name] = { fleet_name: "", msg: {} };
	fleet_fleet_obj_[_message.fleet_name].fleet_name = _message.fleet_name;
	fleet_fleet_obj_[_message.fleet_name].robots = _message.robots;
	// console.log(fleet_fleet_obj_);

	topicMessages_["fleet_fleet_state"] = fleet_fleet_obj_;
});

// ------ path_marker topic ------
let path_marker_sub = new ROSLIB.Topic({
	ros: ros,
	name: 'rss_planner/path_markers',
	messageType: 'visualization_msgs/MarkerArray'
});

path_marker_sub.subscribe(function (_message) {
	topicMessages_["path_marker"] = { msg: _message.markers };
});

// ------ wms_marker topic ------
let wms_marker_sub = new ROSLIB.Topic({
	ros: ros,
	name: '/wms_manager/storage_manager_markers',
	messageType: 'visualization_msgs/MarkerArray'
});

wms_marker_sub.subscribe(function (_message) {
	_message.markers = _message.markers.filter((cell) => { return cell.ns !== "" });

	topicMessages_["wms_marker"] = { msg: _message.markers };
});

// --- Task Timer Tab ---
let flow_state_sub = new ROSLIB.Topic({
	ros: ros,
	name: '/flow_state',
	messageType: 'far_plan_msgs/msg/FlowState'
})

flow_state_sub.subscribe(function (_message) {
	var flow_state = _message.flow_state;
	let fleet = '';
	let jobs = [];

	flow_state.forEach(function (fs) {
		fleet = fs.fleet_name;
		var tasks = fs.tasks;
		tasks.forEach(function (t) {
			let taskObj = {
				task_id: t.task_id,
				task_name: t.task_name,
				complete_percent: t.complete_percent,
				robot_id: t.robot_id,
				start_time: cvtTime2MilliSec(t.start_time),
				end_time: cvtTime2MilliSec(t.end_time),
				state: t.state
			};
			jobs.push(taskObj);
		});
	});

	topicMessages_["flow_state2"] = {
		fleet: fleet,
		msg: jobs
	}
});

const tzOffset_ = new Date().getTimezoneOffset() * 60;

// -- return time in milli-second by UTC standard ---
function cvtTime2MilliSec(_time) {
	return (_time.sec * 1000 + _time.nanosec / 1000000) + tzOffset_;
}

// --- Task Timer Tab ---
let forbidden_area_sub = new ROSLIB.Topic({
	ros: ros,
	name: '/forbidden_area',
	messageType: 'std_msgs/msg/String' // TODO: align the message type
})

forbidden_area_sub.subscribe(function (_message) {
	console.log(_message.data);
	topicMessages_["forbidden_area"] = JSON.parse(_message.data);
});


// ==============================
//    ROS TOPICS Publications 
// ==============================
// --- Publish a task request topic ---
let task_request_pub = new ROSLIB.Topic({
	ros: ros,
	name: '/task_request',
	messageType: 'far_task_msgs/msg/TaskRequest'
});

// --- Publish remove task ---
let remove_task_pub = new ROSLIB.Topic({
	ros: ros,
	name: '/remove_task',
	messageType: 'std_msgs/msg/String'
});

// --- Publish a plan event topic ---
let flow_request_pub = new ROSLIB.Topic({
	ros: ros,
	name: '/plan_event',
	messageType: 'far_plan_msgs/msg/PlanEvent'
});

// --- Publish remove fleet ---
let remove_fleet_pub = new ROSLIB.Topic({
	ros: ros,
	name: '/remove_fleet',
	messageType: 'std_msgs/msg/String'
});

// --- Publish Demo signal ---
let demo_signal_pub = new ROSLIB.Topic({
	ros: ros,
	name: '/demo',
	messageType: 'std_msgs/msg/Bool'
});

let gMappingTopic;
let slamPoseTopic;
let cmdVelTopic;

function rosCmdDispatcher(_data) {
	// console.log(_data);
	_data = JSON.parse(_data);
	var topicName = _data.data.topicName;
	// console.log('Topic Name: ' + topicName);

	var rosMsg;
	if (topicName === 'task_request') {
		rosMsg = new ROSLIB.Message(_data.data.topicMsg);
		task_request_pub.publish(rosMsg);
		return;
	}
	if (topicName === 'remove_task') {
		rosMsg = new ROSLIB.Message(_data.data.topicMsg);
		remove_task_pub.publish(rosMsg);
		return;
	}
	if (topicName === 'flow_request') {
		rosMsg = new ROSLIB.Message(_data.data.topicMsg);
		flow_request_pub.publish(rosMsg);
		return;
	}
	if (topicName === 'remove_fleet') {
		rosMsg = new ROSLIB.Message(_data.data.topicMsg);
		remove_fleet_pub.publish(rosMsg);
		return;
	}
	if (topicName === 'demo_signal') {
		rosMsg = new ROSLIB.Message(_data.data.topicMsg);
		demo_signal_pub.publish(rosMsg);
		return;
	}
	if (topicName === 'apply_config') {
		let applyConfigService = new ROSLIB.Service({
			ros: ros,
			name: '/apply_configurations',
			serviceType: 'std_srvs/srv/Empty',
		});

		let srvRequest = new ROSLIB.ServiceRequest();

		applyConfigService.callService(srvRequest, function (_message) {
			console.log(`apply config. done with messages: ${JSON.stringify(_message)}`);
		})
		return;
	}
	if (topicName === 'cmd_vel') {
		// --- protection: expiry check ---
		var issuedTime = _data.data.issuedTime;
		var currTime = Date.now();
		var timeDiff = Math.abs(currTime - issuedTime);
		// console.log(`time difference: ${timeDiff} ms`);
		if (timeDiff > 600) { return; } // expired if over 600s 

		// --- pubish velocity command procedure ---
		var topicAgent = _data.data.topicAgent;
		if (cmdVelTopic === undefined || cmdVelTopic.name !== `/${topicAgent}/cmd_vel`) {
			cmdVelTopic = new ROSLIB.Topic({
				ros: ros,
				name: `/${topicAgent}/cmd_vel`,
				messageType: 'geometry_msgs/Twist'
			});
		}
		console.log(cmdVelTopic.name);

		// console.log('agent: '+topicAgent)
		// console.log(_data.data.topicMsg)
		var topicMsg = new ROSLIB.Message(_data.data.topicMsg);
		cmdVelTopic.publish(topicMsg);
		return;
	}
	if (topicName === 'slam_init') {
		var topicAgent = _data.data.topicAgent;

		// --- Subscribe to occupancy ---
		gMappingTopic = new ROSLIB.Topic({
			ros: ros,
			name: `/${topicAgent}/mapping`,
			messageType: 'nav_msgs/OccupancyGrid'
		});

		gMappingTopic.subscribe(function (_message) {
			var base64Map = cvtMapData2Base64({ msg: _message });
			topicMessages_["slam_gmapping"] = { msg: base64Map, info: _message.info };
		});

		// --- Subscribe to localizer info ---
		slamPoseTopic = new ROSLIB.Topic({
			ros: ros,
			name: `/${topicAgent}/location`,
			messageType: 'far_fleet_msgs/msg/Location'
		});

		slamPoseTopic.subscribe(function (_message) {
			console.log(`slam pose: ${JSON.stringify(_message)}`);
			topicMessages_["slam_pose"] = { msg: JSON.stringify(_message) };
		});

		// --- Publish twist message to SLAM agent ---
		cmdVelTopic = new ROSLIB.Topic({
			ros: ros,
			name: `/${topicAgent}/cmd_vel`,
			messageType: 'geometry_msgs/Twist'
		});

		return;
	}
}

function cvtMapData2Base64(_slamGmappingObj) {
	// console.log(slam_gmapping_obj_);
	if (_slamGmappingObj === undefined) return;

	var _data = _slamGmappingObj.msg;
	if (_data.info.width <= 0 || _data.info.height <= 0) { return; }

	var width = _data.info.width;
	var height = _data.info.height;
	const canvas = createCanvas(width, height);
	const context = canvas.getContext('2d');
	var mapData = context.createImageData(width, height);

	for (var row = 0; row < height; row++) {
		for (var col = 0; col < width; col++) {
			// --- determine the index into the map data ---
			var mapI = col + ((height - row - 1) * width);
			// --- determine the value ---
			var data = _data.data[mapI];
			var val;
			if (data === 100) {
				val = 0;
			} else if (data === 0) {
				val = 255;
			} else {
				val = 205;
			}

			// determine the index into the image data array
			var i = (col + (row * width)) * 4;
			mapData.data[i] = val;   // r
			mapData.data[++i] = val; // g
			mapData.data[++i] = val; // b
			mapData.data[++i] = 255; // a
		}
	}

	context.putImageData(mapData, 0, 0);

	var dataUrl = canvas.toDataURL();
	// console.log(dataUrl)
	return dataUrl;
}


module.exports = () => {

	function heartbeat() {
		this.isAlive = true;
	}

	let wss = new WebSocket.Server({ port: 8081 })
	wss.on("connection", ws => {
		ws.isAlive = true;
		ws.on('pong', heartbeat);
		topicMessages_['wss_connect_status'] = "on"; console.log("A new client connected!");
		ws.on("message", data => {
			console.log(`Message from client: ${data}`);
			rosCmdDispatcher(data);
		});

	});

	const interval = setInterval(function ping() {
		console.log(`\x1b[31m [Heartbeat] \x1b[0m Number of clients: \x1b[33m ${wss.clients.size} \x1b[0m`);
		wss.clients.forEach(function each(client) {
			if (client.isAlive === false) { return client.terminate(); }
			client.isAlive = false;

			client.send(JSON.stringify(topicMessages_))
			client.ping();
		})
	}, 1000);

	wss.on('close', function close() {
		topicMessages_['wss_connect_status'] = "closed";
		console.log("No WebSocket Connections :( !!");

		clearInterval(interval);
	});

};