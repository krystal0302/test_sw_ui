/*
 * author: John Wu
 * date: 10 May 2021
 * description: interfaces with ros messages
 **/
// =======================
//     Event Loop 
// =======================
setInterval(function () {
  if (!ros_conn_stat) {
    // Create a connection to the rosbridge WebSocket server.
    // ros.connect('ws://localhost:9090');
    ros.connect(`ws://${window.location.hostname}:9090`);
    return;
  }

  // update the map
  if (map_data_.fetched)
    return;

  // Periodically to check map data
  fetchRosMap(map_data_);
}, 1200)

// =============================
//   ROS Web Bridge Connection
// =============================
let ros_conn_stat = false;

let ros = new ROSLIB.Ros();

ros.on('connection', function () {
  document.getElementById('statusIndicator').innerHTML = 'connection: ON';
  document.getElementById('statusIndicator').style.color = '#00D600';
  console.log('Connection made!');
  ros_conn_stat = true;
});

ros.on('close', function () {
  document.getElementById('statusIndicator').innerHTML = 'connection: CLOSED';
  document.getElementById('statusIndicator').style.color = '#FF0000';
  console.log('Connection closed.');
  ros_conn_stat = false;
});

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function (_error) {
  document.getElementById('statusIndicator').innerHTML = 'connection: ERROR';
  document.getElementById('statusIndicator').style.color = '#FF0000';
  console.log(_error);
  ros_conn_stat = false;
});

// ========================
//       ROS SERVICES  
// ========================
let SCALE_ = 1;

let X_OFFSET_ros = 0;
let Y_OFFSET_ros = 0;

let map_data_ = {
  w: 0.0,
  h: 0.0,
  r: 0.025,
  d: null,
  fetched: false,
  updated: false,
};

let mapServiceClient = new ROSLIB.Service({
  ros: ros,
  name: '/map_server/map',
  serviceType: 'nav_msgs/GetMap',
  compression: 'png'
});

let request = new ROSLIB.ServiceRequest();

var fetchCount = 0;
function fetchRosMap(_data) {
  if (fetchCount > 5) return;

  fetchCount++;
  console.log('fetch_map');
  // --- ROS service case ---
  // _data.updated = true;
  mapServiceClient.callService(request, function (_message) {
    _data.w = _message.map.info.width;
    _data.h = _message.map.info.height;
    _data.r = _message.map.info.resolution.toFixed(4);
    _data.d = _message.map.data;
    _data.fetched = true;

    console.log("map_resolution: " + _data.r);
    SCALE_ = 1 / _data.r;
    console.log("scale: " + SCALE_);

    X_OFFSET_ros = _message.map.info.origin.position.x * SCALE_;
    Y_OFFSET_ros = _message.map.info.origin.position.y * SCALE_;
    console.log(`OFFSET_ros (${X_OFFSET_ros}, ${Y_OFFSET_ros}) in fetchMap`);
  })
}

// =============================
//    ROS TOPICS PUBLICATIONS 
// =============================
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

// // --- Publish remove fleet ---
// let remove_fleet_pub = new ROSLIB.Topic({
//   ros: ros,
//   name: '/remove_fleet',
//   messageType: 'std_msgs/msg/String'
// });

// // --- Publish Demo signal ---
// let demo_signal_pub = new ROSLIB.Topic({
//   ros: ros,
//   name: '/demo',
//   messageType: 'std_msgs/msg/Bool'
// });

// ==============================
//    ROS TOPICS SUBSCRIPTIONS 
// ==============================
// ------ Task state topic ------
let count_finished_tasks_ = 0;
let tasks_obj_ = {
  msg: null,
  fleet: null,
  updated: false
};

let task_state_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/task_state',
  messageType: 'far_task_msgs/msg/TaskState'
});

task_state_sub.subscribe(function (_message) {
  tasks_obj_.fleet = _message.fleet_name;
  tasks_obj_.msg = _message.tasks;
  // Todo: solve the conflict
  for (var i = 0; i < tasks_obj_.msg.length; i++) {
    tasks_list_[tasks_obj_.msg[i].task_name] = { 'state': tasks_obj_.msg[i].state, 'task_name': tasks_obj_.msg[i].task_name, 'complete_percent': tasks_obj_.msg[i].complete_percent };

    if (tasks_obj_.msg[i].state == 2) {
      count_finished_tasks_ += 1;
    }
  }
});

// ----------------------------
//     Task State Utilities 
// ----------------------------
// ------ Fleet State topic ------
let robot_list = [];

let fleet_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let fleet_fleet_obj_ = {};

let fleet_state_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/fleet_state',
  messageType: 'far_fleet_msgs/msg/FleetState'
});

fleet_state_sub.subscribe(function (_message) {

  fleet_obj_.msg = _message.robots;

  for (var i = 0; i < fleet_obj_.msg.length; i++) {
    // Display Available robot
    if (robot_list.find(element => element == fleet_obj_.msg[i].robot_id) == undefined) {
      robot_list.push(fleet_obj_.msg[i].robot_id);
    }
  }

  // console.log(fleet_obj_);
  fleet_obj_.updated = true;

  // --- update fleet messages ---
  // console.log(fleet_obj_);
  fleet_fleet_obj_[_message.fleet_name] = { fleet_name: "", msg: {} };
  fleet_fleet_obj_[_message.fleet_name].fleet_name = _message.fleet_name;
  fleet_fleet_obj_[_message.fleet_name].robots = _message.robots;
  // console.log(fleet_fleet_obj_);
});

// ------ Role State topic ------
let role_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let role_state_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/role_state',
  messageType: 'std_msgs/msg/String'
});

role_state_sub.subscribe(function (message) {
  // Display Available roles
  role_obj_.msg = message.data;
});


// ------ Agent Marker topic ------
let agents_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let agent_marker_sub = new ROSLIB.Topic({
  ros: ros,
  name: 'far_sim/agent_markers',
  messageType: 'visualization_msgs/MarkerArray'
});

agent_marker_sub.subscribe(function (_message) {
  agents_obj_.msg = _message.markers;
});

// ------ Graph Marker topic ------
let graph_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let graph_marker_sub = new ROSLIB.Topic({
  ros: ros,
  name: 'rss_planner/graph_markers',
  messageType: 'visualization_msgs/MarkerArray'
});

graph_marker_sub.subscribe(function (_message) {
  graph_obj_.msg = _message.markers;
});

// ------ Path Marker topic ------
let path_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let path_marker_sub = new ROSLIB.Topic({
  ros: ros,
  name: 'rss_planner/path_markers',
  messageType: 'visualization_msgs/MarkerArray'
});

path_marker_sub.subscribe(function (_message) {
  // console.log('receive the path messages');
  path_obj_.msg = _message.markers;
});


// ------ WMS Marker topic ------
let wms_obj_ = {
  msg: null,
  prev_msg: null,
  updated: false
};

let wms_marker_sub = new ROSLIB.Topic({
  ros: ros,
  name: '/wms_manager/storage_manager_markers',
  messageType: 'visualization_msgs/MarkerArray'
});

wms_marker_sub.subscribe(function (_message) {
  _message.markers = _message.markers.filter((cell) => { return cell.ns !== "" });
  wms_obj_.msg = _message.markers;
});



// ================================
//        project events    
// ================================
// ----------------------------
//     ROS Common Utilities 
// ----------------------------
let tasks_list_ = {};
function clearData() {
  // reset the variables
  prev_task_num = 0;
  finished_tasks_num_ = 0;
  tasks_list_ = {};

  // reset the graph
  deliveryChart.data.labels = [];
  deliveryChart.data.datasets[0].data = [];
  deliveryChart.update();
}

// function send_remove_fleet(_fleetName, _roleName) {
//   var dataString = "fleet:" + _fleetName + ",role:" + _roleName;
//   var remove_fleet_msg = new ROSLIB.Message({ data: dataString });
//   remove_fleet_pub.publish(remove_fleet_msg);
//   console.log("=======send remove fleet=======");
//   console.log({ data: dataString });
// }

// function demo() {
//   var demo_msg = new ROSLIB.Message({ data: true });
//   demo_signal_pub.publish(demo_msg);
//   toast('Demo Request Sent');
// }

// ------ Notification Utility ------
function toast(message) {
  Toastify({
    text: message,
    duration: 3000,
    close: false,
    gravity: "top",
    position: "right",
    backgroundColor: "#666",
    stopOnFocus: true,
  }).showToast();
}

const task_state_bgclass = {
  0: 'farobot-info-bg-queued',
  1: 'farobot-info-bg-active',
  2: 'farobot-info-bg-success',
  3: 'farobot-info-bg-error'
}

const task_state = {
  0: 'QUEUED',
  1: 'ACTIVE',
  2: 'COMPLETED',
  3: 'FAILED'
}
