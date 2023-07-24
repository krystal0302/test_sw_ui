let path_obj_;
let wms_obj_;
let fleet_fleet_obj_;
let flow_state_obj2_;
let slam_gmapping_obj_;
let slam_pose_obj_;
let forbidden_area_obj_;
let ros_conn_status_;
let wws_conn_status_;

const worker_ = new SharedWorker("dist/js/pages/far_ros_worker.js");
let queryData_;

// --- Set initial state ---
let webSocketState = WebSocket.CONNECTING;

worker_.port.start();

const broadcastChannel = new BroadcastChannel("WebSocketChannel");
console.log(broadcastChannel);
broadcastChannel.addEventListener("message", event => {
  switch (event.data.type) {
    case "WSState":
      webSocketState = event.data.state;
      // console.log(webSocketState);
      if (webSocketState === WebSocket.CLOSED) {
        genConnectionStatusIndicator('connection: CLOSED', '#FF0000');
      }
      break;
    case "message":
      event.data.data['wss_connect_status'] = 'on';
      handleBroadcast(event.data.data);
      break;
    case "apiMessages":
      queryData_ = event.data;
      break;
  }
});

// --- listen to broadcasts from server ---
function handleBroadcast(data) {
  // console.log(data);
  updateMessageFromWSServer(data);
  updateNavBarConnectionStatus();
}

// --- get the type of browser ---
function detectBrowser() {
  if (navigator.userAgent.indexOf("Chrome") != -1) {
    return 'Chrome';
  } else if (navigator.userAgent.indexOf("Safari") != -1) {
    return 'Safari';
  } else if (navigator.userAgent.indexOf("Firefox") != -1) {
    return 'Firefox';
  } else {
    return 'Unknown';
  }
}

// Use this method to send data to the server.
function postMessageToWSServer(_wsMsg, _pageId = 'any') {
  console.log(webSocketState);
  // if (webSocketState === WebSocket.CONNECTING) {
  //   console.log("Still connecting to the server, try again later!");
  // } else if (
  if (
    webSocketState === WebSocket.CLOSING ||
    webSocketState === WebSocket.CLOSED
  ) {
    console.log("Connection Closed!");
  } else {
    worker_.port.postMessage({
      from: detectBrowser(),
      data: _wsMsg,
      page_id: _pageId
    });
  }
}

function updateMessageFromWSServer(_data) {
  path_obj_ = _data['path_marker'];
  wms_obj_ = _data['wms_marker'];
  fleet_fleet_obj_ = _data['fleet_fleet_state'];
  flow_state_obj2_ = _data['flow_state2'];
  slam_gmapping_obj_ = _data['slam_gmapping'];
  slam_pose_obj_ = _data['slam_pose'];
  ros_conn_status_ = _data['ros_connect_status'];
  wws_conn_status_ = _data['wss_connect_status'];
  forbidden_area_obj_ = _data['forbidden_area'];
}

function updateNavBarConnectionStatus() {
  // console.log(`ros: ${ros_conn_status_}, wws: ${wws_conn_status_}`);
  if (ros_conn_status_ === "on" && wws_conn_status_ === "on") {
    genConnectionStatusIndicator('connection: ON', '#00D600');
  } else if (ros_conn_status_ === "error") {
    genConnectionStatusIndicator('connection: ERROR', '#FF0000');
  } else {
    genConnectionStatusIndicator('connection: CLOSED', '#FF0000');
  }
}

function wsApplyConfig() {
  postMessageToWSServer({ topicName: 'apply_config' });
}

function wsSendROSTopic() {
  var myData = { hello: 'world' };
  postMessageToWSServer({ topicName: 'topic_test', topicMsg: myData });
}

function wsRemoveFleet(_fleetName, _roleName) {
  var dataString = "fleet:" + _fleetName + ",role:" + _roleName;
  // topic: remove_fleet_pub
  postMessageToWSServer({ topicName: 'remove_fleet', topicMsg: dataString });
}

function wsTrigDemo() {
  // topic: demo_signal_pub 
  postMessageToWSServer({ topicName: 'demo_signal', topicMsg: { data: true } });
}

function wsPubTaskReq(_rosMsg) {
  // topic: task_request_pub
  postMessageToWSServer({ topicName: 'task_request', topicMsg: _rosMsg });
}

function wsRemoveTask(_rosMsg) {
  // topic: remove_task_pub
  postMessageToWSServer({ topicName: 'remove_task', topicMsg: _rosMsg });
}

function wsPubFlowReq(_rosMsg) {
  // topic: flow_request_pub
  postMessageToWSServer({ topicName: 'flow_request', topicMsg: _rosMsg });
}

// function wsMoveSomeWhere(_rosMsg) {
//   postMessageToWSServer({ topicName: 'cmd_vel', topicMsg: _rosMsg.velCmd, topicAgent: _rosMsg.slamAgent, issuedTime: _rosMsg.issuedTime });
// }

// function wsSlamTopicsInit(_slamAgent) {
//   postMessageToWSServer({ topicName: 'slam_init', topicAgent: _slamAgent });
//   // return [slam_gmapping_obj_, slam_pose_obj_];
// }

function wsSetGmapping(_targetAgent, _state) {
  postMessageToWSServer({ topicName: 'set_gmapping', topicAgent: _targetAgent, topicMsg: _state });
}

