'use strict';
const express = require('express');
const hostIP = require('../util').getHostIP();
const router = express.Router();

const ROSLIB = require('roslib');
// =============================
//   ROS Web Bridge Connection
// =============================
let ros = new ROSLIB.Ros();
ros.connect(`ws://${hostIP}:9090`);


ros.on('connection', function () {
  console.log('[route] rosbridge Connection MADE!');
});

ros.on('close', function () {
  console.log('[route] rosbridge Connection CLOSED.');
});

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function (_error) {
  console.log(_error);
});

ros.getParams(function (params) {
  console.log(params);
});


router.put('/gmapping/param/', function (req, res) {
  let reqMsg = req.body;
  // console
  var topicAgent = reqMsg.agent;
  var topicMsg = reqMsg.msg;
  console.log(topicAgent);
  console.log(topicMsg);

  var gmParam = new ROSLIB.Param({
    ros: ros,
    name: `/${topicAgent}/far_gmapping:enable`
  })

  // --- send the set-param request ---
  // console.log(topicMsg);
  gmParam.set(topicMsg);
  gmParam.get(function (value) {
    // --- report the set-param response ---
    console.log('the value: ' + value);
    res.send({ gmappingState: value });
  })

});

let flowRequest;
let reqClient = new ROSLIB.Service({
  ros: ros,
  name: '/flow_request',
  serviceType: 'far_plan_msgs/srv/ReqRes'
});

// --- create tasks ---
router.put('/flow_tasks/', flowCreateTasksReqCb);
// --- delete tasks ---
router.delete('/flow_tasks/', flowRemoveTasksReqCb);

// --- flow tasks Request Callback ---
function flowCreateTasksReqCb(req, res) {
  let reqMsg = JSON.stringify(req.body);
  console.log(reqMsg);

  flowRequest = new ROSLIB.ServiceRequest({
    req_msg: reqMsg
  })
  reqClient.callService(flowRequest, function (_message) {
    let jsonRes = JSON.parse(_message.res_msg);
    console.log(_message.res_msg);
    res.send(jsonRes);
  });
}

function flowRemoveTasksReqCb(req, res) {
  let reqMsg = JSON.stringify(req.body);
  console.log(reqMsg);

  flowRequest = new ROSLIB.ServiceRequest({
    req_msg: reqMsg
  })
  reqClient.callService(flowRequest, function (_message) {
    let jsonRes = JSON.parse(_message.res_msg);
    console.log(_message.res_msg);
    res.send(jsonRes);
  });
};


module.exports = router;