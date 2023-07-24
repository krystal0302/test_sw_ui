/*
 * Author: John Wu
 * Date: 9 Mar 2021
 * Description: 
 *   1. Display swarm system infomation on Main Page
 *   2. Add pop-up sidebar features
 **/

// --- agents asset  ---
const avatar_map_ = {
  bbr: "dist/img/sprites/bbr-440x220.png",
  br: "dist/img/sprites/smr1000_300mm-440x220.png",
  hik600: "dist/img/sprites/hik-440x220.png",
  smr250: "dist/img/sprites/smr250-440x220.png",
  smr250black: "dist/img/sprites/smr250black-440x220.png",
  smr1000: "dist/img/sprites/smr1000-440x220.png",
  mir250: "dist/img/sprites/mir-440x220.png",
  none: "dist/img/sprites/noimage-440x220.png"
}

const mode_map_ = {
  0: "IDLE",
  1: "CHARGING",
  2: "MOVING",
  3: "PAUSED",
  4: "WAITING",
  5: "EMERGENCY",
  6: "GOING_HOME",
  7: "DOCKING",
  8: "UNINITIALIZED",
  404: "DISCONNECTED"
};

// ============================
//   Event Loop for Dashboard 
// ============================
setInterval(() => {
  if (tasks_obj_) { taskStateOnUI(tasks_obj_); }
  if (fleet_obj_) { fleetStateOnUI(fleet_obj_); }
  if (role_obj_) { roleStateOnUI(role_obj_); }
}, 1000);

var finished_tasks_num_ = 0;
var finished_cargo_ = 0;
var role_dict = {}; // role:params

var tempTaskMsgArr = [];
var tempFleetMsgArr = [];


// ======================
//       Load Ready 
// ======================
// ------ Render Widgets ------
let deliveryChart;

let deliveryChartData = {
  labels: [],
  datasets: [
    {
      label: 'Digital Goods',
      fill: true,
      borderWidth: 2,
      lineTension: 0,
      spanGaps: true,
      borderColor: '#00A0FF',
      pointRadius: 3,
      pointHoverRadius: 7,
      pointColor: '#00A0FF',
      pointBackgroundColor: '#00A0FF',
      data: []
    }
  ]
}

let deliveryChartOptions = {
  maintainAspectRatio: false,
  responsive: true,
  legend: {
    display: false,
  },
  scales: {
    xAxes: [{
      ticks: {
        fontColor: '#ffffff',
      },
      gridLines: {
        display: false,
        color: '#ffffff',
        drawBorder: false,
      }
    }],
    yAxes: [{
      ticks: {
        stepSize: 2,
        fontColor: '#ffffff',
        beginAtZero: true,
      },
      gridLines: {
        display: true,
        color: '#ffffff',
        zeroLineColor: '#ffffff',
        drawBorder: false,
      }
    }]
  }
}

$(async function () {
  'use strict'
  initDataAsync();

  // ------ dashboard initialization ------
  /* jQueryKnob */
  $('.knob').knob({
    "readOnly": true,
    'fgColor': '#00A0FF'
  });

  try {
    var selFleet = getSavedFleet();
    console.log(`=== current fleet : ${selFleet} ===`);

    // -- fetch fleet_settings --
    var fltSettings = await restGetFleetSettings(selFleet);
    fltSettings = JSON.parse(fltSettings);

    // -- load the agents in the fleet --
    var fltKey = Object.keys(fltSettings)[0];
    var agents = fltSettings[fltKey].agents;
    addAgentCards(agents);

    var isDarkMode = getSavedTheme() === 'dark';
    toggleContentDarkTheme(isDarkMode);
  } catch (err) {
    console.error("load fleet configuration error: " + err);
  }

  // --- Delivery Items graph chart ---
  var deliveryChartCanvas = $('#delivery-chart').get(0).getContext('2d');
  deliveryChart = new Chart(deliveryChartCanvas, {
    type: 'line',
    data: deliveryChartData,
    options: deliveryChartOptions
  })
})

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'dashboard');
}


// ==============================
//    RELATED VARIABLES 
// ==============================
const total_tasks = 0; // TODO: load the value from back-end 
let prev_task_num = 0;
let prev_finished_cargo = 0;


// ==============================
//    ROS MESSAGES ON UI 
// ==============================
function taskStateOnUI(_taskStates) {
  finished_cargo_ = 0;
  finished_tasks_num_ = 0;
  var taskFleet = _taskStates.fleet;
  var tasks = _taskStates.msg;

  if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append('<div class="card card-dark col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
  }
  if (taskFleet !== $('#fleet-select').val()) { return };

  var div = "";
  tasks = tasks.sort(compare).slice(0, 10);
  if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
    console.log(tasks);
    $('#task-schedule-div').html('');
    tasks.forEach(function (task_item, task_index) {
      var start_time = task_item.start_time;
      var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
      var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;

      div +=
        `<div class="col-md-2 col-sm-6 col-12 trigger-div">
          <div class="info-box bg-dark" id="task-status-${task_index}">
            <button type="button" class="delete-trigger-btn" id='${task_item.task_id}'>×</button>
            <span class="info-box-icon"><i class="fas fa-tasks"></i></span>
            <div class="info-box-content col-10" width="50%">
              <span class="info-box-text" title="${sche_date_trans}">${sche_date_trans}</span>
              <marquee direction="left" scrollamount="2" behavior="scroll" id="task-name-${task_index}">${task_item.task_name}</marquee>
              <span class="info-box-text" id="task-robot-${task_index}" title="Assigned robot: ${task_item.robot_id}">Assigned robot: ${task_item.robot_id}</span>
              <span class="info-box-number" id="task-percent-${task_index}">--%</span>
              <div class="progress">
                <div class="progress-bar" id="task-progress-${task_index}" style="width: 0%"></div>
              </div>
            </div>
          </div>
        </div>`;
    });
    $('#task-schedule-div').append(div);
  }
  tempTaskMsgArr = tasks;

  // need update state
  tasks.forEach(function (task_item, task_index) {
    // --- Update on UI ---
    $('#task-status-' + task_index).removeClass(function (index, className) {
      return (className.match(/(^|\s)bg-\S+/g) || []).join(' ');
    });

    $('#task-name-' + task_index).html(`${task_item.task_name} is ${task_state[task_item.state].toLowerCase()}`);
    $('#task-status-' + task_index).addClass(task_state_bgclass[task_item.state]);

    if (task_item.state == 2) {
      // TODO: reflect real data
      $('#task-percent-' + task_index).html('100%');

      if (task_item.task_name.substr(0, 3) == "tra") {
        finished_cargo_++;
      }
      finished_tasks_num_++;
    } else {
      // TODO: reflect real data
      $('#task-percent-' + task_index).html(`${task_item.complete_percent}%`);
      $('#task-progress-' + task_index).css('width', `${task_item.complete_percent}%`)
    }
  });

  if (finished_cargo_ > prev_finished_cargo) {
    var currTime = new Date();
    var datetime = currTime.getHours() + ":" + currTime.getMinutes() + ":" + currTime.getSeconds();

    deliveryChart.data.labels.push(datetime);
    deliveryChart.data.datasets[0].data.push(finished_cargo_);

    deliveryChart.update();
    prev_finished_cargo = finished_cargo_;
  }

  // --- Plan Progress Widget ---
  var rate = (finished_tasks_num_ / total_tasks) * 100;
  $('#plan-progress').val(rate).trigger('change');
}

$(document).on('click', '.delete-trigger-btn', function (e) {
  sendRemoveTaskReq("id", this.id);
  $(this).parent().parent().remove();
});

function sendRemoveTaskReq(type, task) {
  var jsonRemoveTask = { data: `${type}:${task}` };
  wsRemoveTask(jsonRemoveTask);
}

function compare(a, b) {
  var a_sec = a.start_time.sec + a.start_time.nanosec / 1000000000;
  var b_sec = b.start_time.sec + b.start_time.nanosec / 1000000000;
  if (a_sec === b_sec) { return 0; }
  return (a_sec < b_sec) ? -1 : 1;
}

async function switchFleetCallback() {
  initWidgets();

  try {
    var selFleet = getSavedFleet();
    var fltSettings = await restGetFleetSettings(selFleet);
    fltSettings = JSON.parse(fltSettings);
    var fltKey = Object.keys(fltSettings)[0];
    addAgentCards(fltSettings[fltKey].agents);
  }
  catch (err) {
    console.error("Get Fleet Settings error: " + err);
  }
}

function initWidgets() {
  var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  $('#robot-util').val(0).trigger('change');
  $('#plan-progress').val(0).trigger('change');
  resetDeliveryChart();
  var $emptyDiv = $('#smr-card-0').detach();
  $('#agent-card-deck-db').empty();
  $('#agent-card-deck-db').append($emptyDiv);
  $('#task-schedule-div').html('');
  $('#task-schedule-div').append(`<div class="card ${cardClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  tempTaskMsgArr = [];
  tempFleetMsgArr = [];
}

function resetDeliveryChart() {
  let totalData = deliveryChart.data.labels.length;
  while (totalData >= 0) {
    deliveryChart.data.labels.pop();
    deliveryChart.data.datasets[0].data.pop();
    totalData--;
  }
  deliveryChart.update();
}

function addAgentCards(fleet_agents) {
  var emptyAgentData = {
    model: "",
    robot_id: "",
    role: "",
    location: { x: 0, y: 0 },
    mode: 404,
    battery_percent: "--"
  };

  var cardDeck = document.getElementById('agent-card-deck-db');
  fleet_agents.forEach((agent) => {
    console.log(agent);
    emptyAgentData.robot_id = agent;
    const template = document.querySelector('#agent-card-db');
    const node = document.importNode(template.content, true);

    var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
    var endtryNode = node.querySelector('.card-entry');
    endtryNode.id = agent;
    endtryNode.classList.add(cardClass);

    updateAgentCardView(node, emptyAgentData);
    cardDeck.prepend(node);
  });
}

function fleetStateOnUI(_fleetState) {
  // -------- Robots Utilization Widget --------
  var workingAgents = _fleetState.msg.filter((agent) => { return agent.mode !== 0; }); // 0: IDLE
  var rate = (workingAgents.length / _fleetState.msg.length) * 100;

  // -------- Fleet Status Overview  --------
  // image        :                                    [id: .smr-card-X > img.src]
  // model        : _fleetState.msg[i].model           [id: .smr-model-X]
  // role         : _fleetState.msg[i].role            [id: .smr-role-X]
  // task         : _fleetState.msg[i].task_name       [id: .smr-task-name-X]
  // battery      : _fleetState.msg[i].battery_percent [id: .smr-battery-X]
  // TODO: task percent needed

  _fleetState.msg.forEach((agent) => {
    console.log(agent);
    if (agent.fleet_name !== $('#fleet-select').val()) {
      return;
    }

    $('#robot-util').val(rate).trigger('change');
    var targetEl = document.getElementById(agent.robot_id);
    if (!targetEl) {
      console.error("cannot find the DOM by robot_id");
      return;
    }
    updateAgentCardView(targetEl, agent);
  });

  tempFleetMsgArr = _fleetState.msg;
}

function updateAgentCardView(_domNode, _agentData) {
  var imgNode = _domNode.querySelector('.card-img-top');
  var model = _agentData.model.toLowerCase();
  imgNode.src = avatarMap_[model] || avatarMap_['none'];

  // --- agent name ---
  var roleNode = _domNode.querySelector('.smr-name');
  roleNode.textContent = _agentData.robot_id;

  // --- agent role ---
  if (_agentData.role.substr(0, 3) == "tra") {
    _agentData.role = "transport";
  }
  if (_agentData.role.substr(0, 4) == "home" || _agentData.role.substr(0, 6) == "undock") {
    _agentData.role = ""
  }
  var roleNode = _domNode.querySelector('.smr-role');
  roleNode.textContent = _agentData.role;

  var statusNode = _domNode.querySelector('.smr-mode');
  statusNode.textContent = modeMap_[_agentData.mode];

  // --- battery percent ---
  var powerNode = _domNode.querySelector('.battery-percent');
  powerNode.textContent = _agentData.battery_percent + '%';
}

function roleStateOnUI(_roleState) {
  var role_param_list = _roleState.split('&');
  for (i = 0; i < role_param_list.length; i++) {
    role_dict[role_param_list[i].split("@")[0]] = role_param_list[i].split("@")[1];
  }
  var role_list = Object.keys(role_dict);
  // var x = document.getElementById("task_type");
  var option = document.createElement("option");
  var task_type_list = [];
  for (var i = 0; i < role_list.length; i++) {
    if (task_type_list.find(element => element == role_list[i]) == undefined) {
      option.text = role_list[i];
    }
  }
}

// ===========================
//         UTILITIES 
// ===========================
function RGBToHex(r, g, b) {
  r = parseInt(r * 255).toString(16);
  g = parseInt(g * 255).toString(16);
  b = parseInt(b * 255).toString(16);

  if (r.length == 1) { r = "0" + r; }
  if (g.length == 1) { g = "0" + g; }
  if (b.length == 1) { b = "0" + b; }

  return "#" + r + g + b;
}
