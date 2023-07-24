/*
 * Author: John Wu
 * Date: 21 Feb 2022
 * Description: 
 *   1. Display swarm system infomation on Main Page
 *   2. Add pop-up sidebar features
 **/

// --- agents asset ---
let artifactAssets_ = AssetsDict["artifactAssets"];

const avatarMap_ = {
  bbr: "dist/img/sprites/bbr-75x75.png",
  br: "dist/img/sprites/smr1000_300mm-75x75.png",
  hik600: "dist/img/sprites/hik-75x75.png",
  smr250: "dist/img/sprites/smr250-75x75.png",
  smr250black: "dist/img/sprites/smr250black-75x75.png",
  smr1000: "dist/img/sprites/smr1000-75x75.png",
  mir250: "dist/img/sprites/mir-75x75.png",
  none: "dist/img/sprites/no_image-75x75.png",
}

const modeMap_ = {
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

function correctAgentStatus(status) {
  if (status === 'MOVING')
    return 'ACTIVE';
  else
    return status;
}

// ============================
//   Event Loop for Dashboard 
// ============================
function updateRUGraph(chart, _data) {
  chart.data.datasets.pop();
  chart.data.datasets.push(
    {
      data: _data,
      backgroundColor: ['#c4c4c4', '#00d600', '#00a0ff', '#f39c12', '#3c8dbc', '#dc3545', '#17a2b8', '#d2d6d3', '#6c757d'],
      borderColor: ['#373e44', '#373e44', '#373e44', '#373e44', '#373e44', '#373e44', '#373e44', '#373e44', '#373e44'],
    }
  );
  chart.update();
}

function updatePPGraph(chart, _data) {
  chart.data.datasets.pop();
  chart.data.datasets.push(
    {
      data: _data,
      backgroundColor: ['#6c757d', '#00a0ff', '#00d600', '#dc3545', '#f39c12'],
      borderColor: ['#373e44', '#373e44', '#373e44', '#373e44', '#373e44'],
    }
  );
  chart.update();
}

function updateARTraph(chart, _data) {
  chart.data.datasets.pop();
  chart.data.datasets.push(
    {
      data: _data,
      backgroundColor: ['#00d600', '#f39c12', '#dc3545', '#d2d6d3'],
      borderColor: ['#373e44', '#373e44', '#373e44', '#373e44'],
    }
  );
  chart.update();
}

const pieAgnt = {
  'en': ['Idle', 'Charging', 'Active', 'Paused', 'Waiting', 'Emergency', 'Unintialized', 'Disconnected'],
  'zh': ['闲置', '充电', '作动中', '暂停', '等待中', '紧急', '未初始化', '离线',]
};

const pieArtf = {
  'en': ['In service', 'No data received', 'Error', 'Disconnected'],
  'zh': ['服务中', '无接收数据', '错误', '离线']
};

const pieFlow = {
  'en': ['Queued', 'Active', 'Completed', 'Failed', 'Paused'],
  'zh': ['队列', '作动中', '完成', '失败', '暂停']
}

function updateRUGraph_lang(chart, lng) {
  chart.data.labels = pieAgnt[lng]
  chart.update();
}
function updatePPGraph_lang(chart, lng) {
  chart.data.labels = pieFlow[lng]
  chart.update();
}
function updateARTraph_lang(chart, lng) {
  chart.data.labels = pieArtf[lng]
  chart.update();
}

$('.fa-globe').parent().on('click', async () => {
  let lng = getSetLang();
  lng = (lng === "") ? "en" : lng;
  lng = (lng === "en") ? "zh" : "en";

  updatePPGraph_lang(planProgressChart, lng);
  updatePPGraph_lang(taskPieChart, lng);
  updateRUGraph_lang(robotUtilChart, lng);
  updateARTraph_lang(artifactUtilChart, lng);
});

let hideLink = false;

let prevArtData_ = [];
let prevRUData_ = [];
let prevPPData_ = [];
let prevTPData_ = [];
let artData_ = [0, 0, 0, 0];
let ppData_ = [0, 0, 0, 0, 0];
let ruData_ = [0, 0, 0, 0, 0, 0, 0, 0, 0];
var scannedAgents;

var selRobotID;
var selFleet = $('#fleet-select').val();
setInterval(async () => {
  // --- websocket data ---
  // if (fleetDataObj_) { updateFleetWidgets(fleet_obj_); }
  // if (flowDataObj_) { updateFlowsWidgets(tasks_obj_); }

  // --- client pull by native APIs ---
  // if (queryData_ === undefined) { return; }
  // // console.log(queryData_)
  // if (queryData_.flowListView) {
  //   var flowListView = _.groupBy(queryData_.flowListView, flow => flow.fleet_name);
  //   var flowPieChart = _.countBy(flowListView[selFleet], 'state');
  //   updateFlowsWidgets2(flowPieChart, flowListView[selFleet]);
  // }

  // if (queryData_.agentListView) {
  //   var agentListView = _.groupBy(queryData_.agentListView, agent => agent.fleet_name);
  //   var agentPieChart = _.countBy(agentListView[selFleet], 'mode');
  //   updateFleetWidgets2(agentPieChart, agentListView[selFleet]);
  //   updateAgentModalContentFromFleetState(agentListView[selFleet]);
  // }

  // const mockFleetData_ = [
  //   { "robot_id": "fb_0", robot_name: "none", model: "SMR250", "task_id": "", "mode": 8, "battery_percent": 100 },
  //   { "robot_id": "fb_1", robot_name: "none", model: "SMR250", "task_id": "", "mode": 7, "battery_percent": 100 },
  //   { "robot_id": "fb_2", robot_name: "none", model: "SMR250", "task_id": "", "mode": 6, "battery_percent": 100 },
  //   { "robot_id": "fb_3", robot_name: "none", model: "SMR250", "task_id": "", "mode": 5, "battery_percent": 100 },
  //   { "robot_id": "fb_4", robot_name: "none", model: "SMR250", "task_id": "", "mode": 4, "battery_percent": 100 },
  //   { "robot_id": "fb_5", robot_name: "none", model: "SMR250", "task_id": "", "mode": 3, "battery_percent": 100 },
  //   { "robot_id": "fb_6", robot_name: "none", model: "SMR250", "task_id": "", "mode": 2, "battery_percent": 100 },
  //   { "robot_id": "fb_7", robot_name: "none", model: "SMR250", "task_id": "", "mode": 1, "battery_percent": 100 },
  //   { "robot_id": "fb_8", robot_name: "none", model: "SMR250", "task_id": "", "mode": 0, "battery_percent": 100 },
  // ];
  // updateFleetWidgets3(mockFleetData_);

  // --- client pull by swarm APIs ---
  updateFleetWidgets3(fleetData2_);
  updateAgentModalContentFromFleetState(fleetData2_);

  updateFlowsWidgets3(flowsData2_);
  updateArtifactsWidgets3(artifactData2_);
}, 1000);

// ------ Render Widgets ------
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


// ======================
//       Load Ready 
// ======================
$(async function () {
  'use strict'
  initDataAsync();

  // ------ dashboard initialization ------
  try {
    var isDarkMode = getSavedTheme() === 'dark';
    toggleContentDarkTheme(isDarkMode);
  } catch (err) {
    console.error("load fleet configuration error: " + err);
  }

  // --- flow statistics data binding ---
  lcOptions_ = {
    axisLabels: { show: false },
    grid: {
      borderColor: '#ffffff',
      borderWidth: 1,
      tickColor: '#808080' // gray
    },
    series: {
      color: '#3c8dbc',
      lines: {
        lineWidth: 4,
        show: true,
        fill: false,
      },
    },
    xaxis: {
      showTickLabels: 'none',
      color: 'white',
      tickColor: 'white',
      tickLength: 0,
      showTicks: 'major',
      font: { size: 15 }, //critical to font tick font size
      axisLabel: "past interval (sec.)"
    },
    yaxis: {
      min: 0,
      max: 20,
      showTickLabels: 'none',
      color: 'white',
      tickColor: 'white',
      showTicks: 'major',
      font: { size: 15 }, //critical to font tick font size
      // autoScale: 'loose',
      // autoScaleMargin: 1,
      axisLabel: "count"
    }
  };

  lcData_ = [{ data: getSwarmData() }];
  interactivePlot_ = $.plot('#flow-line-chart', lcData_, lcOptions_);


  // --- [event binding] Realtime toggle ---
  // $('#realtime .btn').click(function () {
  //   var toggleState = this.getAttribute('data-toggle');
  //   if (!toggleState) { return; }

  //   rtTrigger = (toggleState === 'on') ? 'on' : 'off';
  //   update();
  // })

  // --- data sourcing --- 
  pollFleetStates();
  pollFlowStates();
  adjustPieChart();

})

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'dashboard');
  if (statusData.role !== "admin") {
    hideLink = true;
  }

  switchViews();
  switchArtifactViews();
  adjustLayouts();
}


// ==============================
//    ROS MESSAGES ON UI 
// ==============================
let flowListCache_ = [];
function updateFlowsWidgets3(_listView) {
  console.log(_listView);
  let flowList = _listView.filter(job => job.pid === 0);
  let fStatus = _.chain(flowList).map('status').countBy().value();
  let flowStates = [fStatus[0] || 0, fStatus[1] || 0, fStatus[2] || 0, fStatus[3] || 0, fStatus[4] || 0];

  let taskList = _listView.filter(job => job.pid !== 0);
  let tStatus = _.chain(taskList).map('status').countBy().value();
  let taskStates = [tStatus[0] || 0, tStatus[1] || 0, tStatus[2] || 0, tStatus[3] || 0, tStatus[4] || 0];

  if (!_.isEqual(prevPPData_, flowStates)) {
    updatePPGraph(planProgressChart, flowStates);
    prevPPData_ = Object.assign([], flowStates);
    ppData_ = Object.assign([], flowStates);
  }

  if (!_.isEqual(prevTPData_, taskStates)) {
    updatePPGraph(taskPieChart, taskStates);
    prevTPData_ = Object.assign([], taskStates);
  }

  // --- task flows panel update ---
  var flowTable = document.querySelector('#flows-table');
  var children = flowTable.querySelectorAll("tr[class*=treegrid-]");
  var tableDiv = $('.table-responsive');
  // console.log(children);

  var flowPanel = $('.flow-list-panel');
  // --- if ID is existing, update. otherwise, append ---
  if ((typeof _listView === 'undefined' || _listView.length === 0)) {
    if (tableDiv.css('display') === 'block') {
      tableDiv.hide();

      const emptyTemplate = document.querySelector('#no-flow-row');
      const emptyNode = document.importNode(emptyTemplate.content, true);
      flowPanel.append(emptyNode);
      applyFontSize(getSavedFontSize(), '.flow-list-panel');
    }
    return;
  }

  flowPanel.find('.no-flows-caption').remove();
  tableDiv.show();

  // --- update ---
  // console.log(_listView);
  if (children.length !== 0) {
    children.forEach(c => {
      const newChild = document.createElement("span");
      const newChildText = document.createTextNode(c.cells[0].lastChild.textContent);
      newChild.appendChild(newChildText);
      c.cells[0].replaceChild(newChild, c.cells[0].lastChild);

      // --- existing job id ---
      var jobId = c.cells[1].firstChild.title;
      var updatedInfo = _listView.find(lv => lv.id === jobId);
      // console.log(updatedInfo);
      if (updatedInfo === undefined) {
        c.remove();
        return;
      }
      // console.log(updatedInfo);
      var statusMsg = (updatedInfo.hasOwnProperty('statusMsg')) ? updatedInfo.statusMsg : "";
      // --- updated only when status changes ---
      // var currState = parseInt($(c.cells[2].firstChild).attr('value'));
      // if (currState !== updatedInfo.status) {
      c.cells[3].innerHTML = statusFormatter(updatedInfo.status, { statusMsg: statusMsg }, "");
      $(c.cells[5].firstChild.firstChild).removeClass().addClass(`fa ${CmdOptions_[updatedInfo.status]}`);
      // }

      c.cells[4].innerHTML = progressFormatter(updatedInfo.progressValue, { status: updatedInfo.status, progressValue: updatedInfo.progressValue }, "");
      _listView = _listView.filter(lv => lv.id !== jobId);
    });
    applyFontSize(getSavedFontSize(), '.flow-list-panel');
  }

  // --- if new flows, append on flows list table ---
  if (!_listView.length) { return; }
  $table.bootstrapTable('append', _listView);
}

function updateFlowsWidgets2(_pieChart, _listView) {
  let arrStates = [0, 0, 0, 0, 0];
  for (let i in _pieChart) {
    arrStates[i] = _pieChart[i];
  }
  // console.log(arrStates);

  if (!_.isEqual(prevPPData_, arrStates)) {
    updatePPGraph(planProgressChart, arrStates);
    updatePPGraph(taskPieChart, arrStates);
    prevPPData_ = Object.assign([], arrStates);
    ppData_ = Object.assign([], arrStates);
  }

  const cvtState = ['QUEUED', 'ACTIVE', 'COMPLETED', 'FAILED', 'PAUSED'];
  const cvtStateColor = ['color-queued', 'color-active', 'color-completed', 'color-failed', 'color-paused'];
  // const cvtBadge = ['badge-info', 'badge-success', 'badge-secondary', 'badge-danger'];
  // const cvtProgressBar = ['bg-info', 'bg-success', 'bg-secondary', 'bg-danger'];

  // --- task flows panel update ---
  var flowTable = document.getElementById('fleet-flows-body');
  var children = flowTable.getElementsByTagName('tr');
  var tableDiv = $('.table-responsive');
  // console.log(children);

  var flowPanel = $('.flow-list-panel');
  // --- if ID is existing, update. otherwise, append ---
  if ((typeof _listView === 'undefined' || _listView.length === 0)) {
    if (tableDiv.css('display') === 'block') {
      // $('#fleet-flows-body').empty();
      const emptyTemplate = document.querySelector('#no-flow-row');
      const emptyNode = document.importNode(emptyTemplate.content, true);

      tableDiv.hide();
      flowPanel.append(emptyNode);
    }
    return;
  }

  flowPanel.find('.no-flows-caption').remove();
  tableDiv.show();

  if (children.length !== 0) {
    for (let child of children) {
      var flowId = child.querySelector('.flow-id').title;
      const target = _listView.filter(ts => ts.flow_id === flowId);
      _listView = _listView.filter(ts => ts.flow_id !== flowId);
      if (target.length !== 0) {
        var subNode = child.querySelector('.flow-status');
        subNode.textContent = cvtState[target[0].state];
        subNode.className = `flow-status badge ${cvtStateColor[target[0].state]}`;

        subNode = child.querySelector('.flow-progress');
        var strPercent = target[0].progress + '%';
        subNode.textContent = strPercent;
        subNode.style.width = strPercent;
        subNode.className = `flow-progress progress-bar progress-bar-striped ${cvtStateColor[target[0].state]}`
      }
      else {
        child.parentNode.removeChild(child);
      }
    }
  }

  _listView.forEach(function (task) {
    // console.log(task);
    const template = document.querySelector('#flow-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.flow-id');
    var flowID = task.flow_id;
    subNode.setAttribute("title", flowID);
    subNode.textContent = (flowID.length > 6) ? `${flowID.substring(0, 6)}...` : flowID;

    subNode = node.querySelector('.flow-name');
    var flowName = task.flow_name;
    subNode.setAttribute("title", flowName);
    subNode.textContent = (flowName.length > 6) ? `${flowName.substring(0, 6)}...` : flowName;

    subNode = node.querySelector('.flow-status');
    subNode.textContent = cvtState[task.state];
    subNode.className = `flow-status badge ${cvtStateColor[task.state]}`;

    subNode = node.querySelector('.flow-progress');
    var taskProgress = task.progress + '%';
    subNode.textContent = taskProgress;
    subNode.style.width = taskProgress;
    subNode.className = `flow-progress progress-bar progress-bar-striped ${cvtStateColor[task.state]}`

    flowTable.appendChild(node);
  });

}

function updateFlowsWidgets(_taskStates) {
  // console.log(_taskStates);
  // --- pie chart update ---
  var states = _taskStates.msg.map(ts => ts.state);
  var cvtStates = _.countBy(states);
  var arrStates = [cvtStates[0] || 0, cvtStates[1] || 0, cvtStates[2] || 0, cvtStates[3] || 0, cvtStates[3] || 0];
  // console.log(arrStates);
  if (!_.isEqual(prevPPData_, arrStates)) {
    updatePPGraph(planProgressChart, arrStates);
    updatePPGraph(taskPieChart, arrStates);
    prevPPData_ = Object.assign([], arrStates);
    ppData_ = Object.assign([], arrStates);
  }

  // --- protections ---
  const cvtState = ['QUEUED', 'ACTIVE', 'COMPLETED', 'FAILED', 'PAUSED'];
  const cvtStateColor = ['color-queued', 'color-active', 'color-completed', 'color-failed', 'color-paused'];
  // const cvtBadge = ['badge-info', 'badge-success', 'badge-secondary', 'badge-danger'];
  // const cvtProgressBar = ['bg-info', 'bg-success', 'bg-secondary', 'bg-danger'];

  var taskFleet = _taskStates.fleet;

  if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append('<div class="card card-dark col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
  }
  if (taskFleet !== $('#fleet-select').val()) { return };

  // --- task flows panel update ---
  var flowTable = document.getElementById('fleet-flows-body');
  var children = flowTable.getElementsByTagName('tr');
  // console.log(children);

  if (children.length !== 0) {
    for (let child of children) {
      var flowId = child.querySelector('.flow-id').textContent;
      // console.log(flowId);
      const target = _taskStates.msg.filter(ts => ts.task_id === flowId);
      // console.log(target);
      if (target.length !== 0) {
        var subNode = child.querySelector('.flow-status');
        subNode.textContent = cvtState[target[0].state];
        subNode.className = `flow-status badge ${cvtStateColor[target[0].state]}`;

        subNode = child.querySelector('.flow-progress');
        var strPercent = target[0].complete_percent + '%';
        subNode.textContent = strPercent;
        subNode.style.width = strPercent;
        subNode.className = `flow-progress progress-bar progress-bar-striped ${cvtStateColor[target[0].state]}`
      }
      else {
        child.parentNode.removeChild(child);
      }
    }
    return;
  }

  // --- if ID is existing, update. otherwise, append ---
  _taskStates.msg.forEach(function (task) {
    // console.log(task);
    const template = document.querySelector('#flow-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.flow-id');
    subNode.textContent = task.task_id;

    subNode = node.querySelector('.flow-name');
    subNode.textContent = task.task_id;

    subNode = node.querySelector('.flow-status');
    subNode.textContent = cvtState[task.state];
    subNode.className = `flow-status badge ${cvtStateColor[task.state]}`;

    subNode = node.querySelector('.flow-progress');
    var taskProgress = task.complete_percent + '%';
    subNode.textContent = taskProgress;
    subNode.style.width = taskProgress;
    subNode.className = `flow-progress progress-bar progress-bar-striped ${cvtStateColor[task.state]}`

    flowTable.appendChild(node);
  });
}

// Artifact
let artifactListCache_ = [];
function updateArtifactsWidgets3(_listView) {
  // --- data transformation ---
  var status = _.chain(_listView).map('status').countBy().value();
  let arrStates = [0, 0, 0, 0];
  let target_dict = {};
  let fake_mode = 0
  if (_listView == undefined) { return; }
  _listView.forEach((artifact) => {
    if (Object.keys(artifact.conf_info).length == 0) {
      artifact.conf_info['version'] = '0.0';
    }

    if (artifact.conf_info.version === '0.0') {
      if (artifact.connection_status == 0) {
        arrStates[0] += 1;
        target_dict[artifact.id] = 0;
      } else {
        arrStates[3] += 1;
        target_dict[artifact.id] = 3;
      }
    } else {
      if (artifact.connection_status == 0) {
        if (artifact.state.hasOwnProperty('state')) {
          if (artifact.state.state === 'InService') {
            arrStates[0] += 1;
            target_dict[artifact.id] = 0;
          } else if (artifact.state.state === 'Error') {
            arrStates[2] += 1;
            target_dict[artifact.id] = 2;
          } else {
            arrStates[1] += 1;
            target_dict[artifact.id] = 1;
          }
        } else {
          arrStates[2] += 1;
          target_dict[artifact.id] = 2;
        }
      } else {
        arrStates[3] += 1;
        target_dict[artifact.id] = 3;
      }
    }
  })

  if (!_.isEqual(prevArtData_, arrStates)) {
    updateARTraph(artifactUtilChart, arrStates);
    prevArtData_ = Object.assign([], arrStates);
    artData_ = Object.assign([], arrStates);
  }

  const cvtMode = ['In service', 'No data received', 'Error', 'Disconnected'];
  const cvtModeColors = ['status-inservice', 'status-no-data-received', 'status-error', 'status-disconnected'];
  var artifactTable = document.getElementById('fleet-artifacts-body');
  var $artifactTable = $('#fleet-artifacts-body');
  var children = artifactTable.getElementsByTagName("li");
  // console.log(children);

  // --- if ID is existing, update. otherwise, append ---
  if (_listView.length === 0) {
    $('#fleet-artifacts-body').empty();
    const emptyTemplate = document.querySelector('#no-artifacts-row');
    const emptyNode = document.importNode(emptyTemplate.content, true);
    $artifactTable.append(emptyNode);
    updateArtifactListSwitchStatus(true);
    applyFontSize(getSavedFontSize(), '#fleet-artifacts-body');
    return;
  }

  $artifactTable.find('.no-artifacts-caption').remove();

  // console.log(_listView);
  if (children.length !== 0) {
    for (let child of children) {
      var artifact_id = child.querySelector('.artifact-id').textContent;
      console.log(artifact_id)
      const target = _listView.find(fs => fs.id === artifact_id);
      if (!target) {
        child.parentNode.removeChild(child);
        continue;
      }

      var subNode = child.querySelector('.artifact-status');
      subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
      subNode.classList.add(cvtModeColors[target_dict[artifact_id]]);  // then add target status class
      subNode.textContent = cvtMode[target_dict[artifact_id]];

      _listView = _listView.filter(fs => fs.id !== artifact_id);
    }
  }

  if (!_listView.length) { return; }

  updateArtifactListSwitchStatus(false);
  // --- if ID is existing, update. otherwise, append ---
  _listView.forEach((artifact) => {
    console.log(artifact);
    const template = document.querySelector('#artifacts-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.artifact-name');
    let artifact_name = artifact.hasOwnProperty('name') ? artifact.name : 'none';
    avoidNoneName(subNode, artifact_name, artifact.id);
    subNode = node.querySelector('.artifact-id');
    subNode.textContent = artifact.id;

    // subNode = node.querySelector('.darkview a');
    // subNode.setAttribute('data-id', artifact.id);

    subNode = node.querySelector('img.thumb');
    var model = artifact.conf_info.type.toLowerCase();
    console.log(model);
    console.log(artifactAssets_[model])
    // subNode.src = avatarMap_[model] || avatarMap_['none'];
    subNode.src = '/images/' + artifactAssets_[model].thumbnail;
    subNode = node.querySelector('.artifact-status');
    subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
    subNode.classList.add(cvtModeColors[target_dict[artifact.id]]);  // then add target status class
    subNode.textContent = cvtMode[target_dict[artifact.id]];

    // console.log(node);
    artifactTable.appendChild(node);
  });
  applyFontSize(getSavedFontSize(), '#fleet-artifacts-body');
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
  var a_sec = a.start_time.sec + a.start_time.nanosec / 1e9;
  var b_sec = b.start_time.sec + b.start_time.nanosec / 1e9;
  if (a_sec === b_sec) { return 0; }
  return (a_sec < b_sec) ? -1 : 1;
}

async function switchFleetCallback() {
  console.log('--- switch fleet initialization ---');
  initWidgets();

  // try {
  // var selFleet = getSavedFleet();
  // selFleet = getSavedFleet();
  // }
  // catch (err) {
  // console.error("Get Fleet Settings error: " + err);
  // }
}

function initWidgets() {
  // --- flush pieChart history ---
  updatePPGraph(planProgressChart, [0, 0, 0, 0, 0]);
  updatePPGraph(taskPieChart, [0, 0, 0, 0, 0]);
  updateRUGraph(robotUtilChart, [0, 0, 0, 0, 0, 0, 0, 0, 0]);
  updateARTraph(artifactUtilChart, [0, 0, 0]);

  // --- flush fleet data history ---
  fleetData2_ = [];
  flowsData2_ = [];
  artifactData2_ = [];

  // --- flush tables history ---
  $('#fleet-agents-body').empty();
  $('#fleet-flows-body').empty();
  $('#fleet-artifacts-body').empty();

}

function updateFleetPieChart(_fleetState) {
  var states = _fleetState.map(fs => fs.mode);
  var cvtStates = _.countBy(states);
  // console.log(cvtStates);
  var arrStates = [cvtStates[0] || 0, cvtStates[1] || 0, cvtStates[2] || 0, cvtStates[3] || 0, cvtStates[4] || 0, cvtStates[5] || 0, cvtStates[6] || 0, cvtStates[7] || 0, cvtStates[8] || 0];
  // console.log(arrStates);
  ruData_ = Object.assign([], arrStates);
  // console.log(ruData_);
  if (!_.isEqual(prevRUData_, arrStates)) {
    updateRUGraph(robotUtilChart, arrStates);
    prevRUData_ = Object.assign([], arrStates);
  }
}

function updateFleetWidgets2(_pieChart, _listView) {
  let arrStates = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  // for (let obj of _pieChart) {
  //   arrStates[obj.mode] = obj.number;
  // }
  for (let i in _pieChart) {
    arrStates[i] = _pieChart[i];
  }
  // console.log(arrStates);

  if (!_.isEqual(prevRUData_, arrStates)) {
    updateRUGraph(robotUtilChart, arrStates);
    prevRUData_ = Object.assign([], arrStates);
    ruData_ = Object.assign([], arrStates);
  }

  const cvtMode = ['IDLE', 'CHARGING', 'MOVING', 'PAUSED', 'WAITING', 'EMERGENCY', 'GOING_HOME', 'DOCKING', 'UNINITIALIZED'];
  const cvtModeColors = ['status-idle', 'status-charging', 'status-moving', 'status-paused', 'status-waiting', 'status-emergency', 'status-going_home', 'status-docking', 'status-uninitialized'];

  // -------- fleet agents update  --------
  var agentTable = document.getElementById('fleet-agents-body');
  var $agentTable = $('#fleet-agents-body');
  var children = agentTable.getElementsByTagName("li");
  // console.log(children);

  // --- if ID is existing, update. otherwise, append ---
  if (typeof _listView === 'undefined' || _listView.length === 0) {
    $('#fleet-agents-body').empty();
    const emptyTemplate = document.querySelector('#no-agent-row');
    const emptyNode = document.importNode(emptyTemplate.content, true);
    // agentTable.appendChild(emptyNode);
    $agentTable.append(emptyNode);
    updateAgentListSwitchStatus(true);
    return;
  }

  $agentTable.find('.no-agents-caption').remove();

  if (children.length !== 0) {
    for (let child of children) {
      var agentId = child.querySelector('.agent-id').textContent;
      const target = _listView.filter(fs => fs.robot_id === agentId);
      _listView = _listView.filter(fs => fs.robot_id !== agentId);
      if (target.length !== 0) {
        var subNode = child.querySelector('.agent-status');
        subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
        subNode.classList.add(cvtModeColors[target.mode]);  // then add target status class
        subNode.textContent = correctAgentStatus(cvtMode[target[0].mode]);
      } else {
        child.parentNode.removeChild(child);
      }
    }
    return;
  }

  if (typeof _listView === 'undefined' || _listView.length === 0) { return; }
  updateAgentListSwitchStatus(false);
  // --- if ID is existing, update. otherwise, append ---
  _listView.forEach((agent) => {
    // console.log(agent);
    const template = document.querySelector('#agent-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.agent-name');
    avoidNoneName(subNode, agent.robot_name, agent.robot_id);
    subNode = node.querySelector('.agent-id');
    subNode.textContent = agent.robot_id;

    subNode = node.querySelector('.darkview a');
    subNode.setAttribute('data-id', agent.robot_id);

    subNode = node.querySelector('img.thumb');
    var model = agent.model.toLowerCase();
    // console.log(model);
    subNode.src = avatarMap_[model] || avatarMap_['none'];

    subNode = node.querySelector('.agent-status');
    subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
    subNode.classList.add(cvtModeColors[agent.mode]);  // then add target status class
    subNode.textContent = correctAgentStatus(cvtMode[agent.mode]);

    subNode = node.querySelector('.agent-task-id');
    subNode.textContent = agent.task_id;

    subNode = node.querySelector('.agent-battery');
    subNode.textContent = agent.battery_percent + "%";

    // console.log(node);
    agentTable.appendChild(node);
  });
}

// ====== [UI] UI data schema and style spec. ======
function updateFleetWidgets3(_listView) {
  console.log(_listView);

  // --- data transformation ---
  var status = _.chain(_listView).map('mode').countBy().value();
  // let arrStates = [status[0] || 0, status[1] || 0, status[2] || 0, status[3] || 0, status[4] || 0, status[5] || 0, status[6] || 0, status[7] || 0, status[8] || 0];
  // TODO: agent mode adaptor
  // let activeCount = status[2] || 0;
  // activeCount += status[6] || 0;
  // activeCount += status[7] || 0;
  // console.log(activeCount);
  // let arrStates = [status[0] || 0, status[1] || 0, activeCount, status[3] || 0, status[4] || 0, status[5] || 0, status[8] || 0];
  let arrStates = [status[0] || 0, status[1] || 0, status[2] || 0, status[3] || 0, status[4] || 0, status[5] || 0, status[6] || 0, status[7] || 0];

  if (!_.isEqual(prevRUData_, arrStates)) {
    updateRUGraph(robotUtilChart, arrStates);
    prevRUData_ = Object.assign([], arrStates);
    ruData_ = Object.assign([], arrStates);
  }

  // const cvtMode = ['IDLE', 'CHARGING', 'MOVING', 'PAUSED', 'WAITING', 'EMERGENCY', 'GOING_HOME', 'DOCKING', 'UNINITIALIZED'];
  // const cvtModeColors = ['status-idle', 'status-charging', 'status-moving', 'status-paused', 'status-waiting', 'status-emergency', 'status-going_home', 'status-docking', 'status-uninitialized'];
  // TODO: agent mode adaptor
  const cvtMode = ['IDLE', 'CHARGING', 'ACTIVE', 'PAUSED', 'WAITING', 'EMERGENCY', 'UNINITIALIZED', 'DISCONNECTED'];
  const cvtModeColors = ['status-idle', 'status-charging', 'status-moving', 'status-paused', 'status-waiting', 'status-emergency', 'status-uninitialized', 'status-disconnected'];


  var agentTable = document.getElementById('fleet-agents-body');
  var $agentTable = $('#fleet-agents-body');
  var children = agentTable.getElementsByTagName("li");
  // console.log(children);

  // --- if ID is existing, update. otherwise, append ---
  if (!_listView || _listView.length === 0) {
    $('#fleet-agents-body').empty();
    const emptyTemplate = document.querySelector('#no-agent-row');
    const emptyNode = document.importNode(emptyTemplate.content, true);
    $agentTable.append(emptyNode);
    updateAgentListSwitchStatus(true);
    applyFontSize(getSavedFontSize(), '#fleet-agents-body');
    return;
  }


  $agentTable.find('.no-agents-caption').remove();

  // TODO: agent mode adaptor
  // _listView.forEach((agent) => {
  //   agent.mode = (agent.mode === 6 || agent.mode === 7) ? 2 : agent.mode;
  //   agent.mode = (agent.mode === 8) ? 6 : agent.mode;
  // });

  // console.log(_listView);
  if (children.length !== 0) {
    for (let child of children) {
      var agentId = child.querySelector('.agent-id').textContent;
      const target = _listView.find(fs => fs.robot_id === agentId);
      if (!target) {
        child.parentNode.removeChild(child);
        continue;
      }

      var subNode = child.querySelector('.agent-status');
      subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
      subNode.classList.add(cvtModeColors[target.mode]);  // then add target status class
      subNode.textContent = correctAgentStatus(cvtMode[target.mode]);
      subNode = child.querySelector('.agent-more a button');
      if (target.connection_status) {
        subNode.setAttribute('class', 'btn btn-danger');
        subNode.textContent = 'DISCONN.';
      } else {
        subNode.setAttribute('class', 'btn btn-secondary');
        subNode.textContent = 'MORE';
      }

      subNode = child.querySelector('.agent-task-id');
      subNode.textContent = `${target.task_id.slice(0, 5)}...`;
      subNode.title = target.task_id;

      subNode = child.querySelector('.agent-battery');
      subNode.textContent = target.battery_percent + "%";
      // console.log(target)

      _listView = _listView.filter(fs => fs.robot_id !== agentId);
    }
  }

  if (!_listView.length) { return; }

  updateAgentListSwitchStatus(false);
  // --- if ID is existing, update. otherwise, append ---
  _listView.forEach((agent) => {
    console.log(agent);
    const template = document.querySelector('#agent-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.agent-name');
    avoidNoneName(subNode, agent.robot_name, agent.robot_id);
    if (agent.robot_id.length > 5) {
      subNode.textContent = `${agent.robot_id.slice(0, 5)}...`;
    } else {
      subNode.textContent = agent.robot_id;
    }
    subNode.title = agent.robot_id;

    subNode = node.querySelector('.agent-id');
    subNode.textContent = agent.robot_id;

    subNode = node.querySelector('.darkview a');
    subNode.setAttribute('data-id', agent.robot_id);

    subNode = node.querySelector('img.thumb');
    console.log(agent);
    console.log(agent.model);
    var model = agent.model.toLowerCase();
    // console.log(model);
    subNode.src = avatarMap_[model] || avatarMap_['none'];

    subNode = node.querySelector('.agent-status');
    subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
    subNode.classList.add(cvtModeColors[agent.mode]);  // then add target status class
    subNode.textContent = correctAgentStatus(cvtMode[agent.mode]);
    subNode.setAttribute('id', `${agent.robot_id}-mode-label`);

    subNode = node.querySelector('.agent-task-id');
    subNode.textContent = agent.task_id;

    subNode = node.querySelector('.agent-battery');
    subNode.textContent = agent.battery_percent + "%";

    // console.log(node);
    agentTable.appendChild(node);
  });
  applyFontSize(getSavedFontSize(), '#fleet-agents-body');
}

function updateFleetWidgets(_fleetState) {
  const cvtMode = ['IDLE', 'CHARGING', 'MOVING', 'PAUSED'];
  const cvtModeColors = ['status-idle', 'status-charging', 'status-moving', 'status-paused'];

  // console.log(_fleetState);
  // --- pie chart update ---
  _fleetState = _fleetState.msg;
  var states = _fleetState.map(ts => ts.mode);
  var cvtStates = _.countBy(states);
  // console.log(cvtStates);
  var arrStates = [cvtStates[0] || 0, cvtStates[1] || 0, cvtStates[2] || 0, cvtStates[3] || 0, cvtStates[4] || 0, cvtStates[5] || 0, cvtStates[6] || 0, cvtStates[7] || 0, cvtStates[8] || 0];
  // console.log(arrStates);
  ruData_ = Object.assign([], arrStates);
  // console.log(ruData_);
  if (!_.isEqual(prevRUData_, arrStates)) {
    updateRUGraph(robotUtilChart, arrStates);
    prevRUData_ = Object.assign([], arrStates);
  }

  // -------- fleet agents update  --------
  var agentTable = document.getElementById('fleet-agents-body');
  var children = agentTable.getElementsByTagName("li");
  // console.log(children);

  if (children.length !== 0) {
    for (let child of children) {
      var agentId = child.querySelector('.agent-id').textContent;

      const target = _fleetState.filter(fs => fs.robot_id === agentId);
      if (target.length !== 0) {
        var subNode = child.querySelector('.agent-status');
        subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
        subNode.classList.add(cvtModeColors[target.mode]);  // then add target status class
        subNode.textContent = correctAgentStatus(cvtMode[target[0].mode]);
      } else {
        child.parentNode.removeChild(child);
      }
    }
    return;
  }

  // --- if ID is existing, update. otherwise, append ---
  _fleetState.forEach((agent) => {
    // console.log(agent);
    const template = document.querySelector('#agent-row');
    const node = document.importNode(template.content, true);

    var subNode = node.querySelector('.agent-name');
    avoidNoneName(subNode, agent.robot_name, agent.robot_id);
    subNode = node.querySelector('.agent-id');
    subNode.textContent = agent.robot_id;

    subNode = node.querySelector('img.thumb');
    var model = agent.model.toLowerCase();
    console.log(model);
    subNode.src = avatarMap_[model] || avatarMap_['none'];

    subNode = node.querySelector('.agent-status');
    subNode.classList.remove(...cvtModeColors); // remove all the status class from cvtModeColors
    subNode.classList.add(cvtModeColors[agent.mode]);  // then add target status class
    subNode.textContent = correctAgentStatus(cvtMode[agent.mode]);

    subNode = node.querySelector('.agent-task-id');
    subNode.textContent = agent.task_id;

    subNode = node.querySelector('.agent-battery');
    subNode.textContent = agent.battery_percent + "%";

    // console.log(node);
    agentTable.appendChild(node);
  });
}


// ===========================
//       PIE CHART 
// ===========================
let per = getFontScalePercent(getSavedFontSize());
let pieTextFontSize = 16 * parseFloat(per) / 100;
let pieCenterTextFontSize_ = pieTextFontSize * 2;
let pieLegendTextFontSize_ = pieTextFontSize;

// --- robots utility ---
var rbtUtilData = {
  labels: [
    'Idle',
    'Charging',
    'Active',
    'Paused',
    'Waiting',
    'Emergency',
    // 'Going_home',
    // 'Docking',
    'Unintialized',
    'Disconnected',
  ],
  datasets: [
    {
      label: '# of Votes',
      // TODO: agent mode adaptor
      // data: [0, 0, 0, 0, 0, 0, 0, 0, 0],
      // backgroundColor: ['#c4c4c4', '#00d600', '#00a0ff', '#f39c12', '#3c8dbc', '#dc3545', '#17a2b8', '#ffc107', '#6c757d'],
      data: [0, 0, 0, 0, 0, 0, 0],
      backgroundColor: ['#c4c4c4', '#00d600', '#00a0ff', '#f39c12', '#3c8dbc', '#dc3545', '#6c757d'],
    }
  ]
}

var robotUtil_init_data = {
  element_id: 'agent-pie-chart',
  chart_data: rbtUtilData,
  style_width: 100,
  style_height: 100,
  sum_data_index: 2
};
var robotUtilChart = new DoughnutChart(robotUtil_init_data).generateDoughnutChart();

// artifact pie chart
var artUtilData = {
  labels: [
    'In service',
    'No data received',
    'Error',
    'Disconnect'
  ],
  datasets: [
    {
      data: [0, 0, 0, 0],
      backgroundColor: ['#00d600', '#f39c12', '#dc3545', '#d2d6d3'],
      // borderColor: [
      //   '#dc3545',
      //   '#dc3545',
      //   '#dc3545',
      //   '#dc3545'
      // ],
      clip: { left: 5, top: false, right: -2, bottom: 0 }
    }
  ]
}

var arti_init_data = {
  element_id: 'artifact-pie-chart',
  chart_data: artUtilData,
  style_width: 250,
  style_height: 250,
  sum_data_index: 0
};
var artifactUtilChart = new DoughnutChart(arti_init_data).generateDoughnutChart();

// --- flow pie chart ---
var flowdata = {
  labels: [
    'Queued',
    'Active',
    'Completed',
    'Failed',
    'Paused'
  ],
  datasets: [
    {
      data: [0, 0, 0, 0, 0],
      backgroundColor: ['#6c757d', '#00a0ff', '#00d600', '#dc3545', '#f39c12'],
    }]
};

var plan_init_data = {
  element_id: 'flow-pie-chart',
  chart_data: flowdata,
  style_width: 100,
  style_height: 100,
  sum_data_index: 1
};
var planProgressChart = new DoughnutChart(plan_init_data).generateDoughnutChart();

// --- task pie chart ---
let taskdata = {
  labels: [
    'Queued',
    'Active',
    'Completed',
    'Failed',
    'Paused'
  ],
  datasets: [
    {
      data: [0, 0, 0, 0, 0],
      backgroundColor: ['#6c757d', '#00a0ff', '#00d600', '#dc3545', '#f39c12'],
    }]
};

let task_init_data = {
  element_id: 'task-pie-chart',
  chart_data: taskdata,
  style_width: 100,
  style_height: 100,
  sum_data_index: 1
};
let taskPieChart = new DoughnutChart(task_init_data).generateDoughnutChart();

function switchViews() {
  $("button.switcher").bind("click", function (e) {
    e.preventDefault();

    var thisID = $(this).attr("id");
    var fltAgents = $("ul#fleet-agents-body");


    $("#listview").removeClass("active");
    $("#gridview").removeClass("active");
    $(this).addClass("active");

    fltAgents.removeClass("list");
    fltAgents.removeClass("grid");

    if (thisID === "gridview") {
      fltAgents.addClass("grid");
      return;
    }

    if (thisID === "listview") {
      fltAgents.addClass("list");
      return;
    }
  });
}

function switchArtifactViews() {
  $("button.artifact_switcher").bind("click", function (e) {
    e.preventDefault();

    var thisID = $(this).attr("id");
    var fltAgents = $("ul#fleet-artifacts-body");


    $("#artifact_listview").removeClass("active");
    $("#artifact_gridview").removeClass("active");
    $(this).addClass("active");

    fltAgents.removeClass("list");
    fltAgents.removeClass("grid");

    if (thisID === "artifact_gridview") {
      fltAgents.addClass("grid");
      return;
    }

    if (thisID === "artifact_listview") {
      fltAgents.addClass("list");
      return;
    }
  });
}

function updateAgentListSwitchStatus(isLocked) {
  $('#gridview').prop('disabled', isLocked);
  $('#listview').prop('disabled', isLocked);
}

function updateArtifactListSwitchStatus(isLocked) {
  $('#artifact_gridview').prop('disabled', isLocked);
  $('#artifact_listview').prop('disabled', isLocked);
}

function adjustLayouts() {
  // --- make the dashboard widgets sortable Using jquery UI ---
  $('.connectedSortable').sortable({
    placeholder: 'sort-highlight',
    connectWith: '.connectedSortable',
    handle: '.card-header, .nav-tabs',
    forcePlaceholderSize: true,
    zIndex: 999999
  })
  $('.connectedSortable .card-header, .connectedSortable .nav-tabs-custom').css('cursor', 'move')
}

let plotData = [];
var samplingPoints = 40;
let swarmData_;
let flowPieChartData_;
let flowListViewData_;

let agentPieChartData_;
let agentListViewData_;

async function updateSwarmStatistics() {
  // swarmData_ = await restPostStatistics();
  if (queryData_ === undefined) { return; }
  console.log(queryData_);
  swarmData_ = queryData_.flowFigures;
  // console.log(swarmData_);
}

function getSwarmData() {
  updateSwarmStatistics();

  // --- remove the trailing element in array ---
  if (plotData.length > 0) {
    plotData.splice(-1);
  }

  // --- Do a random walk ---
  while (plotData.length < samplingPoints) {
    var y = (Math.random() * 40);

    if (y < 0) {
      y = 0;
    } else if (y > 40) {
      y = 40;
    }

    // --- prepend a value within range [0,40] ---
    plotData.unshift(y);
  }

  // --- zip the generated y values with the x values ---
  var res = [];
  for (var i = 0; i < plotData.length; ++i) {
    res.unshift([i, Math.round(plotData[i])]);
  }
  // console.log(swarmData_);
  let flowCounts = [];
  if (swarmData_ !== undefined) {
    let record = 0;
    for (var i = 0; i < samplingPoints; ++i) {
      let target = swarmData_.filter(sd => (sd.diff <= (i + 1)) && (sd.diff >= (i)));
      // console.log(target);
      if (target.length === 0) {
        flowCounts.unshift([i, record]);
        continue;
      }
      var nums = target.map(t => t.number);
      record += _.sum(nums);
      flowCounts.unshift([i, record]);
    }
  } else {
    for (var i = 0; i < samplingPoints; ++i) {
      flowCounts.unshift([i, 0]);
    }
  }
  // console.log(flowCounts);
  return flowCounts;
}

let interactivePlot_;
let lcOptions_;
let lcData_;
let prevHeight_;
let state_ = { folded: true };
let rtTrigger = 'off';
prevHeight_ = $('#flow-statistic-body').height();

console.log(prevHeight_);
function expandLineChartPanel() {
  setupGraph();
  state_.folded = !state_.folded;
  if (state_.folded) {
    $('#flow-statistic-body').height(prevHeight_);
    $('#flow-line-chart').css('height', '150px');
  } else {
    $('#flow-statistic-body').height(prevHeight_ + 150);
    $('#flow-line-chart').css('height', '280px');
  }
}

let isFolded_ = false;
function setupGraph() {
  if (!isFolded_) {
    lcOptions_.xaxis.showTickLabels = 'major';
    lcOptions_.yaxis.showTickLabels = 'major';
  } else {
    lcOptions_.xaxis.showTickLabels = 'none';
    lcOptions_.yaxis.showTickLabels = 'none';
  }

  lcOptions_.axisLabels.show = !lcOptions_.axisLabels.show;
  interactivePlot_ = $.plot("#flow-line-chart", lcData_, lcOptions_);
  isFolded_ = !isFolded_;
}

async function update() {
  interactivePlot_.setData([getSwarmData()])

  // Since the axes don't change, we don't need to call plot.setupGrid()
  interactivePlot_.draw()
  if (rtTrigger === 'on') {
    // setTimeout(update, updateInterval)
    setTimeout(update, 1000);
  }
}

// $('#flow-pie-btn').click(function () {
//   $('#realtime').hide();
// });

// $('#flow-line-btn').click(function () {
//   $('#realtime').show();
// });

$('#flow-switch-tab').click(function () {
  $('#flow-statistics-title').text('Flows Statistics');
})
$('#task-switch-tab').click(function () {
  $('#flow-statistics-title').text('Tasks Statistics');
})

$(document).on('click', '.firstbtn', function () {
  $('.firstbtn').prop('disabled', true);
  selRobotID = $(this).attr('data-id');
  console.log(`${selRobotID} clicked.`);
  resetRobotModalData();
  $('#r-agent-id').text(selRobotID);
  getRobotModalData(selRobotID).then(function () {
    $('#agent-modal').modal('show');
    $('.firstbtn').prop('disabled', false);
  });
});

function resetRobotModalData() {
  $('#agent-modal .p-content').empty().text('-');
}

async function getRobotModalData(robotID_) {
  await getScanRobot();
  if (scannedAgents.robots === undefined) return;
  var agentData = scannedAgents.robots.filter(function (robot) {
    return robot.robot_id === robotID_;
  })[0];
  if (agentData === undefined) {
    notificationMsg(3, `Fail to reach ${robotID_}!`);
    return;
  }
  updateAgentModalContent(agentData);
}

function updateAgentModalContent(data) {
  // console.log(data);
  // $('#r-agent-id').text(data.robot_id);
  // $('#r-agent-name').text(data.robot_name);
  let newName = '';
  if ('none' == data.robot_name.toLowerCase()) {
    newName = data.robot_name + ' - ' + data.robot_id;
  } else {
    newName = data.robot_name;
  }
  $('#r-agent-name').text(newName);


  $('#r-agent-mode').text(data.mode);
  $('#r-agent-model').text(data.model);
  $('#r-agent-version').text(data.sw_version);
  let path = window.location.protocol + "//" + data.ip + ":3000";
  if (hideLink) {
    $('#r-agent-ip').html(`${data.ip}`);
  } else {
    $('#r-agent-ip').html(`${data.ip}&ensp;<a href="${path}" target="_blank"><i class="fas fa-arrow-circle-right fa-lg"></i></a>`);
  }
  $('#r-agent-mac').text(data.mac);
}

function updateAgentModalContentFromFleetState(agentList) {
  // console.log(agentList);
  // var data = agentList.robots.find(robot => robot.robot_id === selRobotID);
  var data = agentList.find(robot => robot.robot_id === selRobotID);
  if (!data) { return; }

  $('#r-agent-status').text(correctAgentStatus(modeMap_[data.mode]));
  // $('#r-agent-task-id').text();
  $('#r-agent-battery').text(`${data.battery_percent}%`);
}

async function getScanRobot() {
  try {
    // await rmtTokenCheck();
    scannedAgents = await fetchScanRobots2(rmtToken_);
    console.log(scannedAgents);
    if (scannedAgents.detail !== undefined) {
      notificationMsg(3, scannedAgents.detail);
    }
  } catch (err) {
    console.error(err);
    notificationMsg(3, 'ROBOT SCANNED ERROR!');
  }
}


// =======================================
//     FLOWS LIST TREE TABLE GRID VIEW 
// =======================================
// --- Tree View Table ---
let flowsData2_ = [];
let fleetData2_ = [];
let artifactData2_ = [];

let $table = null;
$(document).ready(function () {
  $table = $('#flows-table');
  $table.bootstrapTable({
    data: [],
    idField: 'id',
    dataType: 'jsonp',
    columns: [
      { field: 'name', title: 'Name', align: 'left' },
      { field: 'id', title: 'ID', formatter: 'idFormatter' },
      { field: 'assignee', title: 'Assigned to', formatter: 'assigneeFormatter' },
      { field: 'status', title: 'Status', sortable: true, formatter: 'statusFormatter' },
      { field: 'progressValue', title: 'Progress', formatter: 'progressFormatter' },
      { field: 'operate', title: 'Operation', align: 'center', events: operateEvents, formatter: 'operateFormatter' },
    ],
    treeShowField: 'name', // unfolded column
    parentIdField: 'pid', // parent id
    onResetView: function (data) {
      $table.treegrid({
        initialState: 'expanded',// 'collapsed'|'expanded'
        treeColumn: 0,
        onChange: function () {
          $table.bootstrapTable('resetWidth');
        }
      });
    },
  });

  // --- remove `Loading, please wait...` ---
  $('.fixed-table-container').find('.fixed-table-loading').remove();
});

// --- status formatter ---
function idFormatter(value, row, index) {
  return `<div title="${value}">${value.slice(0, 5)}...</div>`
}

// --- assignee formatter ---
function assigneeFormatter(value, row, index) {
  let assigneeName = (value.length > 5) ? `${value.slice(0, 5)}...` : value;
  return `<div title="${value}">${assigneeName}</div>`
}

// --- status formatter ---
function statusFormatter(value, row, index) {
  // console.log(row);
  const cvtState = ['QUEUED', 'ACTIVE', 'COMPLETED', 'FAILED', 'PAUSED'];
  const cvtBadge = ['color-queued', 'color-active', 'color-completed', 'color-failed', 'color-paused'];

  var val = cvtState[value];
  var badge = cvtBadge[value];
  var info = (row.statusMsg) ? (`&nbsp;<i title="${row.statusMsg}" class="fa fa-info-circle"></i>`) : "";

  return `<div align="left" value=${value}><span class="flow-status ribbon badge ${badge}">${val}</span>${info}</div>`;
}

// --- progress formatter ---
function progressFormatter(value, row, index) {
  const cvtProgressBar = ['color-queued', 'color-active', 'color-completed', 'color-failed', 'color-paused'];
  var progressBadge = cvtProgressBar[row.status];
  return `
    <div class="progress active" style="position:relative;">
      <div class="flow-progress progress-bar progress-bar-striped ${progressBadge}" style="width:${value}%;">
        <span style="position:absolute;display:block;width:100%;color:black;">${value}%</sapn>
      </div>
    </div>
  `;
}

// --- operation buttons formatter ----
// flowtaskState    [QUEUED, ACTIVE, COMPLETED, FAILED, PAUSED]
const CmdOptions_ = ['fa-pause', 'fa-pause', 'fa-pause', 'fa-redo', 'fa-play',];
function operateFormatter(value, row, index) {
  console.log(row);
  console.log(this);
  console.log(row.status);
  // var statIcon = (row.status !== 4) ? 'fa-pause' : 'fa-play';
  var statIcon = CmdOptions_[row.status];
  return [
    `<button type="button" class="RoleOfpause btn-small btn-circle" id="edit-${row.name}" style="margin-right:15px;"><i class="fa ${statIcon}"></i></button>`,
    `<button type="button" class="RoleOfdelete btn-small btn-circle" id="delete-${row.name}" style="margin-right:15px;"><i class="fa fa-trash" ></i></button>`
  ].join('');
}

// --- initialization of operation methods ---
const TASK_STATES_ = {
  pause: {
    cmd: 'pause',
    next: 'fa fa-play'
  },
  play: {
    cmd: 'resume',
    next: 'fa fa-pause'
  },
  redo: {
    cmd: 'behavior_retry',
    next: 'fa fa-pause'
  }
};
window.operateEvents = {
  'click .RoleOfdelete': function (e, value, row, index) {
    conductJobCommand(row, 'delete');
  },
  'click .RoleOfpause': async function (e, value, row, index) {
    var domNode = (e.target.nodeName === "BUTTON") ? e.target.firstChild : e.target;
    console.log(domNode)
    var attr = domNode.getAttribute('class');
    console.log(attr);
    var token = attr.split('-')[1] || 'resume'; // fetch the operation keyword
    console.log(token)
    var opCmd = TASK_STATES_[token].cmd;

    var res = await conductJobCommand(row, opCmd);
    if (!res.ok) { return; }

    domNode.setAttribute('class', TASK_STATES_[token].next);
  }
};

// --- operation callbacks ---
async function conductJobCommand(_row, _op) {
  // console.log(_op);
  if (!confirm(`${_op} the Job? (ID: ${_row.id}`)) { return; }

  // --- user choose Yes case ---
  var res = '';
  if (_op === 'delete') {
    res = await fetchDeleteTask(rmtToken_, _row.id)
  } else {
    res = await fetchPutTask(rmtToken_, _row.id, _op);
  }
  const data = await res.json();

  // --- [Connection] health check ---
  if (!res.ok) {
    console.log(data);
    notificationMsg(3, `[CONN.] ${data}`);
    return false;
  }

  // --- [Swarm System] health check ---
  const sysStateStyle = (_.inRange(data.system_status_code, 200, 300)) ? 1 : 3;
  notificationMsg(sysStateStyle, `[SWARM] ${data.system_message}`);
  return true;
}

function pollFlowStates(_inteval = 1000) {
  setInterval(async function () {
    var selFleet = $('#fleet-select').val();
    var res = await fetchGetFlowStates(rmtToken_, selFleet);
    var queryData = {};
    if (res.ok) {
      queryData = await res.json();
    }
    // console.log(queryData);

    flowsData2_ = [];
    for (f of queryData.swarm_data) {
      var flow = { "id": f.flow_id, "pid": 0, "assignee": "-", status: f.state, "name": f.flow_name, "progressValue": f.complete_percent };
      flowsData2_.push(flow);
      if (!f.tasks.length) { continue; }

      f.tasks.forEach((t) => {
        let assignee = (t.robot_id === "") ? "-" : t.robot_id;
        let task = { "id": t.task_id, "pid": f.flow_id, "assignee": assignee, status: t.state, statusMsg: t.status_msg, "name": t.task_name, "progressValue": t.complete_percent };
        flowsData2_.push(task);
      });
    }
    // console.log(flowsData2_);
  }, _inteval);
};

function pollFleetStates(_inteval = 1000) {
  setInterval(async function () {
    var currFleet = $('#fleet-select').val();
    var res = await fetchFleetStates(rmtToken_, currFleet, 'all');
    fleetData2_ = [];
    if (!res.ok) { return; }

    var queryData = await res.json();
    fleetData2_ = queryData.fleet_state.find(flt => flt.fleet_name === currFleet) || [];
    if (!fleetData2_) { return; }

    artifactData2_ = fleetData2_.artifacts;
    fleetData2_ = fleetData2_.robots;

    // ------ DATA ADAPTOR ------
    fleetData2_ = _.map(fleetData2_, (agent) => {
      agent.mode = (agent.mode === 6 || agent.mode === 7) ? 2 : agent.mode;
      agent.mode = (agent.mode === 8) ? 6 : agent.mode;
      agent.mode = (agent.connection_status) ? 7 : agent.mode; // use connection status to determine new status DISCONNECTED.
      return agent;
    });
    // console.log(fleetData2_);
  }, _inteval);
};

function adjustPieChart(_inteval = 1000) {
  setInterval(function () {
    if (window.innerWidth <= 1366 || window.outerWidth <= 1366) {
      robotUtilChart.options.legend.display = false;
      planProgressChart.options.legend.display = false;
      taskPieChart.options.legend.display = false;
      artifactUtilChart.options.legend.display = false;
      $('#agent-pie-chart').css("margin-left", "0px");
      $('#flow-pie-chart').css("margin-left", "0px");
      $('#task-pie-chart').css("margin-left", "0px");
    } else {
      robotUtilChart.options.legend.display = true;
      planProgressChart.options.legend.display = true;
      taskPieChart.options.legend.display = true;
      artifactUtilChart.options.legend.display = true;
      $('#agent-pie-chart').css("margin-left", "-20px");
      $('#flow-pie-chart').css("margin-left", "-20px");
      $('#task-pie-chart').css("margin-left", "-20px");
    }
  }, _inteval);
};