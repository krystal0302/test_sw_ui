/*
 * Author: Angela Kao
 * Contributor: John Wu 
 * Date: 29 Sep 2021
 * Revision: 13 Jan 2022
 * Description: Public file for every pages
 *
 **/

// ------------------------------
//  Data Source Common Utilities 
// ------------------------------

let userAuthFunctions_ = {
  "admin": {
    "dashboard": {
      functionName: 'Dashboard',
      dirPath: 'index.html',
      imgClass: 'fas fa-tachometer-alt',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "fleet": {
      functionName: 'Configuration',
      dirPath: 'fleet.html',
      imgClass: 'fas fa-th',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "liveMap": {
      functionName: 'Live View',
      dirPath: 'map_live.html',
      imgClass: 'fas fa-map-marked-alt',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "operation": {
      functionName: 'Operation',
      dirPath: 'operation.html',
      imgClass: 'fas fa-edit',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "taskTrigger": {
      functionName: 'TASK TRIGGER',
      dirPath: '#',
      lastItemUnderFleetHrchy: true,
      isTreeView: true
    },
    "map": {
      functionName: 'Map',
      dirPath: '#',
      imgClass: 'fas fa-map',
      isTreeView: true,
      treeItem: {
        'Create Map': 'slam.html',
        'Edit Map': 'map.html'
      }
    },
    "settings": {
      functionName: 'Settings',
      dirPath: '#',
      imgClass: 'fas fa-cog',
      isTreeView: true,
      treeItem: {
        'Swarm Core Settings': 'settings.html',
        'User Settings': 'user_settings.html'
      }
    },
    "logs": {
      functionName: 'Logs',
      dirPath: 'log.html',
      imgClass: 'fas fa-clipboard-list',
      isTreeView: false
    },
    "userManagement": {
      functionName: 'User Management',
      dirPath: 'manage_user.html',
      imgClass: 'fas fa-user',
      isTreeView: false
    },
    "help": {
      functionName: 'Help',
      dirPath: '#',
      imgClass: 'fas fa-check-circle',
      isTreeView: false
    }
  },
  "general": {
    "dashboard": {
      functionName: 'Dashboard',
      dirPath: 'index.html',
      imgClass: 'fas fa-tachometer-alt',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "fleet": {
      functionName: 'Configuration',
      dirPath: 'fleet.html',
      imgClass: 'fas fa-th',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "liveMap": {
      functionName: 'Live View',
      dirPath: 'map_live.html',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "operation": {
      functionName: 'Operation',
      dirPath: 'operation.html',
      imgClass: 'fas fa-edit',
      lastItemUnderFleetHrchy: false,
      isTreeView: false
    },
    "taskTrigger": {
      functionName: 'TASK TRIGGER',
      dirPath: '#',
      lastItemUnderFleetHrchy: true,
      isTreeView: true
    },
    "map": {
      functionName: 'Map',
      dirPath: '#',
      imgClass: 'fas fa-map',
      isTreeView: true,
      treeItem: {
        'Create Map': 'slam.html',
        'Edit Map': 'map.html'
      }
    },
    "settings": {
      functionName: 'Settings',
      dirPath: '#',
      imgClass: 'fas fa-cog',
      isTreeView: true,
      treeItem: {
        'Swarm Core Settings': 'settings.html',
        'User Settings': 'user_settings.html'
      }
    },
    "logs": {
      functionName: 'Logs',
      dirPath: 'log.html',
      imgClass: 'fas fa-clipboard-list',
      isTreeView: false
    },
    "help": {
      functionName: 'Help',
      dirPath: '#',
      imgClass: 'fas fa-check-circle',
      isTreeView: false
    }
  }
}


// ------------------------------
//      UI Common Utilities 
// ------------------------------

function genSideBarMenu(role, selPage, selPageLink) {
  createBrandLogo();
  for (const [roleKey, roleVal] of Object.entries(userAuthFunctions_)) {
    if (roleKey === role) {
      for (const [funcKey, funcVal] of Object.entries(roleVal)) {
        // console.log(funcVal);
        createSideBarItem(funcKey, funcVal, selPage, selPageLink);
      }
      chkTriggerListVisible();
    }
  }
}

function createBrandLogo() {
  var logoLink =
    `<a href="index.html" class="brand-link logo-switch">
      <img src="dist/img/FARobotLogo.png" alt="FARobot Logo" class="brand-image img-circle elevation-3 logo-xs">
      <img src="dist/img/FARobotLogo_lg.png" alt="FARobot Logo" class="brand-image logo-xl">
   </a>`;
  $('.sidebar').before(logoLink);
}

function createSideBarItem(key, obj, selPage, selPageLink) {
  let isSelectedPage = key === selPage;
  let isHelpPage = key === "help";
  let isTriggerList = key === 'taskTrigger';

  var $li = $('<li>');
  if (obj.isTreeView) {
    $li = createTreeViewTitle(isSelectedPage || isTriggerList);
  } else {
    $li.addClass('nav-item');
  }
  // fleet group function class
  if (typeof obj.lastItemUnderFleetHrchy !== 'undefined') {
    $li.addClass('fleet-group');
  }
  // trigger function add id for recognize
  if (isTriggerList) {
    $li.attr('id', 'trigger-list');
  }
  $li.append(createSideBarItemLink(isSelectedPage, isHelpPage, obj));
  if (obj.isTreeView) {
    $li.append(createTreeViewItem(obj.treeItem, selPageLink));
  }
  $('.nav-sidebar').append($li);

  // fleet group functions
  if (typeof obj.lastItemUnderFleetHrchy !== 'undefined' && obj.lastItemUnderFleetHrchy) {
    var $div = $('<div>').addClass('user-panel');
    var fleet_li = `<li class="nav-header">
                       <select id="fleet-select" class="form-control select-bold-text"">
                          <option>No Fleets</option>
                       </select>
                    </li>`;

    var $fleetGroupFunc = $('.nav-sidebar .fleet-group').detach();
    var $trigger_li = $('#trigger-list').detach();
    $div.append(fleet_li).append($fleetGroupFunc).append($trigger_li);
    $('.nav-sidebar').append($div);

    // bind switch fleet function
    $('#fleet-select').change(function () { switchFleet(_switchFleetCallback) });
  }
}

function createTreeViewTitle(needOpenMenu = true) {
  var $li = $('<li>').addClass('nav-item has-treeview');
  var classTag = (needOpenMenu) ? 'menu-open' : 'menu-close';
  $li.addClass(classTag);
  return $li;
}

function createSideBarItemLink(isActiveLink = true, isHelpLink = false, obj) {
  var $a = $('<a>');
  if (isHelpLink) {
    let path = window.location.protocol + "//" + window.location.hostname + ":5000/docs";
    $a.attr('href', path).addClass('nav-link');
  } else if (isActiveLink) {
    $a.attr('href', obj.dirPath).addClass('nav-link active');
  } else {
    $a.attr('href', obj.dirPath).addClass('nav-link');
  }

  var $p = $('<p>').text(obj.functionName);
  if (obj.isTreeView) {
    var $rightImg = $('<i>').addClass('right fas fa-angle-left');
    $p.append($rightImg);
  }

  if (typeof obj.imgClass !== 'undefined') {
    var $i = $('<i>').addClass('nav-icon ' + obj.imgClass);
    return $a.append($i).append($p);
  }
  return $a.append($p);
}

function createTreeViewItem(treeItem, selPageLink) {
  var $ul = $('<ul>').addClass('nav nav-treeview');

  if (typeof treeItem === 'undefined') return $ul;

  for (const [itemKey, itemVal] of Object.entries(treeItem)) {
    var $li = $('<li>').addClass('nav-item');
    var $a = $('<a>').attr('href', itemVal).addClass('nav-link');
    if (itemVal === selPageLink) {
      $a.addClass('active');
    }
    var $i = $('<i>').addClass('far fa-circle nav-icon');
    var $p = $('<p>').text(itemKey);
    $a.append($i).append($p);
    $li.append($a);
    $ul.append($li);
  }
  return $ul;
}

async function genTaskTrigger(selFleet = "none") {
  resetTriggerItem();
  try {
    var taskArray = await restGetTasks(selFleet);
    taskArray.forEach(function (jsonData) {
      var info = jsonData.task_info;
      if (info.task_trigger.includes("Manual​")) {
        createTaskTriggerItem(selFleet, jsonData.task, 1);
      }
    });
  } catch (err) {
    console.error("read tasks error: " + err);
  }

  try {
    var flowArray = await restGetFlows(selFleet);
    flowArray.sort(function (a, b) {
      var x = a.flow_name.toLowerCase(), y = b.flow_name.toLowerCase();
      return x < y ? -1 : x > y ? 1 : 0;
    }).forEach(function (dataObj) {
      var jsonData = JSON.parse(JSON.stringify(dataObj));
      if (jsonData.Event.event.includes("Manual")) {
        createTaskTriggerItem(selFleet, jsonData.flow_name, 2);
      }
    });
  } catch (err) {
    console.error("read flow error: ");
    console.error(err);
  }

  chkTriggerListVisible();
}

function createTaskTriggerItem(fleet, taskName, mode) {
  // --- protection ---
  if (![1, 2].includes(mode)) {
    alert("Invalid Mode!");
    return;
  }

  var event = (mode === 1) ? "t" : "f";
  var dom = `
    <li class='nav-item trigger-item'>
      <a href='views/manual_trigger_modal.html' class='nav-link' id='${fleet};${taskName}_link_${event}'>
        <p>${taskName}</p>
      </a>
    </li>
  `;
  $("#trigger-list .nav-treeview").append(dom);
}

function resetTriggerItem() {
  $(".trigger-item").remove();
}

function chkTriggerListVisible() {
  if ($(".trigger-item").length == 0) {
    $(".nav-sidebar #trigger-list").hide();
  } else if ($(".trigger-item").length > 0) {
    $(".nav-sidebar #trigger-list").show();
  }
}


// ------------------------------
//     Public binding events 
// ------------------------------
var trigger_event = "";
var trigger_name = "";

$(document).on('click', '.trigger-item a', function (e) {
  var task = this.id.split('_link_');
  var taskName = task[0].split(';')[1];
  trigger_event = task[1];
  trigger_name = taskName;

  e.preventDefault();
  $("#manual-trigger-confirm-modal").load($(this).attr('href'), function () {
    $('#manual-trigger-confirm-modal .modal-body').html("Are you sure you want to trigger " + taskName + "?");
  });
  $("#manual-trigger-confirm-modal").modal('show');
});

$(document).on('click', '#add-manual', function (e) {
  if (trigger_event == "t") {
    send_ros_taskReq(getSavedFleet(), trigger_name);
    return;
  }
  if (trigger_event == "f") {
    send_ros_flowReq(trigger_name);
    return;
  }
});


// ------------------------------
//         Fleet events 
// ------------------------------

var _switchFleetCallback = function () { };

function setSwitchFleetCallback(_callback) {
  _switchFleetCallback = _callback;
}

async function genFleets() {
  // -- fetch available fleet configurations --
  try {
    var fleets = await restGetFleets();
    genFleetSelectOptionsView(fleets);
  } catch (err) {
    console.error(err);
  }

  var firstFleet = $('#fleet-select').prop("selectedIndex", 0).val();
  if (firstFleet === null) {
    console.log('no fleet found');
    return;
  }

  var fleet = (localStorage.hasOwnProperty('fleet')) ? getSavedFleet() : firstFleet;
  localStorage.setItem("fleet", fleet);

  var savedFleet = getSavedFleet();
  genTaskTrigger(savedFleet);
  $('#fleet-select').val(savedFleet);
}

function genFleetSelectOptionsView(_fleets) {
  $('#fleet-select').find('option').remove();
  if (!_fleets.length) {
    $('#fleet-select').prop('disabled', true).append('<option disabled selected hidden>No Fleets</option>');
    removeSavedFleet();
    return;
  }

  for (var i = 0; i < _fleets.length; i++) {
    _fleets[i] = _fleets[i].split('.').slice(0, -1).join('.');
    $('#fleet-select').append(`<option value='${_fleets[i]}'>${_fleets[i]}</option>`);
  }

  // remove saved fleet if config file is removed
  var savedFleet = getSavedFleet();
  if (!$.inArray(savedFleet, _fleets) != -1) {
    removeSavedFleet();
  }
}

function switchFleet(_callback) {
  saveSwitchFleetOpt();
  _callback();
}

function saveSwitchFleetOpt() {
  var opt = "";
  opt = $("#fleet-select option:selected").text();
  console.log("========fleet: " + opt + "========");
  localStorage.setItem("fleet", opt);
  genTaskTrigger(opt);
}

function resetSavedFleet(_behaviour, _fleetName) {
  var savedFleet = getSavedFleet();
  switch (_behaviour) {
    case 'delete':
      if (savedFleet === _fleetName) {
        var firstFleet = $('#fleet-select').prop("selectedIndex", 0).val();
        localStorage.setItem('fleet', firstFleet !== null ? firstFleet : "");
        // remove fleet option & refresh task trigger list
        $('#fleet-select').find('option[value=' + _fleetName + ']').remove();
        genTaskTrigger(firstFleet);
      }
      break
    default:
      break
  }
}

function getSavedFleet() {
  return localStorage.hasOwnProperty('fleet') ? localStorage.getItem("fleet") : "";
}

function removeSavedFleet() {
  localStorage.removeItem('fleet');
}

// ------------------------------
//     Task Triggers events 
// ------------------------------
var timerTriggerTimes_ = 0;

async function send_ros_taskReq(fleet, taskName, isTimer = false) {
  var new_task = {
    task_id: '',
    task_name: '',
    task_type: '',
    task_params: '',
    priority: 2,
    robot_id: '',
    fleet_name: '',
    timer_scheduled: isTimer,
    task_time: {
      sec: 0,
      nanosec: 0
    }
  };

  var data = await restGetTaskData(fleet, taskName);
  console.log(data);
  var task_param = [];

  console.log(data.task_info.role.role_params);
  for (const role_item of data.task_info.role.role_params) {
    console.log(Object.entries(role_item));
    for (const [role_key, role_value] of Object.entries(role_item.content)) {
      if ((role_key.includes("goal_") && !role_key.includes("goal_label_")) || role_key.includes("cell_id_")) {
        var valArray = role_value.split(";");
        if (valArray.length >= 3) {
          var map = valArray[0];
          var area = valArray[1];
          var cell = valArray[2];

          var mapCells = await restGetMapCells(map + ".json");
          var jsonObj = JSON.parse(mapCells);
          var cellArray = jsonObj[`${area}`];
          var cellObj = Object.values(cellArray).filter(function (elements) {
            return elements.cell_id === cell;
          });
          if (cellObj.length > 0) {
            var cellCoord = cellObj[0].cell_coordinate.toString().replaceAll(',', ';');
            task_param.push(`${role_key}:${cellCoord}`);
          }
        }
      } else if (role_key.includes("mode_")) {
        task_param.push(`${role_key}:coord`);
      } else {
        task_param.push(`${role_key}:${role_value}`);
      }
    }
  }

  var robot_id = (data.task_info.assigned_robot == "auto") ? "" : data.task_info.assigned_robot;
  console.log(robot_id);

  // --- timer trigger ---
  if (typeof data.task_info.task_time_info !== 'undefined') {
    var timeInfo = data.task_info.task_time_info.split(";");
    if (timeInfo.length == 1 && data.task_info.task_time_info !== "") {
      new_task.task_time.sec = new Date(timeInfo[0]).getTime() / 1000;
      new_task.task_time.nanosec = 0
    } else if (timeInfo.length == 3) {
      var startTime = new Date(timeInfo[0]).getTime() / 1000;
      var duration = getSeconds(timeInfo[1]);
      var endTime = new Date(timeInfo[2]).getTime() / 1000;
      var times = Math.floor((endTime - startTime) / duration);
      startTime += duration * timerTriggerTimes_;

      new_task.task_time.sec = startTime;
      new_task.task_time.nanosec = 0

      timerTriggerTimes_ += 1;
      if (timerTriggerTimes_ <= times) {
        send_ros_taskReq(fleet, taskName, true);
      }
    }
  }

  new_task.task_id = genUuid();
  new_task.task_type = data.task_info.role.role_value;
  new_task.task_name = data.task;
  new_task.robot_id = robot_id;
  new_task.task_params = task_param.join(',');
  new_task.fleet_name = fleet;
  console.log(new_task)

  // Fill send_msgs 
  var send_msgs = {
    tasks: [new_task]
  };

  console.log("============ send task =================")
  wsPubTaskReq(send_msgs);
}

function resetTimerTriggerTimes() {
  timerTriggerTimes_ = 0;
}

// async function send_ros_flowReq(flowName) {
//   var new_flow_msg = {
//     event: genUuid(), // flow_id
//     event_type: flowName, // flow_name
//     params: 'ui',
//     timer_scheduled: false,
//     plan_time: {
//       sec: 0,
//       nanosec: 0
//     }
//   };
//   console.log(new_flow_msg)

//   var dt = new Date();
//   var currTime = dt.toISOString();
//   // var currTime = dt.toString();
  
//   var flow = {
//     id: genUuid(),
//     name: flowName
//   }

//   var options = {
//     op: "create",
//     args: {
//       start_time: '',
//       end_time: '',
//       interval: ''
//     }
//   };

//   var res = await restFlowTaskOp(flow, options);
//   console.log(res);
//   if (res.status_code !== 200) {
//     promptBox('error', res.message);
//     return;
//   }
//   promptBox('success', res.message);
// }


// ------------------------------
//      Login Status 
// ------------------------------
function getLoginStatus(status, selPage, selPageLink = "") {
  if (status === "logout") {
    alert("Please login first!");
    window.location.href = "login.html";
  } else {
    console.log(window.history);
    if (status.role !== 'admin' && selPage === 'userManagement') {
      alert("You can't access this page!");

      if (window.history.length < 3) {
        window.location.href = "login.html";
      } else {
        window.history.back();
      }
    }
    $('#user-login-status').html(status.greetingMsg);
    genSideBarMenu(status.role, selPage, selPageLink);
    genFleets();
  }
}

function userActivityDetector() {
  var secsSinceLastActivity = 0;
  var maxInactivitySec = 10 * 60;

  setInterval(function () {
    secsSinceLastActivity++;
    // console.log(secsSinceLastActivity + ' seconds since the user was last active');
    if (secsSinceLastActivity >= maxInactivitySec) {
      console.log('User has been inactive for more than ' + maxInactivitySec + ' seconds');
      alert("Time out. Please login again!");
      // -- log out --
      restLogout();
    }
  }, 1000);

  function activity() {
    secsSinceLastActivity = 0;
  }

  var activityEvents = [
    'mousedown', 'mousemove', 'keydown',
    'scroll', 'touchstart'
  ];
  activityEvents.forEach(function (eventName) {
    document.addEventListener(eventName, activity, true);
  });
}

function genConnectionStatusIndicator(content, hexColor) {
  $('#statusIndicator').html(content);
  $('#statusIndicator').css("color", hexColor);
}

function toast(message, color = "#666") {
  Toastify({
    text: message,
    duration: 3000,
    close: false,
    gravity: "top",
    position: "right",
    backgroundColor: color,
    stopOnFocus: true,
  }).showToast();
}

function promptBox(type, titleMsg) {
  Swal.mixin({
    toast: true,
    position: 'bottom-end',
    showConfirmButton: false,
    timer: 5000
  }).fire({
    type: type,
    title: `<span style='color:${promptColor[type]}'>${titleMsg}</span>`,
    background: promptBgColor[type]
  });
}

const promptColor = {
  "error": 'red',
  "warning": '#f8bb86',
  "success": '#28a745'
}

const promptBgColor = {
  "error": 'rgba(255, 255, 255, 0.8)',
  "warning": 'white',
  "success": 'white'
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
