/*
 * Author: Angela Kao
 * Author: John Wu
 * Date: 5 August 2021
 * Revision: 13 Jan 2022
 * Description:
 **/

let artifactAssets_ = {
  conveyoer: {
    type: 'conveyor_06',
    title: 'Conveyor',
    model: 'conveyor_06',
    avatar: `${getImagePath()}/sprites/conveyor-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/conveyor-150x150.png`,
  },
  conveyor: {
    type: 'conveyor_06',
    title: 'Conveyor',
    model: 'conveyor_06',
    avatar: `${getImagePath()}/sprites/conveyor-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/conveyor-150x150.png`,
  },
  top_module: {
    type: 'top_module',
    title: 'Top Module',
    model: 'top_module',
    avatar: `${getImagePath()}/sprites/top_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/top_module-150x150.png`,
  },
  juluen_top_module: {
    type: 'juluen_top_module',
    title: 'Juluen Top Module',
    model: 'juluen_top_module',
    avatar: `${getImagePath()}/sprites/juluen_top_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/juluen_top_module-150x150.png`,
  },
  lift_module: {
    type: 'lift_module',
    title: 'Lift Module',
    model: 'lift_module',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  },
  lift_module_black: {
    type: 'lift_module_black',
    title: 'Lift Module Black',
    model: 'lift_module_black',
    avatar: `${getImagePath()}/sprites/lift_module_black-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module_black-150x150.png`,
  },
  lift: {
    type: 'lift_module',
    title: 'Lift Module',
    model: 'lift_module',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  },
  auo_eq: {
    type: 'auo_eq',
    title: 'auo_eq',
    model: 'auo_eq',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  },
  auo_cv: {
    type: 'auo_cv',
    title: 'auo_cv',
    model: 'auo_cv',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  },
  auo_mplc: {
    type: 'auo_mplc',
    title: 'auo_mplc',
    model: 'auo_mplc',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  },
  juluen_worktable: {
    type: 'juluen_worktable',
    title: 'Juluen Work Table',
    model: 'juluen_worktable',
    avatar: `${getImagePath()}/sprites/juluen_worktable-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/juluen_worktable-150x150.png`,
  },
  default: {
    type: 'default',
    title: 'default Work Table',
    model: 'default',
    avatar: `${getImagePath()}/sprites/lift_module-440x220.png`,
    thumbnail: `${getImagePath()}/sprites/lift_module-150x150.png`,
  }
};

var currTaskName_ = "";
var currFlowName_ = "";
var isflow = false;
var flowArray = [];
var mapCelldict = {};
var timerInfoString = "";
var timerTriggerTimes = 0;
var timerFileString = "";
var tempTaskMsgArr = [];
var finished_tasks_num_ = 0;
let current_timer_flow_id = '';

let rmTimerFlowList_ = [];

var scannedAgents;
// var rmtToken_;
var bUnsavedChages_ = false;

let settingsCache_ = {
  fleet: {},
  agent: {},
  old_fleet_name: null
};
let openSidebar = true;
let artifact_flatten_dict = {};
let artifact_tab_row_content_dict = {};
let validate_res_dict = {};
/**
 * This function generates the flow statistic DOM.
 * @param _argType - "flow" or "timer"
 * @returns Flow Statistic DOM.
 */

function genTimerInfoDom(_argType) {
  var arg = (_argType === "flow") ? "-flow" : "";
  return `
    <div class="row" id="task-r-info" style="padding: 10px;">
      <div class="col-md-3 col-sm-3 col-3"></div>
      <div class="col-md-6 col-sm-6 col-6" style="text-align: center; margin-top: 15px;"><button type="button" class="btn btn-success" style="margin-right: 10px;" id="r-t${arg}">Run Timer</button><button type="button" class="btn btn-danger" style="margin-left: 10px;" id="r-t-a${arg}">Remove All</button></div>
      <div class="col-md-3 col-sm-3 col-3"></div>
    </div>`;
}

var isRemoveAllflowPressed = false;
var isRemoveAlltaskPressed = false;

const homeUri = '/operation.html';
const cookieExpiration = {
  expires: 1,
  path: homeUri
};

let langTemplateObj_ = {};
$(function () {
  'use strict'

  // $("#flow_edit_dashboard").hide();
  $("#back-div").hide();
  $("#save-flow-div").hide();
  initDataAsync();
  initDatePicker();
  loadFleetSettings();
  modalValidateEvent('flow-modal');
  setInterval(() => {
    // --- updating delete/add task list ---
    // var RunningTasks = queryData_.flowListView.map(f => f.flow_id);
    // rmTaskList_ = rmTaskList_.filter(tl => !RunningTasks.includes(tl));
    // // console.log(RunningTasks);
    // // console.log(rmTaskList_);

    // // subscribe_tasks_schedule2(flow_state_obj2_);
    // // subscribe_tasks_schedule3(queryData_.flowListView);
    // subscribe_tasks_schedule4(queryData_.flowListView);

    // console.log(apiQueryData_);
    // if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('flow_state') && apiQueryData_.flow_state.hasOwnProperty('flow_state')) {
    if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('flow_state') && apiQueryData_.flow_state.hasOwnProperty('swarm_data')) {
      var RunningTasks = apiQueryData_.flow_state.swarm_data.map(f => f.flow_id);
      rmTaskList_ = rmTaskList_.filter(tl => !RunningTasks.includes(tl));

      // console.log(RunningTasks);
      // console.log(rmTaskList_);

      // subscribe_tasks_schedule2(flow_state_obj2_);
      // subscribe_tasks_schedule3(queryData_.flowListView);
      // console.log(apiQueryData_);

      subscribe_tasks_schedule4(apiQueryData_.flow_state.swarm_data);
    }

    if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('timer_flow_state') && apiQueryData_.timer_flow_state.hasOwnProperty('swarm_data')) {
      var RunningTimerFlows = undefined;
      let current_flow_name = $('.editing-flowname').text();
      let current_select_fleet = $('#fleet-select').val();

      apiQueryData_.timer_flow_state.swarm_data.forEach(function (item) {
        if (current_flow_name == item.flow_name && current_select_fleet == item.fleet_name) {
          RunningTimerFlows = {
            "timer_flow_id": item.flow_id,
            "sub_flow_id": item.flows_to_be_generated
          };
          if (current_timer_flow_id == '' && !isRemoveAllflowPressed) {
            current_timer_flow_id = item.flow_id;
          }
        }
      });

      // console.log(RunningTimerFlows);
      rmTimerFlowList_ = rmTimerFlowList_.filter(tl => console.log(!RunningTasks.includes(tl)));
      // console.log(rmTimerFlowList_);
      flow_sub(RunningTimerFlows);
    }

    checkAddFlowBtnFocusState();

    // console.log(apiQueryData_)
    // console.log(` --------  ${current_timer_flow_id} ------------- `)

    // var current_fleet = $("#fleet-select").val();
    // updateJobStatistics("flow");

  }, 1000);

  $.cookie('flow_name', "", cookieExpiration);

  $.cookie("planner", JSON.stringify({
    "plant_event": [],
    "flows": []
  }), cookieExpiration);

  // --- flow state data sourcing ---
  pollFlowStates(apiQueryData_);

  // --- query string parse ---
  const queryString = window.location.search;
  console.log(queryString);
  const urlParams = new URLSearchParams(queryString);
  const flowName = urlParams.get('flow_name');
  console.log(flowName);
  if (flowName !== null) {
    switchOnFlowEdit(flowName);
  }

  validateFlowNameInputEvent();
});

let apiQueryData_ = {};
function pollFlowStates(_dataObj, _inteval = 1000) {
  setInterval(async function () {
    var res = await fetchGetFlowStates(rmtToken_, getSelectedFleet());
    if (res.ok) {
      _dataObj['flow_state'] = await res.json();
    }
    if (!isRemoveAllflowPressed) {
      var res = await fetchGetTimerFlowStatus(rmtToken_, current_timer_flow_id);
      if (res.ok) {
        _dataObj['timer_flow_state'] = await res.json();
      }
    } else {
      let deleteIndex = 0;
      if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('timer_flow_state') && apiQueryData_.timer_flow_state.hasOwnProperty('swarm_data')) {
        apiQueryData_.timer_flow_state.swarm_data.forEach(function (item, i) {
          if (current_timer_flow_id == item.flow_id) {
            deleteIndex = i;
          }
        });
      };

      apiQueryData_.timer_flow_state.swarm_data.splice(deleteIndex, 1);

      displayOverlay("Removing Timer Flow...");

      let i = 0;
      while (i < 15) {
        await sleep(200);
        i++;
      }
      removeOverlay();

      current_timer_flow_id = '';
      isRemoveAllflowPressed = false;
    }

  }, _inteval);
};


async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'operation');

  // wait until sidebar fleet is generated
  await sleep(1000); // sleep 1s
  var selectedFleet = getSelectedFleet();
  await getFleetSettingsData(selectedFleet + '.yaml');

  // for get rmt scan robot
  await getRMTscanRobot(selectedFleet);

  jQuery.ajaxSetup({
    async: false
  });
  getFlowData(selectedFleet);
  jQuery.ajaxSetup({
    async: true
  });

  // ------ [lang] switch language ------
  await initLanguageSupport();

  let lng = getSetLang() || 'en';
  langTemplateObj_ = await restGetTemplateLang(lng, 'flow_config');
}

async function initDatePicker() {
  await rmtTokenCheck();
  console.log(rmtToken_);
  var sys_date_millisec = await fetchGetSystemTime(rmtToken_);
  var sys_date = new Date(sys_date_millisec);
  console.log(' ----- data picker ------- ')
  console.log(sys_date_millisec)
  console.log(new Date(`${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`))
  $(".datepicker").datepicker({
    clearBtn: true,
    format: "yyyy/mm/dd",
    startDate: new Date(`${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`)
  });

  $(".datepicker").each(function () {
    $(this).datepicker('setDate', `${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`);
  });
}

async function getFlowData(selectedFleet) {
  console.log('--------Flow---------')
  await restGetFlows(selectedFleet).then(function(response) {
    // console.log("getFlowData Response:", response);
    flowArray = response;
    response.sort(function (a, b) {
      var x = a.flow_name.toLowerCase(),
        y = b.flow_name.toLowerCase();
      return x < y ? -1 : x > y ? 1 : 0;
    }).forEach(function (dataObj) {
      var jsonData = JSON.parse(JSON.stringify(dataObj))
      addJobButton(jsonData.flow_name, 'flow');
      if (isMobile()) {
        $(".text-col div").addClass("unavailable");
      } else {
        $(".flow-div input[type=text]").addClass("unavailable");
      }

    });
  }).catch(function(error) {
    console.error("getFlowData Error:", error);
  });

  // var flowData = await restGetFlows(selectedFleet);
  // flowArray = flowData;
  // flowData.sort(function (a, b) {
  //   var x = a.flow_name.toLowerCase(),
  //     y = b.flow_name.toLowerCase();
  //   return x < y ? -1 : x > y ? 1 : 0;
  // }).forEach(function (dataObj) {
  //   var jsonData = JSON.parse(JSON.stringify(dataObj))
  //   addJobButton(jsonData.flow_name, 'flow');
  //   if (isMobile()) {
  //     $(".text-col div").addClass("unavailable");
  //   } else {
  //     $(".flow-div input[type=text]").addClass("unavailable");
  //   }

  // });
  applyFontSize(getSavedFontSize(), '#add-list');
}

async function getFleetSettingsData(fleetFileName) {
  var fleetName = fleetFileName.split('.').slice(0, -1).join('.');
  var data = await restGetFleetSettings(fleetName);

  var data_dict = data;
  var agents;
  var roles;
  var maps;
  var rolesArray = [];
  for (let [item_key, item_val] of Object.entries(data_dict)) {
    agents = item_val.agents;
    roles = item_val.roles;
    rolesArray.push(roles);
    maps = item_val.maps;
  }
  maps.forEach(function (maps_item, maps_index) {
    getMapsInfo(`${maps_item}.json`);
  });
  $.cookie('role', rolesArray.join(","), cookieExpiration);

  updateRobotSelectOptionsView(agents);
  updateRoleSelectOptionsView(roles);
}

/**
 * This function is used to get the title and content of the role.
 * @param _roles - The role name, without the .xml extension.
 * @returns None
 */
async function getRolesInfo(_roles) {
  var data_dict = [];
  var rolesMappingData = await restGetRolesMappingData($("#fleet-select").val());
  rolesMappingData.forEach(function (data_item, data_index) {
    if (data_item.role_name === _roles.replace(".xml", "")) {
      data_dict.push({
        'title': data_item.title_name,
        'content': data_item.title_content
      });
    }
  });
  generateRoleParam(data_dict);
  selectAreaFirstOption();
}

/**
 * Cannot generate summary
 * @param _map - The map to get the cells for.
 * @returns None
 */
async function getMapsInfo(_map) {
  var cellsData = await restGetMapCells(_map);

  for (let [item_key, item_val] of Object.entries(JSON.parse(cellsData))) {
    var cellDict = {};
    Object.keys(item_val).map(function (key, index) {
      cellDict[item_val[key].cell_id] = item_val[key].cell_coordinate;
    });
    mapCelldict[`${_map.replace('.json', '')};${item_key}`] = cellDict;
  }
}

function updateRobotSelectOptionsView(agents) {
  $(".robot-select option[value=auto]").nextAll().remove();
  agents.forEach(function (agent) {
    $(".robot-select option[value=auto]").after(`<option value='${agent}'>${agent}</option>`);
  });
}

function updateRoleSelectOptionsView(roles) {
  $(".role-select option[value=new-role]").prevAll().remove();
  $(".role-select").prepend('<option hidden disabled selected value></option>');
  roles.forEach(function (role) {
    $(".role-select option[value=new-role]").before(`<option value='${role}'>${role}</option>`);
  });
}

/**
 * Add a job to the job list.
 * @param _name - the name of the job
 * @param _type - 'task' or 'flow'
 * @returns The function addJobButton returns nothing.
 */
function addJobButton(_name, _type) {
  // - protection -
  if (!['task', 'flow'].includes(_type)) {
    alert(`invalid job type: ${_type}!`);
    return;
  }
  var style = (_type === 'task') ? 'check' : 'clipboard';

  var flowNameDom = `<input class='form-control w-100' type='text' value='${_name}' readonly>`;
  if (isMobile()) {
    flowNameDom = `<div class="text-left w-100" data-value='${_name}'>${_name}</div>`;
  }
  var dom = `
  <div class='col-11 text-center ${_type}-div' id='flowbar-${_name.replace(/\s/g, '_')}'>
    <div class='row form-inline' style="padding:0 1rem">
      <!--<div class='fixed-30 text-right'>
        <i class='fas fa-clipboard-${style} fa-lg fa-fw'></i>
      </div>-->
      <div class='col text-col'>
        ${flowNameDom}
      </div>
      <div title="edit flow" class='fixed-40 btn btn-link edit-flow-btn' id='edit-${_name.replace(/\s/g, '_')}'>
        <i class='fas fa-pen fa-lg'></i>
      </div>
      <div title="remove flow" class='fixed-40 btn btn-link' id='delete-${_type}-${_name.replace(/\s/g, '_')}'>
        <i class='fas fa-times-circle fa-lg'></i>
      </div>
    </div>
  </div>`;
  $("#add-list").append(dom);
  return;
}

/**
 * Cannot generate summary
 * @param _mode - 1 for task, 2 for flow
 * @param _text - the text to be displayed in the dropdown
 * @param _type - "timer" or "event"
 * @returns None
 */
function changeTriggerText(_mode, _text, _type) {
  var modeName = (_mode === 1) ? "task-" : "flow-";
  // $(".trigger-select option[id=" + modeName + "timer]").text(_text);
  $(`.trigger-select option[id=${modeName}${_type}]`).text(_text);
}

function selectAreaFirstOption() {
  $(".area-list").val($(".area-list option:first").val()).change();
}

$(document).on('change', '.area-list', function (e) {
  if ($(this).val() == null) {
    return;
  }
  var map = $(this).val().split(";")[0];
  var area = $(this).val().split(";")[1];
  var id = "#" + $(this).attr('id').replace('area_select', 'cell_select');
  $(id).empty();
  for (let [area_key, area_val] of Object.entries(mapCelldict)) {
    var map_key = area_key.split(";")[0];
    var map_area_key = area_key.split(";")[1];
    if (map_key === map && map_area_key === area) {
      for (let [cell_key, cell_val] of Object.entries(area_val)) {
        $(id).append(`<option value=${cell_key}>${cell_key}</option>`);
      }
    }
  }
});

function generateRoleParam(data_item) {
  $(".role_title_tr").remove();
  $(".role_params_tr").remove();
  $(".blank_tr").remove();
  var tr = "";
  var itemCellObj = new Object();
  console.log(data_item);
  data_item.forEach(function (item, index) {
    if (item.content === "N/A") {
      return;
    }
    tr += '<tr class="role_title_tr"><th scope="rowgroup" colspan="2"><span>' + dictUiTerms[item.title] + '</span></th></tr>';
    for (const [content_key, content_value] of Object.entries(item.content)) {
      if (content_key.includes("goal_label_") || content_key.includes("mode_")) {
        tr += '<tr class="role_params_tr" style="display: none">';
      } else {
        tr += '<tr class="role_params_tr">';
      }
      if ((content_key.includes("goal_") && !content_key.includes("goal_label_")) || content_key.includes("cell_id_")) {
        itemCellObj[content_key] = content_value;
        tr += '<td class="params-title-td"><div>' + rename_showing_label(content_key) + '​</div></td>';
        var option_str = "";
        for (let [map_key, map_val] of Object.entries(mapCelldict)) {
          var mapKey = map_key.split(";")[0];
          var map_area_key = map_key.split(";")[1];
          var valArray = content_value.split(";");
          if (valArray.length >= 3 && mapKey === valArray[0] && map_area_key === valArray[1]) {
            option_str += `<option value=${map_key} selected="selected">${map_area_key}</option>`;
          } else {
            option_str += `<option value=${map_key}>${map_area_key}</option>`;
          }
        }
        tr += '<td class="text-left py-0 align-middle content-td" id="' + content_key + '"><select class="form-control select-bold-text area-list" id="' + content_key + '_area_select" style="width: 45%;">' + option_str + '</select><select class="form-control select-bold-text" id="' + content_key + '_cell_select" style="width: 45%;"></select></td>';
      } else if (content_key.includes("status_")) {
        var option_str = "";
        var status = content_value;
        if (typeof content_value !== 'undefined' && content_value === 'occupied') {
          option_str = '<option value="occupied" selected="selected">Occupied</option><option value="empty">Empty</option>';
        } else if (typeof content_value !== 'undefined' && content_value === 'empty') {
          option_str = '<option value="occupied">Occupied</option><option value="empty" selected="selected">Empty</option>';
        } else {
          option_str = '<option value="occupied">Occupied</option><option value="empty">Empty</option>';
        }
        tr += '<td class="params-title-td"><div>' + rename_showing_label(content_key) + '​</div></td>';
        tr += '<td class="text-right py-0 align-middle content-td" id="' + content_key + '"><select class="form-control select-bold-text" id="' + content_key + '_status_select" style="width: 45%;">' + option_str + '</select></td>';
      } else {
        tr += '<td class="params-title-td"><div>' + content_key + '​</div></td>';
        tr += '<td class="text-right py-0 align-middle content-td" id="' + content_key + '"><input type="text" class="form-control" value="' + content_value + '"></td>';
      }
      tr += '</tr>';
    }
    tr += '<tr class="blank_tr"><td colspan="2"></td></tr>';
  });
  $("#task-field-table tbody tr:last").after(tr);
  $(".blank_tr").prev("tr").children('td.params-title-td').css('border-bottom-left-radius', '10px');
  $(".blank_tr").prev("tr").children('td.content-td').css('border-bottom-right-radius', '10px');
  for (const [cell_key, cell_value] of Object.entries(itemCellObj)) {
    var valArray = cell_value.split(";");
    if (valArray.length >= 3) {
      $('#' + cell_key + '_area_select').val($('#' + cell_key + '_area_select option[value="' + valArray[0] + ';' + valArray[1] + '"]').val()).change();
      $('#' + cell_key + '_cell_select option[value="' + valArray[2] + '"]').attr('selected', 'selected');
    }
  }
}

$("#flow-item").click(function () {
  $(this).attr('data-target', '#flow-modal');
});

function checkAddFlowBtnFocusState() {
  if ($("#flow-item").is(":focus")) {
    $("#flow-item").blur();
  }
}

async function checkFileName(mode, name) {
  var filename_regex = /^[a-zA-Z0-9_]+$/;
  var modeName = mode == 1 ? "task" : "flow";

  var data = await restOperationFileExistence(modeName, name);
  // --- protection ---
  if (data.fileExists) {
    notificationMsg(2, `${name} already exists!`);
    return false;
  }
  if (name === "") {
    notificationMsg(2, `Please fill in ${modeName} name!`);
    return false;
  }
  if (name.indexOf(' ') >= 0) {
    notificationMsg(2, `${modeName} name can't include white space!`);
    return false;
  }
  if (!inputCheck(name)) {
    notificationMsg(2, `${modeName} include invalid characters`);
    return false;
  }

  // --- rendering ---
  // if (mode !== 1 || mode !== 2) { alert("Invalid Mode!"); }
  var jobType = (mode === 1) ? 'task' : 'flow';
  addJobButton(name, jobType);
  let $flowNameElement = isMobile() ? $(`.text-col div`) : $(`.${jobType}-div input[type=text]`);
  changeAvaliableStatus($flowNameElement);
  return true;
}

$("#add-task").click(function (e) {
  $("#task-modal").modal('hide');
  var taskName = $("#task-name").val();
  checkFileName(1, taskName);
});

$("#cancel-add-flow").click(async function () {
  $("#flow-name").css("border-color", '');
  $("#flow-name").val('');
  $('#flow-modal').modal('hide');
})

$("#add-flow").click(async function () {
  var flowName = $("#flow-name").val();
  let flowNameValid = await checkFileName(2, flowName);

  if (flowNameValid) {
    $("#flow-name").css("border-color", '');
    $("#flow-name").val('');
    $('#flow-modal').modal('hide');

    $.cookie("planner", JSON.stringify({
      "plant_event": [],
      "flows": []
    }), cookieExpiration);

    $.cookie('flow_name', flowName, cookieExpiration);
    // bUnsavedChages_ = true;
  } else {
    $("#flow-name").css("border-color", 'red');
  }
});

$("#add-event").click(function () {
  var eventID = $("#event-id").val();

  if (isflow) {
    resetTaskTriggerSelectOptions("flow");
    var test_pln_event = {
      "Event": "Event_" + eventID,
      "flows": currFlowName_
    };
    var planner_cookie = $.parseJSON($.cookie("planner"));
    planner_cookie.plant_event.push(test_pln_event);
    $.cookie("planner", JSON.stringify(planner_cookie), cookieExpiration);

    console.log($.parseJSON($.cookie("planner")));
    set_flow_event("Event_" + eventID);
  } else {
    resetTaskTriggerSelectOptions("task");
  }

  var mode = (isflow ? 2 : 1);
  changeTriggerText(mode, "Event: " + eventID, 'event');
  $('#event-trigger-modal').modal('hide');
});

$("#add-timer").click(function () {
  var needRepeat = $('#repeatCheckbox').is(':checked');
  var regex = /^([0-9]|0[0-9]|1[0-9]|2[0-3]):[0-5][0-9]$/
  if ($("#startDate").val() === "") {
    alert('Please fill in task timer start date !');
  } else if ($("#startTime").val() === "") {
    alert('Please fill in task timer start time !');
  } else if (!regex.test($("#startTime").val())) {
    alert('Task timer start time format is invalid!');
  } else if (needRepeat && $("#endDate").val() === "") {
    alert('Please fill in task timer end date !')
  } else if (needRepeat && $("#endTime").val() === "") {
    alert('Please fill in task timer end time !');
  } else if (needRepeat && !regex.test($("#endTime").val())) {
    alert('Task timer end time format is invalid!');
  } else {
    var startTimeString = $("#startTime").val();
    if (isflow) {
      resetTaskTriggerSelectOptions("flow");
      var test_pln_event;
      var isrepeat = false;

      if (needRepeat) {
        test_pln_event = {
          "Event": "Timer",
          "flows": currFlowName_,
          "Starttime": startTimeString,
          "Endtime": $("#endTime").val(),
          "Duration": $("#cycle-time").val()
        };
        isrepeat = true;
      } else {
        test_pln_event = {
          "Event": "Timer",
          "flows": currFlowName_,
          "Starttime": startTimeString
        };
        isrepeat = false;
      }

      var planner_cookie = $.parseJSON($.cookie("planner"));
      planner_cookie.plant_event.push(test_pln_event);
      $.cookie("planner", JSON.stringify(planner_cookie), cookieExpiration);
      console.log($.parseJSON($.cookie("planner")));
      set_flow_event("Timer");
      get_time_trigger_event(startTimeString, $("#endTime").val(), $("#cycle-time").val(), isrepeat, $('#startDate').val(), $('#endDate').val());
    } else {
      resetTaskTriggerSelectOptions("task");
    }

    startTimeString += needRepeat ? " (Rep)" : ""
    var mode = (isflow ? 2 : 1);
    changeTriggerText(mode, "Timer: " + startTimeString, 'timer');
    $('#timer-trigger-modal').modal('hide');

    var startDate = $("#startDate").val();
    timerInfoString = startDate + " " + $("#startTime").val();
    if (needRepeat) {
      var endDate = $("#endDate").val();
      timerInfoString = timerInfoString.concat(";", $("#cycle-time").val(), ";", endDate, " ", $("#endTime").val());
    }
  }
});

$(document).on('click', "div[id*='delete-']", async function (e) {
  let jobType = $(this).attr('id').split('-')[1];
  let selFlowName = $(this).attr('id').split('-')[2];
  let selectOp = $(this).parent().find('input[type=text]').val();
  let selFleet = $("#fleet-select").val();

  if (confirm("Remove the Flow?")) {
    switch (jobType) {
      case 'task':
        if (selectOp === currTaskName_) {
          currTaskName_ = "";
          $("#task-deck").hide();
        }
        await restDeleteTaskData(selFleet, selectOp);
        break;
      case 'flow':
        $.cookie('flow_name', selFlowName, cookieExpiration);
        await delete_flow();

        flowArray = _.filter(flowArray, ((f) => f.flow_name !== selFlowName));

        // --- remove flow bar from flows list ---
        $(`#flowbar-${selFlowName}`).remove();
        break;
      default:
        break;
    }
  }

  e.stopPropagation();

  // --- delete task trigger if exists ---
  $(".trigger-item p").each(function () {
    if ($(this).text() === selectOp) {
      $(this).parent().parent('li').remove();
    }
    chkTriggerListVisible();
  });
});

// Delete
function delete_flow() {
  flow_name_cookie = $.cookie('flow_name').replace(/\s/g, '_');

  restDeleteFlowData(getSelectedFleet(), flow_name_cookie);
  toast(`${flow_name_cookie} is deleted`);
}

$(document).on('click', '.task-div', function (e) {
  console.log('#### task clicked');
  isflow = false;
  $("#flow_dashboard").show();
  $("#flow_edit_dashboard").hide();
  $("#back-div").hide();
  $("#save-flow-div").hide();

  if (isMobile()) {
    $(this).find(".text-col").children('div').removeClass("available").addClass("unavailable");
  } else {
    $(this).find("div.text-col").children('input[type=text]').removeClass("available").addClass("unavailable");
  }

  if (istaskTimer(currTaskName_)) {
    console.log('---- A task WITH timer ----')
    $('#timer_label').text("Timer schedule")
    $('#timer-tab').css('display', '');
  } else {
    console.log('---- A task WITHOUT timer ----')
    $('#timer-tab').css('display', 'none');
  }

  $(".role_title_tr").css('display', 'none');
  resetTaskOptions();
  checkDisableManual(1);
});

// $(document).on('click', '.flow-div', function (e) {
//   var flowName = $(this).find('input[type=text]').val();
//   switchOnFlowEdit(flowName);
// });

$(document).on('click', '.edit-flow-btn', function (e) {
  var flowName = $(this).parent().find('input[type=text]').val();
  if (isMobile()) {
    flowName = $(this).parent().find('.text-col').children('div').attr('data-value');
  }

  console.log(flowName)
  window.location.href = `/operation_floweditor.html?flowName=${flowName}`;
  // switchOnFlowEdit(flowName);
});

function switchOnFlowEdit(_flowName) {
  isflow = true;
  console.log(_flowName);

  $("#flow_dashboard").hide();
  $("#flow_edit_dashboard").show();
  $("#back-div").show();
  $("#save-flow-div").show();

  currFlowName_ = _flowName;

  $("#flow-deck .editing-flowname").text(_flowName);
  if (isflowTimer(_flowName)) {
    // $("#flow-deck .card-header").html(_flowName);
    $('#timer_label_flow').text("Timer schedule")
    $('#flow-profile-tab').css('display', '');
    $('#flow-home-tab').css('border-top-right-radius', '');
    $('#flow-home-tab').css('padding', '0px');
    $('#flow-home-tab').parent().css("width", "50%");
    $('#flow-home-tab').attr("data-toggle", "tab");
    $('#flow-home-tab').attr("href", "#flow-home");
    $('#flow-home-tab').addClass('active');
  } else {
    // $("#flow-deck .card-header").html(_flowName);
    $('#flow-profile-tab').css('display', 'none');
    $('#flow-home-tab').tab('show');
    $('#flow-home-tab').parent().css("width", "100%");
    $('#flow-home-tab').css("border-top-right-radius", "15px");
    $('#flow-home-tab').css('padding', '0px');
    $('#flow-home-tab').removeAttr("data-toggle");
    $('#flow-home-tab').removeAttr("href");
  }

  resetTaskOptions();
  checkDisableManual(2);

  $.cookie("planner", JSON.stringify({
    "plant_event": [],
    "flows": []
  }), cookieExpiration);

  $.cookie('flow_name', _flowName, cookieExpiration);

  init_flow(_flowName);
}

// $(document).on('click', '#flow-back', function (e) {
//   window.location.href = homeUri;
// });

$(".trigger-select").on('click', function () {
  if (isTouchDevice()) return;
  console.log(`touch device? ${isTouchDevice()}`);
  var platform = navigator.platform;
  console.log(platform + " click trigger type");

  var show_name = "";
  if ($(this).attr("id") == "flow_select") {
    show_name = currFlowName_;
  } else {
    show_name = currTaskName_;
  }

  if ($(this).val() === 'manual') {
    if (isflow) {
      resetTaskTriggerSelectOptions("flow");
      set_flow_event("Manual");
    } else {
      resetTaskTriggerSelectOptions("task");
    }
  } else if ($(this).val() === 'event') {
    $("#event-label").html(show_name + ' event:');
    $("#event-trigger-modal").modal('show');
  } else if ($(this).val() === 'timer') {
    $("#timer-label").html(show_name + ' Timer:');
    $("#timer-trigger-modal").modal('show');
  }
});

$('.trigger-select').on('change', function () {
  if (!isTouchDevice()) return;
  console.log(`touch device? ${isTouchDevice()}`);
  var platform = navigator.platform;
  console.log(platform + " change trigger type");
  var show_name = "";
  if ($(this).attr("id") == "flow_select") {
    show_name = currFlowName_;
  } else {
    show_name = currTaskName_;
  }
  if ($(this).val() === 'manual') {
    if (isflow) {
      resetTaskTriggerSelectOptions("flow");
      set_flow_event("Manual");
    } else {
      resetTaskTriggerSelectOptions("task");
    }
  } else if ($(this).val() === 'event') {
    $("#event-label").html(show_name + ' event:');
    $("#event-trigger-modal").modal('show');
  } else if ($(this).val() === 'timer') {
    $("#timer-label").html(show_name + ' Timer:');
    $("#timer-trigger-modal").modal('show');
  }
});

$(".role-select").change(function () {
  var role = $(this).val();
  getRolesInfo(`${role}.xml`);
  if ($(this).val() === "new-role") {
    window.location.href = "role.html";
  }
});

$("#startTime").keypress(checkTimeFormat);
$("#endTime").keypress(checkTimeFormat);
$("#cycle-time").keypress(checkNumFormat);
$("#save-task").click(saveJSONData);

function changeAvaliableStatus(elements) {
  elements.each(function () {
    if (!$(this).hasClass("unavailable")) {
      $(this).addClass("available");
    } else {
      $(this).addClass("unavailable");
    }
  });
}

function resetSelectedName() {
  currTaskName_ = "";
  currFlowName_ = "";
}

function resetTaskTriggerSelectOptions(modeName) {
  $(`#${modeName}-event`).text('Event');
  $(`#${modeName}-timer`).text('Timer');
}

async function resetTaskOptions() {
  $(".trigger-select").prop('selectedIndex', 0);
  $(".robot-select").prop('selectedIndex', 0);
  $(".role-select").prop('selectedIndex', 0);
  $(".role_params_tr").remove();

  if (isflow) {
    resetTaskTriggerSelectOptions("flow");
    if ($.trim(currFlowName_) == '') {
      return;
    }
    flowArray.forEach(function (flow, flow_index) {
      if (flow.flow_name === currFlowName_) {
        if (flow.Event.event.includes("Manual")) {
          $('#flow-manual').prop("selected", true);
        } else if (flow.Event.event.includes("Event")) {
          changeTriggerText(2, flow.Event.event, 'event');
          $('#flow-event').prop("selected", true);
        } else if (flow.Event.event.includes("Timer")) {
          changeTriggerText(2, flow.Event.event, 'timer');
          $('#flow-timer').prop("selected", true);
          var isrepeatt = false;
          var s_time = flow.Event.start_time.split('/')[3];
          var s_date = `${flow.Event.start_time.split('/')[0]}/${flow.Event.start_time.split('/')[1]}/${flow.Event.start_time.split('/')[2]}`;
          var e_time = "";
          var e_date = "";
          if (flow.Event.end_time !== "") {
            isrepeatt = true;
            e_time = flow.Event.end_time.split('/')[3];
            e_date = `${flow.Event.end_time.split('/')[0]}/${flow.Event.end_time.split('/')[1]}/${flow.Event.end_time.split('/')[2]}`;
          } else {
            e_date = s_date;
          }
          get_time_trigger_event(s_time, e_time, flow.Event.repeat_time, isrepeatt, s_date, e_date);
        }
      }
    });
  } else {
    resetTaskTriggerSelectOptions("task");
    if ($.trim(currTaskName_) == '') {
      return;
    }
    var fleet = $("#fleet-select").val();
    var data = await restGetTaskData(fleet, currTaskName_);
    var info = data.task_info;
    $(".trigger-select").val(info.task_trigger);
    if (info.task_trigger.includes("Event")) {
      changeTriggerText(1, info.task_trigger, 'event');
      $('#task-event').prop("selected", true);
    } else if (info.task_trigger.includes("Timer")) {
      changeTriggerText(1, info.task_trigger, 'timer');
      $('#task-timer').prop("selected", true);
      if (typeof info.task_time_info !== 'undefined') {
        timerFileString = info.task_time_info;
      }
    } else if (info.task_trigger.includes("Manual​")) {
      $('#task-manual').prop("selected", true);
    }
    $(".robot-select").val(info.assigned_robot);
    $(".role-select").val(info.role.role_value);

    generateRoleParam(info.role.role_params);
  }
}

function checkDisableManual(mode) {
  var modeName = (mode == 1) ? "task-" : "flow-";
  var selectedName = (mode == 1) ? currTaskName_ : currFlowName_;
  $(".trigger-select option[id=" + modeName + "manual]").attr("disabled", false);
  $(".trigger-item p").each(function () {
    if ($(this).text() === selectedName) {
      $(".trigger-select option[id=" + modeName + "manual]").attr("disabled", true);
    }
  });
}

function checkTimeFormat(event) {
  var regex = new RegExp("^[0-9:]+$");
  checkInputIsValid(regex, event);
}

function checkNumFormat(event) {
  var regex = new RegExp("^[0-9]+$");
  checkInputIsValid(regex, event);
}

function checkInputIsValid(regexp, event) {
  var regex = new RegExp(regexp);
  var key = String.fromCharCode(!event.charCode ? event.which : event.charCode);
  if (!regex.test(key)) {
    event.preventDefault();
    return false;
  }
}

async function switchFleetCallback() {
  var selFleet = getSelectedFleet();
  await getFleetSettingsData(selFleet + '.yaml');
  $('#add-list').empty();
  getFlowData(selFleet);
  resetSelectedName();
  resetTaskOptions();

  // switch_fleet(selFleet);
  loadFleetSettings();

  var cardModeClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  $('#task-schedule-div').html('');
  $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  tempTaskMsgArr = [];
}

function getDefaultValIfNull(val) {
  return (val === null) ? "" : val;
}

async function saveJSONData() {
  var fleet = $("#fleet-select").val();
  var triggerOpt = $(".trigger-select option:selected").text();
  var isTimer = $(".trigger-select option:selected").val() == 'timer';
  var taskObj = new Object();
  taskObj.task_trigger = triggerOpt;
  if (isTimer && timerInfoString !== "") {
    taskObj.task_time_info = timerInfoString;
  } else if (isTimer && timerFileString !== "") {
    taskObj.task_time_info = timerFileString;
  }
  taskObj.assigned_robot = $(".robot-select").val();

  var roleObj;
  var contentObj;
  var roleArray = [];
  $("tr[class^='role_'][class$='_tr']").each(function () {
    var className = $(this).attr('class');
    if (className === 'role_title_tr') {
      contentObj = new Object();
      roleObj = new Object();
      roleObj.title = dictBtTerms[$(this).find('th').text()];
      roleArray.push(roleObj);
    } else {
      var content_key = $(this).find('td.content-td').attr('id');
      var content_val = "";
      if ((content_key.includes("goal_") && !content_key.includes("goal_label_")) || content_key.includes("cell_id_")) {
        var area = getDefaultValIfNull($(`#${content_key}_area_select`).val());
        var cell = getDefaultValIfNull($(`#${content_key}_cell_select`).val());
        content_val = area + ";" + cell;
      } else if (content_key.includes("status_")) {
        var status = getDefaultValIfNull($(`#${content_key}_status_select`).val());
        content_val = status;
      } else {
        content_val = $(this).find('input').val();
      }
      contentObj[content_key] = content_val;
    }
    roleObj.content = contentObj;
  });

  var role_info = {
    "role_value": $(".role-select").val(),
    "role_params": roleArray
  }
  taskObj.role = role_info;

  var task = {
    "fleet": fleet,
    "task": currTaskName_,
    'task_info': taskObj
  };
  console.log(task);

  await restPostTaskData(task, fleet, currTaskName_);
  // toast(`${currTaskName_} is saved`);

  // add manual trigger list
  var mode = (isflow ? 2 : 1);
}

function getSeconds(time) {
  var timeArray = time.split(":");
  return timeArray.length > 1 ? timeArray[0] * 3600 + timeArray[1] * 60 : timeArray[0] * 60;
}

// --- Add for Timer schedule ---
function istaskTimer(task_name) {
  var fleet = $("#fleet-select").val();
  var isTimer = false;

  restGetTaskData(fleet, task_name, false).done(function (data) {
    if (data.task_info.task_trigger.includes('Timer')) {
      isTimer = true;
    }
  });

  console.log(isTimer)
  return isTimer;
}

function isflowTimer(flow_name) {
  var fleet = $("#fleet-select").val();
  var isTimer = false;

  restGetFlowData(flow_name, `${fleet}-flow-${flow_name}`, false).done(function (data) {
    var jsonData = JSON.parse(data);
    if (jsonData.event_name.event.includes('Timer')) {
      isTimer = true;
    }
  });

  console.log(isTimer)
  return isTimer;
}

// --- Flow tab ---
$(document).on('click', '#flow-profile-tab', function (e) {
  $('.dark-icon-color').addClass('light-icon-color').removeClass('dark-icon-color')
  // var current_fleet = $("#fleet-select").val();
  // flow_sub(current_fleet)
});

$(document).on('click', '#flow-home-tab', function (e) {
  $('.light-icon-color').addClass('dark-icon-color').removeClass('light-icon-color')
});

// --- Task tab ---
$(document).on('click', '#timer-tab', function (e) {
  var current_fleet = $("#fleet-select").val();
  // --- task_sub(current_fleet) ---
  $('#task-timer-sche').empty();
  $('#flow-timer-sche').empty();

  if ($('#task-timer-sche-status').children().length === 0) {
    var dom = genTimerInfoDom('task');
    $('#task-timer-sche-status').append(dom);
  }

  var no_task = '<h1 class="col-md-12 col-sm-12 col-12 no-task" style="text-align: center; margin-top: 40px;">No scheduled tasks.</h1>';
  $('#task-timer-sche').append(no_task);

  // --- taskStateSub_taskSub2(); ---
  if (flow_state_obj2_ === undefined) return;

  var fleet_name = flow_state_obj2_.fleet;
  var tasks = flow_state_obj2_.msg;

  var tasks_num = tasks.length;

  if (isRemoveAlltaskPressed) {
    $('#task-timer-sche').empty();
    $('#task-timer-sche').append(no_task);
  }

  if (tasks_num == 0) {
    isRemoveAlltaskPressed = false;
  } else {
    $('.no-task').remove();
  }

  if (current_fleet !== fleet_name) {
    console.log(`fleet name: ${fleet_name} is not matched!`);
    return;
  }

  tasks.sort(compare);
  var task_name_c = $('#main-tab').find('.card-header').text();
  tasks.forEach(function (task_item, task_index) {
    var uuid = task_item.task_id;
    var task_name = task_item.task_name;
    var complete_percent = task_item.complete_percent;
    var robot_id = task_item.robot_id;
    var start_time = task_item.start_time; // start time SHOULD be in ms unit by UTC standard
    var state = task_item.state;
    // console.log(start_time);
    var fmtStartTime = new moment(start_time).local().format('YYYY-MM-DD HH:mm:ss');
    // console.log(fmtStartTime);

    var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
    var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;

    if (task_name !== task_name_c) {
      return;
    }

    if (state === 2 || state === 3) {
      $('#' + uuid).parent().remove();
      return;
    }

    if ($('#' + uuid).length) {
      update_task_info(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "task");
    } else {
      $('#task-timer-sche').append(genTaskInfoTemplate(uuid, task_name, robot_id, task_state[state], fmtStartTime, complete_percent, "task"));
    }
  });

});

// --- ROS pub ---
function send_remove(type, task) {
  var jsonRemoveTask = {
    data: `${type}:${task}`
  };
  wsRemoveTask(jsonRemoveTask);
}

// --- Task Timer Tab ---
function compare(a, b) {
  let a_start = moment.unix(a.scheduled_time.sec).toDate();
  let b_start = moment.unix(b.scheduled_time.sec).toDate();

  if (a_start < b_start) {
    return -1;
  }
  if (a_start > b_start) {
    return 1;
  }
  return 0;
}

// --- ROS sub ---
function subscribe_tasks_schedule2(_obj) {
  // console.log(flow_state_obj2_);
  console.log(_obj);

  if (_obj === undefined) return;
  var taskFleet = _obj.fleet;
  var tasks = _obj.msg;

  var cardModeClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  }
  if (taskFleet !== $('#fleet-select').val()) {
    if ($('#task-schedule-div').find(".info-box").length > 0) {
      $('#task-schedule-div').empty();
      $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
    }
    return
  };

  tasks = tasks.sort(compare);

  var div = "";
  if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
    $('#task-schedule-div').html('');
    tasks.forEach(function (task_item, task_index) {
      var start_time = task_item.start_time;
      start_time = moment(start_time).local().format('YYYY-MM-DD HH:mm:ss');
      var sche_date_trans = start_time;

      div +=
        `<div class="col-md-2 col-sm-6 col-12 trigger-div">
          <div class="info-box bg-dark" id="task-status-${task_index}">
            <button type="button" class="delete-trigger-btn" id='delete-${task_item.task_id}'>×</button>
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

  // --- TaskState Update ---
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
}

function subscribe_tasks_schedule3(_obj) {
  // console.log(_obj);
  if (_obj === undefined) { return; }

  var currFleet = getSelectedFleet();
  var tasks = _obj.filter(o => o.fleet_name === currFleet);
  // console.log(tasks);

  var cardModeClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';

  if (tasks.length === 0 && !$('#no-data-div').length) {
    $('#task-schedule-div').empty();
    $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  }
  if (currFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  }

  // tasks = tasks.sort(compare);

  var div = "";
  if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
    $('#task-schedule-div').html('');
    tasks.forEach(function (task_item, task_index) {
      var start_time = task_item.start_time;
      start_time = moment(start_time).local().format('YYYY-MM-DD HH:mm:ss');
      var sche_date_trans = start_time;

      div +=
        `<div class="col-md-2 col-sm-6 col-12 trigger-div">
          <div class="info-box bg-dark" id="task-status-${task_index}">
            <button type="button" class="delete-trigger-btn" id='delete-${task_item.flow_id}'>×</button>
            <span class="info-box-icon"><i class="fas fa-tasks"></i></span>
            <div class="info-box-content col-10" width="50%">
              <span class="info-box-text" title="${sche_date_trans}">${sche_date_trans}</span>
              <marquee direction="left" scrollamount="2" behavior="scroll" id="task-name-${task_index}">${task_item.flow_name}</marquee>
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

  // --- TaskState Update ---
  tasks.forEach(function (task_item, task_index) {
    // --- Update on UI ---
    $('#task-status-' + task_index).removeClass(function (index, className) {
      return (className.match(/(^|\s)bg-\S+/g) || []).join(' ');
    });

    $('#task-name-' + task_index).html(`${task_item.flow_name} is ${task_state[task_item.state].toLowerCase()}`);
    $('#task-status-' + task_index).addClass(task_state_bgclass[task_item.state]);

    if (task_item.state == 2) {
      $('#task-percent-' + task_index).html('100%');

      if (task_item.task_name.substr(0, 3) == "tra") {
        finished_cargo_++;
      }
      finished_tasks_num_++;
    } else {
      var taskProgress = task_item.progress;
      $('#task-percent-' + task_index).html(`${taskProgress}%`);
      $('#task-progress-' + task_index).css('width', `${taskProgress}%`)
    }
  });
}

function subscribe_tasks_schedule4(_obj) {
  // console.log(_obj);
  if (_obj === undefined) { return; }

  var currFleet = getSelectedFleet();
  var flows = _obj.filter(o => o.fleet_name === currFleet);
  // console.log(tasks);

  var cardModeClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';

  if (flows.length === 0 && !$('#no-data-div').length) {
    $('#task-schedule-div').empty();
    $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  }
  if (currFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  }

  var taskList = _.map(flows, f => f.tasks);
  taskList = _.flatten(taskList);
  // console.log(taskList);

  var div = "";
  if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(taskList)) {
    $('#task-schedule-div').html('');
    taskList.forEach(function (task_item, task_index) {
      var start_time = task_item.start_time;
      start_time = moment(start_time).local().format('YYYY-MM-DD HH:mm:ss');
      var sche_date_trans = start_time;

      div +=
        `<div class="col-md-2 col-sm-6 col-12 trigger-div">
          <div class="info-box bg-dark" id="task-status-${task_index}">
            <button type="button" class="delete-trigger-btn" id='delete-${task_item.task_id}'>×</button>
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
  tempTaskMsgArr = taskList;

  // --- TaskState Update ---
  taskList.forEach(function (task_item, task_index) {
    // --- Update on UI ---
    $('#task-status-' + task_index).removeClass(function (index, className) {
      return (className.match(/(^|\s)bg-\S+/g) || []).join(' ');
    });

    $('#task-name-' + task_index).html(`${task_item.task_name} is ${task_state[task_item.state].toLowerCase()}`);
    $('#task-status-' + task_index).addClass(task_state_bgclass[task_item.state]);

    if (task_item.state == 2) {
      $('#task-percent-' + task_index).html('100%');

      if (task_item.task_name.substr(0, 3) == "tra") {
        finished_cargo_++;
      }
      finished_tasks_num_++;
    } else {
      var taskProgress = task_item.complete_percent;
      $('#task-percent-' + task_index).html(`${taskProgress}%`);
      $('#task-progress-' + task_index).css('width', `${taskProgress}%`)
    }
  });
}

function subscribe_tasks_schedule() {
  task_state_sub.subscribe(function (_message) {
    var taskFleet = _message.fleet_name;
    var tasks = _message.tasks;
    // console.log(_message.fleet_name);

    var div = "";

    if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
      var cardModeClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
      $('#task-schedule-div').append(`<div class="card ${cardModeClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
    }
    if (taskFleet !== $('#fleet-select').val()) {
      return
    };

    tasks = tasks.sort(compare);
    if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
      // console.log(tasks);
      $('#task-schedule-div').html('');
      tasks.forEach(function (task_item, task_index) {
        var start_time = task_item.start_time;
        var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
        var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;

        div +=
          `<div class="col-md-2 col-sm-6 col-12 trigger-div">
          <div class="info-box bg-dark" id="task-status-${task_index}">
            <button type="button" class="delete-trigger-btn" id='delete-${task_item.task_id}'>×</button>
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
  });
}

// --- RESTful API queries to remove task ---
let rmTaskList_ = [];
async function blockScreen(_arrObj) {
  var i = 0;
  displayOverlay("Removing Task...");
  while (_arrObj.length > 0 && i < 25) {
    await sleep(200);
    i++;
  }
  removeOverlay();
}

$(document).on('click', '.delete-trigger-btn', async function (e) {
  var taskId = this.id.replace("delete-", "");
  console.log();
  const res = await fetchDeleteTask(rmtToken_, taskId);
  if (res.ok) { rmTaskList_.push(taskId) };
  // --- overlay block protection ---
  await blockScreen(rmTaskList_);

  const data = await res.json();

  // if (typeof data === Object && data.hasOwnProperty('detail')) { data = data.detail; }
  // const stateStyle = (res.ok) ? 1 : 3;
  // notificationMsg(stateStyle, data);

  // --- [Connection] health check ---
  if (!res.ok) {
    console.log(data);
    notificationMsg(3, `[CONN.] ${data}`);
    return;
  }
  // --- [Swarm System] health check ---
  const sysStateStyle = (_.inRange(data.system_status_code, 200, 300)) ? 1 : 3;
  notificationMsg(sysStateStyle, `[SWARM] ${data.system_message}`);
});

function update_task_info(uuid, task_name, robot_id, state, start_time, complete_percent, flow_or_task) {
  var target = '#' + uuid;
  if ($(target).find(`.${flow_or_task}-agent`).text() !== robot_id) {
    $(target).find(`.${flow_or_task}-agent`).text(robot_id);
  }

  if ($(target).find(`.${flow_or_task}-state`).text() !== state) {
    $(target).find(`.${flow_or_task}-state`).text(state);
  }

  if ($(target).find(`.${flow_or_task}-complete`).text() !== new String(`${complete_percent}%`)) {
    $(target).find(`.${flow_or_task}-complete`).text(`${complete_percent}%`);
  }

  $(target).find(`.${flow_or_task}-complete-bar`).css('width', `${complete_percent}%`);
}

// --- remove one task/flow ---
$(document).on('click', '.info-box-icon i.fas.fa-trash', async function (e) {
  var taskId = $(this).parent().parent().attr('id');

  displayOverlay("Removing Timer Flow...");

  const res = await fetchDeleteFlow(rmtToken_, taskId);

  if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('timer_flow_state') && apiQueryData_.timer_flow_state.hasOwnProperty('swarm_data')) {
    let current_select_fleet = $('#fleet-select').val();
    let current_flow_name = $('.editing-flowname').text();

    apiQueryData_.timer_flow_state.swarm_data.forEach(function (item) {
      if (current_flow_name == item.flow_name && current_select_fleet == item.fleet_name) {
        let sub_timer_flow_length = item.flows_to_be_generated.length;

        if (sub_timer_flow_length == 1) {
          let left_sub_flow_id = item.flows_to_be_generated[0].flow_id;
          if (left_sub_flow_id == taskId) {
            current_timer_flow_id = '';
            isRemoveAllflowPressed = true;
          }
        }
      }
    });
  }

  await sleep(200);

  if (res.ok) { removeOverlay(); };

  const data = await res.json();

  if (!res.ok) {
    console.log(data);
    notificationMsg(3, `[CONN.] ${data}`);
    return;
  }
  // --- [Swarm System] health check ---
  const sysStateStyle = (_.inRange(data.system_status_code, 200, 300)) ? 1 : 3;
  notificationMsg(sysStateStyle, `[SWARM] ${data.system_message}`);
});

let cachedFleetname_;
$('#edit-flowname-switch').on('click', async function () {
  var target = this.querySelector('.fa');

  if (target.classList.contains('fa-pen')) {
    target.setAttribute("class", "fa fa-eye");
    var targetNode = document.getElementsByClassName('editing-flowname');
    // console.log($(targetNode).text());
    var inputNode = document.createElement('input');
    inputNode.setAttribute("id", 'editing-flowname-input');
    inputNode.setAttribute("class", 'editing-flowname flow-input');
    inputNode.value = $(targetNode).text();

    // --- chore: cache the state ---
    cachedFleetname_ = $(targetNode).text();

    $(targetNode).replaceWith($(inputNode));
    validateFlowNameInputEvent();
    return;
  }

  if (target.classList.contains('fa-eye')) {
    var targetNode = document.getElementsByClassName('editing-flowname');
    var inputNode = document.createElement('span');
    inputNode.setAttribute("id", 'editing-flowname-span');
    inputNode.setAttribute("class", 'editing-flowname flow-input');

    // --- chore: comparison with original and modified ones ---
    var tmpName = cachedFleetname_;
    var modifiedName = $(targetNode).val();

    if (tmpName == modifiedName) {
      target.setAttribute("class", "fa fa-pen");
      $(targetNode).css("border-color", '');
      inputNode.textContent = tmpName;
      $(targetNode).replaceWith($(inputNode));
    } else {
      if (!inputCheck(modifiedName)) {
        $(targetNode).css("border-color", 'red');
        notificationMsg(2, `Flow name include invalid characters`);
      } else {


        let duplicateres = await checkDuplicateFlowname(getSelectedFleet(), cachedFleetname_, modifiedName);
        duplicateres = JSON.parse(duplicateres);

        let rename_needed = false;

        if (duplicateres.statusCode === 200) {
          target.setAttribute("class", "fa fa-pen");
          $(targetNode).css("border-color", '');

          if (cachedFleetname_ !== modifiedName) {
            if (confirm('rename the flow?')) {
              console.log('rename the flee now!');
              tmpName = modifiedName;
              rename_needed = true;
            }
          }
        } else {
          target.setAttribute("class", "fa fa-eye");
          notificationMsg(3, duplicateres.message);
          return
        }

        if (rename_needed) {
          let res = await restRenameFlowname(getSelectedFleet(), cachedFleetname_, modifiedName);
          res = JSON.parse(res);
          if (res.statusCode === 200) {
            inputNode.textContent = tmpName;
            $(targetNode).replaceWith($(inputNode));
            style = 1;
            // --- update global flow name ---
            flow_name = tmpName;
          } else {
            style = 3;
          }

          notificationMsg(style, res.message);
        } else {
          inputNode.textContent = tmpName;
          $(targetNode).replaceWith($(inputNode));
          notificationMsg(0, 'Rename Flow Canceled');
        }
      }
    }
  }
})

function flow_sub(timer_flow_status) {
  $('#task-timer-sche').empty();
  $('#flow-timer-sche').empty();

  if ($('#flow-timer-sche-status').children().length === 0) {
    var dom = genTimerInfoDom('flow');
    $('#flow-timer-sche-status').append(dom);
  }

  var no_task = '<h1 class="col-md-12 col-sm-12 col-12 no-task" style="text-align: center; margin-top: 40px;">No scheduled flows.</h1>';
  $('#flow-timer-sche').append(no_task);

  if (timer_flow_status === undefined) {
    $("#r-t-a-flow").attr("data-main_id", "");
    return;
  }

  var main_timer_flow_id = timer_flow_status.timer_flow_id;
  var tasks = timer_flow_status.sub_flow_id;
  var tasks_num = tasks.length;

  if (isRemoveAllflowPressed) {
    $('#flow-timer-sche').empty();
    $('#flow-timer-sche').append(no_task);
    $("#r-t-a-flow").attr("data-main_id", "");
  }

  if (tasks_num == 0) {
    isRemoveAllflowPressed = false;
  } else {
    $('.no-task').remove();
  }

  var rm_task_li = [];
  tasks.sort(compare);
  // console.log(tasks);
  tasks.forEach(function (task_item, task_index) {
    var timeNow = new Date(Date.now());
    timeNow.setSeconds(0, 0);
    timeNow = timeNow.getTime();

    var uuid = task_item.flow_id;
    var task_name = "";
    var complete_percent = 0;
    var robot_id = '1';
    var start_time = moment.unix(task_item.scheduled_time.sec).toDate();
    var fmtStartTime = moment.unix(task_item.scheduled_time.sec).format('YYYY-MM-DD HH:mm:ss');
    // console.log(fmtStartTime);

    var state = 0;

    rm_task_li.push(robot_id);

    // if (state === 2 || state === 3) {
    //   $('#' + uuid).parent().remove();
    //   updateJobStatistics("flow");
    //   return;
    // }
    $('#flow-timer-sche').append(genTaskInfoTemplate(uuid, task_name, robot_id, task_state[state], fmtStartTime, complete_percent, "flow"));

    if ($('#' + uuid).length) {
      if (start_time >= timeNow) {
        update_task_info(uuid, task_name, robot_id, task_state[state], fmtStartTime, complete_percent, "flow");
      } else {
        clearExpiredJobs();
      }
    } else {
      if (state === 0 || state === 1) {
        if (start_time >= timeNow) {
          $('#flow-timer-sche').append(genTaskInfoTemplate(uuid, task_name, robot_id, task_state[state], fmtStartTime, complete_percent, "flow"));
        } else {
          clearExpiredJobs();
        }
      } else {
        if ($('.no-task').length === 0 && $('.info-box.bg-dark').length === 0) {
          $('#flow-timer-sche').append(no_task);
        } else {
          clearExpiredJobs();
        }
      }
    }
    // updateJobStatistics("flow");
  });

  $("#r-t-a-flow").attr("data-main_id", main_timer_flow_id);
}

function clearExpiredJobs() {
  $('.info-box.bg-dark').each(function (index) {
    // current_list.push($(this).attr('id'));
    var time_date = $(this).find('.time').text();
    var y = time_date.split(' ')[0].split('/')[0];
    var m = time_date.split(' ')[0].split('/')[1];
    var d = time_date.split(' ')[0].split('/')[2];
    var hr = time_date.split(' ')[1].split(':')[0];
    var min = time_date.split(' ')[1].split(':')[1];
    var timeShe = new Date(y, m - 1, d, hr, min).getTime();
    var timeNow = Date.now();
    var flow_state = $(this).find('.flow-state').text();

    if (timeNow > timeShe && flow_state !== 'ACTIVE') {
      $(this).parent().remove();
    }
  });
}

function genTaskInfoTemplate(uuid, task_name, robot_id, state, start_time, complete_percent, flow_or_task) {
  return `
    <div class="col-md-12 col-sm-12 col-12">
      <div class="info-box farobot-info-bg-queued" id="${uuid}">
        <span class="info-box-icon"><i class="fas fa-tasks"></i></span>

        <div class="info-box-content">
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
              <span class="info-box-text">${capitalize(flow_or_task)} ID: </span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <span class="info-box-text ${flow_or_task}-name">${uuid} </span>
            </div>
          </div>
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
              <span class="info-box-text">Start time: </span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <span class="info-box-text time">${start_time} </span>
            </div>
          </div>
        </div>
        <!-- /.info-box-content -->
        <span class="info-box-icon"><i class="fas fa-trash"></i></span>
      </div>
      <!-- /.info-box -->
    </div>`;

  // <div class="row">
  //         <div class="col-md-3 col-sm-3 col-3">
  //           <span class="info-box-text">Agent: </span>
  //         </div>
  //         <div class="col-md-9 col-sm-9 col-9">
  //           <span class="info-box-text ${flow_or_task}-agent">${robot_id} </span>
  //         </div>
  //       </div>
  //       <div class="row">
  //         <div class="col-md-3 col-sm-3 col-3">
  //           <span class="info-box-text">Status: </span>
  //         </div>
  //         <div class="col-md-9 col-sm-9 col-9">
  //           <marquee direction="left" scrollamount="2" behavior="scroll" id="" class="${flow_or_task}-state">${state}</marquee>
  //         </div>
  //       </div>
  // <div class="row">
  //         <div class="col-md-3 col-sm-3 col-3">
  //         <span class="info-box-number ${flow_or_task}-complete" id="">${complete_percent}%</span>
  //         </div>
  //         <div class="col-md-9 col-sm-9 col-9">
  //           <div class="progress">
  //             <div class="progress-bar ${flow_or_task}-complete-bar" style="width: ${complete_percent}%"></div>
  //           </div>
  //         </div>
  //       </div>
}

function updateJobStatistics(_jobType) {
  var jobArr = $('.info-box');
  // console.log(jobArr.length);

  if (jobArr.length === 0) {
    $('#q-l').text(`QUEUED: 0`);
    $('#a-l').text(`ACTIVE: 0`);
    return;
  }

  var numQueued = 0;
  var numActive = 0;

  jobArr.each(function () {
    console.log($(this).find('.time').text());
    switch ($(this).text()) {
      case "QUEUED":
        numQueued += 1;
        break;
      case "ACTIVE":
        numActive += 1;
        break;
      default:
        break;
    }
  })

  // console.log(n_a_l);
  $('#q-l').text(`QUEUED: ${numQueued}`);
  $('#a-l').text(`ACTIVE: ${numActive}`);
}

// for assigned robot
async function rmtTokenCheck() {
  if (!rmtToken_) {
    console.log(rmtToken_);
    try {
      console.log('------ ###### rmt token re-fetch #### ------')
      rmtToken_ = await fetchToken();
    } catch (err) {
      console.error(err);
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

async function getRMTscanRobot(_selFleet) {
  try {
    await rmtTokenCheck();
    scannedAgents = await fetchScanRobots2(rmtToken_);
  } catch (err) {
    console.error(err);
  }
}

$(window).bind('beforeunload', function () {
  if (bUnsavedChages_) {
    return "You have unsaved changes on this page. Are you sure you want to leave?";
  }
});

// Load fleet setting
async function loadFleetSettings() {
  $('#artifact_conf_panel').hide();
  var selFleet = getSelectedFleet();
  await switchFleetContext(selFleet);

  // updateContentColorTheme();
}

async function switchFleetContext(_selFleet) {
  let fltSettings;
  try {
    fltSettings = await restGetFleetSettings(_selFleet);
  } catch (err) {
    console.error(err);
  }

  settingsCache_.fleet = fltSettings;
  settingsCache_.fleet[_selFleet]['name'] = _selFleet;

  // --- fetch artifacts of agent in fleet ---
  let scannedArtifacts = {};
  try {
    // await rmtTokenCheck();
    if (rmtToken_ != undefined) {
      scannedArtifacts = await fetchScanArtifacts2(rmtToken_);
    }
  } catch (err) {
    console.error(err);
  }

  scannedArtifacts = scannedArtifacts.artifacts;

  loadFleetArtifacts(settingsCache_.fleet[_selFleet], scannedArtifacts);
}

// UI Append Artifact
async function loadFleetArtifacts(_data, _scanArtifacts = null) {
  console.log(_data);
  console.log(_scanArtifacts);

  // --- clean the panel first ---
  var $artifactDeck = $('#plan-artifact-deck');
  $artifactDeck.empty();

  var fleetName = _data.name;
  var fleetArtifacts = [];
  console.log(_data);
  try {
    if (_data.artifacts.hasOwnProperty('plan')) {
      var artifacts = _data.artifacts.plan;

      artifacts.forEach((artifactId) => {
        var defaultArtifact = {};
        // TODO: artifact_id / artifact_name
        // TODO: remove the workaround
        var artifactInfo = artifactId.split('@');
        defaultArtifact["artifact_id"] = artifactInfo[1] || "";
        defaultArtifact["artifact_name"] = artifactInfo[1] || "none";
        defaultArtifact["type"] = artifactInfo[0] || "none";
        defaultArtifact["fleet"] = fleetName || "";
        defaultArtifact["ip"] = "127.0.0.1";
        defaultArtifact["mac"] = "xx:xx:xx:xx:xx:xx";
        var artifactsObj = [];
        if (_scanArtifacts !== null) {
          artifactsObj = _scanArtifacts.filter(sa => sa.artifact_id === defaultArtifact.artifact_id);
        }
        // console.log(artifactsObj);
        if (artifactsObj.length) {
          defaultArtifact["mode"] = "active";
          defaultArtifact["sw_version"] = artifactsObj[0].sw_version;
          defaultArtifact["ip"] = artifactsObj[0].ip;
          defaultArtifact["mac"] = artifactsObj[0].mac;
        }

        fleetArtifacts.push(defaultArtifact);
      });

      fleetArtifacts.forEach((_artifact) => {
        var node = createArtifactMemberView(_artifact);
        // $artifactDeck.prepend(node);
        $artifactDeck.append(node);
      });
    }
  } catch (err) {
    console.warn(err);
    console.warn('No Plan Artifacts!');
  }
}

// Add Plan
$('#add-plan-artifact').click(popupPlanArtifactCandidatesAsyncCb);

async function popupPlanArtifactCandidatesAsyncCb() {
  // 1. send /artifacts/scan request
  let scanArtifacts;
  try {
    await rmtTokenCheck();
    scanArtifacts = await fetchScanArtifacts2(rmtToken_);
    // console.log(scanArtifacts);
  } catch (err) {
    console.error(err);
  }

  settingsCache_.scanArtifacts = scanArtifacts;

  // 2. update robots on Data
  var candidates = scanArtifacts.artifacts;

  // 3. update robots on View
  updateArtifactCandidates(candidates);
}

function updateArtifactCandidates(_artifacts) {
  // set saved artifct list
  let selected_Artifact_list = [];

  for (const [key, value] of Object.entries(settingsCache_.fleet[getSelectedFleet()].artifacts)) {
    if (key == 'agent') {
      value.forEach(function (agent_item, index) {
        for (const [agent_key, agent_value] of Object.entries(agent_item)) {
          agent_value.forEach(function (item, index) {
            let agent_artifact_name = item.split('@')[1];

            if (selected_Artifact_list.indexOf(agent_artifact_name) < 0) {
              selected_Artifact_list.push(agent_artifact_name);
            }
          });
        }
      });
    } else if (key == 'external') {
      value.forEach(function (item, index) {
        let agent_artifact_name = item.split('@')[1];
        if (selected_Artifact_list.indexOf(agent_artifact_name) < 0) {
          selected_Artifact_list.push(agent_artifact_name);
        }
      });
    }
    else if (key == 'plan') {
      value.forEach(function (item, index) {
        let agent_artifact_name = item.split('@')[1];
        if (selected_Artifact_list.indexOf(agent_artifact_name) < 0) {
          selected_Artifact_list.push(agent_artifact_name);
        }
      });
    }
  }

  // 1. filter available artifacts
  var avArtifacts = _artifacts.filter(a => !selected_Artifact_list.includes(a.artifact_id) && (a.artifact_category != 'External') && (a.artifact_category != 'Embedded'));

  // 2. append to pop-up list
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  avArtifacts.forEach((artifact) => {
    var node = createArtifactCandidateView(artifact);
    ul.appendChild(node);
  })

  // 3. update modal title
  // $('#available-items-title').html('Available Artifacts');
  // [lang] available artifacts
  $('#available-items-title').html(langTemplateObj_.list.artf.lbl_Title);
}

function createArtifactCandidateView(_data) {
  const template = document.querySelector('#agent-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollArtifactToFleetCb.bind(this, _data));

  // console.log(_data);
  var imgNode = node.querySelector('.product-img > img');
  // console.log(_data);
  // console.log(_data.type);
  // console.log(artifactAssets_);
  // console.log(artifactAssets_[_data.type]);

  if (artifactAssets_.hasOwnProperty(_data.type)) {
    imgNode.src = artifactAssets_[_data.type].thumbnail;
  } else {
    imgNode.src = artifactAssets_['default'].thumbnail;
  }


  var titleNode = node.querySelector('.product-title').childNodes[0];
  console.log(_data);
  // titleNode.textContent = _data.artifact_name;
  titleNode.textContent = _data.artifact_id;

  var statusNode = node.querySelector('.product-title > span');
  statusNode.textContent = _data.status;
  // statusNode.textContent = _data.mode;

  // -- badge style --
  // if (_data.status.toLowerCase() === 'active') {
  //   statusNode.classList.add('badge-success');
  // } else if (_data.status.toLowerCase() === 'manual') {
  //   statusNode.classList.add('badge-warning');
  // } else if (_data.status.toLowerCase() === 'off') {
  //   statusNode.classList.add('badge-danger');
  // }

  var infoNode = node.querySelector('.product-description');
  infoNode.textContent = `Model: ${_data.model}, IP: ${_data.ip}`;

  return node;
}

function enrollArtifactToFleet(_inFleet, _artifactId) {
  // console.log(_artifactId);
  // console.log(_inFleet);

  var flt = Object.keys(_inFleet.fleet)[0];

  if (_inFleet.fleet[flt].artifacts.hasOwnProperty('plan')) {
    let plan_item = _inFleet.fleet[flt].artifacts['plan'];
    if (typeof plan_item === 'string' || plan_item instanceof String) {
      let item_list = [];
      item_list.push(plan_item);
      item_list.push(_artifactId);
      _inFleet.fleet[flt].artifacts['plan'] = item_list;
    } else {
      _inFleet.fleet[flt].artifacts['plan'].push(_artifactId);
    }
  } else {
    _inFleet.fleet[flt].artifacts['plan'] = [];
    _inFleet.fleet[flt].artifacts.plan.push(_artifactId);
  }
}

function enrollArtifactToFleetCb(_artifact) {
  var $artifactDeck = $('#artifacts-deck');
  var node = createArtifactMemberView(_artifact);

  $artifactDeck.append(node);

  // -- update the fleet which the agent belongs to --
  // console.log($("#fleet-select option:selected").text());
  _artifact.fleet = $("#fleet-select option:selected").text();

  enrollArtifactToFleet(settingsCache_, `${_artifact.type}@${_artifact.artifact_id}`);

  // -- refresh the artifact-item --
  var candidates = settingsCache_.scanArtifacts.artifacts;
  updateArtifactCandidates(candidates);

  // save to fleet conf
  savePlanArtifact(_artifact);

  loadFleetSettings();
}

function createArtifactMemberView(_data, bRmBtn = true) {
  console.log(_data);
  const template = document.querySelector('#member-card');
  const node = document.importNode(template.content, true);

  var imgNode = node.querySelector('.card-img-top');
  var type = _data.type;

  if (artifactAssets_.hasOwnProperty(type)) {
    imgNode.src = artifactAssets_[type].avatar;
  } else {
    imgNode.src = artifactAssets_['default'].avatar;
  }

  var titleNode = node.querySelector('.info-box-number');
  titleNode.textContent = _data.artifact_id; // TODO: artifact_id or artifact_name

  var statusNode = node.querySelector('.agent-mode');
  statusNode.textContent = _data.mode || 'Not Available';

  var cardNode = node.querySelector('.card');
  var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  cardNode.classList.add(cardClass);
  var closeNode = node.querySelector('.agent-remove');
  closeNode.addEventListener('click', removeArtifactFromFleetCb.bind(cardNode, _data));

  var settingsNode = node.querySelector('.agent-settings');
  settingsNode.addEventListener('click', updateArtifactConfAsyncCb.bind(this, _data));

  if (!bRmBtn) {
    closeNode.parentNode.removeChild(closeNode);
  }

  if (_data.mode1 == "active") {
    settingsNode.parentNode.removeChild(settingsNode);
  }

  var btnNode = node.querySelector('#config_btn');
  btnNode.id = "btn_" + _data.artifact_id;

  var herfNode = node.querySelector('.art_id');
  var art_link = document.createElement('a');
  art_link.setAttribute('href', `http://${_data.ip}:3001/`);
  art_link.innerHTML = _data.ip;
  herfNode.appendChild(art_link);

  return node;
}

function removeArtifactFromFleetCb(_artifact) {
  if (confirm("Remove the artifact?")) {
    _artifact.fleet = "";
    console.log('press ok')

    // --- remove artifact card from the deck ---
    $(this).CardWidget('remove');

    var fltKey = Object.keys(settingsCache_.fleet)[0];
    // var artifactIndex = settingsCache_.fleet[fltKey].artifacts.indexOf(_artifact.artifact_id);
    var artifactIndex = settingsCache_.fleet[fltKey].artifacts.plan.indexOf(_artifact.artifact_id);
    console.log(settingsCache_.fleet[fltKey].artifacts);
    // settingsCache_.fleet[fltKey].artifacts.splice(artifactIndex, 1);
    settingsCache_.fleet[fltKey].artifacts.plan.splice(artifactIndex, 1);

    removePlanArtifact(fltKey, _artifact.artifact_id);

    loadFleetSettings();
  } else {
    console.log('press cancel')
  }
}

async function updateArtifactConfAsyncCb(_artifact) {
  let data;
  try {
    console.log(_artifact.artifact_id);
    await rmtTokenCheck();

    data = await fetchArtifactSettings(rmtToken_, _artifact.artifact_id);
    if (data.hasOwnProperty('detail')) {
      alert(data.detail);
      return;
    }
    settingsCache_.artifact = data;
  } catch (err) {
    console.error(err);
    alert(err);
  }

  artifact_tab_row_content_dict = {};

  let artifact_obj = data.artifact_settings[0].artifact_conf;
  artifact_obj = artifact_obj.replace(/'/g, '"');

  artifact_obj = JSON.parse(artifact_obj);
  settingsCache_.artifact.artifact_settings[0].artifact_conf = artifact_obj;

  processArtifactConfHtml(artifact_obj);
}

function processArtifactConfHtml(artifact_obj) {
  artifact_flatten_dict = {};
  // console.log(artifact_obj);
  let flatten_artifact_configuration_oj = restGetFlattenArtifactConf(artifact_obj, false);
  let nav_tab_list = Object.keys(artifact_obj);
  artifact_flatten_dict = flatten_artifact_configuration_oj;

  updateArtifactTabDict(flatten_artifact_configuration_oj);
  genPlanArtifactHtml(nav_tab_list);

  $('#artifact_conf_panel').show();
}

function updateArtifactTabDict(config_obj) {
  let cof_index = 0;
  for (const [key, value] of Object.entries(config_obj)) {
    let tab_content_index = key.split('.')[0];
    if (artifact_tab_row_content_dict.hasOwnProperty(tab_content_index)) {
      if (artifact_tab_row_content_dict[tab_content_index].hasOwnProperty('right') && artifact_tab_row_content_dict[tab_content_index].hasOwnProperty('left')) {
        // pass
      } else {
        artifact_tab_row_content_dict[tab_content_index]['right'] = [];
        artifact_tab_row_content_dict[tab_content_index]['left'] = [];
      }
      cof_index += 1;
    } else {
      artifact_tab_row_content_dict[tab_content_index] = { 'right': [], 'left': [] };
      cof_index = 0;
    }

    let left_right = '';
    if (cof_index % 2 == 0) {
      left_right = 'left-col'
    } else {
      left_right = 'right-col'
    }

    let tab_row_content = `
                          <div class="row ${left_right}" style="padding-top:0.3rem; min-height: 72px">
                            <div class="col-5 col-md-6 align-self-center">
                              <div type="text" class="param-label" style="text-align:center; font-size: 1.0rem;">${key}</div>
                            </div>
                            <div class="col-7 col-md-6 align-self-center">
                                <div class="input-group">
                                  <input type="text" class="form-control" readonly="true" id="${key}" value="${value}">
                                  <div class="input-group-append">
                                    <div class="input-group-text param-edit"><i class="fas fa-pen"></i></div>
                                  </div>
                                </div>
                            </div>
                          </div>`;
    if (cof_index % 2 == 0) {
      artifact_tab_row_content_dict[tab_content_index]["left"].push(tab_row_content);
    } else {
      artifact_tab_row_content_dict[tab_content_index]["right"].push(tab_row_content);
    }
  }
}

function genPlanArtifactHtml(nav_tab_list) {
  $('#artifact_content').empty();
  let nav_tab_li = '';
  let nav_tab_content = '';
  // console.log(nav_tab_list);
  nav_tab_list.forEach(function (item, index) {
    let tab_text = item;

    if (index == 0) {
      nav_tab_li += `<li class="nav-item">
                    <a class="nav-link active" data-toggle="tab" font-size="1.5rem" href="#${tab_text}">${tab_text}</a>
                  </li>`;

      nav_tab_content += `
                        <div class="tab-pane active" style="overflow-y:scroll; height:200px" id="${tab_text}">
                          <div class="row col-12">
                            <div class="col-12 col-md-6">
                              ${artifact_tab_row_content_dict[tab_text]['left'].join('')}
                            </div>
                            <div class="col-12 col-md-6">
                              ${artifact_tab_row_content_dict[tab_text]['right'].join('')}
                            </div>
                          </div>
                        </div>`;
    } else {
      nav_tab_li += `<li class="nav-item">
                    <a class="nav-link" data-toggle="tab" font-size="1.5rem" href="#${tab_text}">${tab_text}</a>
                  </li>`;

      nav_tab_content += `
                        <div class="tab-pane" style="overflow-y:scroll; height:200px" id="${tab_text}">
                          <div class="row col-12">
                            <div class="col-12 col-md-6">
                              ${artifact_tab_row_content_dict[tab_text]['left'].join('')}
                            </div>
                            <div class="col-12 col-md-6">
                              ${artifact_tab_row_content_dict[tab_text]['right'].join('')}
                            </div>
                          </div>
                        </div>`;
    }

  });

  let conf_html_str = `
    <div class="card card-dark card-outline" style="font-size: 100%;">
      <div class="card-header p-2" style="font-size: 100%;">
        <ul class="nav nav-tabs" style="font-size: 100%;">
          ${nav_tab_li}
        </ul>
      </div>
      <div class="card-body" style="font-size: 100%;">
        <div id="overlay-div" style="font-size: 100%; display: none;"></div>
        <div class="tab-content" style="font-size: 100%;">
        ${nav_tab_content}
        </div>
      </div>
    </div>
  `;

  $('#artifact_content').append(conf_html_str);
}

$("#artifact_conf_save").click(async function () {
  let val_col_list = ['left-col', 'right-col'];
  val_col_list.forEach(function (item, idx) {
    $(`.${item}`).each(function () {
      let input_element = $(this).find('.form-control')
      let key_ = input_element.attr('id');
      let val_ = input_element.val();
      console.log(key_, val_)
      artifact_flatten_dict[key_] = val_;
    });
  });

  let artifact_settings = settingsCache_.artifact.artifact_settings[0];
  let unflatten_artifact_configuration_oj = restGetUnFlattenArtifactConf(artifact_flatten_dict, false);
  artifact_settings.artifact_conf = unflatten_artifact_configuration_oj;
  let save_data = JSON.stringify(artifact_settings.artifact_conf);
  save_data = save_data.replace(/"/g, "'");

  let artifact_id = artifact_settings.artifact_id;
  await rmtTokenCheck();
  let result = await fetchPutArtifactSettings(rmtToken_, artifact_id, save_data);
  console.log(result)
  if (result["artifact_list"][0] == artifact_id) {
    notificationMsg(1, "Config Update!");
  } else {
    notificationMsg(3, `Config Update Fail! ${result}`);
  }
});

$('body').on('click', '.param-edit', function () {
  let icon_element = $(this);
  let input_element = $(this).parent().parent();

  icon_element.find('.fas').toggleClass("fa-pen");
  icon_element.find('.fas').toggleClass("fa-eye");

  if (input_element.find('.form-control').prop('readonly')) {
    input_element.find('.form-control').removeAttr('readonly')
  } else {
    input_element.find('.form-control').attr('readonly', 'readonly')
  }
});

function savePlanArtifact(_artifact_data) {
  let post_res = restPostFleetPlanArtifactSettings(_artifact_data.fleet, _artifact_data, false);
  notificationMsg(1, post_res);
}

function removePlanArtifact(fleet_name, artifact_name) {
  let post_res = restDeleteFleetPlanArtifactSettings(fleet_name, artifact_name, false);
  notificationMsg(1, post_res);
}

// --- add input values validator ---
function validateFlowNameInputEvent(rule_list = []) {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#save-flow-btn');

  const flowNameRule = new Rule('flow-name', textNameValidation, $('#add-flow'));
  const editflowNameRule = new Rule('editing-flowname-input', textNameValidation, interaction_element);
  const failuretimeoutRule = new Rule('failure_timeout', positivenumberValueValidation, interaction_element);
  const errorTypeRule = new Rule('error_type', textNameValidation, interaction_element);
  const retryLimitRule = new Rule('retry_limit', positivenumberValueValidation, interaction_element);
  const eventNameRule = new Rule('event-id', textNameValidation, $('#add-timer'));
  const startTimeRule = new Rule('startTime', startTimeValidation, $('#add-timer'));
  const endTimeRule = new Rule('endTime', endTimeValidation, $('#add-timer'));

  validatorManager.addValidator(flowNameRule);
  validatorManager.addValidator(editflowNameRule);
  validatorManager.addValidator(failuretimeoutRule);
  validatorManager.addValidator(errorTypeRule);
  validatorManager.addValidator(retryLimitRule);
  validatorManager.addValidator(eventNameRule);
  validatorManager.addValidator(startTimeRule);
  validatorManager.addValidator(endTimeRule);

  rule_list.forEach(element_rule => {
    validatorManager.addValidator(element_rule);
  });
  // console.log(role_validatorManager);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator,
  //                  * run()         : run the valiations,
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {
    const validator = vm.getValidator(this.id);
    if (validator == undefined) { return; }

    const res = validator.run(this.value, false);

    // --- [STYLING] reflect validation result ---

    let switch_id = this.id;
    if (this.id.includes("input-name-")) {
      switch_id = 'input-name';
    }

    // --- [UX] Interaction Logics ---
    switch (switch_id) {
      case 'flow-name':
        $('#add-flow').prop('disabled', !res.bValid);
        break;
      case 'editing-flowname-input':
        $('#save-flow-btn').prop('disabled', !res.bValid);
        break;
      case 'event-id':
        $('#add-event').prop('disabled', !res.bValid);
        break;
      case 'startTime':
        if ($("#endTime").css("border-color") == 'rgb(255, 0, 0)') {
          break;
        }
        $('#add-timer').prop('disabled', !res.bValid);
        break;
      case 'endTime':
        if ($("#startTime").css("border-color") == 'rgb(255, 0, 0)') {
          break;
        }
        $('#add-timer').prop('disabled', !res.bValid);
        break;
      case 'failure_timeout':
        $('#save-flow-btn').prop('disabled', !res.bValid);
        validate_res_dict[this.id] = res.bValid;
        break;
      case 'error_type':
        $('#save-flow-btn').prop('disabled', !res.bValid);
        validate_res_dict[this.id] = res.bValid;
        break;
      case 'retry_limit':
        $('#save-flow-btn').prop('disabled', !res.bValid);
        validate_res_dict[this.id] = res.bValid;
        break;
      case 'input-name':
        $('#save-flow-btn').prop('disabled', !res.bValid);
        let geer_icon = `detailid_${this.id.replace("input-name-", "")}`;
        $(`#${geer_icon}`).prop('disabled', !res.bValid)
        break;

      default:
        $('#save-flow-btn').prop('disabled', !res.bValid);
        break;
    }

  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('flow-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

// Start Time
function startTimeValidation(inputVal) {
  const reg1 = /(^([01]\d|2[0-3]):([0-5]\d)$)|(^24:00$)/;     // 00:00 - 24:00

  let bRes = reg1.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid time format, avaliable 00:00 - 24:00';
  strMsg.push(msg1);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// End Time
function endTimeValidation(inputVal) {
  const reg1 = /(^([01]\d|2[0-3]):([0-5]\d)$)|(^24:00$)/;     // 00:00 - 24:00

  let bRes = reg1.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid time format, avaliable 00:00 - 24:00';
  strMsg.push(msg1);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  if (inputVal.length == 0) {
    bRes = true;
    strMsg = [];
  }

  return { bValid: bRes, strMsg: strMsg }
}