/*
 * Author: Angela Kao
 * Date: 5 Auguest 2021
 * Description:
 **/

var currentTaskName = "";
var currentFlowName = "";
var isflow = false;
var flowArray = [];
// var mapList = [];
var mapCelldict = {};
var timerInfoString = "";
var timerTriggerTimes = 0;
var timerFileString = "";
var tempTaskMsgArr = [];

var timer_info_show = `
<div class="row" id="task-r-info" style="padding: 10px;">
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center;" id="q-l">QUEUED: 0</div>
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center;" id="a-l">ACTIVE: 0</div>
  <div class="col-md-3 col-sm-3 col-3"></div>
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center; margin-top: 15px;"><button type="button" class="btn btn-success" style="margin-right: 10px;" id="r-t">Run Timer</button><button type="button" class="btn btn-danger" style="margin-left: 10px;" id="r-t-a">Remove All</button></div>
  <div class="col-md-3 col-sm-3 col-3"></div>
</div>`;

var timer_info_show_flow = `
<div class="row" id="task-r-info" style="padding: 10px;">
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center;" id="q-l">QUEUED: 0</div>
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center;" id="a-l">ACTIVE: 0</div>
  <div class="col-md-3 col-sm-3 col-3"></div>
  <div class="col-md-6 col-sm-6 col-6" style="text-align: center; margin-top: 15px;"><button type="button" class="btn btn-success" style="margin-right: 10px;" id="r-t-flow">Run Timer</button><button type="button" class="btn btn-danger" style="margin-left: 10px;" id="r-t-a-flow">Remove All</button></div>
  <div class="col-md-3 col-sm-3 col-3"></div>
</div>`;

var isRemoveAllflowPressed = false;
var isRemoveAlltaskPressed = false;

$(function () {
  'use strict'

  $("#flow-row").hide();
  $("#back-div").hide();
  initDataAsync();
  initDatePicker();
  setInterval(() => {
    subscribe_tasks_schedule2();
  }, 1000);

  $.cookie('flow_name', "", {
    expires: 1,
    path: '/operation.html'
  });

  $.cookie("planner", JSON.stringify({
    "plant_event": [],
    "flows": []
  }), {
    expires: 1,
    path: '/operation.html'
  });
});

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'operation');

  // wait until sidebar fleet is generated
  await sleep(1000); // sleep 1s
  var selectedFleet = getSavedFleet();
  var fleetData = await getFleetSettingsData(selectedFleet + '.yaml');

  jQuery.ajaxSetup({
    async: false
  });
  getTaskData(selectedFleet);
  getFlowData(selectedFleet);
  jQuery.ajaxSetup({
    async: true
  });
}

function initDatePicker() {
  var sys_date_millisec = get_system_time();
  var sys_date = new Date(sys_date_millisec);
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

async function getTaskData(selectedFleet) {
  $("#add-list").children('div').remove();
  var taskArray = await restGetTasks(selectedFleet);
  // resetTaskTriggerList();
  taskArray.forEach(function (jsonData) {
    addTaskButton(jsonData.task);
    $(".task-div input[type=text]").addClass("unavailable");
    // var info = jsonData.task_info;
    // if (info.task_trigger.includes("Manual​")) {
    //   setUpTaskTriggerList(jsonData.task, 1);
    // }
  });
}

async function getFlowData(selectedFleet) {
  var flowData = await restGetFlows(selectedFleet);
  flowArray = flowData;
  flowData.sort(function (a, b) {
    var x = a.flow_name.toLowerCase(), y = b.flow_name.toLowerCase();
    return x < y ? -1 : x > y ? 1 : 0;
  }).forEach(function (dataObj) {
    var jsonData = JSON.parse(JSON.stringify(dataObj))
    addFlowButton(jsonData.flow_name);
    $(".flow-div input[type=text]").addClass("unavailable");
    // if (jsonData.Event.event.includes("Manual")) {
    //   // ToDO Check type
    //   console.log(jsonData.Event.event === "Manual")
    //   setUpTaskTriggerList(jsonData.flow_name, 2);
    // }
  });
}

async function getFleetSettingsData(fleetFileName) {
  var fleetName = fleetFileName.split('.').slice(0, -1).join('.');
  var data = await restGetFleetSettings(fleetName);

  var data_dict = JSON.parse(data);
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
    // mapList.push(maps_item);
  });
  $.cookie('role', rolesArray.join(","), {
    expires: 1,
    path: '/role.html'
  });
  updateRobotSelectOptionsView(agents);
  updateRoleSelectOptionsView(roles);

}

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

// function getMapsInfo(_id, _map) {
//   $.get(`/map/cells/${_map}`, function (data, status) {
//     if (status === "success") {
//       mapCelldict = {};
//       var cellList = [];
//       for (let [item_key, item_val] of Object.entries(JSON.parse(data))) {
//         var cellDict = {};
//         Object.keys(item_val).map(function(key, index) {
//           cellDict[item_val[key].cell_id] = item_val[key].cell_coordinate;
//         });
//         mapCelldict[item_key] = cellDict;
//       }

//       $(_id).empty();
//       $(_id).append('<option value="" selected disabled>Select Area ...</option>');
//       for (let [map_key, map_val] of Object.entries(mapCelldict)) {
//         $(_id).append(`<option value=${map_key}>${map_key}</option>`);
//       }
//       var cellSelectID = _id.replace('area_select', 'cell_select');
//       $(cellSelectID).empty();
//       $(cellSelectID).append('<option value="" selected disabled>Select Cell ...</option>')
//     }
//   });
// }

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

function addTaskButton(name) {
  $("#add-list").append("<div class='col-11 text-center task-div'><div class='row form-inline'><div class='fixed-30 text-right'><i class='fas fa-clipboard-check fa-lg fa-fw'></i></div><div class='col text-col'><input class='form-control w-100' type='text' value='" + name + "' readonly></div><div class='fixed-40 text-left btn btn-link' id='delete-" + `task-${name.replace(/\s/g, '_')}` + "'><i class='fas fa-times-circle fa-lg'></i></div></div></div>");
}

function addFlowButton(name) {
  $("#add-list").append("<div class='col-11 text-center flow-div'><div class='row form-inline'><div class='fixed-30 text-right'><i class='fas fa-clipboard-list fa-lg fa-fw'></i></div><div class='col text-col'><input class='form-control w-100' type='text' value='" + name + "' readonly></div><div class='fixed-40 text-left btn btn-link' id='delete-" + `flow-${name.replace(/\s/g, '_')}` + "'><i class='fas fa-times-circle fa-lg'></i></div></div></div>");
}

function changeEventTriggerText(mode, text) {
  var modeName = (mode == 1) ? "task-" : "flow-";
  $(".trigger-select option[id=" + modeName + "event]").text(text);
}

function changeTimerTriggerText(mode, text) {
  var modeName = (mode == 1) ? "task-" : "flow-";
  $(".trigger-select option[id=" + modeName + "timer]").text(text);
}

// function setUpTaskTriggerList(taskName, mode) {
//   var event = "";
//   if (mode == 1) {
//     event = "t";
//   } else if (mode == 2) {
//     event = "f";
//   }

//   $(".sidebar #trigger-list").show();
//   $(".sidebar #trigger-list").after("<li class='nav-item trigger-item'><a href='#' class='nav-link' id='" + taskName + "_link_" + event + "'><p>" + taskName + "</p></a></li>");
// }

// function resetTaskTriggerList() {
//   $(".trigger-item").remove();
//   $(".sidebar #trigger-list").hide();
// }

function selectAreaFirstOption() {
  $(".area-list").val($(".area-list option:first").val()).change();
}

// $(document).on('change', '.map-list', function (e) {
//   var map = $(this).val();
//   var id = "#" + $(this).attr('id').replace('map_select', 'area_select');
//   getMapsInfo(id, `${map}.json`);
// });

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
    if (item.content === "N/A") { return; }
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
        // var option_str = "<option value='' selected disabled>Select Map ...</option>";
        // mapList.forEach(function (map_item, map_index) {
        //   option_str += `<option value=${map_item}>${map_item}</option>`;
        // });
        // $tr.append('<td class="text-right py-0 align-middle content-td" id="' + roleKey + '"><select class="form-control select-bold-text map-list" id="'+roleKey+'_map_select" style="width: 40%;">' + option_str +'</select><select class="form-control select-bold-text area-list" id="'+roleKey+'_area_select" style="width: 40%;"><option value="" selected disabled>Select Area ...</option></select><select class="form-control select-bold-text" id="'+roleKey+'_cell_select" style="width: 40%;"><option value="" selected disabled>Select Cell ...</option></select></td>');
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

$("#add-menu .dropdown-item").click(function () {
  if (this.id == "task-item") {
    $(this).attr('data-target', '#task-modal');
  } else {
    $(this).attr('data-target', '#flow-modal');
  }
});

async function checkFileName(mode, name) {
  var filename_regex = /^[a-zA-Z0-9_]+$/;
  var modeName = mode == 1 ? "task" : "flow";
  var data = await restOperationFileExistence(modeName, name);
  if (data.fileExists) {
    alert(`${name} already exists!`);
  } else if (name === "") {
    alert(`Please fill in ${modeName} name!`);
  } else if (hasWhiteSpace(name)) {
    alert(`${modeName} name can't include white space!`);
  } else if (!filename_regex.test(name)) {
    alert(`${modeName} name only accepts letters and numbers and underscore!`);
  } else if (mode == 1) {
    addTaskButton(name);
    changeAvaliableStatus($(".task-div input[type=text]"));
  } else if (mode == 2) {
    addFlowButton(name);
    changeAvaliableStatus($(".flow-div input[type=text]"));
  }
}

$("#add-task").click(function () {
  $("#task-modal").modal('hide');
  var taskName = $("#task-name").val();
  checkFileName(1, taskName);
});

$("#add-flow").click(function () {
  $('#flow-modal').modal('hide');
  var flowName = $("#flow-name").val();
  checkFileName(2, flowName);

  $.cookie("planner", JSON.stringify({
    "plant_event": [],
    "flows": []
  }), {
    expires: 1,
    path: '/operation.html'
  });

  $.cookie('flow_name', flowName, {
    expires: 1,
    path: '/operation.html'
  });
});

$("#add-event").click(function () {
  var eventID = $("#event-id").val();

  if (isflow) {
    resetTaskTriggerSelectOptions("flow");
    var test_pln_event = {
      "Event": "Event_" + eventID,
      "flows": currentFlowName
    };
    var planner_cookie = $.parseJSON($.cookie("planner"));
    planner_cookie.plant_event.push(test_pln_event);
    $.cookie("planner", JSON.stringify(planner_cookie), {
      expires: 1,
      path: '/operation.html'
    });
    console.log($.parseJSON($.cookie("planner")));
    set_flow_event("Event_" + eventID);
  } else {
    resetTaskTriggerSelectOptions("task");
  }

  var mode = (isflow ? 2 : 1);
  changeEventTriggerText(mode, "Event: " + eventID);
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
          "flows": currentFlowName,
          "Starttime": startTimeString,
          "Endtime": $("#endTime").val(),
          "Duration": $("#cycle-time").val()
        };
        isrepeat = true;
      } else {
        test_pln_event = {
          "Event": "Timer",
          "flows": currentFlowName,
          "Starttime": startTimeString
        };
        isrepeat = false;
      }

      var planner_cookie = $.parseJSON($.cookie("planner"));
      planner_cookie.plant_event.push(test_pln_event);
      $.cookie("planner", JSON.stringify(planner_cookie), {
        expires: 1,
        path: '/operation.html'
      });
      console.log($.parseJSON($.cookie("planner")));
      set_flow_event("Timer");
      get_time_trigger_event(startTimeString, $("#endTime").val(), $("#cycle-time").val(), isrepeat, $('#startDate').val(), $('#endDate').val());
    } else {
      resetTaskTriggerSelectOptions("task");
    }

    startTimeString += needRepeat ? " (Rep)" : ""
    var mode = (isflow ? 2 : 1);
    changeTimerTriggerText(mode, "Timer: " + startTimeString);
    $('#timer-trigger-modal').modal('hide');

    var startDate = $("#startDate").val();
    timerInfoString = startDate + " " + $("#startTime").val();
    if (needRepeat) {
      var endDate = $("#endDate").val();
      timerInfoString = timerInfoString.concat(";", $("#cycle-time").val(), ";", endDate, " ", $("#endTime").val());
    }
  }
});

$(document).on('click', "div[id*='delete-']", function (e) {
  var current_id = $(this).attr('id');
  var current_event = $(this).attr('id').split('-')[1];
  var current_event_name = $(this).attr('id').split('-')[2];
  var selectOp = $(this).parent().find('input[type=text]').val();
  var fleet = $("#fleet-select").val();

  switch (current_event) {
    case 'task':
      if (selectOp === currentTaskName) {
        currentTaskName = "";
        $("#task-deck").hide();
      }

      // var delFileName = `${fleet}-${current_event}-${selectOp}.json`;
      restDeleteTaskData(fleet, selectOp);
      toast(`${selectOp} is deleted`);

      break;
    case 'flow':
      $.cookie('flow_name', current_event_name, {
        expires: 1,
        path: '/operation.html'
      });
      var deleteFlowName = $(this).prev().children('input[type=text]').val();
      flowArray = flowArray.filter(function (flow, flow_index) {
        return flow.flow_name !== deleteFlowName;
      });
      delete_flow();
      break;
    default:
      break;
  }

  e.stopPropagation();
  $(this).parent().parent().remove();

  // delete task trigger if exists
  $(".trigger-item p").each(function () {
    if ($(this).text() === selectOp) {
      $(this).parent().parent('li').remove();
    }
    chkTriggerListVisible();
  });
});

$(document).on('click', '.task-div', function (e) {
  console.log('task clicked')
  isflow = false;
  $("#task-row").show();
  $("#task-deck").show();
  $("#flow-row").hide();
  $("#back-div").hide();
  $('#main-tab').tab('show')

  $(this).find("div.text-col").children('input[type=text]').removeClass("available").addClass("unavailable");
  currentTaskName = $(this).find("div.text-col").children('input[type=text]').val();
  $("#task-deck .card-header").html(currentTaskName);

  if (istaskTimer(currentTaskName)) {
    console.log('---- A task WITH timer ----')
    $('#timer_label').text("Timer schedule")
    $('#timer-tab').css('display', '');
    $('#main-tab').css('border-top-right-radius', '');
    $('#main-tab').parent().css("width", "50%");
    $('#main-tab').attr("data-toggle", "tab");
    $('#main-tab').attr("href", "#main");
    $('#main-tab').addClass('active');
  } else {
    console.log('---- A task WITHOUT timer ----')
    $('#timer-tab').css('display', 'none');
    $('#main-tab').tab('show');
    $('#main-tab').parent().css("width", "100%");
    $('#main-tab').css("border-top-right-radius", "15px");
    $('#main-tab').removeAttr("data-toggle");
    $('#main-tab').removeAttr("href");
    // $('#main-tab').removeClass('active');
  }

  $(".role_title_tr").css('display', 'none');
  resetTaskOptions();
  checkDisableManual(1);
});

$(document).on('click', '.flow-div', function (e) {
  isflow = true;
  var flowName = $(this).find('input[type=text]').val();

  $("#task-row").hide();
  $("#flow-row").show();
  $("#back-div").show();

  $(this).find("div.text-col").children('input[type=text]').removeClass("available").addClass("unavailable");
  currentFlowName = $(this).find("div.text-col").children('input[type=text]').val();

  if (isflowTimer(currentFlowName)) {
    $("#flow-deck .card-header").html(currentFlowName);
    $('#timer_label_flow').text("Timer schedule")
    $('#flow-profile-tab').css('display', '');
    $('#flow-home-tab').css('border-top-right-radius', '');
    $('#flow-home-tab').css('padding', '0px');
    $('#flow-home-tab').parent().css("width", "50%");
    $('#flow-home-tab').attr("data-toggle", "tab");
    $('#flow-home-tab').attr("href", "#flow-home");
    $('#flow-home-tab').addClass('active');
  } else {
    $("#flow-deck .card-header").html(currentFlowName);
    $('#flow-profile-tab').css('display', 'none');
    $('#flow-home-tab').tab('show');
    $('#flow-home-tab').parent().css("width", "100%");
    $('#flow-home-tab').css("border-top-right-radius", "15px");
    $('#flow-home-tab').css('padding', '0px');
    $('#flow-home-tab').removeAttr("data-toggle");
    $('#flow-home-tab').removeAttr("href");
    // $('#flow-home-tab').removeClass('active');
  }

  resetTaskOptions();
  checkDisableManual(2);

  $.cookie("planner", JSON.stringify({
    "plant_event": [],
    "flows": []
  }), {
    expires: 1,
    path: '/operation.html'
  });

  $.cookie('flow_name', flowName, {
    expires: 1,
    path: '/operation.html'
  });

  init_flow(flowName);
});

$(document).on('click', '#flow-back', function (e) {
  window.location.href = "operation.html";
});

$(".trigger-select").on('click', function () {
  console.log($(this).attr("id"));
  var platform = navigator.platform;
  if (platform === 'iPad') { return; }
  console.log(platform + " click trigger type");
  var show_name = "";
  if ($(this).attr("id") == "flow_select") {
    show_name = currentFlowName;
  } else {
    show_name = currentTaskName;
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
  var platform = navigator.platform;
  if (platform !== 'iPad') { return; }
  console.log(platform + " change trigger type");
  var show_name = "";
  if ($(this).attr("id") == "flow_select") {
    show_name = currentFlowName;
  } else {
    show_name = currentTaskName;
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

// $(".trigger-select").change(function () {
//   console.log($(this).attr("id"))
//   var show_name = "";
//   if ($(this).attr("id") == "flow_select") {
//     show_name = currentFlowName;
//   } else {
//     show_name = currentTaskName;
//   }

//   if ($(this).val() === 'manual') {
//     $("#manual-trigger-confirm-modal").modal('show');
//     $("#manual-trigger-confirm-modal .modal-body").html("Are you sure you want to trigger " + show_name + "?");
//   } else if ($(this).val() === 'event') {
//     $("#event-label").html(show_name + ' event:');
//     $("#event-trigger-modal").modal('show');
//   } else {
//     $("#timer-label").html(show_name + ' Timer:');
//     $("#timer-trigger-modal").modal('show');
//   }
// });

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
  currentTaskName = "";
  currentFlowName = "";
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
    if ($.trim(currentFlowName) == '') {
      return;
    }
    flowArray.forEach(function (flow, flow_index) {
      if (flow.flow_name === currentFlowName) {
        if (flow.Event.event.includes("Manual")) {
          $('#flow-manual').prop("selected", true);
        } else if (flow.Event.event.includes("Event")) {
          changeEventTriggerText(2, flow.Event.event);
          $('#flow-event').prop("selected", true);
        } else if (flow.Event.event.includes("Timer")) {
          changeTimerTriggerText(2, flow.Event.event);
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
    if ($.trim(currentTaskName) == '') {
      return;
    }
    var fleet = $("#fleet-select").val();
    var data = await restGetTaskData(fleet, currentTaskName);
    var info = data.task_info;
    $(".trigger-select").val(info.task_trigger);
    if (info.task_trigger.includes("Event")) {
      changeEventTriggerText(1, info.task_trigger);
      $('#task-event').prop("selected", true);
    } else if (info.task_trigger.includes("Timer")) {
      changeTimerTriggerText(1, info.task_trigger);
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
  var selectedName = (mode == 1) ? currentTaskName : currentFlowName;
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
  var selFleet = getSavedFleet();
  var fleetData = await getFleetSettingsData(selFleet + '.yaml');
  getTaskData(selFleet);
  getFlowData(selFleet);
  resetSelectedName();
  resetTaskOptions();
  $("#task-deck").hide();

  switch_fleet(selFleet);

  $('#task-schedule-div').html('');
  $('#task-schedule-div').append('<div class="card col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
  tempTaskMsgArr = [];
}

function hasWhiteSpace(str) {
  return str.indexOf(' ') >= 0;
}

function getDefaultValIfNull(val) {
  return val === null ? "" : val;
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
    "task": currentTaskName,
    'task_info': taskObj
  };
  // var taskString = JSON.stringify(task_info);
  console.log(task);

  await restPostTaskData(task, fleet, currentTaskName);
  toast(`${currentTaskName} is saved`);

  // add manual trigger list
  var mode = (isflow ? 2 : 1);
  if ($('#task_select').val() === "manual" && mode === 1) {
    createTaskTriggerItem(fleet, (isflow ? currentFlowName : currentTaskName), mode);
    chkTriggerListVisible();
    checkDisableManual(mode);
  }
}

function getSeconds(time) {
  var timeArray = time.split(":");
  return timeArray.length > 1 ? timeArray[0] * 3600 + timeArray[1] * 60 : timeArray[0] * 60;
}

// Add for Timer schedule

function istaskTimer(task_name) {
  var fleet = $("#fleet-select").val();
  var istimer = false;

  restGetTaskData(fleet, task_name, false).done(function (data) {
    if (data.task_info.task_trigger.includes('Timer')) {
      istimer = true;
    }
  });

  console.log(istimer)
  return istimer;
}

function isflowTimer(flow_name) {
  var fleet = $("#fleet-select").val();
  var istimer = false;

  restGetFlowData(flow_name, `${fleet}-flow-${flow_name}`, false).done(function (data) {
    var jsonData = JSON.parse(data);
    if (jsonData.event_name.event.includes('Timer')) {
      istimer = true;
    }
  });

  console.log(istimer)
  return istimer;
}

// Flow tab
$(document).on('click', '#flow-profile-tab', function (e) {
  $('.dark-icon-color').addClass('light-icon-color').removeClass('dark-icon-color')
  var current_fleet = $("#fleet-select").val();
  flow_sub(current_fleet)
});

$(document).on('click', '#flow-home-tab', function (e) {
  $('.light-icon-color').addClass('dark-icon-color').removeClass('light-icon-color')
});

// Task tab
$(document).on('click', '#timer-tab', function (e) {
  var current_fleet = $("#fleet-select").val();
  task_sub(current_fleet)
});

// --- ros pub ---
function send_remove(type, task) {
  // let remove_task_msg = new ROSLIB.Message({
  //   data: `${type}:${task}`
  // });
  // remove_task_pub.publish(remove_task_msg);

  var jsonRemoveTask = {
    data: `${type}:${task}`
  };
  wsRemoveTask(jsonRemoveTask);
}

// Task Timer Tab
function compare(a, b) {
  if (a.start_time.sec < b.start_time.sec) {
    return -1;
  }
  if (a.start_time.sec > b.start_time.sec) {
    return 1;
  }
  return 0;
}

// ros sub
function subscribe_tasks_schedule2() {
  console.log(tasks_obj_);
  if (tasks_obj_ === undefined) return;
  var taskFleet = tasks_obj_.fleet;
  var tasks = tasks_obj_.msg;
  // console.log(tasks_obj_.fleet_name);

  var div = "";

  if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append('<div class="card col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
  }
  if (taskFleet !== $('#fleet-select').val()) {
    return
  };

  tasks = tasks.sort(compare);
  if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
    console.log("======get tasks state======");
    console.log(tasks);
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
}

function subscribe_tasks_schedule() {
  task_state_sub.subscribe(function (_message) {
    var taskFleet = _message.fleet_name;
    var tasks = _message.tasks;
    // console.log(_message.fleet_name);

    var div = "";

    if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
      $('#task-schedule-div').append('<div class="card col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
    }
    if (taskFleet !== $('#fleet-select').val()) {
      return
    };

    tasks = tasks.sort(compare);
    if (JSON.stringify(tempTaskMsgArr) !== JSON.stringify(tasks)) {
      console.log("======get tasks state======");
      console.log(tasks);
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

$(document).on('click', '.delete-trigger-btn', function (e) {
  send_remove("id", this.id.replace("delete-", ""));
  $(this).parent().parent().remove();
});

function task_sub(current_fleet) {
  var task_name_c = $('#main-tab').find('.card-header').text();

  $('#task-timer-sche').empty();
  $('#flow-timer-sche').empty();

  if ($('#task-timer-sche-status').children().length == 0) {
    $('#task-timer-sche-status').append(timer_info_show);
  }

  var no_task = '<h1 class="col-md-12 col-sm-12 col-12 no-task" style="text-align: center; margin-top: 40px;">No scheduled tasks.</h1>';
  $('#task-timer-sche').append(no_task);

  taskStateSub_taskSub2();

}

function taskStateSub_taskSub2() {
  // task_state_sub.subscribe(function (_message) {
  if (tasks_obj_ === undefined) return;

  var fleet_name = tasks_obj_.fleet;
  var tasks = tasks_obj_.msg;
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

  console.log(current_fleet)
  console.log(fleet_name)
  console.log(tasks_obj_)
  if (current_fleet === fleet_name) {
    tasks.sort(compare);
    tasks.forEach(function (task_item, task_index) {
      var uuid = task_item.task_id;
      var task_name = task_item.task_name;
      var complete_percent = task_item.complete_percent;
      var robot_id = task_item.robot_id;
      // var priority = task_item.priority;
      // var end_time = task_item.end_time;
      var start_time = task_item.start_time;
      // var role = task_item.role;
      var state = task_item.state;
      // var status = task_item.status;
      // var submission_time = task_item.submission_time;  

      var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
      var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;
      // console.log(c)

      if (task_name === task_name_c) {
        if (state === 2) {
          $('#' + uuid).parent().remove();
        } else if (state === 3) {
          $('#' + uuid).parent().remove();
        } else {
          if ($('#' + uuid).length) {
            update_task_info(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "task");
          } else {
            $('#task-timer-sche').append(task_info_template(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "task"));
          }
        };
        update_info("task");
      }
    });
  } else {
    // $('#task-timer-sche').empty();
  };
  // });
}

// function taskStateSub_taskSub() {
//   task_state_sub.subscribe(function (_message) {
//     var fleet_name = _message.fleet_name;
//     var tasks = _message.tasks;
//     var tasks_num = tasks.length;

//     if (isRemoveAlltaskPressed) {
//       $('#task-timer-sche').empty();
//       $('#task-timer-sche').append(no_task);
//     }

//     if (tasks_num == 0) {
//       isRemoveAlltaskPressed = false;
//     } else {
//       $('.no-task').remove();
//     }

//     console.log(current_fleet)
//     console.log(fleet_name)
//     console.log(_message)
//     if (current_fleet === fleet_name) {
//       tasks.sort(compare);
//       tasks.forEach(function (task_item, task_index) {
//         var uuid = task_item.task_id;
//         var task_name = task_item.task_name;
//         var complete_percent = task_item.complete_percent;
//         var robot_id = task_item.robot_id;
//         // var priority = task_item.priority;
//         // var end_time = task_item.end_time;
//         var start_time = task_item.start_time;
//         // var role = task_item.role;
//         var state = task_item.state;
//         // var status = task_item.status;
//         // var submission_time = task_item.submission_time;  

//         var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
//         var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;
//         // console.log(c)

//         if (task_name === task_name_c) {
//           if (state === 2) {
//             $('#' + uuid).parent().remove();
//           } else if (state === 3) {
//             $('#' + uuid).parent().remove();
//           } else {
//             if ($('#' + uuid).length) {
//               update_task_info(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "task");
//             } else {
//               $('#task-timer-sche').append(task_info_template(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "task"));
//             }
//           };
//           update_info("task");
//         }
//       });
//     } else {
//       // $('#task-timer-sche').empty();
//     };
//   });
// }

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

// run all task timer
$(document).on('click', '#r-t', function (e) {
  var fleet = $("#fleet-select").val();
  var task_name = $('#main-tab').find('.card-header').text();

  resetTimerTriggerTimes();
  send_ros_taskReq(fleet, task_name, true);
});

// remove all task timer
$(document).on('click', '#r-t-a', function (e) {
  var task_name = $('#main-tab').find('.card-header').text();

  $('#task-timer-sche').empty();
  // send cancel all
  console.log(task_name)
  send_remove("name", task_name);
  isRemoveAlltaskPressed = true;
  // console.log(task_name)
  update_info("task");
});

// remove one task/flow
$(document).on('click', '.info-box-icon i.fas.fa-trash', function (e) {
  var task_id = $(this).parent().parent().attr('id');

  // send cancel one
  send_remove("id", task_id);
  $(this).parent().parent().parent().remove();
  // console.log(task_id)
});

// // Task Timer Tab
// let flow_state_sub = new ROSLIB.Topic({
//   ros: ros,
//   name: '/flow_state',
//   messageType: 'far_task_msgs/msg/TaskState'
// });

function flow_sub(current_fleet) {
  var flow_name = $('#flow-home-tab').find('.card-header').text();

  $('#task-timer-sche').empty();
  $('#flow-timer-sche').empty();

  if ($('#flow-timer-sche-status').children().length == 0) {
    $('#flow-timer-sche-status').append(timer_info_show_flow);
  }

  var no_task = '<h1 class="col-md-12 col-sm-12 col-12 no-task" style="text-align: center; margin-top: 40px;">No scheduled flows.</h1>';
  $('#flow-timer-sche').append(no_task);

  // flow_state_sub.subscribe(function (_message) {
  //   var fleet_name = _message.fleet_name;
  //   var tasks = _message.tasks;
  //   var tasks_num = tasks.length;
  //   var current_list = [];

  //   if (isRemoveAllflowPressed) {
  //     $('#flow-timer-sche').empty();
  //     $('#flow-timer-sche').append(no_task);
  //   }

  //   if (tasks_num == 0) {
  //     isRemoveAllflowPressed = false;
  //   } else {
  //     $('.no-task').remove();
  //   }

  //   console.log(_message)
  //   if (current_fleet === fleet_name) {
  //     var rm_task_li = [];
  //     tasks.sort(compare);
  //     tasks.forEach(function (task_item, task_index) {
  //       var timeNow = new Date(Date.now());
  //       timeNow.setSeconds(0, 0);
  //       timeNow = timeNow.getTime();

  //       var uuid = task_item.task_id;
  //       var task_name = task_item.task_name;
  //       var complete_percent = task_item.complete_percent;
  //       var robot_id = task_item.robot_id;
  //       // var priority = task_item.priority;
  //       // var end_time = task_item.end_time;
  //       var start_time = task_item.start_time;
  //       // var role = task_item.role;
  //       var state = task_item.state;
  //       // var status = task_item.status;
  //       // var submission_time = task_item.submission_time;

  //       var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
  //       var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;
  //       // console.log(c)

  //       rm_task_li.push(robot_id);

  //       if (flow_name === task_name) {
  //         if (state === 2) {
  //           $('#' + uuid).parent().remove();
  //         } else if (state === 3) {
  //           $('#' + uuid).parent().remove();
  //         } else {
  //           if ($('#' + uuid).length) {
  //             if (sche_date.getTime() >= timeNow) {
  //               update_task_info(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "flow");
  //             } else {
  //               remove_old_flow();
  //             }
  //           } else {
  //             if (state === 0 || state === 1) {
  //               if (sche_date.getTime() >= timeNow) {
  //                 $('#flow-timer-sche').append(task_info_template(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "flow"));
  //               } else {
  //                 remove_old_flow();
  //               }
  //             } else {
  //               if ($('.no-task').length === 0 && $('.info-box.bg-dark').length === 0) {
  //                 $('#flow-timer-sche').append(no_task);
  //               } else {
  //                 remove_old_flow();
  //               }
  //             }
  //           }
  //         };

  //         update_info("flow");
  //       }
  //     });

  //     // remove missing div
  //     // rm_task_li = new Set(rm_task_li);
  //     // const difference_li = current_list.filter(x => !rm_task_li.has(x));
  //     // difference_li.forEach(function (rm_task_item, rm_task_index) {
  //     //   $(`#${rm_task_item}`).remove();
  //     // });

  //   }

  // });

  var fleet_name = flow_state_obj_.msg.fleet_name;
  var tasks = flow_state_obj_.msg.tasks;
  var tasks_num = tasks.length;
  var current_list = [];

  if (isRemoveAllflowPressed) {
    $('#flow-timer-sche').empty();
    $('#flow-timer-sche').append(no_task);
  }

  if (tasks_num == 0) {
    isRemoveAllflowPressed = false;
  } else {
    $('.no-task').remove();
  }

  console.log(flow_state_obj_.msg)
  if (current_fleet === fleet_name) {
    var rm_task_li = [];
    tasks.sort(compare);
    tasks.forEach(function (task_item, task_index) {
      var timeNow = new Date(Date.now());
      timeNow.setSeconds(0, 0);
      timeNow = timeNow.getTime();

      var uuid = task_item.task_id;
      var task_name = task_item.task_name;
      var complete_percent = task_item.complete_percent;
      var robot_id = task_item.robot_id;
      // var priority = task_item.priority;
      // var end_time = task_item.end_time;
      var start_time = task_item.start_time;
      // var role = task_item.role;
      var state = task_item.state;
      // var status = task_item.status;
      // var submission_time = task_item.submission_time;

      var sche_date = new Date(start_time.sec * 1000 + start_time.nanosec / 1000000);
      var sche_date_trans = `${sche_date.getFullYear()}/${sche_date.getMonth() + 1}/${sche_date.getDate()} ${sche_date.getHours()}:${String(sche_date.getMinutes()).padStart(2, "0")}`;
      // console.log(c)

      rm_task_li.push(robot_id);

      if (flow_name === task_name) {
        if (state === 2) {
          $('#' + uuid).parent().remove();
        } else if (state === 3) {
          $('#' + uuid).parent().remove();
        } else {
          if ($('#' + uuid).length) {
            if (sche_date.getTime() >= timeNow) {
              update_task_info(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "flow");
            } else {
              remove_old_flow();
            }
          } else {
            if (state === 0 || state === 1) {
              if (sche_date.getTime() >= timeNow) {
                $('#flow-timer-sche').append(task_info_template(uuid, task_name, robot_id, task_state[state], sche_date_trans, complete_percent, "flow"));
              } else {
                remove_old_flow();
              }
            } else {
              if ($('.no-task').length === 0 && $('.info-box.bg-dark').length === 0) {
                $('#flow-timer-sche').append(no_task);
              } else {
                remove_old_flow();
              }
            }
          }
        };

        update_info("flow");
      }
    });

    // remove missing div
    // rm_task_li = new Set(rm_task_li);
    // const difference_li = current_list.filter(x => !rm_task_li.has(x));
    // difference_li.forEach(function (rm_task_item, rm_task_index) {
    //   $(`#${rm_task_item}`).remove();
    // });

  }

}

function remove_old_flow() {
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

function task_info_template(uuid, task_name, robot_id, state, start_time, complete_percent, flow_or_task) {
  var tmplt = `
    <div class="col-md-12 col-sm-12 col-12">
      <div class="info-box bg-dark" id="${uuid}">
        <span class="info-box-icon"><i class="fas fa-tasks"></i></span>

        <div class="info-box-content">
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
              <span class="info-box-text">${capitalize(flow_or_task)} name: </span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <span class="info-box-text ${flow_or_task}-name">${task_name} </span>
            </div>
          </div>
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
              <span class="info-box-text">Agent: </span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <span class="info-box-text ${flow_or_task}-agent">${robot_id} </span>
            </div>
          </div>
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
              <span class="info-box-text">Status: </span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <marquee direction="left" scrollamount="2" behavior="scroll" id="" class="${flow_or_task}-state">${state}</marquee>
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
          <div class="row">
            <div class="col-md-3 col-sm-3 col-3">
            <span class="info-box-number ${flow_or_task}-complete" id="">${complete_percent}%</span>
            </div>
            <div class="col-md-9 col-sm-9 col-9">
              <div class="progress">
                <div class="progress-bar ${flow_or_task}-complete-bar" style="width: ${complete_percent}%"></div>
              </div>
            </div>
          </div>
        </div>
        <!-- /.info-box-content -->
        <span class="info-box-icon"><i class="fas fa-trash"></i></span>
      </div>
      <!-- /.info-box -->
    </div>
    `;

  return tmplt
}

function update_info(flow_or_task) {
  var n_q_l = 0;
  var n_a_l = 0;

  $('.info-box').find(`.${flow_or_task}-state`).each(function () {
    switch ($(this).text()) {
      case "QUEUED":
        n_q_l += 1;
        break;
      case "ACTIVE":
        n_a_l += 1;
        break;

      default:
        break;
    }
  })

  $('#q-l').text(`QUEUED: ${n_q_l}`);
  $('#a-l').text(`ACTIVE: ${n_a_l}`);
}
