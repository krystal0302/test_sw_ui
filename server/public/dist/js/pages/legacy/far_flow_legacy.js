/*
 * Author: Angela Kao
 * Date: 9 July 13
 * Description:
 **/

var id;
var conn;
var editor;
var dataToImport;

var fleet_cell_list;

var NodeToDrawFlow_str_list = {};

var fleet_cookie;
var flow_name_cookie;

var node_selecting_id;

var roleMappinglist = [];
var flowdatadict = {};

var flow_name = "";
var event_name = "";

var goal_cell_dict = {};

var start_time = "";
var end_time = "";
var repeat_time = "";

var time_send_times = 0;
var time_start_times = [];

var start_date = '';
var end_date = '';

var artifact_dict = {};

var sys_time = 0;
var istimePassed = false;
var delete_type = '';

var data_init_drawflow = {
  "drawflow": {
    "Home": {
      "data": {
        "2": {
          "id": 2,
          "name": "start",
          "data": {},
          "class": "start",
          "html": "\n      <div id=\"start\">\n        <div class=\"title-box\">\n        <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"16\" height=\"16\" fill=\"currentColor\" class=\"bi bi-play-circle\" viewBox=\"0 0 16 16\">\n          <path d=\"M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z\"/>\n          <path d=\"M6.271 5.055a.5.5 0 0 1 .52.038l3.5 2.5a.5.5 0 0 1 0 .814l-3.5 2.5A.5.5 0 0 1 6 10.5v-5a.5.5 0 0 1 .271-.445z\"/>\n        </svg>\n         Start</div>\n      </div>\n      ",
          "typenode": false,
          "inputs": {},
          "outputs": {
            "output_1": {
              "connections": []
            }
          },
          "pos_x": 59,
          "pos_y": 304
        },
        "3": {
          "id": 3,
          "name": "finish",
          "data": {},
          "class": "finish",
          "html": "\n        <div id=\"finish\">\n          <div class=\"title-box\">\n          <svg xmlns=\"http://www.w3.org/2000/svg\" width=\"16\" height=\"16\" fill=\"currentColor\" class=\"bi bi-record-circle\" viewBox=\"0 0 16 16\">\n            <path d=\"M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z\"/>\n            <path d=\"M11 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0z\"/>\n          </svg>\n           Finish</div>\n        </div>\n        ",
          "typenode": false,
          "inputs": {
            "input_1": {
              "connections": []
            }
          },
          "outputs": {},
          "pos_x": 632,
          "pos_y": 300
        }
      }
    }
  }
};

$(function () {
  // console.log('CCCCCCCCCCCCCCCCCc')
  // console.log(rename_showing_label('cell_id'))

  // $.cookie('fleet', 'fleetA', {
  //   expires: 1,
  //   path: '/test.html'
  // });
});

// Set init func
function init_flow(_flow_name) {
  flow_name = _flow_name.replace(/\s/g, '_');
  fleet_cookie = getSavedFleet();
  flow_name_cookie = $.cookie('flow_name').replace(/\s/g, '_');

  get_mapping_data(fleet_cookie);
  get_fleet_data(`${fleet_cookie}.yaml`);
  load_flow();

  start_dict = {
    'name': 'start',
    'param': {},
    'relation': {
      'current_id': 2,
      'input': []
    }
  };
  finish_dict = {
    'name': 'finish',
    'param': {},
    'relation': {
      'current_id': 3,
      'input': []
    }
  };
  flowdatadict['node-2'] = start_dict;
  flowdatadict['node-3'] = finish_dict;
  console.log(flowdatadict)
}

// Step-1 load flow data
async function get_mapping_data(req_fleet) {
  if (req_fleet === undefined) {
    alert("No Fleet avalible")
  } else {
    var rolesMappingData = await restGetRolesMappingData(req_fleet);
    roleMappinglist = rolesMappingData;
    get_artifact(req_fleet);
  }
}

// Step-2 Get fleet data
async function get_fleet_data(_fleet) {
  $('#Task').empty();

  var fleetName = _fleet.split('.').slice(0, -1).join('.');
  var data = await restGetFleetSettings(fleetName)
  var data_dict = JSON.parse(data);
  var maps;
  var roles;
  for (let [item_key, item_val] of Object.entries(data_dict)) {
    maps = item_val.maps;
    roles = item_val.roles;
  }
  // create map task cell
  maps.forEach(function (maps_item, maps_index) {
    flow_get_map_cell(`${maps_item}.json`);
  });

  roles.forEach(function (roles_item, roles_index) {
    add_roles_cell(roles_item);
    var role_dict = {};
    var role_str_list = [];
    get_roles_info(`${roles_item}`, role_dict, role_str_list);
  });

}

// Task cell realate
// For map
async function flow_get_map_cell(_map) {
  var cellsData = await restGetMapCells(_map);
  fleet_cell_list = JSON.parse(cellsData);
  for (let [item_key, item_val] of Object.entries(fleet_cell_list)) {
    var map_item_name = item_key.replaceAll('-', '');
    var target_select = '';
    var content_dict = {};
    item_val.forEach(function (fleet_map_item, fleet_map_index) {
      if (fleet_map_index == 0) {
        target_select = fleet_map_item.cell_id;
      }
      content_dict[fleet_map_item.cell_id] = fleet_map_item.cell_id;
    });

    roleMappinglist.push({
      "role_name": map_item_name,
      "title_name": 'map@' + _map.replace('.json', ''),
      "title_content": content_dict
    });
  }
}

// For role
function add_roles_cell(_role) {
  var task_cell = `
    <div class="drag-drawflow" draggable="true" ondragstart="drag(event)" data-node="${_role}">
      <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-bookmark-fill" viewBox="0 0 16 16">
        <path d="M2 2v13.5a.5.5 0 0 0 .74.439L8 13.069l5.26 2.87A.5.5 0 0 0 14 15.5V2a2 2 0 0 0-2-2H4a2 2 0 0 0-2 2z"/>
      </svg>
      <span> ${_role}</span>
    </div>
  `;
  $('#Task').append(task_cell);
}

async function get_roles_info(_roles, _dict_value, _taget_str_list) {
  // var data = await restGetRoleBT(_roles);
  // var data_dict = JSON.parse(data);
  // data_dict.forEach(function (roles_seq_item, roles_seq_index) {
    generate_NodeToDrawFlow_str(_roles, _dict_value, _taget_str_list.join(''));
  // });

  // $.get(`/role_BT/${_roles}`, function (data, status) {
  //   if (status === "success") {
  //     var data_dict = JSON.parse(data);
  //     data_dict.forEach(function (roles_seq_item, roles_seq_index) {
  //       generate_NodeToDrawFlow_str(_roles, _dict_value, _taget_str_list.join(''));
  //     });
  //   }
  // }).done(function () {
  //   return true;
  // });
}

// Save
function save_ui_flow_setting(_fleetname, _flowname, _data) {
  restPostFlowUIData(_data, _fleetname, _flowname);
  toast(`${_flowname} panel is saved`);
}

function save_planner_flow_setting(_fleetname, _flowname, _data) {
  restPostFlowData(_data, _fleetname, _flowname);
  toast(`${_flowname} planner is saved`);
}

function save() {
  // console.log(editor.export());
  var sys_json = editor.export().drawflow.Home.data;
  var steps = [];
  for (let [item_key, item_val] of Object.entries(sys_json)) {
    var task_id = item_val.id;
    var task_name = item_val.name;
    var task_data = item_val.data;
    var task_ralation_input = [];
    var task_ralation_output = [];

    if (task_name == 'start' || task_name == 'finish') {
      // No need
      switch (task_name) {
        case 'start':
          var output = item_val.outputs.output_1.connections;
          output.forEach(function (output_item, output_index) {
            task_ralation_output.push(output_item.node)
          });
          break;
        case 'finish':
          var input = item_val.inputs.input_1.connections;
          input.forEach(function (input_item, input_index) {
            task_ralation_input.push(input_item.node)
          });
          break;
      }
    } else {
      var input = item_val.inputs.input_1.connections;
      input.forEach(function (input_item, input_index) {
        task_ralation_input.push(input_item.node)
      });
      var output = item_val.outputs.output_1.connections;
      output.forEach(function (output_item, output_index) {
        task_ralation_output.push(output_item.node)
      });
    }

    steps.push({
      'name': task_name,
      'param': task_data,
      'relation': {
        "current_id": task_id,
        "input": task_ralation_input,
        "input": task_ralation_output
      }
    });
    flowdatadict["node-" + task_id].relation.input = task_ralation_input;
  }

  var flow_data_list = [];
  for (let [item_key, item_val] of Object.entries(flowdatadict)) {
    var dict_tmp = {};
    dict_tmp[item_key] = item_val;
    flow_data_list.push(dict_tmp);
  }

  // update flow data when control sidebar still open
  if ($('.drawflow-node.selected').length > 0) {
    var target_node = "node-" + node_selecting_id;
    updateDatadict(target_node);
  }
  // console.log(flow_data_list);

  var save_data = {
    "flow_name": flow_name,
    "event_name": event_name,
    "flow_data": flow_data_list
  }

  var trigger_name = $("#flow_select option:selected").text();
  var flow_filename = `${getSavedFleet()}-flow-${flow_name}`;
  var nameExists = true;
  restOperationFileExistence("flow", flow_name).done(function (data) {
    nameExists = data.fileExists && data.filename !== flow_filename;

    if (trigger_name.trim().length == 0) {
      alert("Please select a task trigger");
    } else if (nameExists) {
      alert(`${flow_name} already exists!`);
    } else {
      // console.log(editor.export());
      // console.log(JSON.stringify(save_data));
      // save to ui 
      save_ui_flow_setting(getSavedFleet(), flow_name, JSON.stringify(editor.export()));
      // save to planner
      var event = {};

      if (trigger_name.includes("Timer")) {
        event["event"] = "Timer";
        event["start_time"] = start_date + "/" + start_time;
        if (end_time === "") {
          event["end_time"] = "";
        } else {
          event["end_time"] = end_date + "/" + end_time;
        }
        event["repeat_time"] = repeat_time;
      } else {
        event["event"] = trigger_name;
      }
      save_data.event_name = event;
      save_planner_flow_setting(getSavedFleet(), flow_name, save_data);

      // pub time trigger
      if (trigger_name.includes("Timer")) {
        // time_start_times.forEach(function (start_time_item, start_time_index) {
        //   console.log('=============== ROS Timer Trigger Pub ===============')
        //   console.log(new Date(start_time_item * 1000))
        //   send_flow_timer(flow_name, start_time_item);
        // });
      }
    };

    // add manual trigger list
    var mode = (isflow ? 2 : 1);
    if ($('#flow_select').val() === "manual" && mode === 2) {
      createTaskTriggerItem(getSavedFleet(), (isflow ? currentFlowName : currentTaskName), mode);
      chkTriggerListVisible();
      checkDisableManual(mode);
    }
  });
}

// Delete
function delete_flow() {
  fleet_cookie = getSavedFleet();
  flow_name_cookie = $.cookie('flow_name').replace(/\s/g, '_');

  restDeleteFlowData(fleet_cookie, flow_name_cookie);
  toast(`${flow_name_cookie} is deleted`);
  editor.clearModuleSelected();
}

// Load
async function load_flow() {
  var data = await restGetFlowData(flow_name_cookie, `${fleet_cookie}-flowui-${flow_name_cookie}`)
    .fail(function (data, status) {
      editor.import(data_init_drawflow);
      adjust_layout();
    });
  // If fail will return
  dataToImport = JSON.parse(data);
  editor.import(dataToImport);
  adjust_layout();
  load_flow_data()
}

async function load_flow_data() {
  var data = await restGetFlowData(flow_name_cookie, `${fleet_cookie}-flow-${flow_name_cookie}`);
  dataToImport = JSON.parse(data);
  var data_list = dataToImport.flow_data;
  var data_dict = {};
  data_list.forEach(function (node_item, node_index) {
    for (let [item_key, item_val] of Object.entries(node_item)) {
      data_dict[item_key] = item_val;
    };
  });

  // Checking if there are role update
  for (let [item_key, item_val] of Object.entries(data_dict)) {
    if (item_val.name != 'start' && item_val.name != 'finish' && item_val.name != 'logical_and' && item_val.name != 'logical_or' && item_val.name != 'logical_delay') {
      tmp_data_dict = data_dict
      check_role_dict = {}
      roleMappinglist.forEach(function (role_item, role_index) {
        if (role_item.role_name == item_val.name) {
          for (let [title_content_key, title_content_val] of Object.entries(role_item.title_content)) {
            check_role_dict[title_content_key] = title_content_val
          }
        }
      });

      if (item_val.hasOwnProperty("param")) {
        // remove not exist key
        for (let [param_key, param_val] of Object.entries(item_val.param)) {
          if (check_role_dict.hasOwnProperty(param_key)) {

          } else {
            delete tmp_data_dict[item_key].param[param_key]
          }
        }
      }

      // Add missing param
      for (let [title_content_key, title_content_val] of Object.entries(check_role_dict)) {
        if (!tmp_data_dict[item_key].hasOwnProperty("param")) {
          continue;
        }
        if (tmp_data_dict[item_key].param.hasOwnProperty(title_content_key)) {
          tmp_data_dict[item_key].param[title_content_key] = tmp_data_dict[item_key].param[title_content_key]
        } else {
          tmp_data_dict[item_key].param[title_content_key] = title_content_val
        }
      }
      data_dict = tmp_data_dict
    }
  }

  flowdatadict = data_dict;

  // check time is ok timer only
  if (dataToImport.event_name === 'Timer') {
    var t = dataToImport.event_name.start_time.split('/');
    var t_check = new Date(parseInt(t[0]), parseInt(t[1]) - 1, parseInt(t[2]), parseInt(t[3].split(':')[0]), parseInt(t[3].split(':')[1]));
    if (sys_time > t_check.getTime()) {
      istimePassed = true;
    } else {
      istimePassed = false;
    }
  }
}

// Util function
function set_flow_event(_flow_event) {
  event_name = _flow_event;
}

function switch_fleet(fleet) {
  $('#Task').children().slice(2).remove()
  get_fleet_data(`${fleet}.yaml`);
  editor.clearModuleSelected();
}

function flow_uuid() {
  var d = Date.now();
  if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
    d += performance.now(); //use high-precision timer if available
  }
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
    var r = (d + Math.random() * 16) % 16 | 0;
    d = Math.floor(d / 16);
    return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
  });
}

function adjust_layout() {
  var node_update = [];

  if ($('#start').length != 0) {
    $('#start').parent().parent().css("width", "100px");
    editor.updateConnectionNodes("node-2");
  }

  if ($('#finish').length != 0) {
    $('#finish').parent().parent().css("width", "100px");
    editor.updateConnectionNodes("node-3");
  }

  if ($('.logical_and').length != 0) {
    $('.logical_and').each(function (index) {
      node_update.push($(this).attr('id'));
      var id = $(this).attr('id').split('-')[1];
      $(this).find('div#logical_and').attr("id", `logical_and-${id}`);
      $(this).css("width", "100px");
    });
  }

  if ($('.logical_or').length != 0) {
    $('.logical_or').each(function (index) {
      node_update.push($(this).attr('id'));
      var id = $(this).attr('id').split('-')[1];
      $(this).find('div#logical_or').attr("id", `logical_or-${id}`);
      $(this).css("width", "100px");
    });
  }

  if ($('.logical_delay').length != 0) {
    $('.logical_delay').each(function (index) {
      node_update.push($(this).attr('id'));
      var id = $(this).attr('id').split('-')[1];
      $(this).find('div#logical_delay').attr("id", `logical_delay-${id}`);
      $(this).css("width", "100px");
    });
  }

  node_update.forEach(function (node_item, node_index) {
    editor.updateConnectionNodes(node_item);
  });
}

function capitalize(s) {
  return s[0].toUpperCase() + s.slice(1);
}

function rename_showing_label(txt) {
  return txt.replace(/_(\w{1,})/, '');
}

function get_system_time() {
  var time = "";
  $.ajax({
    url: `${window.location.hostname}:5000/settings/get_core_settings`,
    type: 'get',
    data: {},
    async: false,
    success: function (data) { },
    error: function (data) {
      console.log(data.getResponseHeader('Date'))
      var sysdate_milli = Date.parse(data.getResponseHeader('Date'));
      var current_date = new Date(sysdate_milli);
      current_date.setHours(0, 0, 0, 0);
      time = current_date.getTime();
      sys_time = sysdate_milli;
    }
  });
  return time;
}

// Get artifact
function get_artifact(fleet_name) {
  restGetFleetArtifacts(fleet_name).done(function (data) {
    arti_dict = JSON.parse(data);
    artifact_dict = arti_dict;
  });
}

// Ros function
function send_flow_timer(flow_name, _sec) {
  // var new_flow_msg = new ROSLIB.Message({
  //   event: '',
  //   event_type: '',
  //   params: 'ui',
  //   timer_scheduled: true,
  //   plan_time: {
  //     sec: _sec,
  //     nanosec: 0
  //   }
  // });

  // new_flow_msg.event = flow_uuid();
  // new_flow_msg.event_type = flow_name;
  // console.log(new_flow_msg);
  // flow_request_pub.publish(new_flow_msg);

  var jsonNewFlow = {
    event: flow_uuid(),
    event_type: flow_name,
    params: 'ui',
    timer_scheduled: true,
    plan_time: {
      sec: _sec,
      nanosec: 0
    }
  };
  console.log(jsonNewFlow);
  wsPubFlowReq(jsonNewFlow);
}

// Drawflow relate
function generate_NodeToDrawFlow_str(_role, _dict_value, _box_content) {
  var task_cell = `
    <div>
      <div class="title-box">
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-bookmark-fill" viewBox="0 0 16 16">
        <path d="M2 2v13.5a.5.5 0 0 0 .74.439L8 13.069l5.26 2.87A.5.5 0 0 0 14 15.5V2a2 2 0 0 0-2-2H4a2 2 0 0 0-2 2z"/>
        </svg>
        ${_role}
      </div>
      <div class="box">
        <input type="text" df-name="">
      </div>
    </div>
  `;

  NodeToDrawFlow_str_list[`${_role}`] = task_cell;
}

console.log('HI');
id = document.getElementById("drawflow");
editor = new Drawflow(id);

editor.reroute = true;
editor.reroute_fix_curvature = true;
editor.force_first_input = false;

editor.start();
// editor.import(dataToImport);

// Events!
editor.on('nodeCreated', function (id) {
  console.log("Node created " + id);
  var node_id = editor.getNodeFromId(id).id;
  var node_name = editor.getNodeFromId(id).name;

  if ((node_name == 'logical_and') || (node_name == 'logical_or') || (node_name == 'logical_delay')) {
    switch (node_name) {
      case 'logical_and':
        logical_and_dict = {
          'name': 'logical_and',
          'param': {},
          'relation': {
            'current_id': node_id,
            'input': []
          }
        };
        flowdatadict['node-' + node_id] = logical_and_dict;
        break;
      case 'logical_or':
        logical_or_dict = {
          'name': 'logical_or',
          'param': {},
          'relation': {
            'current_id': node_id,
            'input': []
          }
        };
        flowdatadict['node-' + node_id] = logical_or_dict;
        break;
      case 'logical_delay':
        logical_delay_dict = {
          'name': 'logical_delay',
          'param': {},
          'relation': {
            'current_id': node_id,
            'input': []
          }
        };
        flowdatadict['node-' + node_id] = logical_delay_dict;
        break;
      default:
        break;
    }
  } else {
      get_card_data(node_name, node_id);
  };
})

editor.on('nodeRemoved', function (id) {
  // console.log("Node removed " + id);
  // console.log(flowdatadict)

  delete flowdatadict['node-' + id];
  $("#control_sidebar").ControlSidebar('collapse');
  $('#card_content_div').empty();
  node_selecting_id = null;
})

editor.on('nodeSelected', function (id) {
  console.log("Node selected " + id);
  var node_id = editor.getNodeFromId(id).id;
  var node_name = editor.getNodeFromId(id).name;

  if ((node_name == 'start') || (node_name == 'finish') || (node_name == 'logical_and') || (node_name == 'logical_or') || (node_name == 'logical_delay')) {
    node_selecting_id = node_id;
  } else {
    node_selecting_id = node_id;
    $('#card_content_div').empty();
    get_card_data(node_name, node_id);
    $("#control_sidebar").ControlSidebar('show');
    hideSomeInfo();
  }

  var platform = navigator.platform;
  if (platform !== 'iPad') { return; }
  delete_type = 'node';
  $(".btn-delete").css('display', '').text('Delete Node');
})

editor.on('nodeUnselected', function (id) {
  console.log("Node unselected: " + id);
  var node_name = "";

  try {
    if (node_selecting_id != null) {
      node_name = editor.getNodeFromId(node_selecting_id).name;
      console.log(node_name)
      if ((node_name == 'start') || (node_name == 'finish') || (node_name == 'logical_and') || (node_name == 'logical_or') || (node_name == 'logical_delay')) {

      } else {
        var taget_node = "node-" + node_selecting_id;
        updateDatadict(taget_node);
        node_selecting_id = null;

        // $("#control_sidebar").ControlSidebar('collapse'); 
      };
    }

    var platform = navigator.platform;
    if (platform !== 'iPad') { return; }
    delete_type = '';
    $(".btn-delete").css('display', 'none');

  } catch (error) {
    console.error('=== try catch error ====');
    console.error(error);
  }
})

editor.on('moduleCreated', function (name) {
  console.log("Module Created " + name);
})

editor.on('moduleChanged', function (name) {
  console.log("Module Changed " + name);
})

editor.on('connectionCreated', function (connection) {
  console.log('Connection created');
  // console.log(connection);
})

editor.on('connectionRemoved', function (connection) {
  console.log('Connection removed');
  console.log(connection);
})

editor.on('connectionSelected', function (connection) {
  console.log('Connection selected');
  var platform = navigator.platform;
  if (platform !== 'iPad') { return; }
  conn = connection;
  delete_type = 'conn';
  $(".btn-delete").css('display', '').text('Delete Connection');
});

editor.on('connectionUnselected', function (connection) {
  console.log('Connection unselected');
  var platform = navigator.platform;
  if (platform !== 'iPad') { return; }
  conn = connection;
  delete_type = '';
  $(".btn-delete").css('display', 'none');
});

editor.on('mouseMove', function (position) {
  // console.log('Position mouse x:' + position.x + ' y:' + position.y);
})

editor.on('nodeMoved', function (id) {
  console.log("Node moved " + id);
})

editor.on('zoom', function (zoom) {
  console.log('Zoom level ' + zoom);
})

editor.on('translate', function (position) {
  // console.log('Translate x:' + position.x + ' y:' + position.y);
})

editor.on('addReroute', function (id) {
  console.log("Reroute added " + id);
})

editor.on('removeReroute', function (id) {
  console.log("Reroute removed " + id);
})

/* DRAG EVENT */

/* Mouse and Touch Actions */

var elements = document.getElementsByClassName('drag-drawflow');
for (var i = 0; i < elements.length; i++) {
  elements[i].addEventListener('touchend', drop, false);
  elements[i].addEventListener('touchmove', positionMobile, false);
  elements[i].addEventListener('touchstart', drag, false);
}

var mobile_item_selec = '';
var mobile_last_move = null;

function positionMobile(ev) {
  mobile_last_move = ev;
}

function allowDrop(ev) {
  ev.preventDefault();
}

function drag(ev) {
  if (ev.type === "touchstart") {
    mobile_item_selec = ev.target.closest(".drag-drawflow").getAttribute('data-node');
  } else {
    ev.dataTransfer.setData("node", ev.target.getAttribute('data-node'));
  }
}

function drop(ev) {
  if (ev.type === "touchend") {
    var parentdrawflow = document.elementFromPoint(mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY).closest("#drawflow");
    if (parentdrawflow != null) {
      addNodeToDrawFlow(mobile_item_selec, mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY);
    }
    mobile_item_selec = '';
  } else {
    ev.preventDefault();
    var data = ev.dataTransfer.getData("node");
    addNodeToDrawFlow(data, ev.clientX, ev.clientY);
  }

}

function addNodeToDrawFlow(name, pos_x, pos_y) {
  if (editor.editor_mode === 'fixed') {
    return false;
  }
  pos_x = pos_x * (editor.precanvas.clientWidth / (editor.precanvas.clientWidth * editor.zoom)) - (editor.precanvas.getBoundingClientRect().x * (editor.precanvas.clientWidth / (editor.precanvas.clientWidth * editor.zoom)));
  pos_y = pos_y * (editor.precanvas.clientHeight / (editor.precanvas.clientHeight * editor.zoom)) - (editor.precanvas.getBoundingClientRect().y * (editor.precanvas.clientHeight / (editor.precanvas.clientHeight * editor.zoom)));

  if (NodeToDrawFlow_str_list.hasOwnProperty(name)) {
    editor.addNode(`${name}`, 1, 1, pos_x, pos_y, `${name}`, {}, NodeToDrawFlow_str_list[name]);
  }

  switch (name) {
    case 'start':
      var start = `
      <div id="start">
        <div class="title-box" data-widget="control-sidebar">
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-play-circle" viewBox="0 0 16 16">
          <path d="M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z"/>
          <path d="M6.271 5.055a.5.5 0 0 1 .52.038l3.5 2.5a.5.5 0 0 1 0 .814l-3.5 2.5A.5.5 0 0 1 6 10.5v-5a.5.5 0 0 1 .271-.445z"/>
        </svg>
         Start</div>
      </div>
      `;
      if ($('#start').length == 0) {
        editor.addNode('start', 0, 1, pos_x, pos_y, 'start', {}, start);
      } else {
        // Start only one
      }
      break;
    case 'finish':
      var finish = `
        <div id="finish">
          <div class="title-box" data-widget="control-sidebar">
          <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-record-circle" viewBox="0 0 16 16">
            <path d="M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z"/>
            <path d="M11 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0z"/>
          </svg>
           Finish</div>
        </div>
        `;
      if ($('#finish').length == 0) {
        editor.addNode('finish', 1, 0, pos_x, pos_y, 'finish', {}, finish);
        $('#finish').parent().parent().css("width", "100px");
      } else {
        // Finish only one
      }
      break;
    case 'logical_and':
      var logical_and = `
          <div id="logical_and">
            <div class="title-box">
            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-hand-thumbs-up" viewBox="0 0 16 16">
              <path d="M8.864.046C7.908-.193 7.02.53 6.956 1.466c-.072 1.051-.23 2.016-.428 2.59-.125.36-.479 1.013-1.04 1.639-.557.623-1.282 1.178-2.131 1.41C2.685 7.288 2 7.87 2 8.72v4.001c0 .845.682 1.464 1.448 1.545 1.07.114 1.564.415 2.068.723l.048.03c.272.165.578.348.97.484.397.136.861.217 1.466.217h3.5c.937 0 1.599-.477 1.934-1.064a1.86 1.86 0 0 0 .254-.912c0-.152-.023-.312-.077-.464.201-.263.38-.578.488-.901.11-.33.172-.762.004-1.149.069-.13.12-.269.159-.403.077-.27.113-.568.113-.857 0-.288-.036-.585-.113-.856a2.144 2.144 0 0 0-.138-.362 1.9 1.9 0 0 0 .234-1.734c-.206-.592-.682-1.1-1.2-1.272-.847-.282-1.803-.276-2.516-.211a9.84 9.84 0 0 0-.443.05 9.365 9.365 0 0 0-.062-4.509A1.38 1.38 0 0 0 9.125.111L8.864.046zM11.5 14.721H8c-.51 0-.863-.069-1.14-.164-.281-.097-.506-.228-.776-.393l-.04-.024c-.555-.339-1.198-.731-2.49-.868-.333-.036-.554-.29-.554-.55V8.72c0-.254.226-.543.62-.65 1.095-.3 1.977-.996 2.614-1.708.635-.71 1.064-1.475 1.238-1.978.243-.7.407-1.768.482-2.85.025-.362.36-.594.667-.518l.262.066c.16.04.258.143.288.255a8.34 8.34 0 0 1-.145 4.725.5.5 0 0 0 .595.644l.003-.001.014-.003.058-.014a8.908 8.908 0 0 1 1.036-.157c.663-.06 1.457-.054 2.11.164.175.058.45.3.57.65.107.308.087.67-.266 1.022l-.353.353.353.354c.043.043.105.141.154.315.048.167.075.37.075.581 0 .212-.027.414-.075.582-.05.174-.111.272-.154.315l-.353.353.353.354c.047.047.109.177.005.488a2.224 2.224 0 0 1-.505.805l-.353.353.353.354c.006.005.041.05.041.17a.866.866 0 0 1-.121.416c-.165.288-.503.56-1.066.56z"/>
            </svg>
             And</div>
          </div>
          `;
      var node_num = editor.addNode('logical_and', 1, 1, pos_x, pos_y, 'logical_and', {}, logical_and);
      $('#logical_and').parent().parent().css("width", "100px");
      $('#logical_and').attr("id", `logical_and-${node_num}`);
      break;
    case 'logical_or':
      var logical_or = `
            <div id="logical_or">
              <div class="title-box">
              <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-sunglasses" viewBox="0 0 16 16">
                <path d="M3 5a2 2 0 0 0-2 2v.5H.5a.5.5 0 0 0 0 1H1V9a2 2 0 0 0 2 2h1a3 3 0 0 0 3-3 1 1 0 1 1 2 0 3 3 0 0 0 3 3h1a2 2 0 0 0 2-2v-.5h.5a.5.5 0 0 0 0-1H15V7a2 2 0 0 0-2-2h-2a2 2 0 0 0-1.888 1.338A1.99 1.99 0 0 0 8 6a1.99 1.99 0 0 0-1.112.338A2 2 0 0 0 5 5H3zm0 1h.941c.264 0 .348.356.112.474l-.457.228a2 2 0 0 0-.894.894l-.228.457C2.356 8.289 2 8.205 2 7.94V7a1 1 0 0 1 1-1z"/>
              </svg>
               Or</div>
            </div>
            `;
      var node_num = editor.addNode('logical_or', 1, 1, pos_x, pos_y, 'logical_or', {}, logical_or);
      $('#logical_or').parent().parent().css("width", "100px");
      $('#logical_or').attr("id", `logical_or-${node_num}`);
      break;
    case 'logical_delay':
      var logical_delay = `
              <div id="logical_delay">
                <div class="title-box">
                <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-stopwatch" viewBox="0 0 16 16">
                  <path d="M8.5 5.6a.5.5 0 1 0-1 0v2.9h-3a.5.5 0 0 0 0 1H8a.5.5 0 0 0 .5-.5V5.6z"/>
                  <path d="M6.5 1A.5.5 0 0 1 7 .5h2a.5.5 0 0 1 0 1v.57c1.36.196 2.594.78 3.584 1.64a.715.715 0 0 1 .012-.013l.354-.354-.354-.353a.5.5 0 0 1 .707-.708l1.414 1.415a.5.5 0 1 1-.707.707l-.353-.354-.354.354a.512.512 0 0 1-.013.012A7 7 0 1 1 7 2.071V1.5a.5.5 0 0 1-.5-.5zM8 3a6 6 0 1 0 .001 12A6 6 0 0 0 8 3z"/>
                </svg>
                Delay</div>
              </div>
              `;
      var node_num = editor.addNode('logical_delay', 1, 1, pos_x, pos_y, 'logical_delay', {}, logical_delay);
      $('#logical_delay').parent().parent().css("width", "100px");
      $('#logical_delay').attr("id", `logical_delay-${node_num}`);
      break;
    default:
  }
}

var transform = '';

function showpopup(e) {
  e.target.closest(".drawflow-node").style.zIndex = "9999";
  e.target.children[0].style.display = "block";
  //document.getElementById("modalfix").style.display = "block";

  //e.target.children[0].style.transform = 'translate('+translate.x+'px, '+translate.y+'px)';
  transform = editor.precanvas.style.transform;
  editor.precanvas.style.transform = '';
  editor.precanvas.style.left = editor.canvas_x + 'px';
  editor.precanvas.style.top = editor.canvas_y + 'px';
  console.log(transform);

  //e.target.children[0].style.top  =  -editor.canvas_y - editor.container.offsetTop +'px';
  //e.target.children[0].style.left  =  -editor.canvas_x  - editor.container.offsetLeft +'px';
  editor.editor_mode = "fixed";

}

function closemodal(e) {
  e.target.closest(".drawflow-node").style.zIndex = "2";
  e.target.parentElement.parentElement.style.display = "none";
  //document.getElementById("modalfix").style.display = "none";
  editor.precanvas.style.transform = transform;
  editor.precanvas.style.left = '0px';
  editor.precanvas.style.top = '0px';
  editor.editor_mode = "edit";
}

function changeModule(event) {
  var all = document.querySelectorAll(".menu ul li");
  for (var i = 0; i < all.length; i++) {
    all[i].classList.remove('selected');
  }
  event.target.classList.add('selected');
}

function changeMode(option) {
  //console.log(lock.id);
  if (option == 'lock') {
    lock.style.display = 'none';
    unlock.style.display = 'block';
  } else {
    lock.style.display = 'block';
    unlock.style.display = 'none';
  }
}

// Template
function card_template(target_node, card_title, card_subtitle, card_content) {
  var card = `
  <div class="row card-deck" style="border-radius: 15px; ">
    <div class="card col-lg-12 p-3 nav2dmapCanvas farobot-view-bg" data-widget="" style="margin:1rem;">
      <div class="card-header">
        <h4 class="card-title">${transTitleName(card_title, card_subtitle)}</h4>
      </div>
      <div class="card-body">
        ${card_content}
      </div>
    </div>
  </div>
  `;

  // append parameter card only when no node is selected
  if ($('.drawflow-node.selected').length == 0) {
    $('#card_content_div').append(card);
  }
}

function generate_area_select_list_str(map_val) {
  var map_select = '';

  Object.keys(goal_cell_dict[map_val]).forEach(function (map_item, map_index) {
    if (map_index == 0) {
      map_select += `<option value="${map_item}" selected>${capitalize(map_item)}</option>`;
    } else {
      map_select += `<option value="${map_item}">${capitalize(map_item)}</option>`;
    }
  });

  var select_str = `
            <select class="cus-select input-item area" style="margin-top: 10px;">
              ${map_select}
            </select>
            `;

  return select_str;
}

function generate_cell_select_list_str(map_val, area_val) {
  var cell_select = '';

  goal_cell_dict[map_val][area_val].forEach(function (cell_item, cell_index) {
    if (cell_index == 0) {
      cell_select += `<option value="${cell_item}" selected>${cell_item}</option>`;
    } else {
      cell_select += `<option value="${cell_item}">${cell_item}</option>`;
    }
  });

  var select_str = `
            <select class="cus-select input-item cell" style="margin-top: 10px;">
              ${cell_select}
            </select>
            `;
  return select_str;
}

// HTML Relate -- Event
$(document).on("change", ".map", function () {
  var area_select = '';
  var cell_select = '';

  area_select = generate_area_select_list_str($(this).val());

  if ($(this).parent().find('select.area').length == 0) {
    $(this).parent().append(area_select);
    area_select = '';
  } else {
    $(this).parent().children().remove("select.area");
    $(this).parent().children().remove("select.cell");
    $(this).parent().append(area_select);
    area_select = '';
  };

  if ($(this).parent().find('select.cell').length == 0) {
    cell_select = generate_cell_select_list_str($(this).val(), $(this).parent().find('select.area').val());
    $(this).parent().append(cell_select);
    cell_select = '';
  }

});

$(document).on("change", ".area", function () {
  var cell_select = '';

  cell_select = generate_cell_select_list_str($(this).parent().find('select.map').val(), $(this).val());

  if ($(this).parent().find('select.cell').length == 0) {
    $(this).parent().append(cell_select);
    cell_select = '';
  } else {
    $(this).parent().children().remove("select.cell");
    $(this).parent().append(cell_select);
    cell_select = '';
  };
});

$(document).on("click", ".drawflow", function () {
  if ($('.drawflow-node.selected').length == 0) {
    $("#control_sidebar").ControlSidebar('collapse');
  }
});

// drawflow button event
function init_panel() {
  editor.import(data_init_drawflow);
  adjust_layout();
}

function deleteSelectedItem() {
  $(".btn-delete").css('display', 'none');
  switch (delete_type) {
    case 'conn':
      if (typeof conn === 'undefined') { return; }
      // removeSingleConnection(id_output, id_input, output_class, input_class)
      editor.removeSingleConnection(conn.output_id, conn.input_id, conn.output_class, conn.input_class);
      break
    case 'node':
      if (node_selecting_id === null) { return; }
      editor.removeConnectionNodeId('node-'+node_selecting_id);
      var moduleName = editor.getModuleFromNodeId(node_selecting_id);
      if (editor.module === moduleName) {
        document.getElementById('node-'+node_selecting_id).remove();
      }
      delete editor.drawflow.drawflow[moduleName].data[node_selecting_id];
      editor.dispatch('nodeRemoved', node_selecting_id);
      break;
    default:
      break
  }
}

// HTML Relate -- UI
function get_card_data(role_name, node_id) {
  if (roleMappinglist.length != 0) {
    gerate_cell_mapping_dict();

    var title_name = "";
    var type_name = "";
    var card_body_str = "";
    var res = false;

    // for data info    
    var data_dict_id = node_id;
    var data_dict_id_tag = `node-${data_dict_id}`;
    var data_dict_param = {}

    // new created card
    if (!flowdatadict.hasOwnProperty(data_dict_id_tag)) {
      flowdatadict[data_dict_id_tag] = {
        "name": '',
        "param": {},
        "relation": {
          "current_id": data_dict_id,
          "input": []
        }
      };

      roleMappinglist.forEach(function (mapping_item, mapping_index) {
        if (mapping_item.role_name == role_name) {
          title_name = mapping_item.title_name;
          type_name = mapping_item.type;

          if (!$.isEmptyObject(mapping_item.title_content)) {
            if (mapping_item.title_content !== "N/A") {
              for (let [item_key, item_val] of Object.entries(mapping_item.title_content)) {
                var para_name = item_key;
                var para_default = item_val;
                var mapping_string = '';

                if (item_key.includes("status_")) {
                  mapping_string = generate_select_list('status', '');
                } else if (item_key.includes("cell_id_")) {
                  mapping_string = generate_select_list('cellid', para_default);
                } else if (item_key.includes("goal_")) {
                  mapping_string = generate_select_list('goal', para_default);
                } else if (item_key.includes("artifact_")) {
                  mapping_string = generate_select_list('artifact', para_default, mapping_item.type);
                } else {
                  mapping_string = generate_select_list('', para_default);
                }

                if (item_key.includes("recognition_method_")) {
                  card_body_str += `<div class="row" style="padding:0.0rem;margin-top: 10px; display: none;">`
                } else {
                  card_body_str += `<div class="row" style="padding:0.0rem;margin-top: 10px;">`;
                }
                card_body_str += `
                  <div class="col-4">
                    <div type="text" class="input-label" style="text-align: center;font-size: 1.0rem;" name="${para_name}>${rename_showing_label(para_name)} :</div>
                  </div>
                  <div class="col-8">
                    ${mapping_string}
                  </div>
                </div>
              `;

                data_dict_param[para_name] = para_default;
              }
            } else {
              card_body_str += `<div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>`;
            }
            card_template('t', title_name, mapping_item.type, card_body_str);
            card_body_str = "";
            res = true;
            flowdatadict[data_dict_id_tag]["name"] = role_name;
            flowdatadict[data_dict_id_tag]["param"] = data_dict_param;
          } else {
            title_name = mapping_item.title_name;
            res = false
            flowdatadict[data_dict_id_tag]["name"] = role_name;
            flowdatadict[data_dict_id_tag]["param"] = {};
          }
        } else {
          return false
        }
      });

      if (!res) {
        card_body_str = `
              <div class="row" style="padding:0.0rem;margin-top: 10px;">
                <div class="col-12">
                  <div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>
                </div>
              </div>
            `
        card_template('t', title_name, type_name, card_body_str);
        card_body_str = "";
      }
    } else {
      var target_flow_data_dict = flowdatadict[data_dict_id_tag];

      roleMappinglist.forEach(function (mapping_item, mapping_index) {
        if (mapping_item.role_name == role_name) {
          title_name = mapping_item.title_name;
          type_name = mapping_item.type;

          if (!$.isEmptyObject(mapping_item.title_content) && !$.isEmptyObject(target_flow_data_dict.param)) {
            if (mapping_item.title_content !== "N/A") {
              for (let [item_key, item_val] of Object.entries(mapping_item.title_content)) {
                var para_name = item_key;
                var para_default = target_flow_data_dict.param[para_name];
                var mapping_string = '';

                if (item_key.includes("status_")) {
                  mapping_string = generate_select_list('status', para_default);
                } else if (item_key.includes("cell_id_")) {
                  mapping_string = generate_select_list('cellid', para_default);
                } else if (item_key.includes("goal_")) {
                  mapping_string = generate_select_list('goal', para_default);
                } else if (item_key.includes("artifact_")) {
                  mapping_string = generate_select_list('artifact', para_default, mapping_item.type);
                } else {
                  mapping_string = generate_select_list('', para_default);
                }

                if (item_key.includes("recognition_method_")) {
                  card_body_str += `<div class="row" style="padding:0.0rem;margin-top: 10px; display: none;">`
                } else {
                  card_body_str += `<div class="row" style="padding:0.0rem;margin-top: 10px;">`;
                }
                card_body_str += `
                  <div class="col-4">
                    <div type="text" class="input-label" style="text-align: center;font-size: 1.0rem;" name="${para_name}">${rename_showing_label(para_name)} :</div>
                  </div>
                  <div class="col-8">
                    ${mapping_string}
                  </div>
                </div>
              `;

                data_dict_param[para_name] = para_default;
              }
            } else {
              card_body_str += `<div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>`;
            }
            card_template('t', title_name, mapping_item.type, card_body_str);
            card_body_str = "";
            res = true;
          } else {
            title_name = mapping_item.title_name;
            res = false
          }

        } else {
          return false
        }
      });

      if (!res) {
        card_body_str = `
              <div class="row" style="padding:0.0rem;margin-top: 10px;">
                <div class="col-12">
                  <div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>
                </div>
              </div>
            `
        card_template('t', title_name, type_name, card_body_str);
        card_body_str = "";
      }
    }
  } else {
    alert("Fleet is empty");
  }
}

function transTitleName(title, subTitle) {
  var title_ui = dictUiTerms[title];
  if (typeof title_ui === 'undefined') {
    return title;
  } else if (title_ui === "Artifact" && typeof subTitle !== 'undefined') {
    return title_ui + ' - ' + subTitle;
  } else {
    return title_ui;
  }
}

function updateDatadict(node_id) {
  var dict_keys = $('.input-label').toArray().map(function (i) {
    return i.getAttribute("name").trim()
  });
  var dict_values = [];
  $(".input-item").each(function () {
    if ($(this).is("select")) {
      var select_data = '';

      if ($(this).hasClass("map")) {
        var map = $(this).find("option:selected").val();
        var area = $(this).parent().find(".area").find("option:selected").val();
        var cell = $(this).parent().find(".cell").find("option:selected").val();
        select_data = `${map}@${area}@${cell}`;
        dict_values.push(select_data);
      } else if ($(this).hasClass("area")) {
        // skip
      } else if ($(this).hasClass("cell")) {
        // skip
      } else {
        select_data = $(this).find("option:selected").val();
        dict_values.push(select_data);
      }
    } else {
      dict_values.push($(this).val());
    };
  });

  var flow_params = flowdatadict[node_id].param;
  var new_params = {}

  if (!$.isEmptyObject(flow_params)) {
    for (let [item_key, item_val] of Object.entries(flow_params)) {
      var t_idx = dict_keys.indexOf(item_key);
      var new_val = dict_values[t_idx];
      new_params[item_key] = dict_values[t_idx];
    };
  }

  flowdatadict[node_id].param = new_params;
  console.log('=======updateDatadict=======')
  console.log(flowdatadict)
}

function generate_select_list(name, default_value, artifact_type = null) {
  var generate_str = '';

  switch (name) {
    case "status":
      if (typeof default_value != 'undefined' && default_value) {
        switch (default_value) {
          case 'empty':
            generate_str = `
            <select class="cus-select input-item">
              <option value="occupied">Occupied</option>
              <option value="empty" selected>Empty</option>
            </select>
            `;
            break;
          case 'occupied':
            generate_str = `
            <select class="cus-select input-item">
              <option value="occupied" selected>Occupied</option>
              <option value="empty">Empty</option>
            </select>
            `;
            break;
          default:
            generate_str = `
            <select class="cus-select input-item">
              <option value="occupied" selected>Occupied</option>
              <option value="empty">Empty</option>
            </select>
            `;
            break;
        }
      } else {
        generate_str = `
        <select class="cus-select input-item">
          <option value="occupied" selected>Occupied</option>
          <option value="empty">Empty</option>
        </select>
      `;
      }
      break;

    case "goal":
      if (typeof default_value != 'undefined' && default_value) {
        if (default_value.includes('@')) {
          var map_option = '';
          var area_option = '';
          var cell_option = '';

          var map_name = default_value.split('@')[0];
          var area = default_value.split('@')[1];
          var cell = default_value.split('@')[2];


          for (let [map_item_key, map_item_val] of Object.entries(goal_cell_dict)) {
            // load map
            if (map_name == map_item_key) {
              map_option += `<option value="${map_item_key}" selected>${map_item_key}</option>`;

              for (let [area_item_key, area_item_val] of Object.entries(map_item_val)) {
                if (area == area_item_key) {
                  area_option += `<option value="${area_item_key}" selected>${capitalize(area_item_key)}</option>`;
                  area_item_val.forEach(function (cell_item, cell_index) {
                    if (cell == cell_item) {
                      cell_option += `<option value="${cell_item}" selected>${cell_item}</option>`;
                    } else {
                      cell_option += `<option value="${cell_item}">${cell_item}</option>`;
                    }
                  });
                } else {
                  area_option += `<option value="${area_item_key}">${capitalize(area_item_key)}</option>`;
                }
              };
            } else {
              map_option += `<option value="${map_item_key}">${map_item_key}</option>`;
            }
          };

          var area_select_str = `
            <select class="cus-select input-item area" style="margin-top: 10px;">
              ${area_option}
            </select>
            `;

          var cell_select_str = `
            <select class="cus-select input-item cell" style="margin-top: 10px;">
              ${cell_option}
            </select>
            `;

          generate_str = `
            <select class="cus-select input-item map">
              ${map_option}
            </select>
            ${area_select_str}
            ${cell_select_str}
            `;
        } else {
          // First time come in
          var str_option = '';
          var area_select_list = '';
          var cell_select_list = '';

          Object.keys(goal_cell_dict).forEach(function (content_item, content_index) {
            if (content_index == 0) {
              str_option += `<option value="${content_item}" selected>${content_item}</option>`;
              console.log(content_item)
              area_select_list = generate_area_select_list_str(content_item);
              cell_select_list = generate_cell_select_list_str(content_item, Object.keys(goal_cell_dict[content_item])[0]);
            } else {
              str_option += `<option value="${content_item}">${content_item}</option>`;
            }
          });

          generate_str = `
            <select class="cus-select input-item map">
              ${str_option}
            </select>
            ${area_select_list}
            ${cell_select_list}
            `;
        };
      } else {
        // First time come in
        var str_option = '';
        var area_select_list = '';
        var cell_select_list = '';

        Object.keys(goal_cell_dict).forEach(function (content_item, content_index) {
          if (content_index == 0) {
            str_option += `<option value="${content_item}" selected>${content_item}</option>`;
            console.log(content_item)
            area_select_list = generate_area_select_list_str(content_item);
            cell_select_list = generate_cell_select_list_str(content_item, Object.keys(goal_cell_dict[content_item])[0]);
          } else {
            str_option += `<option value="${content_item}">${content_item}</option>`;
          }
        });

        generate_str = `
          <select class="cus-select input-item map">
            ${str_option}
          </select>
          ${area_select_list}
          ${cell_select_list}
          `;
      }
      break;

    case "cellid":
      if (typeof default_value != 'undefined' && default_value) {
        if (default_value.includes('@')) {
          var map_option = '';
          var area_option = '';
          var cell_option = '';

          var map_name = default_value.split('@')[0];
          var area = default_value.split('@')[1];
          var cell = default_value.split('@')[2];


          for (let [map_item_key, map_item_val] of Object.entries(goal_cell_dict)) {
            // load map
            if (map_name == map_item_key) {
              map_option += `<option value="${map_item_key}" selected>${map_item_key}</option>`;
              for (let [area_item_key, area_item_val] of Object.entries(map_item_val)) {
                if (area == area_item_key) {
                  area_option += `<option value="${area_item_key}" selected>${capitalize(area_item_key)}</option>`;
                  area_item_val.forEach(function (cell_item, cell_index) {
                    if (cell == cell_item) {
                      cell_option += `<option value="${cell_item}" selected>${cell_item}</option>`;
                    } else {
                      cell_option += `<option value="${cell_item}">${cell_item}</option>`;
                    }
                  });
                } else {
                  area_option += `<option value="${area_item_key}">${capitalize(area_item_key)}</option>`;
                }
              };
            } else {
              map_option += `<option value="${map_item_key}">${map_item_key}</option>`;
            }
          };

          var area_select_str = `
            <select class="cus-select input-item area" style="margin-top: 10px;">
              ${area_option}
            </select>
            `;

          var cell_select_str = `
            <select class="cus-select input-item cell" style="margin-top: 10px;">
              ${cell_option}
            </select>
            `;

          generate_str = `
            <select class="cus-select input-item map">
              ${map_option}
            </select>
            ${area_select_str}
            ${cell_select_str}
            `;
        } else {
          // First time come in
          var str_option = '';
          var area_select_list = '';
          var cell_select_list = '';

          Object.keys(goal_cell_dict).forEach(function (content_item, content_index) {
            if (content_index == 0) {
              str_option += `<option value="${content_item}" selected>${content_item}</option>`;
              console.log(content_item)
              area_select_list = generate_area_select_list_str(content_item);
              cell_select_list = generate_cell_select_list_str(content_item, Object.keys(goal_cell_dict[content_item])[0]);
            } else {
              str_option += `<option value="${content_item}">${content_item}</option>`;
            }
          });

          generate_str = `
            <select class="cus-select input-item map">
              ${str_option}
            </select>
            ${area_select_list}
            ${cell_select_list}
            `;
        };
      } else {
        // First time come in
        var str_option = '';
        var area_select_list = '';
        var cell_select_list = '';

        Object.keys(goal_cell_dict).forEach(function (content_item, content_index) {
          if (content_index == 0) {
            str_option += `<option value="${content_item}" selected>${content_item}</option>`;
            console.log(content_item)
            area_select_list = generate_area_select_list_str(content_item);
            cell_select_list = generate_cell_select_list_str(content_item, Object.keys(goal_cell_dict[content_item])[0]);
          } else {
            str_option += `<option value="${content_item}">${content_item}</option>`;
          }
        });

        generate_str = `
          <select class="cus-select input-item map">
            ${str_option}
          </select>
          ${area_select_list}
          ${cell_select_list}
          `;
      }
      break;

    case "artifact":
      if (typeof default_value != 'undefined' && default_value && default_value != 'undefined') {
        var str_option = '';
        var cell_select_list = '';

        if (typeof artifact_dict[artifact_type] != 'undefined' && artifact_dict[artifact_type]) {
          console.log('111111111111111111')
          console.log(default_value)

          if (default_value == 'auto') {
            str_option += `<option value="auto" selected>auto</option>`;
          } else {
            str_option += `<option value="auto">auto</option>`;
          }

          artifact_dict[artifact_type].forEach(function (content_item, content_index) {
            // console.log(content_item)
            if (content_item == default_value) {
              str_option += `<option value="${content_item}" selected>${content_item}</option>`;
            } else {
              str_option += `<option value="${content_item}">${content_item}</option>`;
            }
          });
        } else {
          str_option += `<option value="auto">auto</option>`;
          // str_option += `<option value="null" selected>No artifact available</option>`;
        }

        generate_str = `
              <select class="cus-select input-item artifact">
                ${str_option}
              </select>
              ${cell_select_list}
              `;
      } else {
        // First time come in
        var str_option = '';
        var cell_select_list = '';

        if (typeof artifact_dict[artifact_type] != 'undefined' && artifact_dict[artifact_type]) {
          str_option += `<option value="auto" selected>auto</option>`;

          artifact_dict[artifact_type].forEach(function (content_item, content_index) {
            str_option += `<option value="${content_item}">${content_item}</option>`;
          });
        } else {
          str_option += `<option value="auto">auto</option>`;
          // str_option += `<option value="null" selected>No artifact available</option>`;
        }

        generate_str = `
              <select class="cus-select input-item artifact">
                ${str_option}
              </select>
              ${cell_select_list}
              `;
      }
      break;

    default:
      generate_str = `
      <div class="input-group">
        <input type="text" class="form-control input-item" value="${default_value}">
      </div>
      `;
      break;
  };
  return generate_str
}

function gerate_cell_mapping_dict() {
  roleMappinglist.forEach(function (cell_item, cell_index) {
    if (cell_item.title_name.includes('map@')) {
      var map_name = cell_item.title_name.split('@')[1];

      // Add map
      if (goal_cell_dict.hasOwnProperty(map_name)) {
        // Add area
        if (goal_cell_dict[map_name].hasOwnProperty(cell_item.role_name)) {
          // Add cell
          Object.values(cell_item.title_content).forEach(function (content_item, content_index) {
            if (goal_cell_dict[map_name][cell_item.role_name].includes(content_item)) {
              //already have
            } else {
              goal_cell_dict[map_name][cell_item.role_name].push(content_item);
            }
          });
        } else {
          goal_cell_dict[map_name][cell_item.role_name] = Object.values(cell_item.title_content);
        }
      } else {
        var area_dict = {};
        area_dict[cell_item.role_name] = Object.values(cell_item.title_content);
        goal_cell_dict[map_name] = area_dict;
      }
    }
  });

  // add auto to cell
  for (let [map_item_key, map_item_val] of Object.entries(goal_cell_dict)) {
    for (let [area_item_key, area_item_val] of Object.entries(map_item_val)) {
      if (!area_item_val.includes('auto_empty') && !area_item_val.includes('auto_occupied')) {
        area_item_val.unshift('auto_empty', 'auto_occupied')
      }
    };
  };

}

function get_time_trigger_event(start, end, duration, isrepeat, startdate, enddate) {
  time_start_times = [];
  start_time = start;
  end_time = end;
  repeat_time = duration;
  start_date = startdate;
  end_date = enddate;

  if (isrepeat) {
    var start_time_sec = start.split(':')[0] * 60 * 60 + start.split(':')[1] * 60;
    var end_time_sec = end.split(':')[0] * 60 * 60 + end.split(':')[1] * 60;
    var repeat_time_sec = duration * 60;

    start_time_sec = new Date(startdate).getTime() * 0.001 + start_time_sec;
    end_time_sec = new Date(enddate).getTime() * 0.001 + end_time_sec;

    var time_diff = (end_time_sec - start_time_sec) / repeat_time_sec;
    // console.log(time_diff)
    time_send_times = time_diff;
    for (var i = 0; i <= time_diff; i++) {
      var time_sec = start_time_sec + repeat_time_sec * i;
      time_start_times.push(time_sec);
    }
  } else {
    var start_time_sec = start.split(':')[0] * 60 * 60 + start.split(':')[1] * 60;
    time_send_times = 1;
    time_start_times.push(new Date(startdate).getTime() * 0.001 + start_time_sec);
  };
}

// For docking msg
function hideSomeInfo() {
  $('.nav2dmapCanvas').each(function (index) {
    if ($(this).find('h4').text() === 'Docking') {
      $(this).find('.row').each(function (index) {
        if ($(this).find('.input-label').text().includes("rotation") || $(this).find('.input-label').text().includes("size") || $(this).find('.input-label').text().includes("tag_id")) {
          $(this).css("display", "none");
        }
      });
    }
  });
}

// For Artifact (Not use right now)
// function artifact_get() {
//   fetch(`http://${window.location.hostname}:5000/login/access-token/`, {
//       method: "POST",
//       headers: {
//         accept: "application/json",
//         "Content-Type": "application/x-www-form-urlencoded",
//       },
//       body: "username=root&password=root@farobot",
//     })
//     .then(function (response) {
//       return response.json();
//     })
//     .then(function (myJson) {
//       var rmtType = myJson.token_type;
//       var rmtToken = myJson.access_token;
//       console.log(myJson);
//       $.ajax({
//         url: `http://${window.location.hostname}:5000/artifacts/scan`,
//         type: 'GET',
//         headers: {
//           accept: "application/json",
//           Authorization: `${rmtType} ${rmtToken}`,
//         },
//         async: false,
//         success: function (data) {
//           artifact_dict = data;
//         },
//         error: function (err_data) {
//           console.log("err: " + err_data);
//         },
//       });
//     })


// }

// run all task timer
$(document).on('click', '#r-t-flow', function (e) {
  var flow_name = $('#flow-home-tab').find('.card-header').text();

  if (istimePassed) {
    alert("Setting time passed, please modify it and try it again.");
  } else {
    time_start_times.forEach(function (start_time_item, start_time_index) {
      console.log('=============== ROS Timer Trigger Pub ===============')
      console.log(new Date(start_time_item * 1000));
      send_flow_timer(flow_name, start_time_item);
    });
  }
});

// remove all task timer
$(document).on('click', '#r-t-a-flow', function (e) {
  var flow_name = $('#flow-home-tab').find('.card-header').text();

  $('#flow-timer-sche').empty();
  // send cancel all
  send_remove("name", flow_name);
  isRemoveAllflowPressed = true;
  // console.log(flow_name)
  update_info("flow");
});

// Name mapping table
const dictUiTerms = {
  Sequence: "Sequence",
  Patrol: "Patrol",
  Docking: "Docking",
  Rotate: "Rotate",
  InitialPose: "InitialPose",
  ChangeMap: "MapSwitch",
  ChangeLocalization: "ChangeLocalization",
  WaitAction: "Wait",
  Repeat: "Repeat",
  Nav2Client: "Move",
  Artifact: "Artifact",
  ReportFail: "ReportFail",
  TriggerEvent: "TriggerEvent",
  Sync: "Sync",
  ReportWMS: "ReportWMS",
  true: "docking",
  false: "undocking"
};

const dictBtTerms = {
  Sequence: "Sequence",
  Patrol: "Patrol",
  Docking: "Docking",
  Rotate: "Rotate",
  InitialPose: "InitialPose",
  MapSwitch: "ChangeMap",
  ChangeLocalization: "ChangeLocalization",
  Wait: "WaitAction",
  Repeat: "Repeat",
  Move: "Nav2Client",
  Wait: "WaitAction",
  Artifact: "Artifact",
  ReportFail: "ReportFail",
  Sync: "Sync",
  ReportWMS: "ReportWMS",
  docking: true,
  undocking: false
};