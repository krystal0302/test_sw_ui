/*
 * Author: Angela Kao
 * Date: 9 July 13
 * Author: John Wu
 * Date: 18 May 2022
 * Description:
 **/

var conn;
var dataToImport;

var NodeToDrawFlow_str_list = {};

var fleet_cookie;
var flow_name_cookie;

var selectingNodeId_;
var deleteNodeId_;

var roleMappinglist = [];
var flowdatadict = {};

var flow_name = "";
var event_name = "";

var goal_cell_dict = {};
var mapAlias_ = "";

var start_time = "";
var end_time = "";
var repeat_time = "";

var time_send_times = 0;
var time_start_times = [];
var time_args = { 'start_time': undefined, 'end_time': undefined, 'interval': undefined };

var start_date = '';
var end_date = '';

var artifact_dict = {};

var sys_time = 0;
var isTimePassed = false;
var delete_type = '';

// --- sidebar trigger button ---
var bSidebarCollapse_ = true;
var sidebar_mode = 'op';

var flowUUID = "";
var assign_robot_flow_name = "";

let unSelectId = undefined;

let error_handle_list = ["error_handle_task_node", "error_type", "failure_timeout", "retry_limit"];

const data_init_drawflow = {
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

let non_plan_artifact_param_dict = {};
// --- Set init. func. ---
async function init_flow(_flow_name) {
    fetchArtifactsServiceTypesCb()
    flow_name = _flow_name.replace(/\s/g, '_');
    assign_robot_flow_name = flow_name;
    fleet_cookie = getSelectedFleet();
    flow_name_cookie = $.cookie('flow_name').replace(/\s/g, '_');

    // --- set map alias ---
    var data = await restGetAllMapData();
    var objArray = JSON.parse(data);
    mapAlias_ = objArray;

    await get_mapping_data(fleet_cookie);
    await get_fleet_data(`${fleet_cookie}.yaml`);
    console.log(roleMappinglist)

    initDataDict();

    console.log(flowdatadict)
    await load_flow().then(response => {
        console.log('---- load_flow success -----');
        console.log(response);
    }).catch(e => {
        console.log('---- load_flow error -----');
        console.log(e);
    });
}

// Step-1 load flow data
async function get_mapping_data(req_fleet) {
    if (!req_fleet) {
        alert("No Fleet avalible");
        return;
    }
    roleMappinglist = await restGetRolesMappingData(req_fleet);

    get_artifact(req_fleet);
}

// Step-2 Get fleet data
async function get_fleet_data(_fleet) {
    $('#Task').empty();

    var fleetName = _fleet.split('.').slice(0, -1).join('.');
    var data = await restGetFleetSettings(fleetName)
    var data_dict = data;
    var maps = [];
    var roles = [];
    for (let item_val of Object.values(data_dict)) {
        maps = item_val.maps;
        roles = item_val.roles;
    }

    if (maps.length == 0) {
        notificationMsg(3, "No Map in fleet, flow function may have some issue.")
        return;
    }

    // --- create map task cell ---
    maps.forEach((map) => {
        flow_get_map_cell(map);
    });

    if (roles.length == 0) {
        notificationMsg(3, "No role in fleet, flow function may have some issue.")
        return;
    }

    roles.forEach((role) => {
        add_roles_cell(role);
        genRoleInfo(role);
    })

    applyFontSize(getSavedFontSize(), '#Task');
}

function initDataDict() {
    const start_dict = {
        'name': 'start',
        'param': {},
        'relation': {
            'current_id': 2,
            'input': []
        }
    };
    const finish_dict = {
        'name': 'finish',
        'param': {},
        'relation': {
            'current_id': 3,
            'input': []
        }
    };
    flowdatadict['node-2'] = start_dict;
    flowdatadict['node-3'] = finish_dict;
}

function init_panel() {
    if (!confirm('Clear the current flow draft?')) { return; }

    $('#card_content_div').empty();
    selectingNodeId_ = null;
    deleteNodeId_ = null;
    bSidebarCollapse_ = true;
    sidebar_mode = 'op';
    validate_res_dict = {};
    flowdatadict = {};
    initDataDict();
    unSelectId = undefined;

    editor.clearModuleSelected();
    editor.import(data_init_drawflow);
    adjust_layout();
}

// task related cell 
// --- collect all cells in the map --- 
async function flow_get_map_cell(_map) {
    var cellsData = await restGetMapCells(_map);

    if (cellsData.includes('no such file or directory')) {
        notificationMsg(3, 'No cell settings in maps, please check it.')
        return;
    }

    var fleetCellList = JSON.parse(cellsData);
    for (let [item_key, item_val] of Object.entries(fleetCellList)) {
        var map_item_name = item_key.replaceAll('-', '');
        var content_dict = {};
        item_val.forEach((fltMapItem) => {
            content_dict[fltMapItem.cell_id] = fltMapItem.cell_id;
        });

        roleMappinglist.push({
            "role_name": map_item_name,
            "title_name": `map@${_map}`,
            "title_content": content_dict
        });
    }
}

// For role
function add_roles_cell(_role) {
    var domTaskCell = `
    <div class="drag-drawflow" draggable="true" ondragstart="drag(event)" data-node="${_role}">
      <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-bookmark-fill" viewBox="0 0 16 16">
        <path d="M2 2v13.5a.5.5 0 0 0 .74.439L8 13.069l5.26 2.87A.5.5 0 0 0 14 15.5V2a2 2 0 0 0-2-2H4a2 2 0 0 0-2 2z"/>
      </svg>
      <span> ${_role}</span>
    </div>
  `;
    $('#Task').append(domTaskCell);
}

// --- save --- 
async function save_ui_flow_setting(_fleetname, _flowname, _data) {
    await restPostFlowUIData(_data, _fleetname, _flowname);
    notificationMsg(0, `${_flowname} panel saved`);
}

async function save(_saveMode) {
    if (Object.values(validate_res_dict).includes(false)) {
        notificationMsg(2, `Input val have incorrect data! Please check your input.`);
        return;
    } else if ($('#edit-flowname-switch').find('.fa-eye').length > 0) {
        notificationMsg(2, `Rename flow name haven't complete, please check your modification.`);
        return;
    } else if ($('.editing-flowname').length > 0) {
        let flow_name = $('.editing-flowname').text();
        if (!inputCheck(flow_name)) {
            notificationMsg(2, `Flow name include invalid characters.`);
            return;
        }
    }

    var flowName = (_saveMode === 'op') ? flow_name : assign_robot_flow_name;
    // TODO: refactor the global variable name, flow_name;
    console.log(flow_name);

    // console.log(editor.export());
    var flowTasks = editor.export().drawflow.Home.data;
    console.log(flowTasks);
    for (let task of Object.values(flowTasks)) {
        var inRelations = [];
        var errOutRelations = [];
        var nodeInputs = task.inputs;
        var nodeOutputs = task.outputs;
        if (!_.isEmpty(nodeInputs)) {
            inRelations = nodeInputs.input_1.connections.map(c => c.node);
        }
        if (!_.isEmpty(nodeOutputs) && nodeOutputs.hasOwnProperty('output_2')) {
            // outRelations = nodeOutputs.output_1.connections.map(c => c.node);
            errOutRelations = nodeOutputs.output_2.connections.map(c => c.node);
        }
        console.log(nodeOutputs);
        console.log(errOutRelations);

        flowdatadict["node-" + task.id].relation.input = inRelations;

        // --- skip error-handling on `start` and `finish` nodes ---
        if (["start", "finish"].includes(task.name)) { continue; }

        // --- add error-handling on task nodes ---
        errOutRelations = (errOutRelations.length) ? errOutRelations[0] : "";

        // TODO: refactor
        if (flowdatadict["node-" + task.id].hasOwnProperty('error_handle')) {
            flowdatadict["node-" + task.id].error_handle["error_handle_task_node"] = errOutRelations;
        }
    }

    var flow_data_list = [];

    for (let [item_key, item_val] of Object.entries(flowdatadict)) {
        var dict_tmp = {};

        if (item_val.hasOwnProperty('param')) {
            for (const [key, value] of Object.entries(item_val['param'])) {
                if (key.includes('angle_or_goal')) {
                    if (value == 'goal') {
                        let relate_list = key.replace('angle_or_goal_', '').split('_');
                        for (var param_index in relate_list) {
                            if (relate_list[param_index].includes('angle')) {
                                let param_key = relate_list[param_index].replace('@', '_');
                                item_val['param'][param_key] = 'NAN';
                            }
                        }
                    }
                } else if (key.includes('angle')) {
                    if (value == 'undefined') {
                        item_val['param'][key] = 'NAN';
                    }
                }
            }
        }

        dict_tmp[item_key] = item_val;
        flow_data_list.push(dict_tmp);
    }

    // --- update flow data when control sidebar still open ---
    if ($('.drawflow-node.selected').length && (!bSidebarCollapse_ && _saveMode === 'op')) {
        updateDatadict(`node-${selectingNodeId_}`);
    }

    var save_data = {
        "flow_name": flowName,
        "event_name": event_name,
        "flow_data": flow_data_list
    }

    var triggerName = $("#flow_select option:selected").text();
    var flow_filename = `${getSelectedFleet()}-flow-${flowName}`;
    var data = await restOperationFileExistence("flow", flowName);
    // console.log(data);
    var nameExists = data.fileExists && (data.filename !== flow_filename);

    // --- protection for missing input ---
    if (!triggerName.trim().length) {
        alert("Please select a task trigger!");
        return;
    }
    if (nameExists) {
        alert(`${flowName} already exists!`);
        return;
    }

    // --- processing save flow procedure ---
    if (_saveMode === 'op') {
        console.log(editor.export());
        save_ui_flow_setting(getSelectedFleet(), flowName, JSON.stringify(editor.export()));
        bUnsavedChages_ = false;
    }

    // --- save to planner ---
    var event = {};
    if (triggerName.includes("Timer")) {
        event["event"] = "Timer";
        event["start_time"] = start_date + "/" + start_time;
        event["end_time"] = (end_time === "") ? "" : end_date + "/" + end_time;
        event["repeat_time"] = repeat_time;
    } else {
        event["event"] = triggerName;
    }
    save_data.event_name = event;
    console.log(save_data);

    console.log(_saveMode);

    let priority_settings = get_task_priority_list();

    console.log(priority_settings);

    save_data.flow_data.forEach(data => 
        Object.entries(data).forEach(([node_id, node_value]) => {
            if (priority_settings[node_id] !== undefined) {
                node_value.priority = priority_settings[node_id]
            }
        })
    )

    console.log(save_data);

    var res = await restPostFlowData(save_data, getSelectedFleet(), flowName);
    if (res.includes('Flow planner saved')) {
        notificationMsg(1, `[SWARM] ${flowName} saved Successfully! `);
    } else {
        notificationMsg(3, `[SWARM] Failed to save ${flowName}`)
    }

    // --- add manual trigger list on sidebar ---
    var jobMode = 2;
    if ($('#flow_select').val() === "manual" && _saveMode === 'op') {
        createTaskTriggerItem(getSelectedFleet(), flowName, jobMode);
        chkTriggerListVisible();
        checkDisableManual(jobMode);
    }
}

function get_task_priority_list() {
    let priority_list = $('.drawflow-task-priority-select').map(function(i, e) {
        let select_val = $(e).val();
        let node_id = $(e).parent().closest('.drawflow-node').attr('id');
        return {"node_id": node_id, "priority": select_val}
    }).get();

    let priority_dict = {};
    
    priority_list.forEach(setting => 
        priority_dict[setting.node_id] = setting.priority === "0" ? "": setting.priority
    )

    return priority_dict
}

// --- delete flow --- 
async function delete_flow() {
    fleet_cookie = getSelectedFleet();
    console.log($.cookie('flow_name'));
    flow_name_cookie = $.cookie('flow_name').replace(/\s/g, '_');
    console.log(flow_name_cookie);

    console.log(`=== fleet_cookie: ${fleet_cookie}, flow_name_cookie: ${flow_name_cookie} ===`)
    let res = await restDeleteFlowData(fleet_cookie, flow_name_cookie);
    notificationMsg(0, res);
    editor.clearModuleSelected();
}

// --- load flow ---
async function load_flow() {
    var data = await restGetFlowData(flow_name_cookie, `${fleet_cookie}-flowui-${flow_name_cookie}`)
        .fail(function (data, status) {
            editor.import(data_init_drawflow);
            adjust_layout();
        });

    // If fail will return
    dataToImport = JSON.parse(data);
    console.log(dataToImport);
    editor.import(dataToImport);
    // --- styling outputs ---
    var nodes = $('div[id^="node-"]').filter(':not(#node-2)').filter(':not(#node-3)');
    nodes.parent().find(".outputs > .output_1").attr("title", "success").css("background", "lime");
    nodes.parent().find(".outputs > .output_2").attr("title", "failure").css("background", "red");

    load_flow_data();
    adjust_layout();
}

async function load_flow_data() {
    var data = await restGetFlowData(flow_name_cookie, `${fleet_cookie}-flow-${flow_name_cookie}`);
    dataToImport = JSON.parse(data);
    var data_list = dataToImport.flow_data;
    var data_dict = {};
    data_list.forEach((node_item) => {
        for (let [item_key, item_val] of Object.entries(node_item)) {
            data_dict[item_key] = item_val;
        };
    });
    console.log(data_list);
    console.log(data_dict);

    // --- Checking if there are role update ---
    for (let [item_key, item_val] of Object.entries(data_dict)) {
        // --- skip constant param nodes ---
        if (CONST_OPERANDS_.includes(item_val.name)) { continue; }

        // --- collect param values nodes ---
        var check_role_dict = {};
        var roleMappinglist2 = roleMappinglist.filter(r => r.role_name === item_val.name);
        roleMappinglist2.forEach((role_item) => {
            for (let [title_content_key, title_content_val] of Object.entries(role_item.title_content)) {
                check_role_dict[title_content_key] = title_content_val;
            }
        });

        // --- remove not exist key ---
        if (item_val.hasOwnProperty("param")) {
            for (let param_key of Object.keys(item_val.param)) {
                if (check_role_dict.hasOwnProperty(param_key)) { continue; }
                if (param_key.includes('angle_or_goal')) {
                    continue;
                } else {
                    delete data_dict[item_key].param[param_key];
                }
            }
        }

        // --- add missing param --- 
        for (let [tcKey, tcValue] of Object.entries(check_role_dict)) {
            if (!data_dict[item_key].hasOwnProperty("param")) { continue; }
            var paramVal = (data_dict[item_key].param.hasOwnProperty(tcKey)) ? data_dict[item_key].param[tcKey] : tcValue;
            data_dict[item_key].param[tcKey] = paramVal;
        }
    }

    flowdatadict = data_dict;

    // --- check time is ok timer only ---
    if (dataToImport.event_name === 'Timer') {
        var t = dataToImport.event_name.start_time.split('/');
        var t_check = new Date(parseInt(t[0]), parseInt(t[1]) - 1, parseInt(t[2]), parseInt(t[3].split(':')[0]), parseInt(t[3].split(':')[1]));
        isTimePassed = (sys_time > t_check.getTime()) ? true : false;
    }
    console.log('------ END OF LOADING FLOW ------')

    adjustNodeNameInput();
}

function set_flow_event(_flow_event) {
    event_name = _flow_event;
}

function switch_fleet(fleet) {
    $('#Task').children().slice(2).remove()
    get_fleet_data(`${fleet}.yaml`);
    editor.clearModuleSelected();
}

function genUUID() {
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

    if ($('#start').length) {
        $('#start').parent().parent().css("width", "100px");
        editor.updateConnectionNodes("node-2");
    }

    if ($('#finish').length) {
        $('#finish').parent().parent().css("width", "100px");
        editor.updateConnectionNodes("node-3");
    }

    if ($('.logical_and').length) {
        $('.logical_and').each(function (index) {
            node_update.push($(this).attr('id'));
            var id = $(this).attr('id').split('-')[1];
            $(this).find('div#logical_and').attr("id", `logical_and-${id}`);
            $(this).css("width", "100px");
        });
    }

    if ($('.logical_or').length) {
        $('.logical_or').each(function (index) {
            node_update.push($(this).attr('id'));
            var id = $(this).attr('id').split('-')[1];
            $(this).find('div#logical_or').attr("id", `logical_or-${id}`);
            $(this).css("width", "100px");
        });
    }

    if ($('.logical_delay').length) {
        $('.logical_delay').each(function (index) {
            node_update.push($(this).attr('id'));
            var id = $(this).attr('id').split('-')[1];
            $(this).find('div#logical_delay').attr("id", `logical_delay-${id}`);
            $(this).css("width", "100px");
        });
    }

    node_update.forEach((nodeItem) => {
        editor.updateConnectionNodes(nodeItem);
    });

    // for old flow that do not have geer
    $('.drawflow_content_node').each(function (index) {
        haveBoxContent = $(this).find('div.box').length;
        if (haveBoxContent != 0) {
            haveConfContent = $(this).find('div.box').find('i').length;

            if (haveConfContent == 0) {
                icon_str = `<button type="button"><i class="nav-icon fas fa-cog"></i></button>`;
                $(this).find('div.box').append(icon_str);
            }
        }
    });
}

function capitalize(s) {
    return s[0].toUpperCase() + s.slice(1);
}

function rename_showing_label(txt) {
    return txt.replace(/_(\w{1,})/, '');
}

function getMapAlias(_mapName) {
    const mapData = mapAlias_.find(m => m.name === _mapName);
    if (mapData === undefined) { return _mapName; }
    return (mapData.alias_name || _mapName);
}

// Get artifact
async function get_artifact(fleet_name) {
    var fltSettings = await fetchGetFleetConfigs(rmtToken_, fleet_name);
    let artifact_settings = fltSettings["artifacts"];

    if (artifact_settings.hasOwnProperty('external') && artifact_settings['external'].length > 0) {
        artifact_settings['external'].forEach(function (external_artifacts) {
            let artifact_type = external_artifacts.split('@')[0];
            let artifact_id = external_artifacts.split('@')[1];

            if (artifact_dict.hasOwnProperty(artifact_type)) {
                artifact_dict[artifact_type].push(artifact_id);
            } else {
                artifact_dict[artifact_type] = [artifact_id];
            }
        })
    } else {
        console.log('No external artifacts.')
    }

    if (artifact_settings.hasOwnProperty('agent') && artifact_settings['agent'].length > 0) {
        artifact_settings['agent'].forEach(function (agent_artifacts) {
            Object.values(agent_artifacts)[0].forEach(function (atifacts) {
                let artifact_type = atifacts.split('@')[0];
                let artifact_id = atifacts.split('@')[1];

                if (artifact_dict.hasOwnProperty(artifact_type)) {
                    artifact_dict[artifact_type].push(artifact_id);
                } else {
                    artifact_dict[artifact_type] = [artifact_id];
                }
            })
        })
    } else {
        console.log('No agent artifacts.')
    }

    console.log(artifact_dict)
}

function send_flow_timer(flow_name, _sec) {
    var jsonNewFlow = {
        event: genUUID(),
        event_type: flow_name,
        params: 'ui',
        timer_scheduled: true,
        plan_time: {
            sec: _sec,
            nanosec: 0
        }
    };
    wsPubFlowReq(jsonNewFlow);
}

// Drawflow relate
function genRoleInfo(_role) {
    let showRoleName = adjustShowtext(_role);
    var domTaskCell = `
    <div data-bs-toggle="tooltip" data-bs-placement="top" title="${_role}">
      <div class="title-box">
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-bookmark-fill" viewBox="0 0 16 16">
        <path d="M2 2v13.5a.5.5 0 0 0 .74.439L8 13.069l5.26 2.87A.5.5 0 0 0 14 15.5V2a2 2 0 0 0-2-2H4a2 2 0 0 0-2 2z"/>
        </svg>
        ${showRoleName}
      </div>
      <div class="box">
          <div class="row">
          <div class="col-9">
            <select class="form-select drawflow-task-priority-select" aria-label="Task priority select">
                <option value="5">Highest</option>
                <option value="4">High</option>
                <option value="3" selected>Normal</option>
                <option value="2">Low</option>
                <option value="1">Lowest</option>
            </select>
          </div>
        </div>
        <div class="row" style="margin-top: 5px;">
          <div class="col-9">
              <input type="text" df-name="node-name" class="flow-input" placeholder="custom name">
          </div>
          <div class="col-3">
              <button type="button"><i class="nav-icon fas fa-cog"></i></button>
          </div>
        </div>		
      </div>
    </div>
  `;

    NodeToDrawFlow_str_list[`${_role}`] = domTaskCell;
}

let id = document.getElementById("drawflow");
let editor = new Drawflow(id);

editor.reroute = true;
editor.reroute_fix_curvature = true;
editor.force_first_input = false;

editor.start();
// editor.import(dataToImport);

// --- Events! ---
// const OPERANDS_ = ['logical_and', 'logical_or', 'logical_delay'];
const OPERANDS_ = [];
editor.on('nodeCreated', function (id) {
    bUnsavedChages_ = true;
    var nodeId = editor.getNodeFromId(id).id;
    var nodeName = editor.getNodeFromId(id).name;
    var targetNode = "node-" + nodeId;

    if (OPERANDS_.includes(nodeName)) {
        let opObj = {
            'name': nodeName,
            'param': {},
            'relation': {
                'current_id': nodeId,
                'input': []
            }
        };
        flowdatadict[targetNode] = opObj;
    } else {
        get_card_data(nodeName, nodeId);
        updateDatadict(targetNode);
        $('#tmp_select').empty();
        adjustNodeNameInput();
    };
})

editor.on('nodeRemoved', function (id) {
    bUnsavedChages_ = true;

    delete flowdatadict['node-' + id];
    $("#control_sidebar").ControlSidebar('collapse');
    $('#card_content_div').empty();
    selectingNodeId_ = null;
    deleteNodeId_ = null;
    bSidebarCollapse_ = true;
    sidebar_mode = 'op';
    validate_res_dict = {};
    unSelectId = undefined;

    if ($('.detail_icon:disabled').length == 0) {
        $('#save-flow-btn').prop('disabled', false);
    }
})

editor.on('nodeSelected', function (id) {
    console.log("Node selected " + id);
    var node_id = editor.getNodeFromId(id).id;
    var node_name = editor.getNodeFromId(id).name;

    var configBtn = $(`#node-${node_id}`).find('button');
    configBtn.addClass(`detail_icon`).attr('id', `detailid_node-${node_id}`);
    configBtn.bind("click", function () {
        sidebar_mode = 'op';
        sidebarEvent(node_name, node_id);
    });

    // --- Device Styling Alignment (touch devices) ---
    if (!isTouchDevice()) { return; }
    delete_type = 'node';
    deleteNodeId_ = node_id;
    $(".btn-delete").css('display', '').text('Delete Node');
})

editor.on('nodeUnselected', function (id) {
    unSelectId = selectingNodeId_;
    console.log("Node unselected: " + id);
    var node_name = "";

    try {
        if (selectingNodeId_ != null) {
            node_name = editor.getNodeFromId(selectingNodeId_).name;
            // console.log(node_name)
            if (!CONST_OPERANDS_.includes(node_name)) {
                var taget_node = "node-" + selectingNodeId_;
                updateDatadict(taget_node);
                selectingNodeId_ = null;
                // console.log(sidebar_mode)
            }
        }

        // --- Device Styling Alignment (touch devices) ---
        if (!isTouchDevice()) { return; }
        delete_type = '';
        deleteNodeId_ = null;
        $(".btn-delete").css('display', 'none');
    } catch (error) {
        console.error(error);
    }
})

editor.on('connectionSelected', function (connection) {
    console.log('Connection selected');
    if (!isTouchDevice()) { return; }
    conn = connection;
    delete_type = 'conn';
    $(".btn-delete").css('display', '').text('Delete Connection');
});

editor.on('connectionUnselected', function (connection) {
    console.log('Connection unselected');
    if (!isTouchDevice()) { return; }
    conn = connection;
    delete_type = '';
    $(".btn-delete").css('display', 'none');
});


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
    // console.log(ev.type);
    ev.preventDefault();
}

function drag(ev) {
    // console.log(ev.type);
    if (ev.type === "touchstart") {
        mobile_item_selec = ev.target.closest(".drag-drawflow").getAttribute('data-node');
    } else {
        ev.dataTransfer.setData("node", ev.target.getAttribute('data-node'));

        // add the node directly because dragover & drop event can not be triggered on Android device
        if (getMobileOS() !== 'android') return;
        drop(ev);
    }
}

function drop(ev) {
    // console.log(ev.type);
    if (ev.type === "touchend") {
        var parentdrawflow = document.elementFromPoint(mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY).closest("#drawflow");
        if (parentdrawflow != null) {
            addNodeToDrawFlow(mobile_item_selec, mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY);
        }
        mobile_item_selec = '';
    } else if (ev.type === "dragstart") {
        ev.preventDefault();
        var data = ev.dataTransfer.getData("node");
        var drawflowRect = document.getElementById('drawflow').getBoundingClientRect();
        addNodeToDrawFlow(data, drawflowRect.x, drawflowRect.y + 100);
    } else {
        ev.preventDefault();
        var data = ev.dataTransfer.getData("node");
        addNodeToDrawFlow(data, ev.clientX, ev.clientY);
    }
}

const WidgetAssets_ = {
    start: {
        title: 'Start',
        num: { i: 0, o: 1 },
        icon: `
            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-play-circle" viewBox="0 0 16 16">
                <path d="M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z"/>
                <path d="M6.271 5.055a.5.5 0 0 1 .52.038l3.5 2.5a.5.5 0 0 1 0 .814l-3.5 2.5A.5.5 0 0 1 6 10.5v-5a.5.5 0 0 1 .271-.445z"/>
            </svg>
    `},
    finish: {
        title: 'Finish',
        num: { i: 1, o: 0 },
        icon: `
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-record-circle" viewBox="0 0 16 16">
            <path d="M8 15A7 7 0 1 1 8 1a7 7 0 0 1 0 14zm0 1A8 8 0 1 0 8 0a8 8 0 0 0 0 16z"/>
            <path d="M11 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0z"/>
        </svg>
    `}
    // logical_and: {
    // 	title: 'And',
    // 	num: { i: 1, o: 1 },
    // 	icon: `
    // 	<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-hand-thumbs-up" viewBox="0 0 16 16">
    // 		<path d="M8.864.046C7.908-.193 7.02.53 6.956 1.466c-.072 1.051-.23 2.016-.428 2.59-.125.36-.479 1.013-1.04 1.639-.557.623-1.282 1.178-2.131 1.41C2.685 7.288 2 7.87 2 8.72v4.001c0 .845.682 1.464 1.448 1.545 1.07.114 1.564.415 2.068.723l.048.03c.272.165.578.348.97.484.397.136.861.217 1.466.217h3.5c.937 0 1.599-.477 1.934-1.064a1.86 1.86 0 0 0 .254-.912c0-.152-.023-.312-.077-.464.201-.263.38-.578.488-.901.11-.33.172-.762.004-1.149.069-.13.12-.269.159-.403.077-.27.113-.568.113-.857 0-.288-.036-.585-.113-.856a2.144 2.144 0 0 0-.138-.362 1.9 1.9 0 0 0 .234-1.734c-.206-.592-.682-1.1-1.2-1.272-.847-.282-1.803-.276-2.516-.211a9.84 9.84 0 0 0-.443.05 9.365 9.365 0 0 0-.062-4.509A1.38 1.38 0 0 0 9.125.111L8.864.046zM11.5 14.721H8c-.51 0-.863-.069-1.14-.164-.281-.097-.506-.228-.776-.393l-.04-.024c-.555-.339-1.198-.731-2.49-.868-.333-.036-.554-.29-.554-.55V8.72c0-.254.226-.543.62-.65 1.095-.3 1.977-.996 2.614-1.708.635-.71 1.064-1.475 1.238-1.978.243-.7.407-1.768.482-2.85.025-.362.36-.594.667-.518l.262.066c.16.04.258.143.288.255a8.34 8.34 0 0 1-.145 4.725.5.5 0 0 0 .595.644l.003-.001.014-.003.058-.014a8.908 8.908 0 0 1 1.036-.157c.663-.06 1.457-.054 2.11.164.175.058.45.3.57.65.107.308.087.67-.266 1.022l-.353.353.353.354c.043.043.105.141.154.315.048.167.075.37.075.581 0 .212-.027.414-.075.582-.05.174-.111.272-.154.315l-.353.353.353.354c.047.047.109.177.005.488a2.224 2.224 0 0 1-.505.805l-.353.353.353.354c.006.005.041.05.041.17a.866.866 0 0 1-.121.416c-.165.288-.503.56-1.066.56z"/>
    // 	</svg>
    // `},
    // logical_or: {
    // 	title: 'Or',
    // 	num: { i: 1, o: 1 },
    // 	icon: `
    // 	<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-sunglasses" viewBox="0 0 16 16">
    // 			<path d="M3 5a2 2 0 0 0-2 2v.5H.5a.5.5 0 0 0 0 1H1V9a2 2 0 0 0 2 2h1a3 3 0 0 0 3-3 1 1 0 1 1 2 0 3 3 0 0 0 3 3h1a2 2 0 0 0 2-2v-.5h.5a.5.5 0 0 0 0-1H15V7a2 2 0 0 0-2-2h-2a2 2 0 0 0-1.888 1.338A1.99 1.99 0 0 0 8 6a1.99 1.99 0 0 0-1.112.338A2 2 0 0 0 5 5H3zm0 1h.941c.264 0 .348.356.112.474l-.457.228a2 2 0 0 0-.894.894l-.228.457C2.356 8.289 2 8.205 2 7.94V7a1 1 0 0 1 1-1z"/>
    // 	</svg>
    // `},
    // logical_delay: {
    // 	title: 'Delay',
    // 	num: { i: 1, o: 1 },
    // 	icon: `
    // 	<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-stopwatch" viewBox="0 0 16 16">
    // 		<path d="M8.5 5.6a.5.5 0 1 0-1 0v2.9h-3a.5.5 0 0 0 0 1H8a.5.5 0 0 0 .5-.5V5.6z"/>
    // 		<path d="M6.5 1A.5.5 0 0 1 7 .5h2a.5.5 0 0 1 0 1v.57c1.36.196 2.594.78 3.584 1.64a.715.715 0 0 1 .012-.013l.354-.354-.354-.353a.5.5 0 0 1 .707-.708l1.414 1.415a.5.5 0 1 1-.707.707l-.353-.354-.354.354a.512.512 0 0 1-.013.012A7 7 0 1 1 7 2.071V1.5a.5.5 0 0 1-.5-.5zM8 3a6 6 0 1 0 .001 12A6 6 0 0 0 8 3z"/>
    // 	</svg>
    // `}
};

function addNodeToDrawFlow(name, pos_x, pos_y) {
    if (editor.editor_mode === 'fixed') {
        return false;
    }
    var edRect = editor.precanvas.getBoundingClientRect();
    var edZoom = editor.zoom;
    pos_x = (pos_x - edRect.x) / edZoom;
    pos_y = (pos_y - edRect.y) / edZoom;

    if (NodeToDrawFlow_str_list.hasOwnProperty(name)) {
        var className = name.replace(/\s/g, '_');
        var idNum = editor.addNode(`${name}`, 1, 2, pos_x, pos_y, `${className}`, {}, NodeToDrawFlow_str_list[name]);
        if (!idNum) { return; }
        $(`#node-${idNum}`).parent().find(".outputs > .output_1").attr("title", "success").css("background", "lime");
        $(`#node-${idNum}`).parent().find(".outputs > .output_2").attr("title", "failure").css("background", "red");
    }

    var targetKeys = Object.keys(WidgetAssets_);
    if (!targetKeys.includes(name)) { return; }

    var widget = WidgetAssets_[name];
    var dom = `
        <div id="${name}">
            <div class="title-box">
            ${widget.icon}
            ${widget.title}
            </div>
        </div>
        `;

    // --- `start` and `finish` are unique for each flow ---
    if (!name.includes('logical_') && !$(`#${name}`).length) {
        editor.addNode(name, widget.num.i, widget.num.o, pos_x, pos_y, name, {}, dom);
        return;
    }

    // --- `logical_xxx` widgets ---
    var node_num = editor.addNode(name, widget.num.i, widget.num.o, pos_x, pos_y, name, {}, dom);
    $(`#${name}`).parent().parent().css("width", "100px");
    $(`#${name}`).attr("id", `${name}-${node_num}`);
}

var transform = '';

function showpopup(e) {
    e.target.closest(".drawflow-node").style.zIndex = "9999";
    e.target.children[0].style.display = "block";

    transform = editor.precanvas.style.transform;
    editor.precanvas.style.transform = '';
    editor.precanvas.style.left = editor.canvas_x + 'px';
    editor.precanvas.style.top = editor.canvas_y + 'px';
    console.log(transform);

    editor.editor_mode = "fixed";
}

function closemodal(e) {
    e.target.closest(".drawflow-node").style.zIndex = "2";
    e.target.parentElement.parentElement.style.display = "none";
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
    lock.style.display = (option === 'lock') ? 'none' : 'block';
    unlock.style.display = (option === 'lock') ? 'block' : 'none';
}

// Template
function card_template(card_title, card_subtitle, card_content) {
    if (bSidebarCollapse_ && sidebar_mode === 'op') { return; }

    var domCard = `
  <div class="row card-deck" style="border-radius: 15px; ">
    <div class="card col-lg-12 p-3 nav2dmapCanvas farobot-view-bg" data-widget="" style="margin:1rem; background-color:white">
      <div class="card-header">
        <h4 class="card-title">${transTitleName(card_title, card_subtitle)}</h4>
      </div>
      <div class="card-body" style="font-size: 1.0rem;">
        ${card_content}
      </div>
    </div>
  </div>
  `;

    $('#card_content_div').append(domCard);
}

function genAreaOptions(map_val) {
    var map_select = '';

    Object.keys(goal_cell_dict[map_val]).forEach(function (map_item, map_index) {
        var selectedStyle = (map_index === 0) ? "selected" : "";
        map_select += `<option value="${map_item}" ${selectedStyle}>${capitalize(map_item)}</option>`;
    });

    return `
        <select class="cus-select input-item area" style="margin-top: 10px;">
            ${map_select}
        </select>
        `;
}

function genCellOptions(map_val, area_val) {
    var cell_select = '';

    goal_cell_dict[map_val][area_val].forEach(function (cell_item, cell_index) {
        var selectedStyle = (cell_index === 0) ? "selected" : "";
        cell_select += `<option value="${cell_item}" ${selectedStyle}>${cell_item}</option>`;
    });

    return `
        <select class="cus-select input-item cell" style="margin-top: 10px;">
            ${cell_select}
        </select>
        `;
}

// HTML Relate -- Event
$(document).on("change", ".map", function () {
    var selArea = '';
    if ($(this).parent().find('select.area').length) {
        $(this).parent().children().remove("select.area");
        $(this).parent().children().remove("select.cell");
    }

    if (!$(this).parent().find('select.area').length) {
        var area_select = genAreaOptions($(this).val());
        $(this).parent().append(area_select);
        selArea = $(this).parent().find('select.area').val();
    }

    if (!$(this).parent().find('select.cell').length) {
        var cell_select = genCellOptions($(this).val(), selArea);
        $(this).parent().append(cell_select);
    }
});

$(document).on("change", ".area", function () {
    if ($(this).parent().find('select.cell').length) {
        $(this).parent().children().remove("select.cell");
    };

    var cell_select = genCellOptions($(this).parent().find('select.map').val(), $(this).val());
    $(this).parent().append(cell_select);
});

$(document).on("click", ".drawflow", function () {
    if ($('.drawflow-node.selected').length) { return; }
    console.log(validate_res_dict)
    console.log(Object.values(validate_res_dict))
    console.log(Object.values(validate_res_dict).includes(false))
    let current_select_node_id = `node-${unSelectId}`;
    // [FAR-2051] Test situation 3. Side bar in edit flow for "RUN" cannot go back
    if (!flowdatadict.hasOwnProperty(current_select_node_id)) {
        $("#control_sidebar").ControlSidebar('collapse');
        return;
    }
    let node_param_dict = flowdatadict[current_select_node_id]['param'];
    console.log(node_param_dict)

    for (let [vali_key, vali_val] of Object.entries(validate_res_dict)) {
        if (vali_val == false && Object.keys(node_param_dict).includes(vali_key)) {
            if (Object.values(validate_res_dict).includes(false)) {
                $(`#node-${unSelectId}`).addClass('selected');
                editor.node_selected = $(`#node-${unSelectId}`)[0];
                selectingNodeId_ = unSelectId;
                return;
            }
        } else if (vali_val == false && error_handle_list.includes(vali_key)) {
            return;
        } else {
            console.log('Other')
        }
    }

    $("#control_sidebar").ControlSidebar('collapse');
    bSidebarCollapse_ = true;
    sidebar_mode = 'op';
    validate_res_dict = {};
    unSelectId = undefined;
    if ($('.detail_icon:disabled').length == 0) {
        $('#save-flow-btn').prop('disabled', false);
    }
});

$(document).on("change", ".robot_ass", function () {
    var robot_select = '';
    // not working on iOS devices
    // var re = /(?<=node-)[\w+.-]+/;
    // var target_node = `node-${$(this).attr('class').match(re)}`;
    var target_node = '';
    var targetClassArr = $(this).prop('classList');
    targetClassArr.forEach((className) => {
        if (className.includes('node-')) {
            target_node = className;
        }
    });
    var selected_val = $(this).val();
    robot_select = (selected_val === 'auto') ? '' : selected_val;

    flowdatadict[target_node]['assigned_robot'] = robot_select;
});

$(document).on("change", "#flow_select", function () {
    bUnsavedChages_ = true;
});

$(document).on("change", ".angle_or_goal", function () {
    var selected_val = $(this).val();
    var parent_card = $(this).closest('.card-body');

    $(parent_card).find('.row').each(function (index) {
        var attr = $(this).find('.input-label').text().split(' ').shift();
        if (selected_val === 'goal') {
            if (attr === 'goal') {
                $(this).css("display", "");
            } else if (attr === 'angle') {
                $(this).css("display", "none");
            }
        } else if (selected_val === 'angle') {
            if (attr === 'goal') {
                $(this).css("display", "none");
            } else if (attr === 'angle') {
                $(this).css("display", "");
            }
        }
    });
});

function deleteSelectedItem() {
    $(".btn-delete").css('display', 'none');
    if (delete_type === 'conn' && (typeof conn !== 'undefined')) {
        editor.removeSingleConnection(conn.output_id, conn.input_id, conn.output_class, conn.input_class);
        return;
    }

    if (delete_type === 'node' && deleteNodeId_ != null) {
        editor.removeConnectionNodeId('node-' + deleteNodeId_);
        var moduleName = editor.getModuleFromNodeId(deleteNodeId_);
        if (editor.module === moduleName) {
            document.getElementById('node-' + deleteNodeId_).remove();
        }
        delete editor.drawflow.drawflow[moduleName].data[deleteNodeId_];
        editor.dispatch('nodeRemoved', deleteNodeId_);
    }
}

// HTML Relate -- UI
function get_card_data(role_name, node_id) {
    let validate_rule_list = [];
    // ------ protection ------
    if (!roleMappinglist.length) { return false; }

    // --- flows data processing ---
    genCellMappingDict();

    var title_name = "";
    var type_name = "";
    var card_body_str = "";
    var res = false;

    // --- data info ---
    var data_dict_id_tag = `node-${node_id}`;
    var data_dict_param = {};

    // --- new created card ---
    if (!flowdatadict.hasOwnProperty(data_dict_id_tag)) {
        // For dropped show sidebar create
        flowdatadict[data_dict_id_tag] = {
            "name": '',
            "param": {},
            "relation": {
                "current_id": node_id,
                "input": []
            }
        };

        var taskConfigs = roleMappinglist.filter(ta => ta.role_name === role_name);
        console.log(taskConfigs);
        taskConfigs.forEach((taskParam) => {
            if (taskParam.role_name !== role_name) { return; }

            title_name = taskParam.title_name;
            type_name = taskParam.type;

            let artifact_service = undefined;
            if (taskParam.hasOwnProperty('service')) {
                artifact_service = taskParam.service;
            }

            if ($.isEmptyObject(taskParam.title_content)) {
                title_name = taskParam.title_name;
                res = false;
                flowdatadict[data_dict_id_tag]["name"] = role_name;
                flowdatadict[data_dict_id_tag]["param"] = {};
                return;
            }

            if (taskParam.hasOwnProperty('title_content')) {
                for (let [item_key, item_val] of Object.entries(taskParam.title_content)) {
                    var para_name = item_key;
                    var para_default = item_val;
                    var mapping_string = '';

                    // for validation
                    if (item_key.includes("angle_")) {
                        validate_rule_list.push(new Rule(item_key, rotateAngleValueValidation))
                    }

                    if (item_key.includes("status_")) {
                        mapping_string = generate_select_list(para_name, 'status', '', type_name);
                    } else if (item_key.includes("cell_id_")) {
                        mapping_string = generate_select_list(para_name, 'cellid', para_default, type_name);
                    } else if (item_key.includes("goal_")) {
                        mapping_string = generate_select_list(para_name, 'goal', para_default, type_name);
                    } else if (item_key.includes("artifact_")) {
                        mapping_string = generate_select_list(para_name, 'artifact', para_default, type_name, taskParam.type);
                    } else if (item_key.includes("dock_")) {
                        mapping_string = generate_select_list(para_name, 'dock', para_default, type_name);
                    } else {
                        mapping_string = generate_select_list(para_name, '', para_default, type_name, null, artifact_service);
                    }

                    var displayStyle = (item_key.includes("recognition_method_") || item_key.includes("tag_id_") || item_key.includes("offset_")) ? "display: none;" : "";
                    card_body_str += `
                                <div class="row" style="padding:0.0rem;margin-top: 10px; ${displayStyle}">
                  <div class="col-4">
                    <div type="text" class="input-label" style="text-align: center;" name="${data_dict_id_tag}@${para_name}">${rename_showing_label(para_name)} :</div>
                  </div>
                  <div class="col-8">
                    ${mapping_string}
                  </div>
                </div>`;
                    // for add new node fill goal value
                    $('#tmp_select').append(card_body_str);

                    data_dict_param[para_name] = para_default;
                }
            } else {
                card_body_str += `<div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>`;
            }

            flowdatadict[data_dict_id_tag]["name"] = role_name;
            flowdatadict[data_dict_id_tag]["param"] = data_dict_param;

            if (sidebar_mode === 'op') {
                card_template(title_name, taskParam.type, card_body_str);
                card_body_str = "";
                res = true;
            } else if (sidebar_mode === 'test') {
                testflowInit();
            }
        });

        if (!res) {
            card_body_str = `
              <div class="row" style="padding:0.0rem;margin-top: 10px;">
                <div class="col-12">
                  <div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>
                </div>
              </div>`;

            if (sidebar_mode === 'op') {
                card_template(title_name, type_name, card_body_str);
                card_body_str = "";
            } else if (sidebar_mode === 'test') {
                testflowInit();
            }
        }
    } else {
        // For click node
        // --- render Task Config. Settings ---
        if (flowdatadict[data_dict_id_tag].hasOwnProperty('error_handle')) {

            var taskLevelParams = flowdatadict[data_dict_id_tag].error_handle;
            console.log(taskLevelParams);
            var categoryTitle = 'Task - Error Handle';
            var domTaskConfigBody = '';
            for (var paramKey of Object.keys(taskLevelParams)) {
                if (paramKey === 'error_handle_task_node') { continue; }
                var paramVal = taskLevelParams[paramKey];
                var domParamCard = generate_select_list(paramKey, '', paramVal);

                domTaskConfigBody += `
                    <div class="row" style="padding:0.0rem;margin-top: 10px;">
                        <div class="col-5">
                            <div type="text" class="input-label" style="text-align: center;" name="${paramKey}">${paramKey} :</div>
                        </div>
                        <div class="col-7">
                            ${domParamCard}
                        </div>
                    </div>`;
            }
            card_template(categoryTitle, 'test', domTaskConfigBody);
        }

        // --- render Task Behavior Parameters ---
        var target_flow_data_dict = flowdatadict[data_dict_id_tag];

        var taskConfigs = roleMappinglist.filter(ta => ta.role_name === role_name);
        console.log(roleMappinglist);
        console.log(taskConfigs);
        console.log(target_flow_data_dict);
        taskConfigs.forEach((taskParam) => {
            title_name = taskParam.title_name;
            type_name = taskParam.type;

            let artifact_service = undefined;
            if (taskParam.hasOwnProperty('service')) {
                artifact_service = taskParam.service;
            }

            if (!taskParam.title_content || !target_flow_data_dict.param) {
                res = false;
                return;
            }

            // console.log(taskParam);
            if (taskParam.hasOwnProperty('title_content')) {
                for (let item_key of Object.keys(taskParam.title_content)) {
                    var para_name = item_key;
                    var para_default = target_flow_data_dict.param[para_name];
                    var mapping_string = '';

                    // for validation
                    if (item_key.includes("angle_")) {
                        validate_rule_list.push(new Rule(item_key, rotateAngleValueValidation))
                    }

                    if (item_key.includes("status_")) {
                        mapping_string = generate_select_list(para_name, 'status', para_default, type_name);
                    } else if (item_key.includes("cell_id_")) {
                        mapping_string = generate_select_list(para_name, 'cellid', para_default, type_name);
                    } else if (item_key.includes("goal_")) {
                        mapping_string = generate_select_list(para_name, 'goal', para_default, type_name);
                    } else if (item_key.includes("artifact_")) {
                        mapping_string = generate_select_list(para_name, 'artifact', para_default, type_name, taskParam.type);
                    } else if (item_key.includes("dock_")) {
                        mapping_string = generate_select_list(para_name, 'dock', para_default, type_name);
                    } else {
                        mapping_string = generate_select_list(para_name, '', para_default, type_name, null, artifact_service);
                    }

                    var displayStyle = (item_key.includes("recognition_method_") || item_key.includes("tag_id_") || item_key.includes("offset_")) ? "display: none;" : "";
                    card_body_str += `
                                <div class="row" style="padding:0.0rem;margin-top: 10px; ${displayStyle}">
                  <div class="col-4">
                    <div type="text" class="input-label" style="text-align: center;" name="${data_dict_id_tag}@${para_name}">${rename_showing_label(para_name)} :</div>
                  </div>
                  <div class="col-8">
                    ${mapping_string}
                  </div>
                </div>`;

                    data_dict_param[para_name] = para_default;
                }
            } else {
                card_body_str += `<div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>`;
            }

            if (sidebar_mode == 'op') {
                card_template(title_name, taskParam.type, card_body_str);
                card_body_str = "";
                res = true;
            } else if (sidebar_mode == 'test') {
                testflowInit();
            }

        });

        // --- when no behavioral parmemters ---
        if (!res) {
            card_body_str = `
              <div class="row" style="padding:0.0rem;margin-top: 10px;">
                <div class="col-12">
                  <div type="text"  style="text-align: center;font-size: 1.0rem;">No need to set parameters</div>
                </div>
              </div>
            `
            card_template(title_name, type_name, card_body_str);
            card_body_str = "";
        }

    }

    validateEvent();
    validateFlowNameInputEvent(validate_rule_list);
    applyFontSize(getSavedFontSize(), '#control_sidebar');
}

function transTitleName(title, subTitle) {
    var title_ui = dictUiTerms.hasOwnProperty(title) ? dictUiTerms[title] : title;
    if (title_ui === "Artifact") {
        title_ui = (typeof subTitle === 'undefined') ? title_ui : `${title_ui} - ${subTitle}`;
    }
    return title_ui;
}

function updateDatadict(node_id) {
    const prev_data = JSON.stringify(flowdatadict);

    // console.log(sidebar_mode);
    if (sidebar_mode === 'op') {
        var artifact_param_list = [];
        var flow_params = flowdatadict[node_id].param;

        let error_handle_dict_values = [];

        $(".input-item").each(function () {
            let label_name = $(this).closest(".row").find('.input-label').attr('name');
            let flow_param_node = undefined;
            let flow_param_id = undefined;

            if (label_name.includes('@')) {
                flow_param_node = label_name.split('@')[0];
                flow_param_id = label_name.split('@')[1];
            } else {
                flow_param_id = label_name;
            }

            // console.log(flow_param_id, node_id)

            if (flow_param_node != undefined && flow_param_node != node_id) {
                return;
            }

            if (!$(this).is("select")) {
                if ($(this).hasClass("arti_param")) {
                    let param_id = $(this).attr('id');
                    let param_dict = { 'art_val_id': flow_param_id, 'art_param_li': [param_id, $(this).val()] };
                    artifact_param_list.push(param_dict);
                } else {
                    if ($.inArray(flow_param_id, error_handle_list) == -1) {
                        if (flow_param_id.includes("angle") && $(this).val() == 'undefined') {
                            flow_params[flow_param_id] = 'NAN';
                        } else {
                            flow_params[flow_param_id] = $(this).val();
                        }
                    } else {
                        error_handle_dict_values.push($(this).val());
                    }
                }
                return;
            }

            var select_data = '';
            if ($(this).hasClass("map")) {
                var map = $(this).find("option:selected").val();
                var area = $(this).parent().find(".area").find("option:selected").val();
                var cell = $(this).parent().find(".cell").find("option:selected").val();
                select_data = `${map}@${area}@${cell}`;
                flow_params[flow_param_id] = select_data
            } else if ($(this).hasClass("area") || $(this).hasClass("cell")) {
                // skip
            } else {
                select_data = $(this).find("option:selected").val();
                flow_params[flow_param_id] = select_data;
            }
        });
        console.log(flow_params);
        // --- TODO: refactor the mechanism ---
        console.log(flowdatadict[node_id]);
        console.log(flowdatadict[node_id].error_handle);
        if (!flowdatadict[node_id].hasOwnProperty('error_handle')) {
            flowdatadict[node_id]['error_handle'] = {
                error_handle_task_node: "",
                failure_timeout: "10",
                error_type: "Moving",
                retry_limit: "3"
            };
        }
        else {
            flowdatadict[node_id].error_handle['failure_timeout'] = error_handle_dict_values[0];
            flowdatadict[node_id].error_handle['error_type'] = error_handle_dict_values[1];
            flowdatadict[node_id].error_handle['retry_limit'] = error_handle_dict_values[2];
        }
        console.log(artifact_param_list)
        // handle artifact param 
        var artifact_mapping_dict = {};
        artifact_param_list.forEach(function (item, index) {
            let art_id = item["art_val_id"];
            let art_param_key_val = item["art_param_li"];
            if (artifact_mapping_dict.hasOwnProperty(art_id)) {
                artifact_mapping_dict[art_id][art_param_key_val[0]] = art_param_key_val[1];
            } else {
                let tmp_art_param = {};
                tmp_art_param[art_param_key_val[0]] = art_param_key_val[1];
                artifact_mapping_dict[art_id] = tmp_art_param;
            }
        })

        console.log(artifact_mapping_dict)

        for (let [item_key, item_val] of Object.entries(artifact_mapping_dict)) {
            if (flow_params.hasOwnProperty(item_key)) {
                flow_params[item_key] = item_val;
            }
        }
        // var flow_params = flowdatadict[node_id].param;
        // var new_params = {}

        // if (!$.isEmptyObject(flow_params)) {
        // 	for (let item_key of Object.keys(flow_params)) {
        // 		var t_idx = dict_keys.indexOf(item_key);
        // 		new_params[item_key] = dict_values[t_idx];
        // 	};
        // }

        flowdatadict[node_id].param = flow_params;
        // console.log(dict_values);
        console.log(flowdatadict);
    }

    if (prev_data !== JSON.stringify(flowdatadict)) {
        bUnsavedChages_ = true;
    }
}

function generate_select_list(param_key, name, default_value, role_type, artifact_type = null, artifact_service = undefined) {
    var generate_str = '';
    // --- protection ---
    default_value = (typeof default_value === 'undefined') ? '' : default_value;

    switch (name) {
        case "status":
            var selOccupied = '';
            var selEmpty = '';
            // undefined and occupied cases
            selOccupied = (!default_value || default_value === 'occupied') ? 'selected' : '';
            selEmpty = (!default_value || default_value === 'occupied') ? '' : 'selected';

            generate_str = `
        <select class="cus-select input-item">
          <option value="occupied" ${selOccupied}>Occupied</option>
          <option value="empty" ${selEmpty}>Empty</option>
        </select>
      `;
            break;
        // --- status case end ---
        case "goal":
        case "cellid":
            var selMap, selArea, selCell;

            if (default_value && default_value.includes('@')) {
                selMap = default_value.split('@')[0];
                selArea = default_value.split('@')[1];
                selCell = default_value.split('@')[2];

                if (goal_cell_dict.hasOwnProperty(selMap) == false) {
                    selMap = Object.keys(goal_cell_dict)[0];
                    selArea = Object.keys(goal_cell_dict[selMap])[0];
                    selCell = goal_cell_dict[selMap][selArea][0];
                } else {
                    if (goal_cell_dict[selMap].hasOwnProperty(selArea) == false) {
                        selArea = Object.keys(goal_cell_dict[selMap])[0];
                        selCell = goal_cell_dict[selMap][selArea][0];
                    } else {
                        if (goal_cell_dict[selMap][selArea].includes(selCell) == false && selCell != 'auto_empty' && selCell != 'auto_occupied') {
                            selCell = goal_cell_dict[selMap][selArea][0];
                        } else {
                            console.log('Save map param in current fleet');
                        }
                    }
                }
            } else {
                selMap = Object.keys(goal_cell_dict)[0];
                selArea = Object.keys(goal_cell_dict[selMap])[0];
                selCell = goal_cell_dict[selMap][selArea][0];
            }

            var map_options = '';
            var area_options = '';
            var cell_options = '';

            // console.log(goal_cell_dict);
            // --- flesh out the options ---
            var maps = Object.keys(goal_cell_dict);
            maps.forEach(map => {
                var selectedStyle = (map === selMap) ? "selected" : "";
                map_options += `<option value="${map}" ${selectedStyle}>${getMapAlias(map)}</option>`;
            });

            if (goal_cell_dict.hasOwnProperty(selMap)) {
                var areas = Object.keys(goal_cell_dict[selMap]);
                areas.forEach(area => {
                    var selectedStyle = (area === selArea) ? "selected" : "";
                    area_options += `<option value="${area}" ${selectedStyle}>${capitalize(area)}</option>`;
                });

                if (goal_cell_dict[selMap].hasOwnProperty(selArea)) {
                    var cells = goal_cell_dict[selMap][selArea];
                    cells.forEach(cell => {
                        var selectedStyle = (cell === selCell) ? "selected" : "";
                        cell_options += `<option value="${cell}" ${selectedStyle}>${cell}</option>`;
                    });
                }
            }

            generate_str = `
                <select class="cus-select input-item map" id="${param_key}-map">
                    ${map_options}
                </select>
                <select class="cus-select input-item area" id="${param_key}-area" style="margin-top: 10px;">
                    ${area_options}
                </select>
                <select class="cus-select input-item cell" id="${param_key}-cell" style="margin-top: 10px;">
                    ${cell_options}
                </select>
                `;
            break;
        // --- goal case end ---
        // --- cellid case end ---
        case "artifact":
            var artifact_options = '';

            var isAutoSelect = ('auto' === default_value) ? "selected" : "";
            artifact_options += `<option value="auto" ${isAutoSelect}>auto</option>`;

            if (artifact_dict.hasOwnProperty(artifact_type) && artifact_dict[artifact_type].length > 0) {
                artifact_dict[artifact_type].forEach((content_item) => {
                    var selectedStyle = (content_item === default_value) ? "selected" : "";
                    artifact_options += `<option value="${content_item}" ${selectedStyle}>${content_item}</option>`;
                });
            }

            generate_str = `
                <select class="cus-select input-item artifact">
                    ${artifact_options}
                </select>
                `;
            break;
        // --- artifact case end ---
        case "task":
            generate_str = `
      <div class="input-group">
        <input type="text" class="form-control input-item" value="${default_value}">
      </div>
      `;
        case "dock":
            var seldock = '';
            var selundock = '';

            if (default_value === undefined || default_value === 'undefined' || default_value === 'true') {
                seldock = 'selected';
                selundock = '';
            } else {
                seldock = '';
                selundock = 'selected';
            }

            generate_str = `
        <select class="cus-select input-item">
          <option value="true" ${seldock}>dock</option>
          <option value="false" ${selundock}>undock</option>
        </select>
      `;
            break;
        default:
            // console.log(default_value)
            if (param_key.split('_')[0].includes('angle')) {
                if (isNaN(default_value)) {
                    default_value = 'NAN';
                }
            }

            let vali = validate(param_key.split('_')[0], default_value);
            let border_color = vali[0];
            let vali_list = vali[1];
            let needToAddResDict = vali[2];

            if (needToAddResDict) {
                validate_res_dict[param_key] = vali_list;
            }

            if (non_plan_artifact_param_dict[role_type] == undefined) {
                generate_str = `
                            <div class="input-group">
                                <input id="${param_key}" type="text" class="form-control input-item flow-input" value="${default_value}" style="border-color:${border_color};">
                            </div>
                            `;
            } else {
                let serivce_input_str = '';
                // console.log(non_plan_artifact_param_dict[role_type])
                let artifact_service_dict = non_plan_artifact_param_dict[role_type];
                if (artifact_service_dict['artifact_ver'] == '0.0') {
                    generate_str = `
                            <div class="input-group">
                                <input id="${param_key}" type="text" class="form-control input-item" value="${default_value}">
                            </div>
                            `;
                } else {
                    artifact_service_dict["artifact_service_list"].forEach(function (item, index) {

                        for (const [key, value] of Object.entries(item)) {
                            let service_name = key.split('@')[0];
                            let param_name = key.split('@')[1];

                            if (service_name == artifact_service) {
                                let param_data_type = value['data_type'];
                                let param_data_range = value['data_range'];
                                let param_default = undefined;

                                if (default_value[param_name] == undefined) {
                                    param_default = value['default'];
                                } else {
                                    param_default = default_value[param_name];
                                }
                                // console.log(`@@@@@@@ ${param_default}`)

                                let input_type = undefined;
                                let input_option_attr = '';
                                let input_check_class = 'string'

                                if (param_data_type == 'int') {
                                    let min = JSON.parse(param_data_range.replace(':', ','))[0];
                                    let max = JSON.parse(param_data_range.replace(':', ','))[1];
                                    input_type = "number";
                                    input_option_attr = `step='1' min='${min}' max='${max}' pattern="^\d+$"`;
                                    input_check_class = param_data_type;
                                } else if (param_data_type == 'double') {
                                    let min = JSON.parse(param_data_range.replace(':', ','))[0];
                                    let max = JSON.parse(param_data_range.replace(':', ','))[1];
                                    input_type = "number"
                                    input_option_attr = `step='0.01' min='${min}' max='${max}' pattern="(^[0-9]+.[0-9]+$)|(^\d+$)"`;
                                    input_check_class = param_data_type;
                                    console.log(min, max)
                                } else if (param_data_type == 'bool') {
                                    input_check_class = param_data_type;
                                } else {
                                    input_type = "text";
                                }

                                serivce_input_str +=
                                    `<div class="col-12" style="padding: 0;">
                                        <span class="n"><i class="fas fa-info-circle"></i> ${param_name}</span>                                    
                                        <input type="${input_type}" id="${param_name}" class="form-control input-item input_type_${input_check_class} arti_param" value="${param_default}" placeholder="${param_name}" ${input_option_attr}>
                                    </div>`;
                                // console.log(key, value, artifact_service, param_name, serivce_input_str);
                            }
                        }
                    });
                    generate_str = `
                            <div class="input-group">
                                ${serivce_input_str}
                            </div>
                            `;
                }
            }
            break;
    };
    return generate_str;
}

function genCellMappingDict() {
    roleMappinglist.forEach((cell_item) => {
        if (!cell_item.title_name.includes('map@')) { return; }

        var mapName = cell_item.title_name.split('@')[1];
        // --- map level ---
        if (!goal_cell_dict.hasOwnProperty(mapName)) {
            var area_dict = {};
            area_dict[cell_item.role_name] = Object.values(cell_item.title_content);
            goal_cell_dict[mapName] = area_dict;
            return;
        }
        // --- area level ---
        if (!goal_cell_dict[mapName].hasOwnProperty(cell_item.role_name)) {
            goal_cell_dict[mapName][cell_item.role_name] = Object.values(cell_item.title_content);
            return;
        }
        // --- cell level ---
        Object.values(cell_item.title_content).forEach((content_item) => {
            if (goal_cell_dict[mapName][cell_item.role_name].includes(content_item)) { return; }
            goal_cell_dict[mapName][cell_item.role_name].push(content_item);
        });
    });

    // --- add auto- to cell options ---
    for (let paramsObj of Object.values(goal_cell_dict)) {
        for (let opts of Object.values(paramsObj)) {
            if (!opts.includes('auto_occupied')) {
                opts.unshift('auto_occupied')
            }
            if (!opts.includes('auto_empty')) {
                opts.unshift('auto_empty')
            }
        }
    }
}

function get_time_trigger_event(start, end, duration, isrepeat, startdate, enddate) {
    time_start_times = [];
    start_time = start;
    end_time = end;
    repeat_time = duration;
    start_date = startdate;
    end_date = enddate;

    console.log(' ---- get_time_trigger_event ---- ')
    console.log(start_date, start_time, end_date, end_time, repeat_time)

    saved_start_time = new Date(start_date)
    saved_end_time = new Date(end_date)
    saved_start_time.setHours(start_time.split(':')[0])
    saved_start_time.setMinutes(start_time.split(':')[1])
    saved_end_time.setHours(end_time.split(':')[0])
    saved_end_time.setMinutes(end_time.split(':')[1])

    saved_start_time_iso = moment(saved_start_time).format()
    saved_end_time_iso = moment(saved_end_time).format()

    console.log(saved_start_time, saved_end_time)
    console.log(saved_start_time_iso, saved_end_time_iso)

    time_args["start_time"] = saved_start_time_iso;
    time_args["end_time"] = saved_end_time_iso;
    time_args["interval"] = `PT${repeat_time}M`;

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
function processInfoNoNeedShow(node_id) {
    $('.nav2dmapCanvas').each(function (index) {
        let tag = $(this).find('h4').text();

        switch (tag) {
            case 'Dock':
                processMsgInfoRow($(this), undefined);
                break;
            case 'Undock':
                processMsgInfoRow($(this), ['undock']);
                break;
            case 'Rotate':
                let select_id = '';
                $(this).find('.row').each(function (index) {
                    var relat_id = $(this).find('.input-label').attr('name');
                    if (!relat_id.includes('angle_or_goal')) {
                        select_id += `_${relat_id.split('_')[0]}@${relat_id.split('_')[1]}`;
                    }
                })
                let special_setting = generateRotateAngleSelect(node_id, select_id);
                processMsgInfoRow($(this), special_setting);
                break;
            default:
                break;
        }
    });
}

function processMsgInfoRow(card, sp_setting) {
    let hidingAttrs = ['rotation', 'size', 'tag_id'];

    $(card).find('.row').each(function (index) {
        var attr = $(this).find('.input-label').text().split(' ').shift();

        if (sp_setting == undefined) {
            if (hidingAttrs.includes(attr)) {
                $(this).css("display", "none");
            }
        } else {
            let sp_setting_mode = sp_setting[0];

            if (sp_setting_mode == 'rotate') {
                let angle_or_goal = sp_setting[2];
                if (attr == 'angle') {
                    $(sp_setting[1]).insertBefore($(this));
                    if (angle_or_goal == 'goal') {
                        $(this).css("display", "none");
                    } else {
                        $(this).css("display", "");
                    }
                } else {
                    if (angle_or_goal == 'goal') {
                        $(this).css("display", "");
                    } else {
                        $(this).css("display", "none");
                    }
                }
            } else if (sp_setting_mode == 'undock') {
                $(this).css("display", "none");
            }
        }
    });
}

function generateRotateAngleSelect(node_id, select_id) {
    let angle_is_select = '';
    let goal_is_select = 'selected';
    let angle_or_goal = 'goal';
    let check_flow_param_dict = flowdatadict[`node-${node_id}`].param;

    for (const [key, value] of Object.entries(check_flow_param_dict)) {
        if (key.includes('angle_or_goal') && key == `angle_or_goal${select_id}`) {
            if (value == 'angle') {
                angle_is_select = 'selected';
                goal_is_select = '';
                angle_or_goal = 'angle';
            } else {
                angle_is_select = '';
                goal_is_select = 'selected';
                angle_or_goal = 'goal';
            }
        }
    }

    let rotate_select_str = `
    <div class="row" style="padding:0.0rem;margin-top: 10px; ">
        <div class="col-4">
        <div type="text" class="input-label" style="text-align: center;" name="angle_or_goal${select_id}">Mode :</div>
        </div>
        <div class="col-8">
            <select class="cus-select input-item angle_or_goal">
                <option value="goal" ${goal_is_select}>goal</option>
                <option value="angle" ${angle_is_select}>angle</option>
            </select>
        </div>
      </div>`;
    return ['rotate', rotate_select_str, angle_or_goal];
}

const CONST_OPERANDS_ = ['start', 'finish']; // , 'logical_and', 'logical_or', 'logical_delay'
function sidebarEvent(node_name, node_id) {
    console.log(bSidebarCollapse_);
    if (selectingNodeId_ != node_id) {
        bSidebarCollapse_ = true;
    }
    if (!bSidebarCollapse_) { return; }

    selectingNodeId_ = node_id;
    if (!CONST_OPERANDS_.includes(node_name)) {
        bSidebarCollapse_ = !bSidebarCollapse_; // toggle off
        $('#card_content_div').empty();
        get_card_data(node_name, node_id);
        $("#control_sidebar").ControlSidebar('show');
        processInfoNoNeedShow(node_id);
    }
}

// for test flow relate
function genAssignRobotDom(assign_robot, node_id) {
    var assign_robot_select = '';
    let current_fleet = getSelectedFleet();

    if (scannedAgents['total'] > 0) {
        assign_robot_select += `<option value="auto" selected>auto</option>`;
        scannedAgents['robots'].forEach((item) => {
            if (current_fleet == item['fleet_name']) {
                var robot_id = item['robot_id'];
                var selectedStyle = (assign_robot === robot_id) ? "selected" : "";
                assign_robot_select += `<option value="${robot_id}" ${selectedStyle}>${robot_id}</option>`;
            }
        });
    } else {
        assign_robot_select += `<option value="auto" selected>auto</option>`;
    }

    return `
        <div class="col-12" style="padding-left: 0px;">
        <div class="col-4" style="float: left;margin-top: 5px;">
            Robot:
        </div> 
        <div class="col-8" style="float: right;">
            <select class="form-select robot_ass ${node_id}" style="">
            ${assign_robot_select}
            </select>
        </div>
        </div>
        `;
}

function appendConfirmBtnOnSidebar() {
    if (!bSidebarCollapse_) { return; }
    var domTestFlow = `
        <div class="col-12 text-center">
        <button type="button" class="btn btn-primary btn-lg" onclick="test_flow()">Confirm</button>
        </div>
        `;
    $('#card_content_div').append(domTestFlow);
}

function testflowInit() {
    let checkDatalist = Object.keys(flowdatadict).filter(ele => {
        return ele !== 'node-2' && ele !== 'node-3'
    })

    if (checkDatalist.length == 0) {
        $('#card_content_div').empty();
        const template = document.querySelector('#no-need-set-parameter');
        const node = document.importNode(template.content, true);
        $('#card_content_div').append(node);
    } else {
        $('.drawflow-node').each(function (index) {
            if ($(this).hasClass('selected')) {
                var node_id = $(this).attr('id')
                updateDatadict(node_id);
            }
        });
        console.log(flowdatadict)

        sidebar_mode = 'test';
        bSidebarCollapse_ = true;
        $('#card_content_div').empty();

        for (let [node_id, node_val] of Object.entries(flowdatadict)) {
            var role_name = node_val.name;
            var assign_robot = node_val.assigned_robot;
            if (!CONST_OPERANDS_.includes(role_name)) {
                var assign_robot_str = genAssignRobotDom(assign_robot, node_id)
                card_template(`Role: ${role_name}`, role_name, assign_robot_str);
            }
        }

        appendConfirmBtnOnSidebar();
        applyFontSize(getSavedFontSize(), '#control_sidebar');
    }

    $("#control_sidebar").ControlSidebar('show');
}

function test_flow() {
    var dfd = $.Deferred();
    flowUUID = genUuid().replace(/-/g, '');
    assign_robot_flow_name = `${flow_name}_${flowUUID}`;
    fleet_cookie = getSelectedFleet();
    save('test').then(function () {
        setTimeout(function () {
            sendFlowTask(assign_robot_flow_name, false);
            // sendFlow();
        }, 500);
        setTimeout(function () {
            restDeleteFlowData(fleet_cookie, assign_robot_flow_name);
            notificationMsg(0, `${assign_robot_flow_name} is deleted`);
            dfd.resolve();
        }, 1000);
    })
    return dfd.promise();
}

async function sendFlow() {
    const jsonNewFlow = {
        "op": 'create',
        "id": flowUUID,
        "name": assign_robot_flow_name,
        "args": {
            "start_time": "",
            "end_time": "",
            "interval": ""
        }
    };
    var res = await sendFlowRequest(jsonNewFlow);
    if (res['status_code'] === 200) {
        notificationMsg(1, 'Flow Send!');
    } else {
        notificationMsg(3, `Flow Send Fail! ${res['message']}`);
    }
}

// run all task timer
$(document).on('click', '#r-t-flow', async function (e) {
    var flow_name = $('#flow-home-tab').find('.card-header').text().trim();

    if (isTimePassed) {
        alert("Setting time passed, please modify it and try it again.");
    } else {
        console.log(time_start_times)
        console.log(' ------ send flow --------')
        current_timer_flow_id = await sendFlowTask(flow_name, true, time_args);

        time_start_times.forEach((start_time_item) => {

            // send_flow_timer(flow_name, start_time_item);
        });
    }
});

// remove all task timer
$(document).on('click', '#r-t-a-flow', async function (e) {
    // var flow_name = $('#flow-home-tab').find('.card-header').text().trim();
    var timer_flow_id = $(this).attr("data-main_id");
    console.log(timer_flow_id)

    $('#flow-timer-sche').empty();
    // send cancel all
    // send_remove("name", flow_name);
    const res = await fetchDeleteFlow(rmtToken_, timer_flow_id);

    if (res.ok) {
        isRemoveAllflowPressed = true;
    } else {
        const data = res.json();
        notificationMsg(3, `[CONN.] ${data}`);
    }
    // console.log(flow_name)
    // update_info("flow");
});

$(document).on('change', '.drawflow-task-priority-select', function (e) {
    let selected_val = $(this).val();
    let node_id = $(this).parent().closest('.drawflow-node').attr('id').replace('node-', '');
    let component_html = editor.getNodeFromId(node_id).html;
    let component_select = $(component_html).find(".drawflow-task-priority-select");
    component_select.find("option").removeAttr('selected')
    component_select.find(`option[value="${selected_val}"]`).attr('selected', true);
    
    // console.log('ASDAS')
    // console.log($(this))
    // console.log(component_select.html())
    
    const placeholder = document.createElement("div");
    placeholder.innerHTML = component_html;
    let node = placeholder.firstElementChild;
    let current_select = node
    current_select.getElementsByTagName('select')[0].innerHTML = component_select.html()
    // console.log(current_select.outerHTML)
    // console.log(component_html)
    editor.drawflow.drawflow.Home.data[node_id].html = current_select.outerHTML
    
    // console.log(editor.getNodeFromId(node_id).html)
});

// Name mapping table
const dictUiTerms = {
    Sequence: "Sequence",
    Patrol: "Patrol",
    Docking: "Docking",
    Dock: "Dock",
    Undock: "Undock",
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
    Dock: "Dock",
    Undock: "Undock",
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

// ======================
//     Http Requests 
// ======================

function isObject(val) {
    return val instanceof Object;
}

async function fetchArtifactsServiceTypesCb() {
    let artifactObj = {};
    let data = [];

    // --- Native APIs ---
    // data = await restArtifactsServiceTypes();
    // console.log(data);
    // data.forEach(d => {
    // 	let service_ver = undefined;
    // 	let service_list = [];
    // 	if (d.hasOwnProperty('services')) {
    // 		let service_param_dict = d.services;
    // 		for (const [key, value] of Object.entries(service_param_dict)) {
    // 			let service_dict = {};
    // 			if (isObject(value) && d.hasOwnProperty('version') && d["version"] != '0.0') {
    // 				service_ver = d["version"];
    // 				for (const [service_param_key, service_param_value] of Object.entries(value["request"])) {
    // 					service_dict[`${key}@${service_param_key}`] = service_param_value;
    // 				}
    // 			} else {
    // 				service_ver = '0.0';
    // 				service_dict[key] = value;
    // 			}
    // 			service_list.push(service_dict)
    // 		}
    // 	}
    // 	artifactObj[d.type] = { 'artifact_ver': service_ver, 'artifact_service_list': service_list };
    // 	//   console.log(Object.keys(d.services));
    // 	//   artifactObj[d.type] = Object.keys(d.services);
    // })
    // console.log(artifactObj);

    // --- Swarm Core APIs ---
    data = await fetchGetArtifactTypes(rmtToken_);
    // --- data transformation ---
    data.forEach(d => {
        const type = d.type;
        d.services.forEach(s => {
            const service = s.service;
            s.parameter.forEach(p => {
                let artifactServiceList = [];
                artifactServiceList[`${service}@${p.param_name}`] = { data_range: p.data_range, data_type: p.data_type, default: p.default };
                // --- initial case ---
                if (!artifactObj.hasOwnProperty(type)) {
                    // TODO: remove version property, it may be obsolete in the near future.
                    artifactObj[type] = { artifact_ver: '1.0', artifact_service_list: [artifactServiceList] };
                    return;
                }

                // --- iterating case ---
                artifactObj[type].artifact_service_list.push(artifactServiceList);
            });
        })
    });
    // console.log(artifactObj);

    // --- assignment ----
    non_plan_artifact_param_dict = artifactObj;
    console.log(non_plan_artifact_param_dict);
}

function validateEvent() {
    // TODO For validate input val
    $('.input-item').on('keyup', function () {
        // $(this).closest(".row").css("background", "yellow")
        // $(this).parent(".row").css("background", "red")
        // $(this).parents(".row").css("background", "blue")
        let parent_row = $(this).closest('.row');

        let label_name = parent_row.find('.input-label').attr('name');
        let param_node = undefined;
        let role_val_id = undefined;

        if (label_name.includes('@')) {
            param_node = label_name.split('@')[0];
            role_val_id = label_name.split('@')[1];
        } else {
            role_val_id = label_name;
        }

        let param_name = role_val_id.split('_')[0];
        let param_val = $(this).val();
        let valMin = $(this).attr('min');
        let valMax = $(this).attr('max');

        // console.log(parent_row)
        // console.log(param_name)
        // console.log(param_val)

        let vali = validate(param_name, param_val);
        let css_border_color = vali[0];
        let vali_res = vali[1];
        let needToAddResDict = vali[2];

        if (needToAddResDict) {
            validate_res_dict[role_val_id] = vali_res;
        }

        if (PARAM_PROPERTIES[param_name] != undefined) {
            $(this).attr("placeholder", PARAM_PROPERTIES[param_name].placeholder);
            $(this).prop("title", "");
            if (!vali_res) {
                $(this).prop("title", vali[3]);
            }

            $(this).css('border-color', css_border_color)
            $(this).attr("placeholder", PARAM_PROPERTIES[param_name].placeholder);
            $(this).prop("title", "");
            if (!vali_res) {
                $(this).prop("title", vali[3]);
            }

            // --- artifact value range validation ---
            if (!$(this).hasClass("arti_param")) { return; }

            if (!IsValueValid(param_val, valMax, valMin)) {
                let domID = $(this).attr('id');
                console.log(domID);
                // validate_res_dict[domID] = false;
                validate_res_dict[role_val_id] = false;
                $(this).css('border-color', 'red')
                console.log(validate_res_dict)
                return;
            }

            validate_res_dict[role_val_id] = true;
            console.log(validate_res_dict);
            $(this).css('border-color', css_border_color);
        } else {
            console.log("Skip-" + PARAM_PROPERTIES[param_name])
        }
    });
}

function IsValueValid(value, max, min) {
    if (value !== undefined && Number(value) < min) {
        console.log(value)
        return false;
    }
    if (value !== undefined && Number(value) > max) {
        console.log(value)
        return false;
    }

    const reg = new RegExp("^[+-]?[0-9.]+$", 'y');
    // console.log(reg);
    var regRes = reg.test(String(value));
    console.log(regRes);
    return regRes;
}

function validate(param_name, param_val) {
    let css_border_color = '';
    let valid_res = false;
    let valid_msg = '';
    let isParamNeedAddToCheckDict = false;

    if (Object.keys(PARAM_PROPERTIES).includes(param_name)) {
        isParamNeedAddToCheckDict = true;
        ret = validateUserInput(param_val, param_name);
        valid_res = ret.res;
        valid_msg = ret.msg;
        css_border_color = (valid_res) ? 'black' : 'red';
    }

    return [css_border_color, valid_res, isParamNeedAddToCheckDict, valid_msg];
}

const PARAM_PROPERTIES = {
    'duration': {
        validate: validateDuration,
        placeholder: 'range: 0-1e6 (ms)'
    },
    'percentage': {
        validate: validatePercentage,
        placeholder: 'range: 0-100 (%)'
    },
    'angle': {
        validate: validateAngle,
        placeholder: 'range: -180~180 (deg)'
    }
};

function validateDuration(_value) {
    _value = parseInt(_value);
    return _.inRange(_value, 0, 1e6 + 1); // range: 0-1000s
}

function validatePercentage(_value) {
    _value = parseInt(_value);
    return _.inRange(_value, 0, 101); // range: 0-100
}

function validateAngle(_value) {
    _value = parseFloat(_value);
    return _.inRange(_value, -180, 181); // range: -180 to 180 degrees
}

function validateUserInput(input_str, _type) {
    let inputStr = String(input_str);
    let invalidLen = inputStr.replace(/[\+\-\.0-9]/g, '').length;
    console.log(invalidLen);
    if (_type == 'angle' && inputStr == 'NAN') { return { res: true, msg: "" }; }
    if (invalidLen > 0) { return { res: false, msg: "value includes invalid charater" }; }
    let withinRange = PARAM_PROPERTIES[_type].validate(inputStr);
    console.log(withinRange);
    if (!withinRange) { return { res: false, msg: "value is out of valid range" }; }
    return { res: true, msg: "" };
}

function adjustNodeNameInput() {
    let validate_rule_list = [];
    $('.drawflow-node').find('input').each(function () {
        // console.log($(this))
        if (!$(this).hasClass("flow-input")) {
            $(this).addClass("flow-input")
        }
        let parent_id = $(this).parent().parent().parent().parent().attr('id');
        let input_name_id = `input-name-${parent_id}`;
        // console.log(parent_id)
        validate_rule_list.push(new Rule(input_name_id, nodeNameValidation));
        $(this).attr('id', input_name_id);
    });
    validateFlowNameInputEvent(validate_rule_list);
}

function rotateAngleValueValidation(inputVal) {
    const reg1 = /(^[-]?(0?[0-9]{1,2}|1[0-7][0-9]|180)$)|(^NAN$)/;                // (+|-)0-9
    const bRes = reg1.test(inputVal);
    let strMsg = "";
    strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters<br /> range -180 ~ 180, or NAN';
    return { bValid: bRes, strMsg: strMsg }
}

// ------ generic validation rules ------
function nodeNameValidation(inputVal) {
    const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
    const reg2 = /^\./;                                     // cannot start with dot (.)
    const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
    const reg4 = /\s/;                                      // space are not allow
    const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z

    let bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal);
    let strMsg = [];
    const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $ + - = ~ ` # %';
    strMsg.push(msg1);
    const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
    strMsg.push(msg2);
    const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
    strMsg.push(msg3);
    const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
    strMsg.push(msg4);
    const msg5 = (reg5.test(inputVal)) ? '' : '- must end with number or A-Za-z';
    strMsg.push(msg5);
    strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

    if (inputVal.length == 0) {
        bRes = true;
        strMsg = [];
    }

    return { bValid: bRes, strMsg: strMsg }
}