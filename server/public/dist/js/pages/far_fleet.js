/*
 * Author: John Wu
 * Date: 9 June 20,
 * Description:
 **/

// ======================
//        Models
// ======================

let agentAssets_ = AssetsDict["agentAssets"];
let artifactAssets_ = AssetsDict["artifactAssets"];

let allFleetArtifacts_ = [];

let settingsCache_ = {
  fleet: {},
  agent: {},
  old_fleet_name: null
};
var fleetVer = {};

let agent_remove_list = [];
let agent_add_list = [];

let deleteOperation = {
  flows: [],
  tasks: []
};

var deletedRole = [];
var operationTextArr = [];
var roleRemoved = false
let mapAlias_ = "";
let conf_definition = {};
let tmp_agent_conf = undefined;
let tmp_artifact_conf = undefined;
let return_artifact_conf_all = false;
var unsavedChanges = false;

// ======================
//       Load Ready
// ======================
let langTemplateObj_ = {};
$(function () {
  'use strict'
  initRundownAsync();
  $('#modal-lg').on('shown.bs.modal', onModalOpen);
  $('#modal-lg').on('hidden.bs.modal', onModalClose);
  $('#conf-search').on('input', onConfSearchChange);
  pollFleetStatesWithUpdates();
  pollArtifactStatusWithUpdates();
  initAgentSettingValidation();
  validateFleetNameInputEvent();
});

var modalCaller = "";
function onModalOpen(event) {
  // - auto focus whenever modal is shown
  $('#conf-search').focus();

  // - Update modalCaller whenevery '#modal-lg' is triggered
  var obj = $(event.relatedTarget) // Object that triggered the modal
  modalCaller = obj.data('caller') // Get the caller name from data-caller attribute
}

function onModalClose() {
  // - Clear search text after '#modal-lg' is closed
  $("#conf-search").val('');
}

var timerknock;
function onConfSearchChange() {
  if (timerknock) {
    // - Reset the timer to avoid consecutive events triggering the AsyncCb
    clearTimeout(timerknock);
  }

  // - Start a new timer, call the AsyncCb if timed out
  timerknock = setTimeout(
    function () {
      if (modalCaller == 'agent')
        popupAgentCandidatesAsyncCb();
      else if (modalCaller == 'artifact')
        popupArtifactCandidatesAsyncCb();
      else if (modalCaller == 'map')
        popupMapCandidatesAsync();
      else if (modalCaller == 'role')
        popupRoleCandidatesAsyncCb();
    },
    500 // 500 ms timeout
  );
}

let allFleetAgents_ = [];
let allFleetsName_ = [];
const asyncReadFleetConfig = async (fltArray) => {
  for (const flt in fltArray) {
    // var fleetName = fltArray[flt]?.split('.').slice(0, -1).join('.');
    var fleetName = fltArray[flt];
    // var fltSettings = await restGetFleetSettings(fleetName);
    var fltSettings = await fetchGetFleetConfigs(rmtToken_, fleetName);
    allFleetsName_.push(fleetName);
    let fleetSettingSettingDict = {}
    fleetSettingSettingDict[fleetName] = fltSettings;
    allFleetAgents_.push(fleetSettingSettingDict);
    await sleep(200); // sleep in ms
  }
  // console.log(allFleetsName_);
  // console.log(allFleetAgents_);
};

async function rmtTokenCheck() {
  if (rmtToken_ === undefined) {
    try {
      rmtToken_ = await fetchToken();
    } catch (err) {
      // console.error(err);
      // console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

let timeout = 3000;
let robotStatusTimeout; // handle to setTimeout
let refreshMax = 20;    // 10 seconds
let refreshCnt = 0;
async function refreshRobots() {
  refreshCnt++;
  console.log('refreshCnt: ', refreshCnt);
  if (refreshCnt > refreshMax) {
    // It will be called after 20 seconds
    clearTimeout(robotStatusTimeout);
    return;
  }
  robotStatusTimeout = setTimeout(refreshRobots, timeout);
  var selFleet = $('#fleet-select').val();
  await switchFleetContext(selFleet);
}

async function initRundownAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'fleet');
  await switchFleetCallback();

  bindTSTooltipEvts();
  initCreateFleetForm();
  initRenameFleetForm();

  // ------ language switch ------
  await initLanguageSupport();
  let lng = getSetLang() || 'en';
  langTemplateObj_ = await restGetTemplateLang(lng, 'fleet_config');
}

function initCreateFleetForm() {
  $.validator.addMethod("chkDupFleetName", function (value, element) {
    return allFleetsName_.indexOf(value) == -1;
  });

  $('#create-fleet-form').validate({
    rules: {
      fleetFileName: {
        required: true,
        chkDupFleetName: true,
        chkInputVal: true
      }
    },
    messages: {
      fleetFileName: {
        required: 'Please enter fleet name',
        chkDupFleetName: 'Fleet name already exists',
        chkInputVal: 'Fleet Name include invalid characters'
      }
    },
    errorElement: 'div',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.after(error);
    }
  });

  $('#create-fleet-form').on('submit', createFleet);
}

function initRenameFleetForm() {
  $.validator.addMethod("chkUnchangedFleetName", function (value, element) {
    let fleetName = $('#fleet-name').val();
    let new_fleetname = value;
    return (new_fleetname !== fleetName) ? true : false;
  });

  $('#rename-fleet-form').validate({
    rules: {
      newFleetName: {
        required: true,
        chkUnchangedFleetName: true,
        chkDupFleetName: true,
        chkInputVal: true
      }
    },
    messages: {
      newFleetName: {
        required: 'Please enter fleet name',
        chkUnchangedFleetName: 'Fleet name not changed',
        chkDupFleetName: 'Fleet name already exists',
        chkInputVal: 'Fleet Name include invalid characters'
      }
    },
    errorElement: 'div',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.after(error);
    }
  });

  $('#rename-fleet-form').on('submit', renameFleet);
}

function initAgentSettingValidation() {
  $.validator.addMethod("chkValidFormat", function (value, element) {
    $(element).parent().parent().find('div.error').remove();
    let param = value;
    let paramKey = $(element).attr('name');
    var type = conf_definition[`${paramKey}.valid_value.data_type`];
    var range = conf_definition[`${paramKey}.valid_value.data_range`].replace(/[[\]]/g, '');
    if (range.split(":").length <= 1) return true;

    switch (type) {
      case 'double': {
        if (isEmptyString(param)) return true;
        var minVal = parseFloat(range.split(":")[0]);
        var maxVal = parseFloat(range.split(":")[1]);
        var doubleVal = parseFloat(param);
        if (isNaN(minVal) || isNaN(maxVal)) return true;
        if (!$.isNumeric(value)) {
          $.validator.messages.chkValidFormat = "value is not numeric";
          return false;
        }
        if (!$.isNumeric(doubleVal)) {
          $.validator.messages.chkValidFormat = "value type should be " + type;
          return false;
        }
        if (doubleVal > maxVal || doubleVal < minVal) {
          $.validator.messages.chkValidFormat = "value must be in the range of " + range.replace(':', '~');
          return false;
        }
        return true;
      }
      case 'int': {
        if (isEmptyString(param)) return true;
        var minVal = parseInt(range.split(":")[0]);
        var maxVal = parseInt(range.split(":")[1]);
        var intVal = parseInt(param);
        if (isNaN(minVal) || isNaN(maxVal)) return true;
        if (!$.isNumeric(value)) {
          $.validator.messages.chkValidFormat = "value is not numeric";
          return false;
        }
        if (!Number.isInteger(Number(value))) {
          $.validator.messages.chkValidFormat = "value type should be " + type;
          return false;
        }
        if (intVal > maxVal || intVal < minVal) {
          $.validator.messages.chkValidFormat = "value must be in the range of " + range.replace(':', '~');
          return false;
        }
        return true;
      }
      case 'string': {
        var minLen = range.split(":")[0];
        var maxLen = range.split(":")[1];
        var strLen = param.length;
        // console.log(value, paramKey)
        if (strLen < minLen) {
          $.validator.messages.chkValidFormat = `value must have at least ${minLen} characters`;
          return false;
        } else if (strLen > maxLen) {
          $.validator.messages.chkValidFormat = `value is greater than ${maxLen} characters`;
          return false;
        }

        // for footprint
        if (paramKey == "planning.footprint") {
          let non_numeric_list = [];
          try {
            let foorprint_obj = JSON.parse(value);
            if (foorprint_obj instanceof Array === false) {
              $.validator.messages.chkValidFormat = 'value is not a correct format';
              return false;
            }

            console.log(foorprint_obj.length);

            if (foorprint_obj.length != 4) {
              $.validator.messages.chkValidFormat = 'value is not a correct format';
              return false;
            } else {
              for (var i in foorprint_obj) {
                let footprint_val = foorprint_obj[i];
                console.log(footprint_val);

                if (footprint_val instanceof Array === false) {
                  $.validator.messages.chkValidFormat = 'value is not a correct format';
                  return false;
                }

                if (footprint_val.length != 2) {
                  $.validator.messages.chkValidFormat = 'value is not a correct format';

                  return false;
                } else {
                  footprint_val.forEach(v => {
                    // console.log(v)
                    // console.log($.isNumeric(v))
                    if (!$.isNumeric(v)) {
                      non_numeric_list.push(v)
                    }
                  });
                }
              }

              if (non_numeric_list.length != 0) {
                $.validator.messages.chkValidFormat = "value is not numeric";
                return false;
              }
            }
          } catch (e) {
            $.validator.messages.chkValidFormat = 'value is not a correct format';
            return false;
          }

        }

        return true;
      }
      default: {
        return true;
      }
    }

  }, $.validator.messages.chkValidFormat);

  $('#save-settings-form').validate({
    ignore: "ui-tabs-hide",
    errorElement: 'div',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.parent().after(error);
    }
  });

  $.validator.addClassRules({
    'param-check': {
      chkValidFormat: true
    }
  });
}

function updateConfDefaultFileName(prefixID_) {
  var today = new Date();
  var todayString = today.toISOString().slice(0, 10).replace(/-/g, "");
  var defaultFileName = `conf-${todayString}`;
  defaultFileName = prefixID_ + '-' + defaultFileName;
  $('#conf-filename').val(defaultFileName);
}

function updateFleetNameTextInput(fleet_name) {
  $('#fleet-name').val(fleet_name);
}

function showRenameFleetContent() {
  // $('#rename-fleet-div').css('display', 'block');
}

function hideRenameFleetContent() {
  // $('#rename-fleet-div').css('display', 'none');
}

function toggleRenameFleetContent() {
  // console.log(settingsCache_.fleet);
  let fltSettingsObj = settingsCache_.fleet;
  if (isEmptyObjValue(fltSettingsObj)) {
    var currFleet = Object.keys(settingsCache_.fleet)[0];
    // console.log(currFleet);
    updateFleetNameTextInput(currFleet);
    showRenameFleetContent();
  } else {
    hideRenameFleetContent();
  }
}

function isEmptyObjValue(object) {
  return Object.values(object).every(v => v && typeof v === 'object'
    ? isEmptyObjValue(v)
    : v.length === 0 || v === null || v === ''
  );
}

async function switchFleetContext(_selFleet) {

  // --- fetch fleet_settings ---
  let fltSettings;
  try {
    console.log(_selFleet);
    fltSettings = await restGetFleetSettings(_selFleet);
    // let res = await fetchGetFleetConfigs(rmtToken_, _selFleet);
    // if (res.ok) {
    // console.log(res.json())
    // fltSettings = res.json();
    // }
  } catch (err) {
    fltSettings = []
    console.error(err);
  }

  console.log(fltSettings);
  settingsCache_.fleet = fltSettings;

  // --- fetch artifacts of agent in fleet ---
  let scannedAgents;
  try {
    await rmtTokenCheck();
    scannedAgents = await fetchScanRobots2(rmtToken_);
  } catch (err) {
    console.error(err);
  }

  // console.log(scannedAgents.robots);
  scannedAgents = scannedAgents.robots;
  // console.log(scannedAgents);
  // var flt = Object.keys(settingsCache_.fleet)[0];
  // settingsCache_['scan'] = scannedAgents;
  // console.log(fltAgents);

  // // --- previous version ---
  // let fltAgents = settingsCache_.fleet[_selFleet].agents;
  // let agentArtifacts = [];
  // if (typeof scannedAgents !== 'undefined' && scannedAgents.length > 0) {
  //   scannedAgents.forEach(sa => {
  //     if (fltAgents.includes(sa.robot_id)) {
  //       var robotId = sa.robot_id;
  //       var arrArtifacts = sa.artifacts.split(';');
  //       arrArtifacts = arrArtifacts.filter(aa => aa !== "");
  //       var artifactObj = {};
  //       artifactObj[robotId] = arrArtifacts;
  //       agentArtifacts.push(artifactObj);
  //     }
  //   });
  // }

  // settingsCache_.fleet[_selFleet].artifacts['agent'] = agentArtifacts;
  // console.log(settingsCache_);

  // --- fetch artifacts of agent in fleet ---
  let scannedArtifacts = {};
  let artifactsProperty = [];
  try {
    await rmtTokenCheck();
    scannedArtifacts = await fetchScanArtifacts2(rmtToken_);
    artifactsProperty = await fetchGetArtifactsProperty(rmtToken_);
  } catch (err) {
    console.error(err);
  }

  scannedArtifacts = scannedArtifacts.artifacts;
  updateWholePageView(settingsCache_.fleet[_selFleet], scannedAgents, scannedArtifacts, artifactsProperty);
}

async function loadFleetSettings() {
  var selFleet = $('#fleet-select').val();
  await switchFleetContext(selFleet);

  toggleRenameFleetContent();

  // --- fetch the software list ---
  let swList;
  try {
    await rmtTokenCheck();
    swList = await fetchGetSwVerList(rmtToken_);
    // console.log(swList);
  } catch (err) {
    console.error(err);
  }

  // --- get the array of agents in the fleet ---
  fleetVer = {};
  let fltAgents = [];
  if (settingsCache_?.fleet[selFleet]?.agents) {
    fltAgents = settingsCache_.fleet[selFleet].agents;
  }
  // console.log(fltAgents);
  try {
    // TODO: improve the way to generate string.
    var agentStr = '[';
    if (fltAgents.length > 0) {
      if (!$.isArray(fltAgents)) {
        console.error('❌ Agents format from fleet file is not an array');
      }
      fltAgents.forEach(a => {
        agentStr += `"${a}",`;
      });
      agentStr = agentStr.slice(0, -1);
    }
    agentStr += ']';
    console.log(agentStr);

    await rmtTokenCheck();
    let res = await fetchPostAgentSwVer(rmtToken_, agentStr);
    // console.log(fleetVer.agents);
    if (res.ok) {
      fleetVer.agents = await res.json();
    } else {
      fleetVer.agents = {
        "agent_sw_version": []
      };
    }
  } catch (err) {
    console.error(err);
  }
  console.log(fleetVer.agents);

  updateFirmwareView(swList, fleetVer);

  updateContentColorTheme();
}

function updateContentColorTheme() {
  // toggle color theme again after DOM elements are all generated
  var isDarkMode = getSavedTheme() === 'dark';
  toggleContentDarkTheme(isDarkMode);
}

function updateFirmwareView(_verList, _fltVer) {
  console.log(_verList);

  // --- current version number ---
  var currAgentVer = _fltVer.agents.agent_sw_version;

  // --- latest version number ---
  var agentVerList = _verList.agent_sw_list.map(v => v.sw_version)
  agentVerList.sort();

  if (_verList.agent_sw_list.length > 0) {
    $('#core-firmware-ver').empty();

    _verList.agent_sw_list.forEach(function (cv) {
      var ver = cv.sw_version;
      var selAttr = (ver === currAgentVer) ? 'selected' : '';
      $('#core-firmware-ver').append(`<option value='${ver}' ${selAttr}>ver. ${ver}</option>`);
    });
  }
}

async function updateWholePageView(_data, _agentScan = null, _artifactScan = null, _artifactProperty = []) {
  console.log(_data)

  // --- [protection] ---
  if (_data == undefined) { return; }
  if (Object.keys(_data).length === 0) { return; }

  // 1. --- update agents ---
  console.log(_data);
  loadFleetAgents(_data, _agentScan)

  // 2. --- update artifacts ---
  allFleetArtifacts_ = await loadFleetArtifacts(_data, _artifactScan, _artifactProperty);
  console.log(allFleetArtifacts_);

  // 3. --- update maps ---
  loadFleetMaps(_data.maps).then(function () { applyFontSize(getSavedFontSize(), '#maps-deck'); })

  // 4. --- update roles ---
  loadFleetRoles(_data.roles);
}

function loadFleetAgents(_data, _scanAgents) {
  console.log(_data);
  console.log(_scanAgents);

  // --- clean the panel first ---
  let $agentDeck = $('#agents-deck');
  $agentDeck.empty();

  // --- [protection] on condition, no agents in a fleet ---
  let agents = _data.agents;
  if (agents === null) {
    return;
  }

  let fleetAgents = [];

  // --- updated version (previous ver check legacy) ---
  if (agents.length == 0) { return; }
  console.log(agents);

  let selFleet = $('#fleet-select').val();
  agents.forEach((agentId) => {
    let arr = _scanAgents.filter(sa => sa.robot_id === agentId);
    let defaultAgent = {};
    defaultAgent["robot_id"] = agentId;
    defaultAgent["robot_name"] = (arr.length) ? arr[0].robot_name : "none";
    defaultAgent["model"] = (arr.length) ? arr[0].model : "none";
    defaultAgent["mode"] = (arr.length) ? arr[0].mode : "not available";
    defaultAgent["sw_version"] = (arr.length) ? arr[0].sw_version : "0.0.0";
    defaultAgent["ip"] = (arr.length) ? arr[0].ip : "127.0.0.1";
    defaultAgent["mac"] = (arr.length) ? arr[0].mac : "xx:xx:xx:xx:xx:xx";
    defaultAgent["fleet"] = (arr.length) ? arr[0].fleet_name : selFleet;

    fleetAgents.push(defaultAgent);
  })

  // console.log(fleetAgents);
  fleetAgents.forEach((_agent) => {
    if (agents.includes(_agent.robot_id)) {
      console.log(_agent);
      let node = createAgentMemberView(_agent);
      $agentDeck.append(node.agent);
    }
  });

  applyFontSize(getSavedFontSize(), '#agents-deck');
}

async function loadFleetArtifacts(_data, _scanArtifacts = null, _artifactProperty = []) {
  // console.log(_data);
  // console.log(_scanArtifacts);
  let allArtifacts = [];

  // --- clean the panel first ---
  var $artifactDeck = $('#artifacts-deck');
  $artifactDeck.empty();

  // --- [protection] on condition, no agents in a fleet ---
  var agents = _data.agents;
  if (agents === null) { return; }

  var fleetName = $('#fleet-select').val();
  var fleetArtifacts = [];

  console.log(_data);
  try {
    var artifacts = _data.artifacts.external;
    console.log(artifacts);

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
      let artifactsObj = [];
      if (_scanArtifacts !== null) {
        artifactsObj = _scanArtifacts.filter(sa => sa.artifact_id === defaultArtifact.artifact_id);
      }

      // --- data re-assignment by scanned data ---
      // console.log(artifactsObj);
      if (artifactsObj.length) {
        defaultArtifact["artifact_id"] = artifactsObj[0].artifact_id;
        defaultArtifact["artifact_name"] = artifactsObj[0].artifact_name;
        defaultArtifact["wrapper_id"] = artifactsObj[0].wrapper_id;
        defaultArtifact["sw_version"] = artifactsObj[0].sw_version;
        defaultArtifact["mode"] = "active";
      }

      // --- data fusion with artifact property ---
      let targetArtifact = _artifactProperty.find(ap => ap.artifact_id === defaultArtifact["artifact_id"]);
      console.log(targetArtifact);
      if (targetArtifact) {
        defaultArtifact["avatar"] = targetArtifact.avatar;
        defaultArtifact["thumbnail"] = targetArtifact.thumbnail;
      }

      fleetArtifacts.push(defaultArtifact);
    });
    console.log(fleetArtifacts);

    fleetArtifacts.forEach((_artifact) => {
      var node = createArtifactMemberView(_artifact);
      // $artifactDeck.prepend(node);
      $artifactDeck.append(node);
    });
    // allArtifacts = [...fleetArtifacts];
    allArtifacts.push(...fleetArtifacts);
  } catch (err) {
    console.warn(err);
    console.warn('No External Artifacts!');
  }

  try {
    // --- load artifacts of agent in fleet ---
    var agentArtifacts = _data.artifacts.agent;
    var arrArtifacts = [];
    agentArtifacts.forEach((artifactObj) => {
      var flt = Object.keys(artifactObj)[0];
      var artifactArr = artifactObj[flt];
      console.log(artifactArr);
      artifactArr.forEach(artStr => {
        let defaultArtifact = {};
        let artifactInfo = artStr.split('@');
        defaultArtifact["artifact_id"] = artifactInfo[1] || "";
        defaultArtifact["artifact_name"] = artifactInfo[1] || "none";
        defaultArtifact["type"] = artifactInfo[0] || "none";
        defaultArtifact["fleet"] = fleetName || "none";
        defaultArtifact["ip"] = "127.0.0.1";
        defaultArtifact["mac"] = "xx:xx:xx:xx:xx:xx";

        let artifactsObj = []
        if (_scanArtifacts !== null) {
          artifactsObj = _scanArtifacts.filter(sa => sa.artifact_id === defaultArtifact.artifact_id);
        }
        // console.log(artifactsObj);

        // --- data re-assignment by scanned data ---
        if (artifactsObj.length) {
          defaultArtifact["artifact_id"] = artifactsObj[0].artifact_id;
          defaultArtifact["artifact_name"] = artifactsObj[0].artifact_name;
          defaultArtifact["wrapper_id"] = artifactsObj[0].wrapper_id;
          defaultArtifact["sw_version"] = artifactsObj[0].sw_version;
          defaultArtifact["mode"] = "active";
        }

        // --- data fusion with artifact property ---
        let targetArtifact = _artifactProperty.find(ap => ap.artifact_id === defaultArtifact["artifact_id"]);
        console.log(targetArtifact);
        if (targetArtifact) {
          defaultArtifact["avatar"] = targetArtifact.avatar;
          defaultArtifact["thumbnail"] = targetArtifact.thumbnail;
        }

        arrArtifacts.push(defaultArtifact);
      });

    });
    console.log(arrArtifacts);

    arrArtifacts.forEach((_artifact) => {
      let node = createArtifactMemberView(_artifact, false);
      // $artifactDeck.prepend(node);
      $artifactDeck.append(node);
    });

    allArtifacts.push(...arrArtifacts);
  } catch (err) {
    console.warn(err);
    console.warn('No Agent Artifacts!');
  }
  applyFontSize(getSavedFontSize(), '#artifacts-deck');
  return allArtifacts;
}

async function loadFleetMaps(_data) {
  var $mapDeck = $('#maps-deck');
  $mapDeck.empty();

  var data = await restGetAllMapData();
  var objArray = JSON.parse(data);
  mapAlias_ = objArray;
  const insMaps = _.intersectionWith(mapAlias_, _data, (o, mapname) => o.name == mapname);
  console.log('maps in db: ', insMaps);

  for (let _map of insMaps) {
    var node = await createMapMemberView({
      map_alias: _map.alias_name,
      map_name: _map.name
    });
    $mapDeck.append(node);
  }

  var dbData = mapAlias_.map(function (item) {
    return item.name;
  });
  var diffMaps = _.differenceWith(_data, dbData);
  if (diffMaps.length === 0) return;
  console.log('fleet file maps: ' + _data);
  console.log('maps not in db: ' + diffMaps);

  for (let _map of diffMaps) {
    var node = await createMapMemberView({
      map_alias: _map,
      map_name: null
    });
    $mapDeck.append(node);
  }
}

function loadFleetRoles(_data) {
  var $roleDeck = $('#roles-deck');
  $roleDeck.empty();

  _data.forEach((_role) => {
    console.log(_role);
    // loadRoleInFleet(_role);
    var node = createRoleMemberView({
      role_name: _role
    });
    $roleDeck.append(node);
  });

  applyFontSize(getSavedFontSize(), '#roles-deck');
}


// =======================
//     Event Callbacks
// =======================
async function updateArtifactSidebarAsyncCb(_artifact) {
  // Hide hyperlink button when updatArtifact
  var hyperCol = document.getElementById('hyperlink-amr-col');
  hyperCol.style.display = 'none';

  // https://farobottech.atlassian.net/jira/software/projects/FAR/boards/2?assignee=624ea4cef6a269006961929e&selectedIssue=FAR-1385
  let name = document.getElementById('agent-artifact-name');
  // name.textContent = 'Artifact Name:';
  name.querySelector("[data-i18n='fleet_config.sidebar.lbl_AgntArtfName']").textContent = langTemplateObj_.artf.lbl_SidebarName;

  // console.log('updateArtifactSidebarAsyncCb');
  // console.log(_artifact);
  openSidebar = !openSidebar;
  hideInitPoseConfirmButton();
  showSaveArtifactButton();
  removeConfActionButtons();

  // --- update agent profile on sidebar ---
  updateArtifactProfileView(_artifact);

  // --- clear previous pane ---
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  removeAllChildNodes(navTab);
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  removeAllChildNodes(tabContent);

  //--- add reboot button ---
  var rebootNode = document.querySelector('.sb-agent-reboot');

  rebootNode.classList.remove('re_agent');
  rebootNode.classList.remove('re_artifact');
  rebootNode.classList.add('re_artifact');

  // --- update agent settings on sidebar ---
  let data;
  try {
    console.log(_artifact.artifact_id);
    await rmtTokenCheck();
    data = await fetchArtifactSettings(rmtToken_, _artifact.artifact_id);
    let artifact_conf = JSON.parse(data["artifact_settings"][0]["artifact_conf"].replace(/'/g, '"'));

    updateArtifactConfActionView(artifact_conf);

    if (artifact_conf.hasOwnProperty("configuration")) {
      tmp_artifact_conf = artifact_conf;
      return_artifact_conf_all = true;
      artifact_conf = artifact_conf["configuration"];
    }
    let flatten_artifact_conf = await restGetFlattenArtifactConf(artifact_conf, false);
    data["artifact_settings"][0]["artifact_conf"] = flatten_artifact_conf;

    if (data.hasOwnProperty('detail')) {
      alert(data.detail);
      return;
    }
    settingsCache_.artifact = data;
  } catch (err) {
    console.error(err);
  }

  updateArtifactParamView(data);

  // upgrade list button
  var btnHolder = $('#core-firmware');
  btnHolder.empty();
  var btnArtifactSwUpdate = document.createElement('button');
  btnArtifactSwUpdate.innerHTML = ' ';
  btnArtifactSwUpdate.setAttribute('class', 'fas fa-bullhorn');
  btnArtifactSwUpdate.addEventListener('click', updateArtifactSw.bind(this, _artifact.artifact_id));
  btnHolder.append(btnArtifactSwUpdate);

  // --- update save button binding ---
  var placeholder = $('#save-placeholder');
  placeholder.empty();
  var saveBtn = document.createElement('button');
  saveBtn.setAttribute('class', 'btn btn-primary');
  saveBtn.innerHTML = "Save";
  saveBtn.addEventListener('click', function (event) {
    saveArtifactSettingsCb(event, _artifact.artifact_id)
  }
  );

  placeholder.append(saveBtn);

  applyFontSize(getSavedFontSize(), '.os-content');
}

async function saveArtifactSettingsCb(e, _artifactId) {
  e.preventDefault();
  $("#save-settings-form").validate({ debug: true });
  if (!$("#save-settings-form").valid()) { return; }

  var tabContent = document.querySelector('#sidebar-param-card > .card-header > .nav-tabs');
  var tabs = tabContent.childNodes;
  // console.log(tabs.length);
  var tabsId = [];
  tabs.forEach((tab) => {
    // console.log(tab.firstChild.href);
    var hrefNode = tab.firstChild.href;
    var arr = hrefNode.split('#');
    tabsId.push(arr[1]);
  });
  // console.log(tabsId);
  var jsonObj = {};
  tabsId.forEach((id) => {
    var ctgObj = {};
    var leftNode = document.querySelector('#' + id + '-left-col');
    // console.log(leftNode);
    var paramRow = leftNode.querySelectorAll('.row');
    paramRow.forEach((pr) => {
      var name = pr.querySelector('.param-label').innerHTML;
      var value = pr.querySelector('.form-control').value;
      let obj_id = pr.querySelector('.form-control').id;

      if (obj_id === 'network.bridge_mode') {
        value = value === 'on' ? 'zenoh' : 'dds';
      }

      settingsCache_.artifact["artifact_settings"][0]["artifact_conf"][obj_id] = value;
      ctgObj[name] = value;
    });

    var rightNode = document.querySelector('#' + id + '-right-col');
    // console.log(leftNode);
    paramRow = rightNode.querySelectorAll('.row');
    paramRow.forEach((pr) => {
      var name = pr.querySelector('.param-label').innerHTML;
      var value = pr.querySelector('.form-control').value;
      let obj_id = pr.querySelector('.form-control').id;

      if (obj_id === 'network.bridge_mode') {
        value = value === 'on' ? 'zenoh' : 'dds';
      }

      settingsCache_.artifact["artifact_settings"][0]["artifact_conf"][obj_id] = value;
      ctgObj[name] = value;
    });
    jsonObj[id] = ctgObj;

  });
  // console.log(jsonObj);
  // console.log(settingsCache_.artifact["artifact_settings"][0]["artifact_conf"])
  let config_unflatten = await restGetUnFlattenArtifactConf(settingsCache_.artifact["artifact_settings"][0]["artifact_conf"], false);
  // console.log(config_unflatten)

  if (return_artifact_conf_all) {
    tmp_artifact_conf["configuration"] = config_unflatten;
    config_unflatten = tmp_artifact_conf;
  }

  // --- JSON to string process ---
  var servicesStr = JSON.stringify(config_unflatten);
  // console.log(servicesStr);
  servicesStr = servicesStr.replace(/true/g, 'True').replace(/false/g, 'False');
  servicesStr = servicesStr.replace(/"/g, "'");
  console.log(servicesStr);

  displayOverlay('Artifact Setting Updating...'); // lock the screen
  try {
    // --- PUT Settings to Backend
    await rmtTokenCheck();
    let response = await fetchPutArtifactSettings(rmtToken_, _artifactId, servicesStr);
    console.log(response);
    // let data = await response.json();
    // console.log(data);
    // if (response.status === 200) {
    notificationMsg(0, 'Set Artifact Settings on Success!');
    // } else {
    //   notificationMsg(3, `[Error] ${data}`);
    // }

    return_artifact_conf_all = false;
    tmp_artifact_conf = undefined;
  } catch (err) {
    console.error(err);
  }
  removeOverlay();
}

function updateArtifactProfileView(_artifact) {
  // console.log(_artifact);
  let imgSrc = "";
  if (_artifact.hasOwnProperty('avatar')) {
    imgSrc = `data:image/png;base64, ${_artifact['avatar']}`;
  } else {
    // imgSrc = '/images/' + artifactAssets_[_artifact.type].thumbnail;
    imgSrc = artifactAssets_[_artifact.type.toLowerCase()].thumbnail;
  }
  $('#sb-agent-icon').attr('src', imgSrc);

  // $('#sb-agent-name').val(_artifact.artifact_name);
  avoidNoneVal($('#sb-agent-name'), _artifact.artifact_name, _artifact.artifact_id);
  $('#sb-agent-id').val(_artifact.artifact_id);
  $('#sb-agent-mode').val(_artifact.mode);
  $('#sb-agent-model').val(_artifact.type);
  $('#sb-agent-firmware').val(_artifact.sw_version);
}

function updateArtifactParamView(_data) {
  $('#sidebar-param-card .nav-tabs').empty();
  $('#sidebar-param-card .tab-content').empty();
  var configData = JSON.stringify(_data);
  // console.log(configData);

  // -- visualize data --
  updateArtifactSidebarParamPane(configData);

  // --- update artifact settings on sidebar ---
  unlockArtifactParamView();
}

function updateArtifactSidebarParamPane(_data) {
  var conf = _data;
  var servicesObj = JSON.parse(conf);
  servicesObj = servicesObj.artifact_settings[0].artifact_conf;
  // console.log(servicesObj);

  // servicesObj = servicesObj.replace(/true/g, 'True').replace(/false/g, 'False');
  // servicesObj = servicesObj.replace(/'/g, '"');
  // servicesObj = JSON.parse(servicesObj);

  // --- fetch artifact WMS data ---
  if (servicesObj.hasOwnProperty('wms')) {
    var artifactWmsCells = servicesObj.wms;
    console.log(artifactWmsCells);
    var sbParamCard = document.getElementById('sidebar-param-card');
    var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
    removeAllChildNodes(navTab);
    var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
    removeAllChildNodes(tabContent);

    artifactWmsCells.forEach(awc => {
      // --- append tabs ---
      var liEl = document.createElement("li");
      liEl.setAttribute("class", "nav-item");
      var aEl = document.createElement("a");
      var classTag = "nav-link";
      aEl.setAttribute("class", classTag);
      aEl.setAttribute("data-toggle", "tab");
      aEl.setAttribute("font-size", "1.5rem");
      aEl.setAttribute("href", `#${awc.area}`);
      aEl.textContent = awc.area;
      liEl.append(aEl);

      navTab.append(liEl);

      // --- append panes ---
      const template = document.querySelector('#params-tab-pane');
      const node = document.importNode(template.content, true);

      var paneNode = node.querySelector('.tab-pane');
      paneNode.id = awc.area;

      var leftColNode = node.querySelector('.left-col');
      leftColNode.id = `${awc.area}-left-col`;

      var rightColNode = node.querySelector('.right-col');
      rightColNode.id = `${awc.area}-right-col`;

      tabContent.append(node);

      // --- append params in the pane ---
      var count = 0;
      console.log(awc);
      for (k in awc) {
        const template = document.querySelector('#param-group-row');
        const node = document.importNode(template.content, true);

        var nameNode = node.querySelector('.param-label');
        nameNode.innerHTML = `<span>${k}</span>`;
        var valNode = node.querySelector('.form-control');
        valNode.value = awc[k];
        var btnEditNode = node.querySelector('.param-edit');
        var inputGroupNode = node.querySelector('.input-group');
        btnEditNode.addEventListener('click', editButtonSwitch.bind(inputGroupNode));

        var tag = (count % 2) ? `#${awc.area}-right-col` : `#${awc.area}-left-col`;
        var $paramCol = $(tag);
        console.log($paramCol);
        $paramCol.append(node);

        count++;
      }
    });
  }

  // --- fetch artifact services data ---
  console.log(servicesObj);
  // var artifactServices = servicesObj.services;
  // console.log(artifactServices);
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  // removeAllChildNodes(tabContent);

  var pCount = 0
  for (const [key, value] of Object.entries(servicesObj)) {
    // console.log(`${key}: ${value}`);
    let tab_name = undefined;
    let input_label = undefined;

    if (key.match(/\./g) != null) {
      let key_split_list = key.split('.');
      tab_name = key_split_list[0];

      if (key_split_list.length <= 1) {
        input_label = key_split_list[key_split_list.length - 1];
      } else {
        input_label = `${key_split_list[key_split_list.length - 2]}_${key_split_list[key_split_list.length - 1]}`
      }

    } else {
      tab_name = key;
      input_label = key;
    }

    // console.log(`${tab_name}: ${input_label}`);

    var liEl = document.createElement("li");
    liEl.setAttribute("class", "nav-item");
    var aEl = document.createElement("a");
    var classTag = "nav-link";
    aEl.setAttribute("class", classTag);
    aEl.setAttribute("data-toggle", "tab");
    aEl.setAttribute("font-size", "1.5rem");
    aEl.setAttribute("href", `#${tab_name}`);
    aEl.textContent = tab_name;
    liEl.append(aEl);

    if ($(`#sidebar-param-card .nav-link:contains(${tab_name})`).length == 0) {
      navTab.append(liEl);
    }

    const template = document.querySelector('#params-tab-pane');
    const node = document.importNode(template.content, true);

    var paneNode = node.querySelector('.tab-pane');
    paneNode.id = tab_name;

    var leftColNode = node.querySelector('.left-col');
    leftColNode.id = `${tab_name}-left-col`;

    var rightColNode = node.querySelector('.right-col');
    rightColNode.id = `${tab_name}-right-col`;

    if ($(`[id=${tab_name}]`).length == 0) {
      tabContent.append(node);
      pCount = 0;
    }

    const row_template = document.querySelector('#param-group-row');
    const row_node = document.importNode(row_template.content, true);

    var nameNode = row_node.querySelector('.param-label');
    nameNode.innerHTML = `<span>${input_label}</span>`;
    var valNode = row_node.querySelector('.form-control');
    valNode.id = key;
    valNode.value = value;
    var btnEditNode = row_node.querySelector('.param-edit');
    var inputGroupNode = row_node.querySelector('.input-group');

    if (key === 'network.bridge_mode') {
      const inputGroupAppend = row_node.querySelector('.input-group-append');
      const btn = document.createElement('button');
      btn.innerHTML = `Set ${value === 'zenoh' ? 'Off' : 'On'}`;
      btn.classList.add('btn', 'btn-sm', 'btn-secondary');
      inputGroupAppend.replaceWith(btn);

      const inputGroupNode = row_node.querySelector('.input-group');
      btn.addEventListener('click', bridgeModeButtonSwitch.bind(inputGroupNode));

      valNode.value = value === 'zenoh' ? 'on' : 'off';
    }

    // btnEditNode.addEventListener('click', editButtonSwitch.bind(inputGroupNode));
    // btnEditNode.parentNode.removeChild(btnEditNode);

    switch (key) {
      default:
        // if (!isEditMode) { break; }

        if (typeof value == "boolean" || value == "false" || value == "true") {
          var stringVal = undefined;
          if (typeof value == "boolean") {
            stringVal = (!value).toString();
          } else {
            if (value == "false") {
              stringVal = "true";
            } else {
              stringVal = "false";
            }
          }
          var boolString = stringVal.charAt(0).toUpperCase() + stringVal.slice(1);

          row_node.querySelector('.input-group-text').remove();
          var inputGroupAppend = row_node.querySelector('.input-group-append');
          var btn = document.createElement('button');
          btn.innerHTML = `Set ${boolString}`;
          btn.classList.add('btn', 'btn-sm', 'btn-secondary');
          inputGroupAppend.appendChild(btn);

          var inputGroupNode = row_node.querySelector('.input-group');
          btn.addEventListener('click', boolButtonSwitch.bind(inputGroupNode));
        } else {
          // --- enable edit switch ---
          var btnEditNode = row_node.querySelector('.param-edit');
          var inputGroupNode = row_node.querySelector('.input-group');
          if (btnEditNode !== null) {
            btnEditNode.addEventListener('click', editButtonSwitch.bind(inputGroupNode));
          }
        }
        break;
    }
    // console.log(key);

    var tag = (pCount % 2) ? `#${tab_name}-right-col` : `#${tab_name}-left-col`;
    var $paramCol = $(tag);
    $paramCol.append(row_node);

    pCount++;
  }

  // --- activate first pane ---
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  var firstChild = navTab.lastChild.firstChild;
  // console.log(firstChild);
  firstChild.classList.add('active');
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  // console.log(tabContent)
  var lastIndex = tabContent.children.length - 1;
  var tcFirstChild = tabContent.children[lastIndex];
  tcFirstChild.classList.add('active');
}

function updateArtifactConfActionView(_data) {
  // --- update modal objects ---
  $('#import-conf-btn').off('click');
  $('#import-conf-btn').on('click', function () {
    importConfFile('artifact');
  });

  let artifactID = $('#sb-agent-id').val();
  updateConfDefaultFileName(artifactID);
  $('#export-conf-btn').off('click');
  $('#export-conf-btn').click(_data, exportConfFile);

  $('#reset-conf-btn').off('click');
  $('#reset-conf-btn').click('artifact', resetConfFile);

  // --- generate action buttons ---
  var btnGroupDiv = $('#settings-btns-group');
  btnGroupDiv.empty();

  var importBtn = document.createElement('button');
  importBtn.setAttribute('class', 'btn btn-primary mx-1');
  importBtn.setAttribute('data-toggle', 'modal');
  importBtn.setAttribute('data-target', '#upload-conf-modal');
  // [lang] import button
  let lang = getSetLang() || 'en';
  console.log(lang);
  // importBtn.innerHTML = `<i class="fas fa-file-import"></i> Import`;
  importBtn.innerHTML = `<i class="fas fa-file-import"></i> ${langTemplateObj_.btn_SidebarImport}`;
  btnGroupDiv.append(importBtn);

  var exportBtn = document.createElement('button');
  exportBtn.setAttribute('class', 'btn btn-primary mx-1');
  exportBtn.setAttribute('data-toggle', 'modal');
  exportBtn.setAttribute('data-target', '#save-conf-modal');
  // [lang] export button
  // exportBtn.innerHTML = `<i class="fas fa-file-export"></i> Export`;
  exportBtn.innerHTML = `<i class="fas fa-file-export"></i> ${langTemplateObj_.btn_SidebarExport}`;
  btnGroupDiv.append(exportBtn);

  var resetDiv = $('#reset-div');
  resetDiv.empty();

  var resetText = document.createElement('div');
  resetText.innerHTML = 'Erase all data:';
  resetDiv.append(resetText);

  var resetBtn = document.createElement('button');
  resetBtn.setAttribute('class', 'btn btn-primary mx-2');
  resetBtn.setAttribute('data-toggle', 'modal');
  resetBtn.setAttribute('data-target', '#delete-conf-modal');
  resetBtn.innerHTML = `<i class="fas fa-trash"></i> Delete`;
  resetDiv.append(resetBtn);

  $('a[href="#reset"]').show();
}

let openSidebar = true; // check whether sidebar needs to be opened
let langData_ = {};
async function updateSidebarAsyncCb(_agent) {
  // https://farobottech.atlassian.net/jira/software/projects/FAR/boards/2?assignee=624ea4cef6a269006961929e&selectedIssue=FAR-1385
  let name = document.getElementById('agent-artifact-name');
  name.querySelector("[data-i18n='fleet_config.sidebar.lbl_AgntArtfName']").textContent = langTemplateObj_.agnt?.lbl_SidebarName || 'Name';

  if (openSidebar) {
    removeConfActionButtons();
    lockAgentParamView();
    lockAgentSettings();
    $('.control-sidebar').ControlSidebar('show');
    openSidebar = false;
  } else {
    $('.control-sidebar').ControlSidebar('collapse');
    openSidebar = true;
    return;
  }
  // console.log(_agent);
  // --- update agent profile on sidebar ---
  updateAgentProfileView(_agent);
  // upgrade list button
  var btnHolder = $('#core-firmware');
  btnHolder.empty();
  var btnAgentSwUpdate = document.createElement('button');
  btnAgentSwUpdate.innerHTML = ' ';
  btnAgentSwUpdate.setAttribute('class', 'fas fa-bullhorn');
  btnAgentSwUpdate.addEventListener(
    'click',
    updateAgentSw.bind(this, _agent.robot_id)
  );
  btnHolder.append(btnAgentSwUpdate);

  //--- add reboot button ---
  var rebootNode = document.querySelector('.sb-agent-reboot');

  rebootNode.classList.remove('re_agent');
  rebootNode.classList.remove('re_artifact');
  rebootNode.classList.add('re_agent');

  // var hyper = $('#hyperlink-amr');
  var hyperCol = document.getElementById('hyperlink-amr-col');
  hyperCol.style.display = 'block';
  var hyper = document.getElementById('hyperlink-amr');
  hyper.setAttribute('href', 'http://' + _agent.ip + ':3000');
  hyper.style.display = 'block';
  hyper.style.color = 'white';

  // --- update agent settings on sidebar ---
  console.log(_agent.robot_id);
  let data;
  let metadata;
  try {
    await rmtTokenCheck();
    metadata = await fetchAgentMetadata(rmtToken_, _agent.robot_id);
    data = await fetchAgentSettings(rmtToken_, _agent.robot_id);
    let lang = getSetLang() || 'en';
    langData_ = await restGetAgentParamTranslation(lang);
    console.log(langData_);
    settingsCache_.agent = data;
  } catch (err) {
    console.error(err);
  }
  createAgentViewButton(_agent.robot_id);
  updateAgentDefinitions(metadata);
  updateAgentParamView(data);

  applyFontSize(getSavedFontSize(), '.os-content');
}

// --- Utility to make a array into string to send to server ---
function getArrayStr(agents) {
  var agentStr = '[';
  if (agents.length > 0) {
    agents.forEach((r) => {
      agentStr += `"${r}",`;
    });
    agentStr = agentStr.slice(0, -1);
  }
  agentStr += ']';
  return agentStr;
}

/**
// --- The following 2 functions both call updateAgentArtiReused,
// with different API endpoints, and different "Draw Progress".
*/
async function updateAgentSw(_robot_id) {
  updateAgentArtiReused(
    _robot_id,
    fetchPutAgentSwVer,
    updateAgentVerProgress
  );
}

async function updateArtifactSw(_artifact_id) {
  updateAgentArtiReused(
    _artifact_id,
    fetchPutArtifactSwVer,
    updateArtifactVerProgress
  );
}

// --- Shared part of update Agents or Artifacts ---
// duplicated codes from far_settings.js updateCoreFw(), could be refactored?
async function genSoftwareVerObj() {
  let tmpObj = {};
  try {
    let agentsVer = fleetVer.agents.agent_sw_version.map(agent => agent.sw_version);
    agentsVer = _.uniq(agentsVer);
    let agentVer = agentsVer.length === 1 ? agentsVer[0] : '';
    tmpObj.agent = agentVer;
    let artifactsVer = allFleetArtifacts_.map(artifact => artifact.sw_version);
    artifactsVer = _.uniq(artifactsVer);
    let artifactVer = artifactsVer.length === 1 ? artifactsVer[0] : '';
    tmpObj.artifact = artifactVer;
    let coreVer = await fetchGetCoreSwVer(rmtToken_);
    tmpObj.core = coreVer.core_sw_version;
  } catch (err) {
    console.error(err);
  }
  return tmpObj;
}

async function updateAgentArtiReused(_updateId, _updateFunc, _progressFunc) {
  // 1. --- get selected version ---
  var select = document.getElementById('core-firmware-ver');
  var value = select.options[select.selectedIndex].value;
  // --- pop-up modal ---
  if (confirm('Update software?')) {
    // reset the freshCnt to get more seconds of current timeout.
    refreshCnt = 0;
    robotStatusTimeout = setTimeout(refreshRobots, timeout);
    // console.log('Firmware Updating!');

    // --- firmware update procedure ---
    // 2. --- apply the selected version to agent ---
    var agents = [];
    agents.push(_updateId);
    try {
      let agentStr = getArrayStr(agents);
      let data = await _updateFunc(rmtToken_, agentStr, value);
      if (!data.ok) {
        var rtnMsg = await data.json();
        console.log(rtnMsg);
        notificationMsg(3, rtnMsg);
        return;
      }
      // 3. update the status on VIEW
      await _progressFunc(agents);
    } catch (err) {
      console.error(err);
    }
  } else {
    console.log('Update Cancel!');
  }
}

// --- Update Agent Version ---
async function updateAgentVerProgress(_agent_ids) {
  updateAgentArtiVerProgressReused(
    _agent_ids,
    fetchPostAgentsSwUpdateStatus,
    'agent_sw_update_status'
  );
}

async function updateArtifactVerProgress(_agent_ids) {
  updateAgentArtiVerProgressReused(
    _agent_ids,
    fetchPostArtifactSwUpdateStatus,
    'artifact_sw_update_status'
  );
}

// duplicated codes from far_settings.js updateCoreFw(), could be refactored?
async function updateAgentArtiVerProgressReused(_agent_ids, _progressFunc, _statusName) {
  const asyncUpdateProgress = async () => {
    var progress;
    var updateFailed = false;
    // --- show the progress bar ---
    while (progress !== 'Succeeded') {
      try {
        let agentStr = getArrayStr(_agent_ids);
        var progressObj = await _progressFunc(rmtToken_, agentStr);
        console.log('progressObj: ', progressObj);
      } catch (err) {
        console.error(err);
      }
      let update_status = progressObj[_statusName];
      progress = update_status[0].sw_update_status;
      console.log('progress: ', progress);
      if (update_status.length === '') break;
      updateFailed = progress.toLowerCase().indexOf('fail') >= 0;
      if (updateFailed) { break; }

      // --- update on the view ---
      var barDom = $('#core-progress > div');
      if (progress.includes('%')) {
        var percent = progress
          .substring(progress.indexOf(':') + 1, progress.lastIndexOf('%'))
          .trim();
        console.log('percent: ', percent);
        $('#core-progress').show();
        barDom.css('width', `${percent}%`);
      }
      barDom.text(progress);
      await sleep(1000); // sleep 1s
    }

    const _updateType = _statusName.split('_')[0];
    const _updateTypeStr = _updateType.charAt(0).toUpperCase() + _updateType.slice(1);
    if (updateFailed) {
      notificationMsg(3, `${_updateTypeStr} Software Update Failed!`);
      return;
    }
    // --- software update guide notice ---
    var select = document.getElementById('core-firmware-ver');
    var value = select.options[select.selectedIndex].value;
    var verObj = await genSoftwareVerObj();
    var cmpCoreVer = compareVersion(value, verObj.core);
    var cmpAgentVer = compareVersion(value, verObj.agent);
    var cmpArtifactVer = compareVersion(value, verObj.artifact);
    if (_updateType === 'agent' && cmpArtifactVer !== 0) {
      alert('Don\'t forget to update the artifact software to same version!');
    } else if (_updateType === 'artifact' && cmpAgentVer !== 0) {
      alert('Don\'t forget to update the agent software to same version!');
    } else if (cmpCoreVer !== 0) {
      alert('Don\'t forget to update the core software to same version!');
    }
    // --- hide the progress bar ---
    $('#core-progress').hide();
    notificationMsg(0, `${_updateTypeStr} Software Updated!`);
  };

  asyncUpdateProgress();
}

function createSaveAgentButton(robot_id) {
  $('#sidebar-param-footer').show();
  // --- update save button binding ---
  var placeholder = $('#save-placeholder');
  placeholder.empty();
  var saveBtn = document.createElement('button');
  saveBtn.setAttribute('class', 'btn btn-primary save-agent');
  saveBtn.innerHTML = "Save";
  saveBtn.addEventListener('click', function (event) {
    saveAgentSettingsCb(event, robot_id)
  }
  );

  placeholder.append(saveBtn);
}

function removeSaveAgentButton() {
  $('#sidebar-param-footer').hide();
  $('.save-agent').remove();
}

function showSaveArtifactButton() {
  $('#sidebar-param-footer').show();
}

async function saveAgentSettingsCb(e, _agentId) {
  e.preventDefault();
  // $.validator.addClassRules({
  //   'param-check': {
  //     chkValidFormat: true
  //   }
  // });
  $("#save-settings-form").validate({ debug: true });
  if (!$("#save-settings-form").valid()) return;

  var tabContent = document.querySelector('#sidebar-param-card > .card-header > .nav-tabs');
  // console.log(tabContent);
  var tabs = tabContent.childNodes;
  var tabsId = [];
  tabs.forEach((tab) => {
    // console.log(tab.firstChild.href);
    var hrefNode = tab.firstChild.href;
    var arr = hrefNode.split('#');
    tabsId.push(arr[1]);
  });
  // console.log(tabsId);
  var jsonObj = {};

  tabsId.forEach((id) => {
    var leftNode = document.querySelector('#' + id + '-left-col');
    // console.log(leftNode);
    var paramRow = leftNode.querySelectorAll('.row');
    paramRow.forEach((pr) => {
      // var name = pr.querySelector('.param-label').innerHTML;
      var value = pr.querySelector('.form-control').value;
      var id = pr.querySelector('.form-control').id;
      // console.log(`${name}: ${value}`);
      // name = keyMappingTable_[name];
      var type = typeof (paramRecord_[id]);
      console.log(type);
      if (type === "number") {
        jsonObj[id] = Number(value);
      } else if (type === "boolean") {
        jsonObj[id] = (value === 'true' || value === 'True');
      } else {
        jsonObj[id] = value;
      }

      // [protection] --- prevent change value on text field ---
      if (id === 'system.fleet_name') {
        if (jsonObj[id] === paramRecord_[id]) return;
        jsonObj[id] = paramRecord_[id];
      } else if (id === 'system.initial_pose_confirmed') {
        jsonObj[id] = enableConfirm;
      } else if (id === 'network.bridge_mode') {
        jsonObj[id] = jsonObj[id] === 'on' ? 'zenoh' : 'dds';
      }
    });

    var rightNode = document.querySelector('#' + id + '-right-col');
    // console.log(leftNode);
    paramRow = rightNode.querySelectorAll('.row');
    paramRow.forEach((pr) => {
      // var name = pr.querySelector('.param-label').innerHTML;
      var value = pr.querySelector('.form-control').value;
      var id = pr.querySelector('.form-control').id;
      // console.log(`${name}: ${value}`);
      // name = keyMappingTable_[name];
      var type = typeof (paramRecord_[id]);
      console.log(type);
      if (type === "number") {
        jsonObj[id] = Number(value);
      } else if (type === "boolean") {
        jsonObj[id] = (value === 'true' || value === 'True');
      } else {
        jsonObj[id] = value;
      }

      // [protection] --- prevent change value on text field ---
      if (id === 'system.fleet_name') {
        if (jsonObj[id] === paramRecord_[id]) return;
        jsonObj[id] = paramRecord_[id];
      } else if (id === 'system.initial_pose_confirmed') {
        jsonObj[id] = enableConfirm;
      } else if (id === 'network.bridge_mode') {
        jsonObj[id] = jsonObj[id] === 'on' ? 'zenoh' : 'dds';
      }
    });
  });

  // console.log(jsonObj);
  var originalJsonStr = getUnflattenObj(JSON.stringify(jsonObj));
  // console.log(originalJsonStr);
  var configData = originalJsonStr;
  configData = configData.replace(/true/g, 'True').replace(/false/g, 'False');
  configData = configData.replace(/"/g, "'");
  // console.log(configData);

  // --- PUT Settings to Backend ---
  await rmtTokenCheck();
  let res = await fetchPutAgentSettings(rmtToken_, _agentId, configData);

  if (res.hasOwnProperty("robot_list")) {
    notificationMsg(0, 'Agent Settings Set!');
  } else {
    await notificationMsg(0, 'Agent Settings Set Fail!');
  }
}

function updateAgentProfileView(_agent) {
  console.log(_agent);
  current_sidebar_mode = "";
  var agentKey = _agent.model.toLowerCase();
  // $('#sb-agent-icon').attr('src', '/images/' + agentAssets_[agentKey].thumbnail);
  $('#sb-agent-icon').attr('src', agentAssets_[agentKey].thumbnail);
  // $('#sb-agent-name').val(_agent.robot_name);
  avoidNoneVal($('#sb-agent-name'), _agent.robot_name, _agent.robot_id);
  $('#sb-agent-id').val(_agent.robot_id);
  $('#sb-agent-mode').val(_agent.mode); // take 'mode' instead of 'status'.
  $('#sb-agent-model').val(_agent.model);
  $('#sb-agent-firmware').val(_agent.sw_version);
}

function updateAgentDefinitions(_data) {
  if (!_data.hasOwnProperty('agent_metadata') || (_data.hasOwnProperty('agent_metadata') && _data.agent_metadata.length === 0)) return;
  var confDefData = _data.agent_metadata[0].nav_conf_def.replace(/'/g, '"');
  confDefData = JSON.parse(confDefData);
  confDefData = flattenJSON(confDefData);
  conf_definition = confDefData;
}

function updateAgentParamView(_data) {
  if (!_data.hasOwnProperty('agent_settings') || (_data.hasOwnProperty('agent_settings') && _data.agent_settings.length === 0)) {
    notificationMsg(3, 'Failed to get agent settings by RMT!');
    unlockAgentSettings();
    $('.control-sidebar').ControlSidebar('collapse');
    openSidebar = true;
    return;
  }

  // -- extract data --
  // console.log(_data);
  var configData = _data.agent_settings[0].nav_conf;

  // -- rectify data --
  configData = configData.replace(/'/g, '"');
  configData = configData.replace(/True/g, 'true').replace(/False/g, 'false');
  tmp_agent_conf = configData;
  console.log(configData);

  // -- visualize data --
  // TODO: encapsualte langData
  updateParamTabsView(configData, langData_);
  updateParamMembersView(configData, langData_);
  updateConfActionView(configData);

  // -- unlock the elements after data update --
  unlockAgentSettings();
  unlockAgentParamView();
}

function createAgentViewButton(_agentId) {
  displayInitPoseConfirmLoadingStyle();
  updateSaveButtonByMode(_agentId);
}

function updateSaveButtonByMode(_agentId) {
  if (!enableConfirm) {
    createSaveAgentButton(_agentId);
  } else {
    removeSaveAgentButton();
  }
}

function buildKeyTable(_data) {
  var table = {};
  for (let key in _data) {
    var keyValue = key.split(".");

    var iParam = {};
    iParam.pkg = keyValue[0];
    iParam.node = keyValue[1];
    if (keyValue.length !== 6) {
      iParam.param = keyValue[keyValue.length - 1];
    } else {
      iParam.param = keyValue[keyValue.length - 2];
    }

    table[iParam.param] = key;
  }
  console.log(table);
  return table;
}

function extractParamData(_data) {
  let new_res = [];
  for (let key in _data) {
    let keyValue = key.split(".");
    let iParam = {};

    iParam.pkg = keyValue[0];
    iParam.node = keyValue[1];
    if (keyValue.length !== 6) {
      iParam.param = keyValue[keyValue.length - 1];
    } else {
      iParam.param = keyValue[keyValue.length - 2];
    }

    iParam.value = _data[key];
    new_res.push(iParam);
  }
  // console.log(new_res);
  return new_res;
}

function editButtonSwitch() {
  if (!$("#save-settings-form").valid()) return;
  // console.log(this);
  let inputNode = this.querySelector('.form-control');
  let btnIconNode = this.querySelector('.fas');
  // console.log(btnIconNode);

  inputNode.readOnly = !inputNode.readOnly;
  // --- mount input validation mechanism ---
  if (!inputNode.readOnly) {
    validateCellInputEvent(inputNode);
  }

  btnIconNode.classList.toggle("fa-pen");
  btnIconNode.classList.toggle("fa-eye");
}

function boolButtonSwitch(e) {
  e.preventDefault();
  let inputNode = this.querySelector('.form-control');
  let inputBoolVal = inputNode.value.toLowerCase() == "true";
  inputNode.value = !inputBoolVal;

  let stringVal = inputBoolVal.toString();
  let boolString = stringVal.charAt(0).toUpperCase() + stringVal.slice(1);
  let boolBtn = this.querySelector('.btn');
  boolBtn.innerHTML = `Set ${boolString}`;
}

function bridgeModeButtonSwitch(e) {
  e.preventDefault();
  const inputNode = this.querySelector('.form-control');
  const switchBtn = this.querySelector('.btn');
  const inputString = inputNode.value.charAt(0).toUpperCase() + inputNode.value.slice(1);
  switchBtn.innerHTML = `Set ${inputString}`;
  inputNode.value = inputNode.value.toLowerCase() == 'on' ? 'off' : 'on';
}

function checkInvalidParam() {
  $('#save-settings-form').data('validator').element($(`input[name="${this.name}"]`));
  // var keyID = this.id;
  // var uiName = keyID.split('.')[1] || '';
  // var validDataTypeKey = keyID.concat('.', 'valid_value.data_type');
  // var validDataRangeKey = keyID.concat('.', 'valid_value.data_range');
  // var validDataType = conf_definition[validDataTypeKey];
  // var validDataRange = conf_definition[validDataRangeKey].replace(/[[\]]/g, '');
  // if (validDataRange.split(":").length <= 1) return;
  // switch (validDataType) {
  //   case 'double': {
  //     var minVal = parseFloat(validDataRange.split(":")[0]);
  //     var maxVal = parseFloat(validDataRange.split(":")[1]);
  //     var doubleVal = parseFloat(this.value);
  //     if (isNaN(minVal) || isNaN(maxVal) || isNaN(doubleVal)) {
  //       return;
  //     }
  //     if (doubleVal > maxVal || doubleVal < minVal) {
  //       alert(`${uiName} value NOT-IN-RANGE!!`);
  //     }
  //     break;
  //   }
  //   case 'int': {
  //     var minVal = parseInt(validDataRange.split(":")[0]);
  //     var maxVal = parseInt(validDataRange.split(":")[1]);
  //     var intVal = parseInt(this.value);
  //     if (isNaN(minVal) || isNaN(maxVal) || isNaN(intVal)) {
  //       return;
  //     }
  //     if (intVal > maxVal || intVal < minVal) {
  //       alert(`${uiName} value NOT-IN-RANGE!!`);
  //     }
  //     break;
  //   }
  //   case 'string': {
  //     var minLen = validDataRange.split(":")[0];
  //     var strLen = this.value.length;
  //     if (strLen < minLen) {
  //       alert(`${uiName} must have at least ${minLen} characters!!`)
  //     }
  //     break;
  //   }
  //   default: {
  //     break;
  //   }
  // }
}

function setInputValue(selNode) {
  const inputNode = this.querySelector('.form-control');
  inputNode.value = selNode.value;
}

function addOption(selNode, opt, selOpt) {
  const mapOption = document.createElement('option');
  mapOption.value = opt;
  mapOption.text = opt;
  mapOption.selected = (opt === selOpt);
  selNode.add(mapOption);
}

// ------ add agent event ------
$('#add-agent').click(popupAgentCandidatesAsyncCb);

async function popupAgentCandidatesAsyncCb() {
  // 1. send /robots/scan request
  let scanRobots;
  try {
    await rmtTokenCheck();
    scanRobots = await fetchScanRobots2(rmtToken_);
    // console.log(scanRobots);
  } catch (err) {
    console.error(err);
  }

  // CRITICAL!! SHOULD NOT UPDATE HERE
  settingsCache_.scanAgents = scanRobots;
  // 2. update robots on Data
  console.log(settingsCache_);
  var candidates = getAgentCandidates(settingsCache_);
  console.log(candidates);

  // -- filter those are grouped in other fleets ---
  // - get current fleet -
  var currFleet = Object.keys(settingsCache_.fleet)[0]; // suppose only one key-value pair

  // - filter out current fleet -
  var otherFleet = allFleetAgents_.filter(flt => Object.keys(flt)[0] !== currFleet);
  // console.log(otherFleet);

  // - all grouped agents -
  var groupedAgents = [];
  otherFleet.forEach(fltObj => {
    var flt = Object.keys(fltObj)[0]; // suppose only one key-value pair
    groupedAgents = groupedAgents.concat(fltObj[flt].agents);
  });
  groupedAgents = groupedAgents.filter(ga => ga !== null);
  // console.log(groupedAgents);

  // - filter out current fleet -
  candidates = candidates.filter(c => !groupedAgents.includes(c.robot_id));
  console.log(candidates);

  // - filter out by search name
  var keyword = $("#conf-search").val();
  if (keyword) {
    // search candidates start with `keyword` if keyword is not empty
    candidates = candidates.filter(c =>
      c.robot_id.includes(keyword) ||
      (c.robot_name.includes(keyword))
      // Need check if everything is ok remove it
      // (c.robot_name != "none" && c.robot_name.includes(keyword)
    );
  }

  settingsCache_.scanAgents.total = candidates.length;
  settingsCache_.scanAgents.robots = candidates;

  // 3. update robots on View
  updateAgentCandidates(candidates);
}

function isEmpty(obj) {
  for (var k in obj) {
    if (obj.hasOwnProperty(k)) return false;
  }
  return true;
}

function getAgentCandidates(_inCache) {
  // console.log(_inCache);
  var fltKey = Object.keys(_inCache.fleet)[0]; // suppose only one key-value pair

  // [protection] --- if the fleet does not exist. ---
  if (isEmpty(_inCache.fleet))
    return [];

  // [protection] --- if the selected fleet without agents  ---
  if (_inCache.fleet[fltKey].agents === null)
    return _inCache.scanAgents.robots;

  console.log(_inCache.fleet);
  console.log(_inCache.scanAgents);
  var candidates = [];
  if (typeof _inCache.scanAgents.robots !== 'undefined') {
    candidates = _inCache.scanAgents.robots.filter((obj) => _inCache.fleet[fltKey].agents.indexOf(obj.robot_id) < 0);
  }

  return candidates
}

function updateAgentCandidates(_agents) {
  console.log(_agents);
  // 1. filter available agents
  var avAgents = _agents;

  // 2. append to pop-up list
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  avAgents.forEach((agent) => {
    var node = createAgentCandidateView(agent);
    ul.appendChild(node)
  })

  // 3. update modal title
  // $('#available-items-title').html('Available Agents');
  $('#available-items-title').html(langTemplateObj_.agnt.lbl_AvailTitle);
  applyFontSize(getSavedFontSize(), '.products-list');
}

function enrollAgentToFleet(_inFleet, _agentId) {
  var fltKey = Object.keys(_inFleet.fleet)[0];
  _inFleet.fleet[fltKey].agents.push(_agentId);

}

async function enrollAgentToFleetCb(_agent) {
  unsavedChanges = true;

  // console.log(_agent);
  var $agentDeck = $('#agents-deck');
  var node = createAgentMemberView(_agent);

  $agentDeck.append(node.agent);

  enrollAgentToFleet(settingsCache_, _agent.robot_id);
  console.log(settingsCache_);

  // -- add agent-artifacts in cache --
  var agentArtifacts = [];
  var robotId = _agent.robot_id;
  var arrArtifacts = _agent.artifacts.split(';');
  arrArtifacts = arrArtifacts.filter(aa => aa !== "");
  var artifactObj = {};
  artifactObj[robotId] = arrArtifacts;
  agentArtifacts.push(artifactObj);
  // console.log(agentArtifacts);

  var fltKey = Object.keys(settingsCache_.fleet)[0]; // suppose only one key-value pair
  settingsCache_.fleet[fltKey].artifacts.agent = settingsCache_.fleet[fltKey].artifacts.agent.concat(agentArtifacts);
  // console.log(settingsCache_.fleet);

  var candidates = getAgentCandidates(settingsCache_);
  console.log(candidates);
  updateAgentCandidates(candidates);

  // --- update artifacts agent candidates ---
  // console.log(settingsCache_.fleet[fltKey]);
  allFleetArtifacts_ = await loadFleetArtifacts(settingsCache_.fleet[fltKey]);

  // --- remove agent from the cache ---
  agent_remove_list = agent_remove_list.filter(arl => arl !== _agent.robot_id);
  agent_add_list.push(_agent.robot_id);
  // console.log(agent_remove_list);
}

async function removeAgentFromFleetCb(_agent) {
  console.log(_agent);
  if (confirm("Remove the agent?")) {
    unsavedChanges = true;
    _agent.fleet = "";

    agent_remove_list.push(_agent.robot_id);
    agent_add_list = agent_add_list.filter(aal => aal !== _agent.robot_id);

    //  --- remove agent card from the deck ---
    $(this).remove();

    var fltKey = Object.keys(settingsCache_.fleet)[0]; // suppose only one key-value pair
    var agentIndex = settingsCache_.fleet[fltKey].agents.indexOf(_agent.robot_id);

    // --- remove agents from cache ---
    settingsCache_.fleet[fltKey].agents.splice(agentIndex, 1);
    // console.log(settingsCache_.fleet[fltKey]);

    // --- remove agent_artifacts from cache ---
    settingsCache_.fleet[fltKey].artifacts.agent = settingsCache_.fleet[fltKey].artifacts.agent.filter(aa => Object.keys(aa)[0] !== _agent.robot_id);
    console.log(settingsCache_.fleet[fltKey].artifacts.agent);
    console.log(settingsCache_.fleet[fltKey].artifacts);

    // -- update artifacts Fleet status ---
    allFleetArtifacts_ = await loadFleetArtifacts(settingsCache_.fleet[fltKey]);
  }
}

// ------ add artifacts event ------
$('#add-artifact').click(popupArtifactCandidatesAsyncCb);

async function popupArtifactCandidatesAsyncCb() {
  // 1. send /artifacts/scan request
  let scanArtifacts = {};
  let artifactsProperty = [];
  try {
    await rmtTokenCheck();
    scanArtifacts = await fetchScanArtifacts2(rmtToken_);
    artifactsProperty = await fetchGetArtifactsProperty(rmtToken_);
    // console.log(scanArtifacts);
  } catch (err) {
    console.error(err);
  }

  settingsCache_.scanArtifacts = scanArtifacts;

  // 2. update robots on Data
  // console.log(settingsCache_);
  var candidates = getArtifactCandidates(settingsCache_);

  // 2-1. extend data by fusing data from artifact property
  candidates = candidates.map(c => {
    let target = artifactsProperty.find(ap => ap.artifact_id === c.artifact_id);
    return (target) ? { ...c, avatar: target.avatar, thumbnail: target.thumbnail } : c;
  })
  console.log(candidates);

  // -- filter those are grouped in other fleets --
  // - get current fleet -
  var currFleet = Object.keys(settingsCache_.fleet)[0];

  // -filter out current fleet -
  var otherFleet = allFleetAgents_.filter(flt => Object.keys(flt)[0] !== currFleet);
  // console.log(allFleetAgents_);
  // console.log(otherFleet);

  // - all grouped artifacts -
  var groupedArtifacts = [];
  otherFleet.forEach(fltObj => {
    var flt = Object.keys(fltObj)[0];

    // groupedArtifacts = groupedArtifacts.concat(fltObj[flt].artifacts);
    // console.log(fltObj[flt]);
    if (fltObj[flt].hasOwnProperty('artifacts') && fltObj[flt].artifacts.length > 0) {
      // console.log(fltObj[flt].artifacts);
      fltObj[flt].artifacts = fltObj[flt].artifacts.map(a => a.split('@')[1]);
      // console.log(fltObj[flt].artifacts);
      groupedArtifacts = groupedArtifacts.concat(fltObj[flt].artifacts);
    }
  });
  groupedArtifacts = groupedArtifacts.filter(ga => ga !== null);

  // -filter out current fleet -
  // console.log(groupedArtifacts);
  // console.log(candidates);
  candidates = candidates.filter(c => !groupedArtifacts.includes(c.artifact_id));
  // console.log(candidates);

  // - filter out existing artifacts in fleet -
  // console.log(settingsCache_);
  // console.log(allFleetAgents_);
  var currFleet = allFleetAgents_.filter(flt => Object.keys(flt)[0] === currFleet);
  // console.log(currFleet[0]);
  var flt = Object.keys(currFleet[0]);

  var existingArtifacts = currFleet[0][flt].artifacts.external;
  // console.log(existingArtifacts);
  existingArtifacts = existingArtifacts.map(a => a.split('@')[1]);
  // console.log(existingArtifacts);
  candidates = candidates.filter(c => !existingArtifacts.includes(c.artifact_id));
  console.log(candidates);

  // - filter out by search name
  var keyword = $("#conf-search").val();
  if (keyword) {
    // search candidates start with `keyword` if keyword is not empty
    candidates = candidates.filter(c =>
      c.artifact_id.includes(keyword) ||
      (c.artifact_name.includes(keyword))
    );
    // Need check if everything is ok remove it
    // (c.artifact_name != "none" && c.artifact_name.includes(keyword)
  }

  settingsCache_.scanArtifacts.total = candidates.length;
  settingsCache_.scanArtifacts.artifacts = candidates;

  // 3. update robots on View
  updateArtifactCandidates(candidates);
}

function getArtifactCandidates(_inCache) {
  // console.log(_inCache);
  var fltKey = Object.keys(_inCache.fleet)[0];

  // console.log(_inCache.scanArtifacts);
  return _inCache.scanArtifacts.artifacts.filter(obj => _inCache.fleet[fltKey].artifacts.external.indexOf(obj.artifact_id) < 0);
}

function updateArtifactCandidates(_artifacts) {
  console.log(_artifacts);
  // 1. filter available artifacts
  // var selArtifact = settingsCache_.fleet[getSavedFleet()].artifacts.external;
  // selArtifact = selArtifact.map(function (value, index, array) {
  //   var v = value.split('@');
  //   return v[1];
  // });

  var selFleet = $('#fleet-select').val();
  let selected_Artifact_list = [];

  for (const [key, value] of Object.entries(settingsCache_.fleet[selFleet].artifacts)) {
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

  // console.log(settingsCache_);
  // console.log(settingsCache_.fleet[selFleet].artifacts.external);
  // console.log(selected_Artifact_list);

  // var avArtifacts = _artifacts;
  var avArtifacts = _artifacts.filter(a => !selected_Artifact_list.includes(a.artifact_id) && (a.artifact_category != 'Planning') && (a.artifact_category != 'Embedded'));

  // 2. append to pop-up list
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  avArtifacts.forEach((artifact) => {
    var node = createArtifactCandidateView(artifact);
    ul.appendChild(node)
  })

  // 3. update modal title
  // $('#available-items-title').html('Available Artifacts');
  $('#available-items-title').html(langTemplateObj_.artf.lbl_AvailTitle);
}

function enrollArtifactToFleet(_inFleet, _artifactId) {
  // console.log(_artifactId);
  var flt = Object.keys(_inFleet.fleet)[0];
  _inFleet.fleet[flt].artifacts.external.push(_artifactId);
  console.log(_inFleet.fleet[flt]);
}

function enrollArtifactToFleetCb(_artifact) {
  unsavedChanges = true;

  // console.log(_artifact);
  var $artifactDeck = $('#artifacts-deck');
  var node = createArtifactMemberView(_artifact);

  $artifactDeck.append(node);

  // -- update the fleet which the artifact belongs to --
  _artifact.fleet = $('#fleet-select').val();
  enrollArtifactToFleet(settingsCache_, `${_artifact.type}@${_artifact.artifact_id}`);
  console.log(settingsCache_);

  // -- refresh the artifact-item --
  var candidates = getArtifactCandidates(settingsCache_);
  console.log(candidates);
  updateArtifactCandidates(candidates);
}

async function syncAgentFromFleetCb(_agent) {
  console.log(_agent);
  var syncModal = document.getElementById('sync-confirm-modal');
  var title = syncModal.querySelector('.modal-title');
  title.textContent = `Sync ${_agent.robot_id} Confiuration`

  var badge = syncModal.querySelector('.sync-target');
  badge.textContent = _agent.robot_id;

  var confirm = syncModal.querySelector('.sync-confirm');
  console.log(confirm);
  confirm.addEventListener('click', confirmSyncAgentFromFleetCb.bind(syncModal, _agent));
}

async function confirmRebootAgentFromFleetCb(_agent_id) {
  // console.log(_agent_id);
  $(this).modal('hide');
  var confirm = this.querySelector('.reset-confirm');
  confirm.replaceWith(confirm.cloneNode(true));

  // 1. send the reboot request
  var res = await fetchPutRebootAgent(rmtToken_, _agent_id);

  // 2. handle the response
  if (res.ok) {
    notificationMsg(0, 'Reboot agent successfully!');
  } else {
    notificationMsg(2, 'Failed to reboot agent!');
  }
}

async function confirmSyncAgentFromFleetCb(_agent) {
  // console.log(_agent);
  $(this).modal('hide');
  var confirm = this.querySelector('.sync-confirm');
  confirm.replaceWith(confirm.cloneNode(true));

  const index = agent_add_list.indexOf(_agent.robot_id);
  if (index > -1) {
    agent_add_list.splice(index, 1);
  }

  agent_add_list.push(_agent.robot_id);
  btnSaveFleetSettings();
}

async function confirmRebootArtifactFromFleetCb(_artifact_id) {
  // console.log(_artifact_id);
  $(this).modal('hide');
  var confirm = this.querySelector('.reset-confirm');
  confirm.replaceWith(confirm.cloneNode(true));

  // 1. send the reboot request
  var res = await fetchPutRebootArtifact(rmtToken_, _artifact_id);

  // 2. handle the response
  if (res.ok) {
    notificationMsg(0, `Reboot artifact successfully!`);
  } else {
    notificationMsg(2, `Failed to reboot artifact!`);
  }
}

function removeArtifactFromFleetCb(_artifact) {
  if (confirm("Remove the artifact?")) {
    unsavedChanges = true;
    _artifact.fleet = "";
    console.log('press ok')

    // --- remove artifact card from the deck ---
    $(this).CardWidget('remove');

    var fltKey = Object.keys(settingsCache_.fleet)[0];
    // var artifactIndex = settingsCache_.fleet[fltKey].artifacts.indexOf(_artifact.artifact_id);
    var artifactIndex = settingsCache_.fleet[fltKey].artifacts.external.indexOf(_artifact.artifact_id);
    console.log(settingsCache_.fleet[fltKey].artifacts);
    console.log(_artifact);
    var strArtifact = `${_artifact.type}@${_artifact.artifact_id}`;
    console.log(strArtifact);
    // settingsCache_.fleet[fltKey].artifacts.splice(artifactIndex, 1);
    // settingsCache_.fleet[fltKey].artifacts.external.splice(artifactIndex, 1);
    console.log(settingsCache_.fleet[fltKey].artifacts.external);
    settingsCache_.fleet[fltKey].artifacts.external = settingsCache_.fleet[fltKey].artifacts.external.filter(ae => ae !== strArtifact);
    console.log(settingsCache_.fleet[fltKey].artifacts.external);
  } else {
    console.log('press cancel')
  }
}

// ------ add map event ------
$('#add-map').click(popupMapCandidatesAsync);

async function popupMapCandidatesAsync() {
  // 1. get data of map candidates
  let scanMaps;
  try {
    await rmtTokenCheck();
    scanMaps = await fetchMaps(rmtToken_);
    settingsCache_.scanMaps = scanMaps;
  } catch (err) {
    console.error(err);
  }

  // 2. update maps on Data
  var candidates = getMapCandidates(settingsCache_);

  // -- filter out by search name
  var keyword = $("#conf-search").val();
  if (keyword) {
    // search candidates start with `keyword` if keyword is not empty
    candidates = candidates.filter(c => c.map_name.includes(keyword));
  }

  settingsCache_.scanMaps.total = candidates.length;
  settingsCache_.scanMaps.maps = candidates;

  // 3. update the maps on View
  updateMapCandidates(candidates);
}

function getMapCandidates(_inCache) {
  console.log(_inCache);
  var candidates = _inCache.scanMaps.maps.filter((obj) => {
    var fltKey = Object.keys(_inCache.fleet)[0]; // suppose only one key-value pair
    return (_inCache.fleet[fltKey].maps.indexOf(obj.map_name) < 0)
  });
  console.log(candidates);
  candidates.filter((obj) => {
    var mapObj = _.filter(mapAlias_, ['name', obj.map_name]);
    obj['map_alias'] = mapObj[0].alias_name || 'none';
  });
  return candidates;
}

async function updateMapCandidates(_maps) {
  // console.log(_maps);
  // 1. filter available maps
  var avMaps = _maps;

  // 2. append to pop-up list
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  avMaps.forEach(async (map) => {
    var node = await createMapCandidateView(map);
    ul.appendChild(node)
  })

  // 3. update modal title
  // $('#available-items-title').html('Available Maps');
  $('#available-items-title').html(langTemplateObj_.maps.lbl_AvailTitle);

}

function enrollMapToFleet(_inFleet, _mapName) {
  var fltKey = Object.keys(_inFleet.fleet)[0]; // suppose only one key-value pair
  _inFleet.fleet[fltKey].maps.push(_mapName);
}

async function enrollMapToFleetCb(_map) {
  // console.log(_map);
  var $mapsDeck = $('#maps-deck');
  var node = await createMapMemberView(_map);
  $mapsDeck.append(node);

  // -- refresh the agent-item --
  enrollMapToFleet(settingsCache_, _map.map_name);
  var candidates = getMapCandidates(settingsCache_);
  await updateMapCandidates(candidates);
}

function removeMapFromFleetCb(_map) {
  console.log(_map);
  // pop-up modal
  var txt;
  if (confirm("Remove the map?")) {
    txt = "You pressed OK!";

    // --- remove map card from the deck ---
    this.remove();

    var fltKey = Object.keys(settingsCache_.fleet)[0];
    var mapIndex = settingsCache_.fleet[fltKey].maps.indexOf(_map.map_name);
    console.log(mapIndex);
    settingsCache_.fleet[fltKey].maps.splice(mapIndex, 1);
    console.log(settingsCache_.fleet[fltKey].maps);
  } else {
    txt = "You pressed Cancel!";
    console.log('press cancel')
  }
  document.getElementById("confirm-modal").innerHTML = txt;
}

// ------ add role event ------
$('#add-role').click(popupRoleCandidatesAsyncCb);

async function popupRoleCandidatesAsyncCb() {
  let scanRoles;
  try {
    await rmtTokenCheck();
    scanRoles = await fetchRoles(rmtToken_);
    settingsCache_.scanRoles = scanRoles;
  } catch (err) {
    console.error(err);
  }

  var candidates = getRoleCandidates(settingsCache_);

  // -- filter out by search name
  var keyword = $("#conf-search").val();
  if (keyword) {
    // search candidates start with `keyword` if keyword is not empty
    candidates = candidates.filter(c => c.role_name.includes(keyword));
  }

  settingsCache_.scanRoles.total = candidates.length;
  settingsCache_.scanRoles.roles = candidates;

  updateRoleCandidates(candidates);
}

function getRoleCandidates(_inCache) {
  var candidates = _inCache.scanRoles.roles.filter((obj) => {
    var fltKey = Object.keys(_inCache.fleet)[0]; // suppose only one key-value pair
    return (_inCache.fleet[fltKey].roles.indexOf(obj.role_name) < 0);
  })
  console.log(candidates);
  return candidates;
}

function updateRoleCandidates(_roles) {
  console.log(_roles);
  // 1. filter available roles
  var avRoles = _roles;
  console.log(avRoles);

  // 2. append to pop-up list
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  avRoles.forEach((role) => {
    var node = createRoleCandidateView(role);
    ul.appendChild(node)
  })

  // 3. update modal title
  // $('#available-items-title').html('Available Roles');
  $('#available-items-title').html(langTemplateObj_.role.lbl_AvailTitle);
};

function enrollRoleToFleet(_inFleet, _roleName) {
  var fltKey = Object.keys(_inFleet.fleet)[0]; // suppose only one key-value pair
  _inFleet.fleet[fltKey].roles.push(_roleName);
}

function enrollRoleToFleetCb(_role) {
  // console.log(_role);
  var $rolePanel = $('#roles-deck');
  var node = createRoleMemberView(_role);
  $rolePanel.append(node);

  // -- refresh the agent-item --
  enrollRoleToFleet(settingsCache_, _role.role_name);
  var candidates = getRoleCandidates(settingsCache_);
  updateRoleCandidates(candidates);
}

function removeRoleFromFleetCb(_role) {
  if (confirm("Remove the role?")) {
    console.log('press ok')
    // --- remove role card from deck ---
    this.remove();
    // --- use to check if there are flows still using the roles ---
    deletedRole.push(_role);

    var fltKey = Object.keys(settingsCache_.fleet)[0];
    var roleIndex = settingsCache_.fleet[fltKey].roles.indexOf(_role);
    settingsCache_.fleet[fltKey].roles.splice(roleIndex, 1);

    roleRemoved = true;
  } else {
    console.log('press cancel')
  }
}

async function btnSaveFleetSettings() {
  // console.log(settingsCache_.fleet);

  // --- update core settings to swarm core ---
  // var coreConfig = settingsCache_.core;
  // // TODO: --- swarm parameters settings. Configuration and settings should be done separately ---
  // coreConfig['swarm_control_frequency'] = Number($('#swarm_control_frequency').val());

  // var batteryThresh = $('#low_battery_threshold').val();
  // coreConfig['low_battery_threshold'] = Number(batteryThresh) + 0.000001; // walkaround to convert the value to float
  // console.log(coreConfig);

  // coreConfig = JSON.stringify(coreConfig);
  // coreConfig = coreConfig.replace(/"/g, "'");
  // coreConfig = `{'far_sys': { 'swarm_core': ${coreConfig}}}`;
  // console.log(coreConfig);

  // await fetchPutCoreSettings(coreConfig);

  console.log(settingsCache_.fleet);
  var result = JSON.stringify(settingsCache_.fleet);

  var fleetName = $('#fleet-select').val();

  executingFlow = await checkProtection();
  // console.log(executingFlow)

  if (Object.keys(executingFlow["using_fleets"]).includes(fleetName)) {
    notificationMsg(3, 'Fleet has executing task, cannot save.');
    return;
  }

  // console.log(`selected fleet: ${fleetName}`);
  if (fleetName === "") {
    notificationMsg(2, `Fleet Name is empty`);
    return;
  }
  if (!inputCheck(fleetName)) {
    notificationMsg(2, `Fleet Name include invalid characters`);
    return;
  }

  var jsonData = JSON.parse(result);
  let haveTasks = jsonData.hasOwnProperty(fleetName) && jsonData[fleetName].hasOwnProperty('tasks') && jsonData[fleetName].tasks.length > 0;
  let haveFlows = jsonData.hasOwnProperty(fleetName) && jsonData[fleetName].hasOwnProperty('flows') && jsonData[fleetName].flows.length > 0;

  operationTextArr = [];

  if (roleRemoved && haveTasks) {
    var tasks = jsonData[fleetName].tasks;
    var newTasksArray = tasks.map(task => `${fleetName}-task-${task}.json`);
    chkRoleInUsedOperations(fleetName, newTasksArray, deletedRole, 'tasks');
  }
  if (roleRemoved && haveFlows) {
    var flows = jsonData[fleetName].flows;
    var newFlowsArray = flows.map(flow => `${fleetName}-flow-${flow}.json`);
    chkRoleInUsedOperations(fleetName, newFlowsArray, deletedRole, 'flows');
  }
  if ((roleRemoved && !haveTasks && !haveFlows) || !roleRemoved) {
    fleetName = $('#fleet-select').val();
    saveFleetSettings(fleetName, settingsCache_.old_fleet_name);
  }

  roleRemoved = false;
}

async function saveFleetSettings(fleetName, oldFleetName) {
  // --- save fleet settings to fleet config. yaml file ---
  // console.log(settingsCache_.fleet);
  // var result = JSON.stringify(settingsCache_.fleet);
  // await restPostFleetSettings(fleetName, result, oldFleetName);
  console.log(JSON.stringify(settingsCache_.fleet[fleetName]));
  let res = await fetchPutFleetConfigs(rmtToken_, fleetName, settingsCache_.fleet[fleetName]);
  console.log(res);
  if (res.ok) {
    notificationMsg(0, `Fleet:${fleetName} Settings Saved!`);
  } else {
    notificationMsg(3, `Fleet: Failed to save ${fleetName} Configuration!`);
    return;
  }

  // [FAR-822] filter out agents those are deployed maps by scanned
  // --- fetch artifacts of agent in fleet ---
  let scannedAgents;
  try {
    await rmtTokenCheck();
    scannedAgents = await fetchScanRobots2(rmtToken_);
  } catch (err) {
    console.error(err);
  }

  scannedAgents = scannedAgents.robots.map(r => r.robot_id);
  console.log(scannedAgents);

  // --- deploy maps asynchronously to fleet ---
  var fltKey = Object.keys(settingsCache_.fleet)[0];
  var fltMaps = settingsCache_.fleet[fltKey].maps;
  var fltAgents = settingsCache_.fleet[fltKey].agents;

  var agentStr = '[';
  if (fltAgents.length > 0) {
    fltAgents.forEach(r => {
      // --- only available agents in the fleet can conduct map-deployment ---
      if (scannedAgents.includes(r)) {
        agentStr += `"${r}",`;
      }
    });
    agentStr = agentStr.slice(0, -1);
  }
  agentStr += ']';
  // console.log(agentStr);

  const asyncDeploy = async (mapArray) => {
    for (const m in mapArray) {
      console.log(mapArray[m]);
      await fetchPostFleetMapDeplyment(rmtToken_, agentStr, mapArray[m]);
      await sleep(1500); // sleep 1s
    }
    notificationMsg(0, `Fleet:${fleetName} map deployed!`);
  };

  displayOverlay('Deploying...'); // lock the screen
  try {
    await rmtTokenCheck();
    await asyncDeploy(fltMaps);
  } catch (err) {
    console.error(err);
  }
  removeOverlay(); // unlock the screen


  // ------ [FAR-1176] data synchronization ------
  // -- alignment and notification --
  const asyncAgentMapCheck = async (agentArray) => {
    for (const idx in agentArray) {
      console.log(agentArray[idx]);
      // await fetchPostFleetMapDeplyment(rmtToken_, agentStr, mapArray[m]);
      var runningMap = await getAgentRunningMap(fltAgents[idx]);
      console.log(runningMap);
      // --- skip scan no agent/map case ---
      if (runningMap === "") { continue; }

      // --- skip aligned case ---
      if (fltMaps.includes(runningMap) || runningMap === 'none') { continue; }

      notificationMsg(3, `The fleet NOT contain map:${runningMap} used by ${fltAgents[idx]}`);
      await sleep(1000); // sleep 1s
    }
    notificationMsg(0, `Fleet:${fleetName} map resources aligned!`);
  };


  let bAligned = true;
  let selFleet = $('#fleet-select').val();

  const asyncFleetName = async (arrRemovedAgents, arrAddAgents) => {
    if (arrRemovedAgents.length > 0) {
      for (const idx in arrRemovedAgents) {
        console.log(arrRemovedAgents[idx]);
        await update_agent_fleet(arrRemovedAgents[idx], "none");
        await sleep(1000); // sleep 1s
      }

      arrRemovedAgents.forEach(function (agent_item, agent_index) {
        update_agent_fleet(agent_item, "none");
      });
    }

    if (arrAddAgents.length > 0) {
      for (const idx in arrAddAgents) {
        // console.log(arrAddAgents[idx]);
        var bRes = await update_agent_fleet(arrAddAgents[idx], selFleet);
        bAligned = bAligned && bRes;
        console.log(`${arrAddAgents[idx]} align fleet result: ${bAligned}`);
        await sleep(1000); // sleep 1s
      }
    }

    notificationMsg(0, `Fleet name:${fleetName} is aligned!`);
  };

  displayOverlay('Data Sync...'); // lock the screen
  try {
    await rmtTokenCheck();
    await asyncAgentMapCheck(fltAgents);
    await asyncFleetName(agent_remove_list, agent_add_list);
  } catch (err) {
    console.error(err);
  }
  unsavedChanges = false;
  removeOverlay(); // unlock the screen

  // --- add prevention from saving config file when RMT connection is unstable. ---
  if (!bAligned) {
    notificationMsg(3, 'Failed to align fleet name by RMT!');
    return;
  }

  // --- reset fleet agents array ---
  allFleetsName_ = [];
  // let fleets = await restGetFleets();
  let fleets = await fetchGetFleets(rmtToken_);
  fleets = Object.keys(fleets);
  // allFleetsName_ = fleets.map(fleet => fleet.split('.').slice(0, -1).join('.'));
  console.log(allFleetsName_);
  // --- reset settings cache ---
  resetOldFleetNameCache();
  // --- save fleet local storage ---
  saveSwitchFleetOpt();

  toggleRenameFleetContent();
}


// ========================
//     RESTful Requests
// ========================
async function chkRoleInUsedOperations(_fleetName, _fileNameArray, _deletedRole, modeName) {
  var isLastChecked = modeName === 'flows';
  var data = await restRoleInUsedOperations(_fileNameArray, _deletedRole);
  if (modeName === 'flows') {
    deleteOperation.flows = Object.values(data);
  } else if (modeName === 'tasks') {
    deleteOperation.tasks = Object.values(data);
  }

  if (data.length > 0) {
    operationTextArr.push(modeName);
  }
  if (isLastChecked && operationTextArr.length > 0) {
    $("#save-fleet-lbl").text("There are " + operationTextArr.join(",") + " still using this role. Are you sure you want to save the changes?");
    $("#save-fleet-modal").modal('show');
  } else if (isLastChecked && operationTextArr.length == 0) {
    saveFleetSettings(_fleetName, settingsCache_.old_fleet_name);
  }
}


// ======================
//         Views
// ======================
// ------ agent view ------
function createAgentCandidateView(_data) {
  console.log(_data);
  const template = document.querySelector('#agent-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollAgentToFleetCb.bind(this, _data));

  var agentKey = _data.model.toLowerCase();
  var imgNode = node.querySelector('.product-img > img');
  // imgNode.src = '/images/' + agentAssets_[_data.model.toLowerCase()].thumbnail;
  imgNode.src = agentAssets_[agentKey].thumbnail;

  var titleNode = node.querySelector('.product-title').childNodes[0];
  avoidNoneName(titleNode, _data.robot_name, _data.robot_id);

  var statusNode = node.querySelector('.product-title > span');
  statusNode.textContent = _data.mode;

  // -- badge style --
  if (_data.mode.toLowerCase() === 'active') {
    statusNode.classList.add('badge-success');
  } else if (_data.mode.toLowerCase() === 'manual') {
    statusNode.classList.add('badge-warning');
  } else if (_data.mode.toLowerCase() === 'off') {
    statusNode.classList.add('badge-danger');
  }

  var infoNode = node.querySelector('.product-description');
  infoNode.textContent = `Model: ${_data.model}, IP: ${_data.ip}`;

  return node;
}

function createAgentMemberView(_data) {
  console.log(_data);
  const template = document.querySelector('#member-card');
  const node = document.importNode(template.content, true);

  var imgNode = node.querySelector('.card-img-top');

  // --- default model: 'smr250' ---
  _data.model = (_data.model !== 'None') ? _data.model : 'smr250';

  // --- get avatar ---
  var model = _data.model.toLowerCase();
  // imgNode.src = '/images/' + agentAssets_[model].avatar;
  imgNode.src = agentAssets_[model].avatar;
  // console.log(imgNode.src);

  console.log(_data);
  var titleNode = node.querySelector('.info-box-number');
  titleNode.dataset.robotId = _data.robot_id;
  avoidNoneName(titleNode, _data.robot_name, _data.robot_id);

  var modeNode = node.querySelector('.agent-mode');
  modeNode.textContent = _data.mode;
  if ('not available' == modeNode.textContent.toLowerCase().trim()) {
    modeNode.style.color = 'Red';
  } else if ('maintain' == modeNode.textContent.toLowerCase().trim()) {
    modeNode.style.color = 'Yellow';
  }

  // --- reboot agent ---
  var rebootNode = node.querySelector('.agent-inconsistency');
  if (modeNode.textContent === 'Inconsistency') {
    rebootNode.style.visibility = "visible";
  } else {
    rebootNode.style.visibility = "hidden";
  }

  rebootNode.setAttribute('data-toggle', 'modal');
  rebootNode.setAttribute('data-target', '#sync-confirm-modal');
  rebootNode.addEventListener('click', syncAgentFromFleetCb.bind(cardNode, _data));


  // --- remove agent ---
  var cardNode = node.querySelector('.card');
  var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  cardNode.classList.add(cardClass);
  var closeNode = node.querySelector('.agent-remove');
  closeNode.addEventListener('click', removeAgentFromFleetCb.bind(cardNode, _data));
  closeNode.setAttribute('data-btn-id', `${_data.robot_id}-delete-btn`);

  var settingsNode = node.querySelector('.agent-settings');
  settingsNode.addEventListener('click', updateSidebarAsyncCb.bind(this, _data));
  settingsNode.setAttribute('data-btn-id', `${_data.robot_id}-edit-btn`);

  var syncClass = getSavedTheme() === 'dark' ? 'dark-cloud-sync' : 'light-cloud-sync';
  console.log(node.querySelector('.fa'))
  var syncNode = node.querySelector('.fa');
  syncNode.textContent = '';
  syncNode.classList.remove('fa');
  syncNode.classList.add(syncClass);

  return { agent: node };
}

// ------ artifact view ------
function createArtifactCandidateView(_data) {
  const template = document.querySelector('#agent-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollArtifactToFleetCb.bind(this, _data));

  // console.log(_data);
  var imgNode = node.querySelector('.product-img > img');
  let imgSrc = ""
  if (_data.hasOwnProperty('thumbnail')) {
    imgSrc = `data:image/png;base64, ${_data['thumbnail']}`;
  } else {
    // console.log(artifactAssets_[_data.type]);
    // imgSrc = '/images/' + artifactAssets_[_data.type].thumbnail;
    imgSrc = artifactAssets_[_data.type.toLowerCase()].thumbnail;
  }
  imgNode.src = imgSrc;

  var titleNode = node.querySelector('.product-title').childNodes[0];
  console.log(_data);
  avoidNoneName(titleNode, _data.artifact_name, _data.artifact_id);

  var statusNode = node.querySelector('.product-title > span');
  statusNode.textContent = _data.status;

  var infoNode = node.querySelector('.product-description');
  infoNode.textContent = `Model: ${_data.model}, IP: ${_data.ip}`;

  return node;
}

function createArtifactMemberView(_data, bRmBtn = true) {
  console.log(_data);
  const template = document.querySelector('#member-card');
  const node = document.importNode(template.content, true);

  var imgNode = node.querySelector('.card-img-top');
  //  if data contains avatar attribute, show
  //  else fallback by referencing to its type
  let imgSrc = "";
  if (_data.hasOwnProperty('avatar')) {
    imgSrc = `data:image/png;base64, ${_data['avatar']}`;
  } else {
    const type = _data.type;
    // imgSrc = '/images/' + artifactAssets_[type].avatar;
    imgSrc = artifactAssets_[type.toLowerCase()] === undefined ? artifactAssets_['none'].avatar : artifactAssets_[type.toLowerCase()].avatar;
  }
  imgNode.src = imgSrc;

  var titleNode = node.querySelector('.info-box-number');
  titleNode.id = _data.artifact_id;
  avoidNoneName(titleNode, _data.artifact_name, _data.artifact_id);

  var statusNode = node.querySelector('.agent-mode');
  statusNode.textContent = _data.mode || 'not available';
  if ('not available' == statusNode.textContent.toLowerCase().trim()) {
    statusNode.style.color = 'Red';
  } else if ('maintain' == statusNode.textContent.toLowerCase().trim()) {
    statusNode.style.color = 'Yellow';
  }


  var cardNode = node.querySelector('.card');
  var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  cardNode.classList.add(cardClass);

  // --- reboot artifact ---
  var rebootNode = node.querySelector('.agent-inconsistency');
  rebootNode.style.visibility = "hidden";
  // if (statusNode.textContent === 'not available') {
  //   rebootNode.style.visibility = "hidden";
  // }

  // ------ remove artifact ------
  var closeNode = node.querySelector('.agent-remove');
  closeNode.addEventListener('click', removeArtifactFromFleetCb.bind(cardNode, _data));

  var settingsNode = node.querySelector('.agent-settings');
  settingsNode.addEventListener('click', updateArtifactSidebarAsyncCb.bind(this, _data));

  if (!bRmBtn) {
    closeNode.parentNode.removeChild(closeNode);
  }

  if (_data.mode1 == "active") {
    settingsNode.parentNode.removeChild(settingsNode);
  }

  return node;
}

// ------ map view ------
async function createMapCandidateView(_data) {
  // console.log(_data);
  const template = document.querySelector('#agent-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollMapToFleetCb.bind(this, _data));

  var imgNode = node.querySelector('.product-img > img');
  let reqData = await restGetMapThumbnail(_data.map_name);
  imgNode.src = reqData.data;

  let scaleFontSize = getFontScaleSize(16);
  var titleNode = node.querySelector('.product-title');
  titleNode.style.fontSize = `${scaleFontSize}px`;
  titleNode.childNodes[0].textContent = _data.map_alias;

  return node;
}

async function createMapMemberView(_data) {
  console.log(_data);
  const template = document.querySelector('#map-card');
  const node = document.importNode(template.content, true);

  var titleNode = node.querySelector('.map-name');
  titleNode.textContent = _data.map_alias;

  var imgSrc = '';
  var altText = 'image not found';
  if (_data.map_name !== null) {
    let reqData = await restGetMapThumbnail(_data.map_name);
    console.log(reqData);
    imgSrc = reqData.data;
    altText = _data.map_name + '.png';
  }

  var imgNode = node.querySelector('.map-thumbnail');
  imgNode.src = imgSrc;
  imgNode.alt = altText;

  var cardNode = node.querySelector('.card');
  var removeNode = node.querySelector('.remove-map');

  removeNode.addEventListener('click', removeMapFromFleetCb.bind(cardNode, _data));

  return node;
}

// ------ role view ------
function createRoleCandidateView(_data) {
  const template = document.querySelector('#agent-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollRoleToFleetCb.bind(this, _data));

  var imgNode = node.querySelector('.product-img > img');
  imgNode.src = `${getImagePath()}/sprites/role-150x150-light.png`;

  let scaleFontSize = getFontScaleSize(16);
  var titleNode = node.querySelector('.product-title');
  titleNode.style.fontSize = `${scaleFontSize}px`;
  titleNode.childNodes[0].textContent = _data.role_name;

  return node
}

function createRoleMemberView(_data) {
  const template = document.querySelector('#role-card');
  const node = document.importNode(template.content, true);

  var titleNode = node.querySelector('.role-name');
  titleNode.textContent = _data.role_name;
  console.log(_data);

  var cardNode = node.querySelector('.card');
  var removeNode = node.querySelector('.remove-role');

  removeNode.addEventListener('click', removeRoleFromFleetCb.bind(cardNode, _data.role_name));

  return node
}


// -----------------------
//     Firmware Update
// -----------------------
// const agentFirmware = document.getElementById('agent-firmware');
// agentFirmware.addEventListener('click', updateAgentFirmware);
// function updateAgentFirmware() {
//   // console.log('agent-firmware check !');
//   const badge = this.querySelector('.badge');
//   console.log(badge);
//   let verDisplay = this.previousElementSibling;
//   console.log(verDisplay);

//   // --- pop-up modal ---
//   var txt;
//   if (confirm("Update firmware?")) {
//     txt = "Firmware Updating!";
//     console.log(txt)

//     badge.textContent = '';

//     //  TODO: --- firmware update procedure ---
//     // --- get selected version ---
//     var select = document.getElementById('agent-firmware-ver');
//     var value = select.options[select.selectedIndex].value;

//     // --- send the requseted version to agent ---
//     console.log(settingsCache_); // get the agents
//     var fleet = settingsCache_.fleet;
//     var fltKey = Object.keys(fleet)[0]; // suppose only one key-value pair
//     var robots = fleet[fltKey].agents;
//     // console.log(robots);

//     // TODO: improve the way to generate string.
//     var agentStr = '[';
//     if (robots.length > 0) {
//       robots.forEach(r => {
//         agentStr += `"${r}",`;
//       });
//       agentStr = agentStr.slice(0, -1);
//     }
//     agentStr += ']';
//     // console.log(agentStr);

//     fetchPutAgentSwVer(agentStr, value);

//     //  --- Notification ---
//     toast('Agent Firmware Updated!');
//   } else {
//     txt = "Update Cancel!";
//     console.log(txt)
//   }
//   document.getElementById("confirm-modal").innerHTML = txt;
// }


const artifactMessages_ = '{"artifact_id": "conveyor_06", "artifact_name": "test", "type": "conveyoer", "sim": true, "services": {"convey_forward": {"state": true, "time_s": 10}, "convey_backward": {"state": false, "time_s": 10}, "sensor_front": {"wms_id": "l0_in", "state": false, "enable": false}, "sensor_back": {"wms_id": "l0_out", "state": false, "enable": false}}}';

$('#sidebar-test2').on('click', btnArtifactConf);

function btnArtifactConf() {
  $('#sb-agent-icon').attr('src', `${getImagePath()}/sprites/conveyor-150x150.png`);
  $('#sb-agent-name').val('Conveyor');
  $('#sb-agent-mode').val('Active');
  $('#sb-agent-model').val('Conveyor');
  $('#sb-model-label').text('Wrapper: ');

  // --- create category panel dynamically ---
  var artifactServices = JSON.parse(artifactMessages_)
  // console.log(artifactServices);
  artifactServices = artifactServices.services;
  console.log(artifactServices);
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  removeAllChildNodes(navTab);
  // console.log(navTab);
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  removeAllChildNodes(tabContent);

  for (key in artifactServices) {
    // --- append tabs ---
    var liEl = document.createElement("li");
    liEl.setAttribute("class", "nav-item");
    var aEl = document.createElement("a");
    var classTag = "nav-link";
    aEl.setAttribute("class", classTag);
    aEl.setAttribute("data-toggle", "tab");
    aEl.setAttribute("font-size", "1.5rem");
    aEl.setAttribute("href", `#${key}`);
    aEl.textContent = key;
    liEl.append(aEl);

    navTab.append(liEl);

    // --- append panes ---
    // console.log(param);
    const template = document.querySelector('#params-tab-pane');
    const node = document.importNode(template.content, true);

    var paneNode = node.querySelector('.tab-pane');
    paneNode.id = key;

    var leftColNode = node.querySelector('.left-col');
    leftColNode.id = `${key}-left-col`;

    var rightColNode = node.querySelector('.right-col');
    rightColNode.id = `${key}-right-col`;

    tabContent.append(node);

    // --- append params in the pane ---
    var params = artifactServices[key];
    var pCount = 0;
    for (pKey in params) {
      const template = document.querySelector('#param-group-row');
      const node = document.importNode(template.content, true);

      var nameNode = node.querySelector('.param-label');
      nameNode.innerHTML = `<span>${pKey}</span>`;
      var valNode = node.querySelector('.form-control');
      valNode.value = params[pKey];
      var btnEditNode = node.querySelector('.param-edit');
      var inputGroupNode = node.querySelector('.input-group');
      btnEditNode.addEventListener('click', editButtonSwitch.bind(inputGroupNode));

      console.log(key);
      var tag = (pCount % 2) ? `#${key}-left-col` : `#${key}-right-col`;
      var $paramCol = $(tag);
      console.log($paramCol);
      $paramCol.append(node);

      pCount++;
    }
  }

  // --- activate first pane ---
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  var firstChild = navTab.firstChild.firstChild;
  console.log(firstChild);
  firstChild.classList.add('active');
  // navTab.firstChild.classList.add('active');
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  console.log(tabContent)
  var tcFirstChild = tabContent.children[0];
  // var tcFirstChild = tabContent.firstChild; // FAIL!
  // console.log(tcFirstChild);
  tcFirstChild.classList.add('active');
}

let keyMappingTable_ = {};

function updateParamTabsView(_data, _langData) {
  var conf = _data;
  var paramObj = conf;
  if (typeof conf !== 'object') {
    paramObj = JSON.parse(conf);
  }
  console.log(paramObj);

  var flattedParamObj = flattenJSON(paramObj);
  console.log(flattedParamObj);
  console.log(Object.keys(flattedParamObj).length);

  keyMappingTable_ = buildKeyTable(flattedParamObj);
  console.log(keyMappingTable_);
  flattedParamObj = extractParamData(flattedParamObj);
  console.log(flattedParamObj);

  // --- get param-category ---
  var paramTabs = [];
  flattedParamObj.forEach((obj) => {
    if (paramTabs.indexOf(obj.pkg) == -1) paramTabs.push(obj.pkg);
  });
  console.log(paramTabs);

  // --- create category panel dynamically ---
  var sbParamCard = document.getElementById('sidebar-param-card');
  var navTab = sbParamCard.querySelector('.card-header > .nav-tabs');
  removeAllChildNodes(navTab);
  // console.log(navTab);
  var tabContent = sbParamCard.querySelector('.card-body > .tab-content');
  removeAllChildNodes(tabContent);

  var count = 0;
  let langCategory = _langData.category;
  paramTabs.forEach((param) => {
    // --- append tabs ---
    var liEl = document.createElement("li");
    liEl.setAttribute("class", "nav-item");
    var aEl = document.createElement("a");
    if (count === 0) {
      aEl.classList.add("nav-link", "active");
    } else {
      aEl.classList.add("nav-link");
    }
    aEl.setAttribute("data-toggle", "tab");
    aEl.setAttribute("font-size", "1.5rem");
    aEl.setAttribute("href", `#${param}`);
    // aEl.textContent = param;
    aEl.textContent = (langCategory[param]) ? langCategory[param] : param;
    liEl.append(aEl);

    navTab.append(liEl);

    // --- append panes ---
    const template = document.querySelector('#params-tab-pane');
    const node = document.importNode(template.content, true);

    var paneNode = node.querySelector('.tab-pane');
    if (count === 0) {
      paneNode.classList.add('active');
    }

    paneNode.id = param;

    var leftColNode = node.querySelector('.left-col');
    leftColNode.id = `${param}-left-col`;

    var rightColNode = node.querySelector('.right-col');
    rightColNode.id = `${param}-right-col`;
    // console.log(rightColNode);

    tabContent.append(node);
    count++;
  });
}

let paramRecord_ = {};

function updateParamMembersView(_data, _langData) {
  let confData = _data;
  let paramObj = confData;
  if (typeof confData !== 'object') {
    paramObj = JSON.parse(confData);
  }
  console.log(paramObj);

  let flattedParamObj = flattenJSON(paramObj);
  // console.log(flattedParamObj);
  // console.log(_langData);
  paramRecord_ = flattedParamObj;

  enableConfirm = paramObj.system.initial_pose_confirmed;
  toggleInitPoseConfirmStatus();

  // --- get params and append to panel ---
  paramCounter = {};
  for (var key in flattedParamObj) {
    // console.log(key);
    const template = document.querySelector('#param-group-row');
    const node = document.importNode(template.content, true);

    var fullNames = key.split('.');
    var nameLen = fullNames.length;
    let category = fullNames[0];
    // console.log(category);
    let uiName = fullNames[nameLen - 1];
    if (nameLen > 3) {
      uiName = fullNames[nameLen - 2] + '.' + uiName;
    }

    var nameNode = node.querySelector('.param-label');
    nameNode.innerHTML = `<span>${_langData[category][uiName].title}</span>`;
    var pen_node = node.querySelector('.input-group-append');

    // parameter description
    var descKey = key.concat('.', 'description');
    var rangeKey = key.concat('.', 'valid_value.data_range');
    var unitKey = key.concat('.', 'unit');
    // var desc = conf_definition[descKey] || '';
    var desc = _langData[category][uiName].description || '';
    var range = conf_definition[rangeKey] || '';
    var unit = conf_definition[unitKey] || '';
    var rangeTranStr = range.replace(/[[\]]/g, '').replace(':', '~');

    if (!isEmptyString(desc) || !isEmptyString(rangeTranStr) || !isEmptyString(unit)) {
      // var descStr = !isEmptyString(desc) ? `Description: ${desc}\n` : '';
      // var rangeStr = !isEmptyString(rangeTranStr) ? `\nInput range: ${rangeTranStr}\n` : '';
      // var unitStr = !isEmptyString(unit) ? `\nUnit: ${unit}` : '';
      var descStr = !isEmptyString(desc) ? `${langTemplateObj_.agnt.ttp_SidebarDesc}: ${desc}\n` : '';
      var rangeStr = !isEmptyString(rangeTranStr) ? `\n${langTemplateObj_.agnt.ttp_SidebarRange}: ${rangeTranStr}\n` : '';
      var unitStr = !isEmptyString(unit) ? `\n${langTemplateObj_.agnt.ttp_SidebarUnit}: ${unit}` : '';

      var tooltip = document.createElement('i');
      tooltip.classList.add('fa', 'fa-info-circle', 'custom-tooltip');
      tooltip.setAttribute('data-toggle', 'tooltip');
      tooltip.title = descStr + rangeStr + unitStr;
      nameNode.appendChild(document.createTextNode(" "));
      nameNode.appendChild(tooltip);
    }

    var valNode = node.querySelector('.form-control');
    valNode.value = flattedParamObj[key];
    valNode.id = key;
    valNode.name = key;

    let noNeedCount = false;
    var isEditMode = !enableConfirm;
    switch (key) {
      case "system.initial_pose_x":
      case "system.initial_pose_y":
      case "system.initial_pose_yaw":
        var inputGroupNode = node.querySelector('.input-group').closest('div[class="row"]');
        inputGroupNode.style.display = 'none';
        noNeedCount = true;
        break;
      case "system.fleet_name":
      case "system.initial_pose_confirmed":
        pen_node.remove();
        break;
      case "system.map":
        var inputGroupNode = node.querySelector('.input-group');
        inputGroupNode.style.display = 'none';

        // -- generate map select options list --
        var fltKey = Object.keys(settingsCache_.fleet)[0];
        var mapCacheArray = settingsCache_.fleet[fltKey].maps;
        var mapVal = flattedParamObj[key];
        console.log('map value from agent settings: ' + mapVal);
        var mapSelect = document.createElement('select');
        mapSelect.id = 'map-select';
        mapSelect.setAttribute('class', 'form-control');
        mapSelect.disabled = !isEditMode;
        if (mapCacheArray.length > 0) {
          mapCacheArray.forEach((map) => {
            addOption(mapSelect, map, mapVal);
          });
          // --- protection for map name 'none' ---
          if (!mapCacheArray.includes(mapVal)) {
            addOption(mapSelect, mapVal, mapVal);
          }
        } else {
          addOption(mapSelect, 'none', mapVal);
        }

        mapSelect.addEventListener('change', setInputValue.bind(inputGroupNode, mapSelect));
        inputGroupNode.parentNode.appendChild(mapSelect);

        // -- set input value to currently selected map --
        var mapSelectNode = node.querySelector('#map-select');
        valNode.value = mapSelectNode.value;
        break;
      case "network.bridge_mode":
        valNode.value = flattedParamObj[key] === 'zenoh' ? 'on' : 'off';

        if (!isEditMode) { break; }

        node.querySelector('.input-group-text').remove();
        var inputGroupAppend = node.querySelector('.input-group-append');
        var btn = document.createElement('button');
        btn.innerHTML = `Set ${flattedParamObj[key] === 'zenoh' ? 'Off' : 'On'}`;
        btn.classList.add('btn', 'btn-sm', 'btn-secondary');
        inputGroupAppend.appendChild(btn);

        var inputGroupNode = node.querySelector('.input-group');
        btn.addEventListener('click', bridgeModeButtonSwitch.bind(inputGroupNode));
        break;
      default:
        if (!isEditMode) { break; }

        if (typeof flattedParamObj[key] == "boolean") {
          // --- replace edit button to boolean switch button ---
          var stringVal = (!flattedParamObj[key]).toString();
          var boolString = stringVal.charAt(0).toUpperCase() + stringVal.slice(1);

          node.querySelector('.input-group-text').remove();
          var inputGroupAppend = node.querySelector('.input-group-append');
          var btn = document.createElement('button');
          btn.innerHTML = `Set ${boolString}`;
          btn.classList.add('btn', 'btn-sm', 'btn-secondary');
          inputGroupAppend.appendChild(btn);

          var inputGroupNode = node.querySelector('.input-group');
          btn.addEventListener('click', boolButtonSwitch.bind(inputGroupNode));
        } else {
          // --- add valid range hint text and limit string length --
          var validDataTypeKey = key.concat('.', 'valid_value.data_type');
          var validDataRangeKey = key.concat('.', 'valid_value.data_range');
          var unitKey = key.concat('.', 'unit');
          if (conf_definition.hasOwnProperty(validDataTypeKey) && conf_definition.hasOwnProperty(validDataRangeKey)) {
            var validDataType = conf_definition[validDataTypeKey];
            var validDataRange = conf_definition[validDataRangeKey].replace(/[[\]]/g, '');
            var unit = conf_definition[unitKey];
            var valNode = node.querySelector('.form-control');
            if (validDataType === 'int' || validDataType === 'double') {
              valNode.placeholder = `${validDataRange.replace(':', '~')} ${unit}`;
            } else if (validDataType === 'string' && validDataRange.split(':').length === 2) {
              let maxVal = validDataRange.split(':')[1];
              valNode.maxLength = maxVal;
              valNode.classList.add("agent-params");
              // valNode.classList.add("param-check");
              // valNode.addEventListener('keyup', checkInvalidParam);
            }
            valNode.classList.add("param-check");
            valNode.addEventListener('keyup', checkInvalidParam);
          }

          // --- enable edit switch ---
          var btnEditNode = node.querySelector('.param-edit');
          var inputGroupNode = node.querySelector('.input-group');
          if (btnEditNode !== null) {
            btnEditNode.addEventListener('click', editButtonSwitch.bind(inputGroupNode));
          }
        }
        break;
    }

    var iPkg = fullNames[0];
    var tag;

    if (!paramCounter.hasOwnProperty(iPkg)) {
      paramCounter[iPkg] = {
        count: 0
      };
    }

    if (!noNeedCount) {
      paramCounter[iPkg].count++;
    }

    var name = iPkg;
    tag = (paramCounter[iPkg].count % 2) ? `#${name}-left-col` : `#${name}-right-col`;

    var $paramCol = $(tag);
    console.log(paramCounter)
    $paramCol.append(node);
  }
}

function updateConfActionView(_data) {
  // --- update modal objects ---
  $('#import-conf-btn').off('click');
  $('#import-conf-btn').on('click', function () {
    importConfFile('agent', _data);
  });

  _data = JSON.parse(_data);
  let agentID = $('#sb-agent-id').val();
  updateConfDefaultFileName(agentID);
  $('#export-conf-btn').off('click');
  $('#export-conf-btn').click(_data, exportConfFile);

  $('#reset-conf-btn').off('click');
  $('#reset-conf-btn').click('agent', resetConfFile);

  // --- generate action buttons ---
  var btnGroupDiv = $('#settings-btns-group');
  btnGroupDiv.empty();

  var importBtn = document.createElement('button');
  importBtn.setAttribute('class', 'btn btn-primary mx-1');
  importBtn.setAttribute('data-toggle', 'modal');
  importBtn.setAttribute('data-target', '#upload-conf-modal');
  // [lang] import button
  let lang = getSetLang() || 'en';
  console.log(lang);
  // importBtn.innerHTML = `<i class="fas fa-file-import"></i> Import`;
  importBtn.innerHTML = `<i class="fas fa-file-import"></i> ${langTemplateObj_.btn_SidebarImport}`;

  btnGroupDiv.append(importBtn);

  var exportBtn = document.createElement('button');
  exportBtn.setAttribute('class', 'btn btn-primary mx-1');
  exportBtn.setAttribute('data-toggle', 'modal');
  exportBtn.setAttribute('data-target', '#save-conf-modal');
  // exportBtn.innerHTML = `<i class="fas fa-file-export"></i> Export`;
  exportBtn.innerHTML = `<i class="fas fa-file-export"></i> ${langTemplateObj_.btn_SidebarExport}`;
  btnGroupDiv.append(exportBtn);

  let agent_mode = $('#sb-agent-mode').val();
  updateResetViewByMode(agent_mode);

  activateFirstActionView();
}

function updateResetViewByMode(mode) {
  if (mode === 'manual' || mode === 'maintain') {
    $('a[href="#reset"]').show();
    var resetDiv = $('#reset-div');
    resetDiv.empty();

    var resetText = document.createElement('div');
    resetText.innerHTML = 'Erase all data:';
    resetDiv.append(resetText);

    var resetBtn = document.createElement('button');
    resetBtn.setAttribute('class', 'btn btn-primary mx-2');
    resetBtn.setAttribute('data-toggle', 'modal');
    resetBtn.setAttribute('data-target', '#delete-conf-modal');
    resetBtn.innerHTML = `<i class="fas fa-trash"></i> Delete`;
    resetDiv.append(resetBtn);
  } else {
    $('a[href="#reset"]').hide();
  }
}

function activateFirstActionView() {
  // --- activate first pane ---
  let sbFuncCard = document.getElementById('sidebar-func-card');
  let navTab = sbFuncCard.querySelector('.card-header > .nav-tabs');
  navTab.querySelector('.active')?.classList?.remove("active");
  let firstChild = getFirstChild(navTab).firstChild;
  // console.log(firstChild);
  firstChild?.classList?.add('active');
  let tabContent = sbFuncCard.querySelector('.card-body > .tab-content');
  tabContent.querySelector('.active').classList.remove("active");
  let tcFirstChild = tabContent.children[0];
  // console.log(tcFirstChild);
  tcFirstChild.classList.add('active');
}

function getFirstChild(el) {
  var firstChild = el.firstChild;
  while (firstChild != null && firstChild.nodeType == 3) { // skip TextNodes
    firstChild = firstChild.nextSibling;
  }
  return firstChild;
}

async function switchFleetCallback() {
  deletedRole = [];

  // --- flush agents in each fleet ---
  allFleetAgents_ = [];
  allFleetsName_ = [];

  resetOldFleetNameCache();

  // -- load all fleet configurations --
  let fleets;
  try {
    // fleets = await restGetFleets();
    await rmtTokenCheck();
    fleets = await fetchGetFleets(rmtToken_);

    // fleets = fleets.fleet_list;
    fleets = Object.keys(fleets);

    // -- check fleet is saved or not --
    let chkVal = await chkFleetFileSaved(fleets);
    if (!chkVal) return;

    asyncReadFleetConfig(fleets);
  } catch (err) {
    console.error(err);
  }

  await loadFleetSettings();
}

let isFleetSaved;
async function chkFleetFileSaved(fleets) {
  var fleetCache = Object.keys(settingsCache_.fleet)[0];
  // console.log(fleetCache);

  if (fleetCache === undefined) return true;

  isFleetSaved = false;
  // console.log(fleets)
  $.each(fleets, function (i) {
    // let fleet = fleets[i].split('.').slice(0, -1).join('.');
    let fleet = fleets[i];
    if (fleet === fleetCache) {
      isFleetSaved = true;
    }
  });

  if (!isFleetSaved) {
    alert('You haven\'t save the fleet configuration!');
    $('#fleet-select').val(fleetCache);

    // -- reset side menu trigget list --
    await sleep(200); // sleep in ms
    resetTriggerItem();
    chkTriggerListVisible();
  }
  return isFleetSaved;
}

function renameFleet(e) {
  e.preventDefault();
  if (!$('#rename-fleet-form').valid()) {
    return;
  }

  $('#rename-fleet-modal').modal('hide');

  var fleetName = $('#fleet-name').val();
  var newFleetName = $('#new-fleetname').val();

  // -- rename cache data fleet name
  settingsCache_.fleet[newFleetName] = settingsCache_.fleet[fleetName];
  delete settingsCache_.fleet[fleetName];
  settingsCache_.old_fleet_name = fleetName;

  updateFleetNameTextInput(newFleetName);
  $(`#fleet-select option:contains("${fleetName}")`).val(`${newFleetName}`);
  $(`#fleet-select option:contains("${fleetName}")`).text(`${newFleetName}`);
}

async function createFleet(e) {
  e.preventDefault();
  if (!$('#create-fleet-form').valid()) { return; }
  $('#new-fleet-modal').modal('hide');

  // --- append the new fleet name to select drop-down menu ---
  var fleetName = $('#fleet-filename').val();
  console.log(fleetName);
  $('#fleet-select').append(`<option value='${fleetName}'>${fleetName}</option>`);
  var newFleetName = $('#fleet-select option:last').val();
  $('#fleet-select').val(newFleetName);

  // --- clear the context ---
  var $agentDeck = $('#agents-deck');
  $agentDeck.empty();

  var $artifactDeck = $('#artifacts-deck');
  $artifactDeck.empty();

  var $roleDeck = $('#roles-deck');
  $roleDeck.empty();

  var $mapDeck = $('#maps-deck');
  $mapDeck.empty();

  // --- update SettingsCache_ ---
  var cacheFleet = `{"${fleetName}":{"agents":[],"artifacts":{"external":[], "agent":[]},"maps":[],"roles":[], "flows":[]}}`;
  settingsCache_.fleet = JSON.parse(cacheFleet);

  toggleRenameFleetContent();

  // ------ generate task trigger list ------
  genTaskTrigger(fleetName);

  // --- update core settings ---
  // var coreSettings = await fetchGetCoreSettings(rmtToken_);
  // coreSettings = coreSettings.core_settings;
  // coreSettings = coreSettings.replace(/'/g, '"');
  // coreSettings = JSON.parse(coreSettings);
  // coreSettings = coreSettings['far_sys'].swarm_core;
  // settingsCache_.core = coreSettings;
  // updateCoreSettingsView(coreSettings);

  // ------ create a new fleet ------
  let res = await fetchPostFleetConfigs(rmtToken_, fleetName);
  if (res.ok) {
    notificationMsg(0, `${fleetName} is created!`);
  }
}

async function checkFleet() {
  executingFlow = await checkProtection();
  // console.log(executingFlow)

  // --- get fleet name from select ---
  let fleetName = $('#fleet-select').val();

  if (Object.keys(executingFlow["using_fleets"]).includes(fleetName)) {
    notificationMsg(3, 'Fleet has executing task, cannot delete.');
    return;
  }

  // let fleets = await restGetFleets();
  let fleets = await fetchGetFleets(rmtToken_);
  console.log('--- updated fleet ---');
  console.log(JSON.stringify(fleets));
  fleets = Object.keys(fleets);
  console.log(JSON.stringify(fleets));
  // console.log(`Fleet count: ${fleets.length}`);

  // fleets = fleets.map(fleet => fleet.split('.').slice(0, -1).join('.'));
  // console.log(fleets);
  // console.log(fleetName);
  if (fleets.indexOf(fleetName) == -1) {
    alert('Fleet is NOT save yet!');
    return;
  }
  if (fleets.length === 1) {
    alert('Delete failed. Must have at least one fleet!');
    return;
  }

  // var selectedFleet = fleetName + '.yaml';
  let fltSettings;
  try {
    // fltSettings = await restGetFleetSettings(fleetName);
    fltSettings = await fetchGetFleetConfigs(rmtToken_, fleetName);
    console.log(JSON.stringify(fltSettings));
  }
  catch (err) {
    fltSettings = {};
    console.error(err);
  }

  let flows = fltSettings[fleetName]?.flows || [];
  let tasks = fltSettings[fleetName]?.tasks || [];
  let operation_text_arr = [];
  if (typeof tasks !== 'undefined' && tasks.length > 0) {
    operation_text_arr.push("tasks");
    deleteOperation.tasks = tasks;
  }
  if (typeof flows !== 'undefined' && flows.length > 0) {
    operation_text_arr.push("flows");
    deleteOperation.flows = flows;
  }
  if (operation_text_arr.length > 0) {
    $("#delete-operation-lbl").text("There are " + operation_text_arr.join(",") + " still using this fleet. Are you sure you want to delete?");
    $("#delete-fleet-modal").modal('hide');
    $("#delete-operation-modal").modal('show');
  } else {
    deleteFleet(fleetName);
  }
}

async function deleteRelatedOperations() {
  deleteOperations('fleet');
}

async function deleteRoleRelatedOperations() {
  const fleetName = $('#fleet-select').val();
  await saveFleetSettings(fleetName, settingsCache_.old_fleet_name);
  deleteOperations('fleetRole');
}

function deleteOperations(relatedType) {
  var fleetName = $('#fleet-select').val();
  var isFleet = relatedType === 'fleet';

  deleteOperation.tasks.forEach((task) => {
    var taskName = isFleet ? task : task.replace(`${fleetName}-task-`, '').replace('.json', '');
    restDeleteTaskData(fleetName, taskName, false);
    notificationMsg(0, `${taskName} is deleted`);
    deleteOperation.tasks = deleteOperation.tasks.filter((item) => item !== task);
  });
  deleteOperation.flows.forEach((flow) => {
    var flowName = isFleet ? flow : flow.replace(`${fleetName}-flow-`, '').replace('.json', '');
    restDeleteFlowData(fleetName, flowName, false);
    notificationMsg(0, `${flowName} is deleted`);
    deleteOperation.flows = deleteOperation.flows.filter((item) => item !== flow);
  });
  if (isFleet) {
    deleteFleet(fleetName);
  } else {
    wsRemoveFleet(fleetName, deletedRole.join(";"));
  }
}

async function deleteFleet(fleetName) {
  // --- send delete request to server ---
  try {
    // await restDeleteFleetConfData(fleetName + '.yaml');
    res = await fetchDeleteFleetConfigs(rmtToken_, fleetName);
    console.log(res);
    var delFleetIndex = allFleetsName_.indexOf(fleetName);
    allFleetsName_.splice(delFleetIndex, 1);
  } catch (err) {
    console.error(err);
  }

  // --- pop-up the modal to hint user ---
  notificationMsg(0, `Fleet:${fleetName} is deleted`)
  resetSavedFleet('delete', fleetName);

  // --- reload the options ---
  await loadFleetSettings();
}

$('#upload-conf-modal').on('hidden.bs.modal', function (e) {
  $("#conf-file-input").val('');
});

$('#hide-sidebar').on('click', function () {
  // $('.control-sidebar').ControlSidebar('toggle');
  if (openSidebar) {
    $('.control-sidebar').ControlSidebar('show');
    openSidebar = false;
  } else {
    $('.control-sidebar').ControlSidebar('collapse');
    openSidebar = true;
  }
});

$('#init-pose-confirm-btn').on('click', setInitPoseConfirmed);
let enableConfirm;
async function setInitPoseConfirmed() {
  displayInitPoseConfirmLoadingStyle();
  let agentID = $('#sb-agent-id').val();
  await setAgentSettings(agentID, !enableConfirm);

  let data = await getAgentSettings(agentID);
  if (isEmptyObjValue(data)) {
    notificationMsg(3, 'Failed to get initial pose confirmed state!');
    $('.control-sidebar').ControlSidebar('collapse');
    openSidebar = true;
    return;
  }
  enableConfirm = data.system.initial_pose_confirmed;
  console.log('agent settings init pose confiremd: ' + enableConfirm);

  updateParamTabsView(data, langData_);
  updateParamMembersView(data, langData_);
  updateSaveButtonByMode(agentID);

  notificationMsg(0, 'Initial pose confirmed set!');
}

function toggleInitPoseConfirmStatus() {
  if (enableConfirm) {
    $('#init-pose-confirm-btn').addClass('btn-success').removeClass('btn-danger btn-default');
    // $('#init-pose-confirm-btn').text('Ready');
    $('#init-pose-confirm-btn').text(langTemplateObj_.agnt.btn_Ready);
  } else {
    $('#init-pose-confirm-btn').addClass('btn-danger').removeClass('btn-success btn-default');
    // $('#init-pose-confirm-btn').text('Confirm Initial Pose');
    $('#init-pose-confirm-btn').text(langTemplateObj_.agnt.btn_Init);
  }
  $('#init-pose-confirm-btn').prop('disabled', false);
}

function showInitPoseConfirmButton() {
  $('#init-pose-confirm-btn').css("display", "block");
  $('#init-pose-confirm-btn-col').css("display", "block");
}

function hideInitPoseConfirmButton() {
  $('#init-pose-confirm-btn').css("display", "none");
  $('#init-pose-confirm-btn-col').css("display", "none");
}

function displayInitPoseConfirmLoadingStyle() {
  showInitPoseConfirmButton();
  $('#init-pose-confirm-btn').addClass('btn-default').removeClass('btn-success btn-danger');
  // $('#init-pose-confirm-btn').text('Loading...').prop('disabled', true);
  $('#init-pose-confirm-btn').text(langTemplateObj_.agnt.btn_Load).prop('disabled', true);
}

function lockAgentSettings() {
  $('.agent-settings').prop('disabled', true);
  $('.save-agent').prop('disabled', true);
}

function unlockAgentSettings() {
  $('.agent-settings').prop('disabled', false);
  $('.save-agent').prop('disabled', false);
}

// function toggleInitPoseConfirmParam() {
//   $('#sidebar-param-card input').each(function () {
//     if (this.id === 'system.initial_pose_confirmed') {
//       $(this).val(enableConfirm);
//     }
//   });
// }

function lockAgentParamView() {
  $('#overlay-div').show();
}

function unlockAgentParamView() {
  $('#overlay-div').hide();
}

function unlockArtifactParamView() {
  $('#overlay-div').css("display", "none");
}

function removeConfActionButtons() {
  $('#settings-btns-group').empty();
  $('#reset-div').empty();
}

function compareVersion(version1, version2) {
  console.log(`version1: ${version1}`);
  console.log(`version2: ${version2}`);
  if (version1 === undefined || version2 === undefined) {
    console.log('version numbers not availble');
    return;
  }

  const levels1 = version1.split('.');
  const levels2 = version2.split('.');

  const length = Math.max(levels1.length, levels2.length);

  for (let i = 0; i < length; i++) {
    const v1 = i < levels1.length ? parseInt(levels1[i]) : 0;
    const v2 = i < levels2.length ? parseInt(levels2[i]) : 0;

    if (v1 > v2) return 1;
    if (v2 > v1) return -1;
  }

  return 0;
};

async function getAgentRunningMap(_agentId) {
  var paramObj = await getAgentSettings(_agentId);
  if (Object.keys(paramObj).length === 0) { return ""; }
  console.log(paramObj);
  console.log(paramObj.system.map);
  return paramObj.system.map;
}

async function update_agent_fleet(_agent_id, _update_data) {
  var paramObj = await getAgentSettings(_agent_id);
  if (Object.keys(paramObj).length === 0) { return false; }
  paramObj.system.fleet_name = _update_data;

  var originalObj = unflattenJSON(paramObj);

  var configData = JSON.stringify(originalObj);
  configData = configData.replace(/true/g, 'True').replace(/false/g, 'False');
  configData = configData.replace(/"/g, "'");

  await rmtTokenCheck();
  var result = await fetchPutAgentSettings(rmtToken_, _agent_id, configData);
  console.log(result);
  if (result.hasOwnProperty('detail')) { return false; }
  return true;
}

async function getAgentSettings(_agent_id) {
  let data;
  try {
    await rmtTokenCheck();
    data = await fetchAgentSettings(rmtToken_, _agent_id);
  } catch (err) {
    console.error(err);
  }

  console.log(data);
  if (data.hasOwnProperty('detail') || !data.hasOwnProperty('agent_settings')
    || (data.hasOwnProperty('agent_settings') && data.agent_settings.length === 0)) {
    return {};
  }

  var nav_conf = data.agent_settings[0].nav_conf;
  nav_conf = nav_conf.replace(/'/g, '"');
  nav_conf = nav_conf.replace(/True/g, 'true').replace(/False/g, 'false');
  var paramObj = JSON.parse(nav_conf);
  return paramObj;
}

async function setAgentSettings(_agent_id, _update_data) {
  var jsonObj = {
    'system': {
      'initial_pose_confirmed': _update_data
    }
  };
  console.log(jsonObj);

  var configData = JSON.stringify(jsonObj);
  configData = configData.replace(/true/g, 'True').replace(/false/g, 'False');
  configData = configData.replace(/"/g, "'");
  console.log(configData);

  await rmtTokenCheck();
  await fetchPutAgentSettings(rmtToken_, _agent_id, configData);
}

function resetOldFleetNameCache() {
  settingsCache_.old_fleet_name = null;
}

async function exportConfFile(event) {
  const filename = $('#conf-filename').val();
  if (isEmptyString(filename)) {
    alert('Please enter file name!');
    return;
  }

  // console.log(event.data);
  let data = event.data;

  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], {
    type: "application/json"
  }));
  a.setAttribute("download", `${filename}.json`);
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  $('#save-conf-modal').modal('hide');
}

function importConfFile(_confType, _data = null) {
  const files = document.getElementById('conf-file-input').files;
  if (files.length === 0) return;
  const reader = new FileReader();
  reader.onload = function (event) {
    handleFileLoad(event, _confType, _data);
  };
  reader.readAsText(files[0]);
}

async function handleFileLoad(event, type, data) {
  var jsonFileContent = event.target.result;

  try {
    jsonFileContent = JSON.parse(jsonFileContent);

    if (data !== null) {
      var dataKeys = flattenJSON(data);
      dataKeys = Object.keys(dataKeys);

      var jsonFileKeys = flattenJSON(jsonFileContent);
      jsonFileKeys = Object.keys(jsonFileKeys);

      let matchKeys = _.intersection(dataKeys, jsonFileKeys);
      if (matchKeys.length === 0) {
        alert('Wrong JSON schema! Please check your file!');
        return;
      }
    }

    var configData = JSON.stringify(jsonFileContent);
    configData = configData.replace(/true/g, 'True').replace(/false/g, 'False');
    configData = configData.replace(/"/g, "'");
    // console.log(configData);

    let agentID = $('#sb-agent-id').val();
    await rmtTokenCheck();
    switch (type) {
      case 'agent': {
        let data = await fetchPutAgentSettings(rmtToken_, agentID, configData);
        if (data.hasOwnProperty('detail')) {
          notificationMsg(3, 'Import Agent Settings Failed!');
        } else {
          notificationMsg(0, 'Agent Settings Import!');
        }
        break;
      }
      case 'artifact': {
        let data = await fetchPutArtifactSettings(rmtToken_, agentID, configData);
        if (data.hasOwnProperty('detail')) {
          notificationMsg(3, 'Import Artifact Settings Failed!');
        } else {
          notificationMsg(0, 'Artifact Settings Import!');
        }
        break;
      }
      default: {
        notificationMsg(3, 'Import Failed!');
        return;
      }
    }

    $('#upload-conf-modal').modal('hide');
    $('.control-sidebar').ControlSidebar('collapse');
    openSidebar = true;

  } catch (e) {
    alert('Wrong JSON format! Please check your file!');
  }
}

async function resetConfFile(e) {
  let agentID = $('#sb-agent-id').val();
  await rmtTokenCheck();

  const confType = e.data;
  switch (confType) {
    case 'agent': {
      let data = await fetchResetAgentSettings(rmtToken_, agentID);
      console.log(data);
      notificationMsg(0, 'Agent Settings Reset!');
      break;
    }
    case 'artifact': {
      let data = await fetchResetArtifactSettings(rmtToken_, agentID);
      console.log(data);
      notificationMsg(0, 'Artifact Settings Reset!');
      break;
    }
    default: {
      notificationMsg(3, 'Reset Failed!');
      return;
    }
  }

  $('.control-sidebar').ControlSidebar('collapse');
  openSidebar = true;
}

let current_sidebar_mode;
function pollFleetStatesWithUpdates(_inteval = 1000) {

  setInterval(async function () {
    var currFleet = $('#fleet-select').val();
    let res;
    let scanned_agent_data = undefined;
    let agent_data = undefined;

    try {
      res = await fetchFleetStates(rmtToken_, currFleet);
    } catch (err) {
      console.error(err);
    }
    // console.log(res);
    let fleetData = [];
    if (res == undefined || !res.ok) { return; }

    var queryData = await res.json();
    fleetData = queryData.fleet_state.find(flt => flt.fleet_name === currFleet) || [];
    if (!fleetData) { return; }

    agent_data = fleetData.robots;
    // console.log(agent_data);

    // console.log(' ---- fetch agent -----')
    let scannedAgents;
    try {
      await rmtTokenCheck();
      scannedAgents = await fetchScanRobots2(rmtToken_);
    } catch (err) {
      console.error(err);
    }

    scanned_agent_data = scannedAgents.robots;

    // --- ui update ---
    var uiAgents = document.getElementById('agents-deck').querySelectorAll('.info-box-content');
    // console.log(uiAgents);

    const arrChildren = Array.apply(null, uiAgents);
    arrChildren.forEach(child => {
      // console.log(child);
      var titleNode = child.querySelector('.info-box-number');
      var id = titleNode.dataset.robotId;

      let target_agent_data = agent_data.filter(agent => agent.robot_id === id);
      let target_agent_scan_data = scanned_agent_data.filter(agent => agent.robot_id === id);

      let agent_id = undefined;
      let agent_name = undefined;
      let agent_fleet = undefined;
      let agent_mode = 'not availiable';

      if (target_agent_scan_data.length > 0) {
        agent_id = target_agent_scan_data[0].robot_id;
        agent_name = target_agent_scan_data[0].robot_name;
        agent_fleet = target_agent_scan_data[0].fleet_name;
        agent_mode = target_agent_scan_data[0].mode;
      } else {
        if (target_agent_data.length === 0) { return; }
        agent_id = target_agent_data[0].robot_id;
        agent_name = target_agent_data[0].robot_name;
        agent_fleet = target_agent_data[0].fleet_name;

        let connection_status = target_agent_data[0].connection_status;
        if (connection_status === 0) {
          agent_mode = target_agent_data[0].mode;
        }
      }

      avoidNoneName(titleNode, agent_name, agent_id);

      let status_label = $(child).find('.agent-mode');
      let colorStyle = 'red';
      let inconsistency_visibility = 'hidden';

      if (agent_fleet !== currFleet) {
        agent_mode = 'Inconsistency';
        colorStyle = 'red';
        inconsistency_visibility = 'visible';
      } else {
        if (agent_mode == 'manual') {
          colorStyle = 'red';
        } else if (agent_mode == 'maintain') {
          colorStyle = 'yellow';
        } else if (agent_mode == 'auto') {
          colorStyle = '';
        }

        // For sidebar update
        let current_sidebar_id = $('#sb-agent-id').val();
        if (current_sidebar_id == id && !openSidebar) {
          console.log(agent_name, agent_id)
          avoidNoneVal($('#sb-agent-name'), agent_name, agent_id);
          $('#sb-agent-mode').val(agent_mode);

          if (current_sidebar_mode === agent_mode) return;
          console.log(`${id} mode is change to ${agent_mode}`);
          if (tmp_agent_conf !== undefined) {
            console.log(tmp_agent_conf);
            updateParamTabsView(tmp_agent_conf, langData_);
            updateParamMembersView(tmp_agent_conf, langData_);
          }
          updateSaveButtonByMode(id);
          updateResetViewByMode(agent_mode);
          activateFirstActionView();
          current_sidebar_mode = agent_mode;
        }
      }

      child.querySelector('.agent-inconsistency').style.visibility = inconsistency_visibility;
      status_label.css('color', colorStyle);
      status_label.text(agent_mode);
    })

  }, _inteval);
};

function pollArtifactStatusWithUpdates(_inteval = 1000) {
  setInterval(async function () {
    var currFleet = $('#fleet-select').val();
    var allFltArtifId = allFleetArtifacts_.map(afa => afa.artifact_id);
    var res = await fetchArtifactStatus(rmtToken_, currFleet);
    let artifact_res = await fetchScanArtifacts2(rmtToken_);
    let artifactData = [];
    if (!res.ok) { return; }

    var queryData = await res.json();
    // console.log(queryData);
    artifactData = queryData.artifacts.filter(artif => allFltArtifId.includes(artif.id)) || [];
    if (!artifactData) { return; }

    // --- ui update ---
    // console.log(artifactData);
    var liveArtifacts = artifactData.map(a => a.id);
    var uiArtifacts = document.getElementById('artifacts-deck').querySelectorAll('.info-box-content');
    var artifactName_dict = {};
    artifact_res.artifacts.map(a => artifactName_dict[a.artifact_id] = a.artifact_name);
    // console.log(artifactData);
    // console.log(artifactName_dict)
    // console.log(uiArtifacts);

    const arrChildren = Array.apply(null, uiArtifacts);
    arrChildren.forEach(child => {
      // console.log(child);
      var id = child.querySelector('.info-box-number').id;
      if (id.includes('-')) {
        id = id.split('-')[1].trim();
      }
      var titleNode = child.querySelector('.info-box-number');

      let artfact_name = artifactName_dict[id];
      if (artfact_name == undefined) {
        artfact_name = 'none'
      }
      avoidNoneName(titleNode, artfact_name, id);

      if (liveArtifacts.includes(id)) {
        child.querySelector('.agent-mode').textContent = 'active';
        child.querySelector('.agent-mode').style.color = 'white';
        // child.querySelector('.agent-inconsistency').style.visibility = 'visible';
      } else {
        child.querySelector('.agent-mode').textContent = 'not available';
        child.querySelector('.agent-mode').style.color = 'red';
        // child.querySelector('.agent-inconsistency').style.visibility = 'hidden';
      }
    })
  }, _inteval);
};


$(document).on('click', '.re_agent', async function () {
  let agent_id = $('#sb-agent-id').val()

  let resetModal = document.getElementById('reboot-confirm-modal');
  let title = resetModal.querySelector('.modal-title');
  title.textContent = `Reboot ${agent_id}`

  let badge = resetModal.querySelector('.reset-target');
  badge.textContent = agent_id;

  const confirm = resetModal.querySelector('.reset-confirm');
  console.log(confirm);
  confirm.addEventListener('click', confirmRebootAgentFromFleetCb.bind(resetModal, agent_id));

  $('#reboot-confirm-modal').modal('show');
})

$(document).on('click', '.re_artifact', function () {
  let artifact_id = $('#sb-agent-id').val()

  let resetModal = document.getElementById('reboot-confirm-modal');
  let title = resetModal.querySelector('.modal-title');
  title.textContent = `Reboot ${artifact_id}`

  let badge = resetModal.querySelector('.reset-target');
  badge.textContent = artifact_id;

  let confirm = resetModal.querySelector('.reset-confirm');
  console.log(confirm);

  confirm.addEventListener('click', confirmRebootArtifactFromFleetCb.bind(resetModal, artifact_id));

  $('#reboot-confirm-modal').modal('show');
})

// --- add input values validator ---
function validateCellInputEvent(inputNode) {
  function debounce(func, delay = 200) {
    let timer = null;

    return () => {
      let context = this;
      let args = arguments;

      clearTimeout(timer);
      timer = setTimeout(() => {
        func.apply(context, args);
      }, delay)
    }
  }

  /*
  // --- [CONFIG] 1. Define Validation Flow ---
  //                  * run()         : run the valiations,
  //                  * do the styling and interaction logics
  **/
  function validationFlow() {
    console.log(this.id);
    this.value = this.value.replaceAll(' ', '');
    let validator;
    if (this.id === 'planning.footprint') {
      validator = new Rule('planning.footprint', footprintValidation);
    } else if (this.id === 'system.agent_name') {
      validator = new Rule('system.agent_name', textNameValidation);
    }
    if (this.id.includes('control') ||
      this.id.includes('localization') ||
      (this.id.includes('planning') && !this.id.includes('footprint')) ||
      this.id.includes('perception')) {
      validator = new Rule(this.id, numberValueValidation);
    }

    if (validator == undefined) { return; }

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---
    if (res.bValid) {
      $(this).css('box-shadow', '');
      $(this).css('border-color', "");
      $(this).css('outline-color', "");
    } else {
      $(this).css('box-shadow', '0 0 10px #CC0000');
      $(this).css('border-color', "red");
      $(this).css('outline-color', "red");
    }

    // vm.showPowerTip($(this), res.strMsg);

    // --- [UX] Interaction Logics ---
    $('#save-placeholder > button').prop('disabled', !res.bValid);
  }

  /*
  // --- [CONFIG] 2. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  if (!inputNode.classList.contains("keyup-handler")) {
    inputNode.classList.add("keyup-handler");
    inputNode.addEventListener("keyup", debounce(validationFlow.bind(inputNode)));
  }
}

function footprintValidation(inputVal) {
  console.log(inputVal);
  const reg1 = /^[-+., 0-9 \[\]]+$/;       // allow characters -+., 0-9, []

  const bRes = reg1.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- only allow -+., 0-9, []';
  strMsg.push(msg1);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg };
}

$(window).bind('beforeunload', function () {
  if (unsavedChanges) {
    return "You have unsaved changes on this page. Are you sure you want to leave?";
  }
});

// --- add input values validator ---
function validateFleetNameInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#create_fleet_btn');
  const fleetNameRule = new Rule('fleet-filename', textNameValidation, interaction_element);

  validatorManager.addValidator(fleetNameRule);
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

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---

    // --- [UX] Interaction Logics ---

  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('fleet-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}