/*
 * Author: John Wu
 * Date: 9 June 20,
 * Description:
 **/


// ======================
//        Models
// ======================
// let rmtToken_;

let settingsCache_ = {
  fleet: {},
  agent: {}
};
var fleetVer = {};
let core_definition = {};
var $form = $('#update-user-form');
var $chg_pwd_form = $('#change-pwd-form');
var $save_core_form = $('#save-core-settings-form');
var modifiedPwd = {
  name: "modifiedPwd",
  value: ""
};
// var uploadFailed = false;
// var checkCnt = 0;
let selAgents = [];
let licenseStepper;


// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  initRundownAsync();
  initCoreSettingValidation();
  initUserSettings();

  // --- load LINE notify groups ---
  getNotifyGroups();

  // --- [lang] load the default language ---
  // let lang = getSetLang() || 'en';
  // $("#sel-lang-env").val(lang);
});

async function initRundownAsync() {
  try {
    // ------ register user activity detector ------
    userActivityDetector();

    // ------ get login status ------
    var statusData = await restLoginStatus();
    await getLoginStatus(statusData, 'settings', 'settings2.html');

    // ------ adjust tab content according to the role ------
    removeAdminComponent(statusData);
    switchTab();

    // --- fetch RMT token initially --- 
    try {
      rmtToken_ = await fetchToken();
      // notificationMsg(1, 'RMT ONLINE!');
    } catch (err) {
      console.error(err);
      // notificationMsg(3, 'RMT OFFLINE!');
      // toast('Error: Fail to reach RMT!', 'rgba(255,0,0,0.6)');
    }

    // 1. fetch core settings
    var coreMetaData = await fetchCoreMetadata(rmtToken_);
    var coreSettings = await fetchGetCoreSettings(rmtToken_);

    // 2. fetch core version list
    var swList = await fetchGetSwVerList(rmtToken_);
    // console.log(swList);
    let lng = getSetLang() || 'en';
    console.log(lng);
    var langMsg = await restGetLangMsg(lng);


    // 3. fetch current version
    await genSoftwareVerObj();

  } catch (err) {
    console.error(err);
  }

  // 4. update core settings
  console.log(coreMetaData);
  console.log(coreSettings);
  updateCoreDefinitions(coreMetaData);
  updateCoreSettings(coreSettings, langMsg);

  // 5. update the status on VIEW 
  updateFirmwareView(swList, fleetVer);

  // 6. update current version
  updateCurrentFirmwareVer(fleetVer);

  // 7. license check and hardware signature
  // await sysLicenseCheck();
  // await sysHardwareSignature();
  await updateLicenseInfo();
  await initLicenseStepper();

  // 8. EULA
  await updateEulaStatus();

  bindButtonEvents();
  pollOtaConnStatus();

  // ------ language switch ------
  await initLanguageSupport();
}

async function genSoftwareVerObj() {
  fleetVer.core = await fetchGetCoreSwVer(rmtToken_);

  var scanned_robots = await getScannedAgents();
  scanned_robots = _.reject(scanned_robots, ['fleet_name', 'none']);
  var agentsVer = scanned_robots.map(r => r.sw_version);
  agentsVer = _.uniq(agentsVer);
  fleetVer.agent = agentsVer.length === 1 ? agentsVer[0] : '';

  if (scanned_robots.length === 0) {
    fleetVer.artifact = '';
    return;
  }

  var scannedArtifacts;
  try {
    scannedArtifacts = await fetchScanArtifacts2(rmtToken_);
  } catch (err) {
    console.error(err);
  }
  if (scannedArtifacts.hasOwnProperty('artifacts')) {
    var artifactsVer = scannedArtifacts.artifacts.map(a => a.sw_version);
    artifactsVer = _.uniq(artifactsVer);
    fleetVer.artifact = artifactsVer.length === 1 ? artifactsVer[0] : '';
  }
}

async function getScannedAgents() {
  var scannedAgents = [];
  try {
    scannedAgents = await fetchScanRobots2(rmtToken_);
    if (scannedAgents.hasOwnProperty('robots')) {
      return scannedAgents.robots;
    }
    return scannedAgents;
  } catch (err) {
    console.error(err);
    return scannedAgents;
  }
}

async function sysLicenseCheck() {
  let validateState = await fetchGetLicenseValidation(rmtToken_);
  console.log(validateState);
  let licenseState = { valid: 1 };
  if (!validateState.hasOwnProperty('license_check_result')) { licenseState.valid = 0; };

  var resultObj = validateState.license_check_result;
  for (let pkg in resultObj) {
    if (resultObj[pkg].includes("OK")) { continue; }
    licenseState.valid = 0;
  }
  console.log(licenseState);
  if (licenseState.valid === 0) {
    $('#lic-expiry-date').text('No License');
    // $('#hw-sig').text('No Hardware Signature');
    // $('.fa-medal').css('color', 'gray').prop('title', 'invalid license');
    notificationMsg(3, 'Invalid License!');
  }
  if (licenseState.valid === 1) {
    let expiry;
    try {
      expiry = await restGetLicExpiryDate();
      console.log(expiry);
    } catch (e) {
      console.error(e);
    }

    // --- [protection] exceptional case ---
    if (expiry === undefined || expiry.hasOwnProperty('error')) {
      $('#lic-expiry-date').text('No License');
      // $('.fa-medal').css('color', 'gold').prop('title', `authorized with flaw`);
      notificationMsg(2, 'Can NOT get expiry date!');
      return;
    }

    // --- success case ---
    $('#lic-expiry-date').text(`Expired by ${expiry.date}`);
    // $('.fa-medal').css('color', 'gold').prop('title', `authorized by ${expiry.date}`);
    notificationMsg(1, 'Swarm Service Authorized!');
  }
}

async function sysHardwareSignature() {
  // [API] get hardware signature
  let hwSig = "No hardware signature!";
  try {
    let res = await restGetHardwareSignature();
    console.log(res);

    // [ERROR HANDLING] get no hardware signature
    hwSig = (res?.hwSig) ? res.hwSig : "No hardware signature!";

  } catch (err) {
    console.error(err);
  }

  $('#hw-sig').text(hwSig);
}


async function initUserSettings() {
  var currUserData = await restGetCurrentUserData();
  var jsonData = JSON.parse(currUserData);
  console.log(jsonData)
  // var obj = $.parseJSON(jsonData);
  // console.log(jsonData[0].user_id);
  parseUserCardInfo(jsonData[0]);

  initUpdateUserForm();
}

function removeAdminComponent(statusData) {
  var role = statusData.role;
  // Here we remove the 'Admin' HTML for 'General' users.
  if (role !== "admin") {
    $("#swarm-core").remove();
    $("#tab-swarm-core").remove();
    $("#others").remove();
    $("#tab-others").remove();
  }
}

function switchTab() {
  // switch to the first tab content
  var tabIndex = 0;
  $('#settings-tabs .nav-link').removeClass('active');
  $(`#settings-tabs a:eq(${tabIndex})`).addClass('active');
  $('.tab-content .tab-pane').removeClass('active');
  $(`.tab-content .tab-pane:eq(${tabIndex})`).addClass('active');
}

// ===========================
//   Views - Firmware Update
// ===========================
function updateFirmwareView(_verList, _fltVer) {
  // --- current version number ---
  var currCoreVer = _fltVer.core.core_sw_version;

  // --- latest version number ---
  // TODO: if no newer version, return.
  var coreVerList = _verList.core_sw_list.map(v => v.sw_version)
  coreVerList.sort();
  console.log(coreVerList);
  var latestCoreVer = coreVerList[coreVerList.length - 1];
  var cmpVer = compareVersion(currCoreVer, latestCoreVer);

  // -- cmpVer < 0: add the badge, cmpVer == 0: version are equal --
  // var updateDom = $('#core-firmware > .input-group-text > .fa-bullhorn');
  // if (cmpVer < 0 && updateDom.find('span.badge').length === 0) {
  //   updateDom.prepend('<span class="badge bg-warning">!</span>');
  // }

  // --- apppend all option on the drop menu ---
  if (_verList.core_sw_list.length > 0) {
    $('#core-firmware-ver').empty();

    _verList.core_sw_list.forEach(function (cv) {
      var ver = cv.sw_version;
      var sel = (ver === currCoreVer) ? 'selected' : '';
      $('#core-firmware-ver').append(`<option value='${ver}' ${sel}>ver. ${ver}</option>`);
    });
  }
  // --- version number fleet-agents alignment comparison ---
}

function updateCurrentFirmwareVer(_fltVer) {
  var currCoreVer = _fltVer.core.core_sw_version;
  $('#core-firmware-cur-ver').val(currCoreVer);
}

function bindButtonEvents() {
  const btnDeployOta = document.getElementById('deploy-ota-btn');
  btnDeployOta.addEventListener('click', deployOtaSw);
  const btnOpenAgents = document.getElementById('open-agents-btn');
  btnOpenAgents.addEventListener('click', searchAgents);
  const btnOpenOtaAgents = document.getElementById('open-ota-agents-btn');
  btnOpenOtaAgents.addEventListener('click', searchOtaAgents);
}

async function pollOtaConnStatus(_inteval = 1000) {
  setInterval(async function () {
    let otaTarget = await fetchOtaUpdateTarget(rmtToken_);
    // console.log(otaTarget);
    let otaStatus = otaTarget.hasOwnProperty('check_result') ? otaTarget.check_result : "";
    let otaStatusText = document.getElementById('ota-status');
    otaStatusText.textContent = otaStatus;

    if (otaTarget.hasOwnProperty('target_version')) {
      let ota_ver = otaTarget.target_version;
      ota_ver = ota_ver.replace(/[{}]/g, "");
      if (isEmptyString(ota_ver)) return;
      let ota_ver_arr = ota_ver.split(",");
      ota_ver_arr.forEach((ver, i) => {
        let ecu_type = ver.split(":")[0];
        if (ecu_type === "FAR_APP") {
          fleetVer.ota = ver.split(":")[1];
          let otaVersionInput = document.getElementById('ota-version');
          otaVersionInput.value = `New version: ${fleetVer.ota}`;
        }
      });
    }
  }, _inteval);
}

async function deployOtaSw() {
  let otaVersion = document.getElementById('ota-version').value;
  console.log(otaVersion);
  if (otaVersion === "not available") {
    alert('Can not reach OTA version!');
    return;
  }

  if (confirm("Update software?")) {
    console.log("Software Updating!")

    try {
      let data = await fetchPutOtaSw(rmtToken_);
      console.log(data);
    } catch (err) {
      console.error(err);
    }

    // --- update progress bars on VIEW ---
    updateOtaSwProgress();
  } else {
    console.log("Update Cancel!")
  }

}

async function searchAgents() {
  selAgents = [];

  let scannedRobots = await getScannedAgents();
  console.log(scannedRobots);

  scannedRobots.forEach(robot => {
    let select_core_ver = document.getElementById('core-firmware-ver').value;
    robot.to_sw_version = select_core_ver;
  });

  let lng = document.getElementById('sel-lang-env').value;
  $("#agent-jsGrid").jsGrid({
    width: "100%",
    height: "auto",
    sorting: true,
    autoload: true,
    paging: false,
    noDataContent: "No agent data found",
    data: scannedRobots,
    fields: [
      {
        title: "", type: "number", align: "center", width: 30,
        itemTemplate: function (val, item) {
          return this._grid.data.indexOf(item) + 1;
        }
      },
      {
        name: "model", title: "",
        itemTemplate: function (val, item) {
          let imgPath = avatarMap_[val.toLowerCase()] || avatarMap_['none'];
          let robot_img = document.createElement('img');
          robot_img.src = imgPath;
          return robot_img
        },
        align: "center",
        width: 100
      },
      {
        name: "robot_name", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].name;
        }
      },
      {
        name: "robot_id", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].id;
        }
      },
      {
        name: "sw_version", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].sw_ver;
        }
      },
      {
        name: "to_sw_version",
        headerTemplate: function () {
          return langAgentTableColumn[lng].update_ver;
        },
        itemTemplate: function (val, item) {
          let upd_sw_ver_span = document.createElement('span');
          upd_sw_ver_span.classList.add('badge', 'bg-danger');
          upd_sw_ver_span.textContent = val;
          return upd_sw_ver_span;
        },
        align: "center"
      },
      {
        type: "checkbox", sorting: false,
        headerTemplate: function () {
          return langAgentTableColumn[lng].check;
        },
        itemTemplate: function (value, item) {
          return $("<input>").addClass('row_check').attr("type", "checkbox")
            .attr("checked", value || item.Checked)
            .on("change", function () {
              item.Checked = $(this).is(":checked");
              if (item.Checked) {
                selAgents = _.union(selAgents, [item]);
              } else {
                _.remove(selAgents, obj => _.isEqual(obj, item));
              }
              // console.log(selAgents);
              // console.log(selAgents.length);

              // --- toggle update button ---
              var checkedCount = document.querySelectorAll('.row_check:checked').length;
              document.getElementById('batch_local_deploy_btn').disabled = (checkedCount == 0);
            });
        }
      }
      // ,{
      //   type: "control", width: 100, editButton: false, deleteButton: false,
      //   itemTemplate: function (val, item) {
      //     var $result = jsGrid.fields.control.prototype.itemTemplate.apply(this, arguments);

      //     var customDeployBtn = document.createElement('button');
      //     customDeployBtn.classList.add('btn', 'btn-primary');
      //     customDeployBtn.textContent = 'Deploy';
      //     customDeployBtn.addEventListener(
      //       'click', function (event) {
      //         deployAgentsOtaSw(event, item)
      //       }
      //     );
      //     return $result.add(customDeployBtn);
      //   }
      // }
    ]
  });

  // let agentTableBody = document.getElementById('agent-table-body');
  // agentTableBody.innerHTML = "";

  // scannedRobots = await getScannedAgents();
  // console.log(scannedRobots)
  // if (scannedRobots.length <= 0) {
  //   notificationMsg(2, 'No available agents!');
  //   return;
  // }

  // await genSoftwareVerObj();

  // for (var i in scannedRobots) {
  //   let robotObj = scannedRobots[i];
  //   let node = createAgentRowView(i, robotObj);
  //   agentTableBody.append(node);
  // }

  // --- append update button ---
  let modalBtnDiv = document.getElementById('update-footer-div');
  removeAllChildNodes(modalBtnDiv);
  let modalBtn = createAgentModalButton();
  modalBtnDiv.append(modalBtn);
  // --- display modal ---
  $('#search-agents-modal').modal('show');
}

async function searchOtaAgents() {
  selAgents = [];

  let scannedRobots = await getScannedAgents();
  await updateAgentsOtaInfo(scannedRobots);
  console.log(scannedRobots);

  let lng = document.getElementById('sel-lang-env').value;
  $("#agent-jsGrid").jsGrid({
    width: "100%",
    height: "auto",
    sorting: true,
    autoload: true,
    paging: false,
    noDataContent: "No agent data found",
    data: scannedRobots,
    fields: [
      {
        title: "", type: "number", align: "center", width: 30,
        itemTemplate: function (val, item) {
          return this._grid.data.indexOf(item) + 1;
        }
      },
      {
        name: "model", title: "",
        itemTemplate: function (val, item) {
          let imgPath = avatarMap_[val.toLowerCase()] || avatarMap_['none'];
          let robot_img = document.createElement('img');
          robot_img.src = imgPath;
          return robot_img
        },
        align: "center",
        width: 100
      },
      {
        name: "robot_name", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].name;
        }
      },
      {
        name: "robot_id", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].id;
        }
      },
      {
        name: "ota_status", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].ota_status;
        }
      },
      { name: "vcu", title: "VCU", type: "text", align: "center" },
      {
        name: "to_vcu_version",
        headerTemplate: function () {
          return langAgentTableColumn[lng].update_ver;
        },
        itemTemplate: function (val, item) {
          let upd_sw_ver_span = document.createElement('span');
          upd_sw_ver_span.classList.add('badge', 'bg-danger');
          upd_sw_ver_span.textContent = val;
          return upd_sw_ver_span;
        },
        align: "center"
      },
      {
        name: "sw_version", type: "text", align: "center",
        headerTemplate: function () {
          return langAgentTableColumn[lng].sw_ver;
        }
      },
      {
        name: "to_sw_version",
        headerTemplate: function () {
          return langAgentTableColumn[lng].update_ver;
        },
        itemTemplate: function (val, item) {
          let upd_sw_ver_span = document.createElement('span');
          upd_sw_ver_span.classList.add('badge', 'bg-danger');
          upd_sw_ver_span.textContent = val;
          return upd_sw_ver_span;
        },
        align: "center"
      },
      {
        type: "checkbox", sorting: false,
        headerTemplate: function () {
          return langAgentTableColumn[lng].check;
        },
        itemTemplate: function (value, item) {
          return $("<input>").addClass('row_check').attr("type", "checkbox")
            .attr("checked", value || item.Checked)
            .on("change", function () {
              item.Checked = $(this).is(":checked");
              if (item.Checked) {
                selAgents = _.union(selAgents, [item]);
              } else {
                _.remove(selAgents, obj => _.isEqual(obj, item));
              }
              // console.log(selAgents);
              // console.log(selAgents.length);

              // --- toggle update button ---
              var checkedCount = document.querySelectorAll('.row_check:checked').length;
              document.getElementById('batch_ota_deploy_btn').disabled = (checkedCount == 0);
            });
        }
      }
    ]
  });

  // --- append update button ---
  let modalBtnDiv = document.getElementById('update-footer-div');
  removeAllChildNodes(modalBtnDiv);
  let modalBtn = createAgentModalButton('ota');
  modalBtnDiv.append(modalBtn);
  // --- display modal ---
  $('#search-agents-modal').modal('show');
}

async function updateAgentsOtaInfo(_robots) {
  let robots = _.map(_robots, function (element, idx) {
    return element.robot_id;
  });
  let robotString = JSON.stringify(robots);
  // console.log(robotString);

  let otaTarget = await fetchAgentOtaSystemStatus(rmtToken_, robotString);
  otaTarget = otaTarget.agent_ota_system_status;
  // console.log(otaTarget);

  for (const robot of _robots) {
    let sys_status = _.find(otaTarget, { robot_id: robot.robot_id });
    if (sys_status === undefined) continue;
    sys_status = sys_status.ota_system_status;
    sys_status = JSON.parse(sys_status);
    robot.ota_status = sys_status.ota_target.check_result || '';
    robot.vcu = sys_status.ota_availability.vcu || 'not available';
    robot.to_vcu_version = 'not available';
    robot.to_sw_version = 'not available';

    let target_ver = sys_status.ota_target.target_version || '';
    target_ver = target_ver.replace(/[{}\s]/g, "");
    if (isEmptyString(target_ver)) continue;
    console.log(target_ver);
    let target_ver_arr = target_ver.split(",");
    target_ver_arr.forEach((ver, i) => {
      console.log(ver)
      let ecu_type = ver.split(":")[0].trim();
      if (ecu_type === "FAR_APP") {
        robot.to_sw_version = ver.split(":")[1];
      } else if (ecu_type === "VCU") {
        robot.to_vcu_version = ver.split(":")[1];
      }
    });
  }
}

// function deployAgentsOtaSw(e, item) {
//   console.log(item);
//   e.stopPropagation();
// }

async function batchLocalDeployAgent() {
  console.log(selAgents);
  let robots = _.map(selAgents, function (element, idx) {
    return element.robot_id;
  });
  let robotString = JSON.stringify(robots);
  // console.log(robotString);

  if (confirm('Batch local update software?')) {
    // --- lock modal buttons ---
    toggleSearchAgentsModalBtns();

    try {
      let sw_ver = document.getElementById('core-firmware-ver').value;
      let data = await fetchPutAgentSwVer(rmtToken_, robotString, sw_ver);
      if (!data.ok) {
        var rtnMsg = await data.json();
        console.log(rtnMsg);
        notificationMsg(3, rtnMsg);
        // --- unlock modal buttons ---
        toggleSearchAgentsModalBtns(false);
        return;
      }
      // --- update progress bars on VIEW ---
      batchLocalUpdateProgress(robots);
    } catch (err) {
      console.error(err);
    }
  } else {
    console.log('Cancel batch local update!');
  }
}

async function batchOtaDeployAgent() {
  console.log(selAgents);
  let robots = _.map(selAgents, function (element, idx) {
    return element.robot_id;
  });
  let robotString = JSON.stringify(robots);
  // console.log(robotString);

  // --- TODO: check ota version ---
  if (confirm('Batch OTA update software?')) {
    // --- lock modal buttons ---
    toggleSearchAgentsModalBtns(true, 'ota');
    try {
      let data = await fetchPutAgentOtaSw(rmtToken_, robotString);
      console.log(data);
    } catch (err) {
      console.error(err);
    }
    // --- update progress bars on VIEW ---
    batchOtaUpdateProgress(robotString);
  } else {
    console.log('Cancel batch OTA update!');
  }
}

function createAgentModalButton(_method = 'local') {
  const template = document.querySelector('#modal-update-btn');
  const updateBtnNode = document.importNode(template.content, true);
  const updateBtn = updateBtnNode.querySelector('.btn');
  updateBtn.id = `batch_${_method}_deploy_btn`;
  if (_method === 'local') {
    updateBtn.addEventListener('click', batchLocalDeployAgent);
  } else {
    updateBtn.addEventListener('click', batchOtaDeployAgent);
  }
  return updateBtnNode;
}

function createAgentRowView(_idx, _data) {
  console.log(_data);
  const template = document.querySelector('#agent-row');
  const node = document.importNode(template.content, true);

  var agentTr = node.querySelector('.agent-progress-tr');
  agentTr.setAttribute('id', `tr_${_data.robot_id}`);

  var seqNode = node.querySelector('.agent-seq');
  seqNode.innerHTML = Number(_idx) + 1;

  var imgNode = node.querySelector('.agent-img');
  var model = _data.model.toLowerCase();
  imgNode.src = avatarMap_[model] || avatarMap_['none'];

  var nameNode = node.querySelector('.agent-name');
  nameNode.innerHTML = _data.robot_name;

  var idNode = node.querySelector('.agent-id');
  idNode.innerHTML = _data.robot_id;

  var versionNode = node.querySelector('.agent-sw-version');
  versionNode.innerHTML = _data.sw_version;

  var badgeVersionNode = node.querySelector('.sw-ver-badge');
  badgeVersionNode.innerHTML = fleetVer.core.core_sw_version || 'not available';
  return node;
}

function createProgressBar(_progressText, _idx) {
  const template = document.querySelector('#progress-bar-row');
  const node = document.importNode(template.content, true);

  var containerNode = node.querySelector('.progress-container');
  containerNode.setAttribute('id', `progress_${_idx}`);
  containerNode.style.display = 'block';

  var progressBarNode = node.querySelector(`#progress_${_idx} > div > div`);
  if (_progressText.includes('%')) {
    var percent = _progressText.substring(_progressText.lastIndexOf(":") + 1, _progressText.lastIndexOf("%")).trim();
    progressBarNode.style.width = `${percent}%`;
  } else {
    progressBarNode.style.width = '100%';
  }
  progressBarNode.textContent = _progressText;
  return node;
}

// ===========================
//   Views - Core Settings
// ===========================
function updateCoreDefinitions(_data) {
  if (!_data.hasOwnProperty('core_metadata')) return;
  var coreDefData = _data.core_metadata.replace(/'/g, '"');
  coreDefData = JSON.parse(coreDefData);
  // --- data hack ---
  coreDefData['system'] = {
    "system_restart": {
      "description": "Restart swarm core system",
      "valid_value": {
        "data_type": "trigger",
        "data_range": "[]"
      },
      "unit": ""
    },
  }
  console.log(coreDefData)

  coreDefData = flattenJSON(coreDefData);
  core_definition = coreDefData;
}

function updateCoreSettings(_data, _lang) {
  console.log(_lang)
  $('#preload-overlay').hide();
  addEmptyCoreSettingsView();

  if (!_data.hasOwnProperty('core_settings')) return;
  _data = _data.core_settings;
  _data = _data.replace(/'/g, '"');
  _data = _data.replace(/True/g, 'true').replace(/False/g, 'false');
  _data = JSON.parse(_data);
  _data['system'] = { 'system_restart': false };
  console.log(_data);
  updateCoreSettingsView(_data, _lang);
}

function addEmptyCoreSettingsView() {
  const template = document.querySelector('#no-core-settings');
  const node = document.importNode(template.content, true);
  $save_core_form.append(node);
}

function updateCoreSettingsView(_data, _lang) {
  var flattedSettingsObj = flattenJSON(_data);
  var flattedSettingsMsg = flattenJSON(_lang);
  console.log(flattedSettingsObj);
  console.log(flattedSettingsMsg);

  if (isEmpty(flattedSettingsObj)) return;
  $('#no-core-settings-div').hide();
  console.log(core_definition);

  for (var key in flattedSettingsObj) {
    // console.log(key);
    const inputName = key.split('.').pop();
    if (inputName === 'swarm_core_ip') { continue; }

    const template = document.querySelector('#core-settings-post');
    const node = document.importNode(template.content, true);

    // const titleString = key.split('.').pop().replaceAll('_', ' ');
    // const transTitle = titleString.charAt(0).toUpperCase() + titleString.slice(1);
    // titleNode.innerHTML = `<b>Swarm Core: ${transTitle}</b>`
    const titleKey = key.concat('.', 'title');
    const transTitle2 = flattedSettingsMsg[titleKey];
    var titleNode = node.querySelector('.settings-title');
    titleNode.innerHTML = `<b>Swarm Core: ${transTitle2 || inputName}</b>`

    const descKey = key.concat('.', 'description');
    // const desc = core_definition[descKey];
    // descNode.textContent = desc;
    var descNode = node.querySelector('.settings-desc');
    const desc2 = flattedSettingsMsg[descKey];
    descNode.textContent = desc2;

    var validType = core_definition[key.concat('.', 'valid_value.data_type')] || 'unknown';
    var inputNode = node.querySelector('.form-control');
    const isCrossLan = validType === 'string' && inputName === 'bridge_mode';

    if (validType === 'bool' || isCrossLan) {
      let isChecked = false;
      if (typeof flattedSettingsObj[key] === 'boolean') {
        isChecked = flattedSettingsObj[key];
      } else {
        isChecked = (flattedSettingsObj[key] === 'zenoh');
      }
      inputNode.remove();
      var switchDiv = document.createElement('div');
      switchDiv.classList.add('custom-control', 'custom-switch');

      var switchChkBox = document.createElement('input');
      switchChkBox_key = key.replace('.', '_');
      switchChkBox.id = switchChkBox_key;
      switchChkBox.dataset.id = key;
      switchChkBox.type = 'checkbox';
      switchChkBox.checked = isChecked;
      switchChkBox.classList.add('custom-control-input');
      switchDiv.appendChild(switchChkBox);

      var switchLabel = document.createElement('label');
      switchLabel.classList.add('custom-control-label');
      switchLabel.setAttribute('for', switchChkBox_key);
      switchDiv.appendChild(switchLabel);

      var userBlockNode = node.querySelector('.user-block');
      userBlockNode.appendChild(switchDiv);
    }
    else if (validType === 'trigger') {
      var triggerBtn = document.createElement('button');
      triggerBtn.id = key;
      triggerBtn.type = 'button';
      triggerBtn.textContent = flattedSettingsMsg['btn_SystemRestart'];
      triggerBtn.classList.add('btn', 'btn-primary', 'btn-md');
      // --- add a callback ---
      triggerBtn.addEventListener('click', btnEventCallbacks);

      inputNode.replaceWith(triggerBtn)
    }
    else {
      var validRange = core_definition[key.concat('.', 'valid_value.data_range')] || '';
      validRange = validRange.replace(/[[\]]/g, '').replace(':', '-');
      var unit = core_definition[key.concat('.', 'unit')] || '';
      inputNode.id = key.replace('.', '_');
      inputNode.dataset.id = key;
      inputNode.name = inputName;
      inputNode.classList.add('param-check');
      inputNode.classList.add('settings-input');
      inputNode.value = flattedSettingsObj[key];
      inputNode.placeholder = `${validRange}${unit}`;
    }

    $save_core_form.append(node);
  }
  const template = document.querySelector('#save-btn-div');
  const saveBtnNode = document.importNode(template.content, true);
  const saveBtn = saveBtnNode.querySelector('.btn');
  saveBtn.addEventListener('click', saveCoreSettings);
  // saveBtn.textContent = (lang === 'en') ? 'Save' : '保存';
  saveBtn.textContent = flattedSettingsMsg['btn_SaveSettings'];
  // saveBtn.setAttribute('data-i18n', 'sys_set.swarm_core.btn_SaveSettings');
  $save_core_form.append(saveBtnNode);

  applyFontSize(getSavedFontSize(), '#save-core-settings-form');
  validateSettingInputEvent();
}

async function btnEventCallbacks(e) {
  if (e.target.id === 'system.system_restart') {
    // --- confirm prompt modal ---
    if (!confirm("Restart Swarm Core?")) {
      console.log("Restart Command Cancelled!")
      e.target.disabled = false;
      return;
    }

    // --- send the restart callback ---
    let res = await fetchPutRebootSwarmCore(rmtToken_);
    // --- handle the response ---
    if (res.ok) {
      notificationMsg(0, 'Reboot Swarm Core successfully!');
      setTimeout(querySwarmCoreStatus, 1000);
      // --- update the button status ---
      e.target.disabled = true;
    } else {
      notificationMsg(2, 'Failed to reboot Swarm Core!');
    }
  }
}

async function querySwarmCoreStatus() {
  let res = {};
  try {
    res = await fetchGetSwarmCoreLiveliness(rmtToken_);
  } catch (e) {
    console.log(e);
    setTimeout(querySwarmCoreStatus, 1000);
    return;
  }
  if (!res.ok) {
    setTimeout(querySwarmCoreStatus, 1000);
    return;
  }

  // --- swarm core is ready ---
  let myBtn = document.getElementById('system.system_restart');
  myBtn.disabled = false;
}

async function saveCoreSettings() {
  // $.validator.addClassRules({
  //   'param-check': {
  //     chkValidFormat: true
  //   }
  // });
  if (!$save_core_form.valid()) return;

  var jsonObj = {};
  $save_core_form.find('input,select').each(function (i, val) {
    var dictKey = $(val).attr('data-id');
    var dictVal = '';
    var valType = core_definition[`${dictKey}.valid_value.data_type`];
    if (valType === 'int' || valType === 'double') {
      dictVal = $(val).val();
      jsonObj[dictKey] = Number(dictVal);
    } else if (valType === 'bool') {
      dictVal = $(val).is(":checked");
      jsonObj[dictKey] = (dictVal === true);
    } else {
      if (dictKey === 'network.bridge_mode') {
        dictVal = $(val).is(":checked") ? 'zenoh' : 'dds';
      } else {
        dictVal = $(val).val();
      }
      jsonObj[dictKey] = dictVal;
    }
  });

  var originalObj = unflattenJSON(jsonObj);
  // console.log(originalObj);
  var coreSettings = JSON.stringify(originalObj);
  coreSettings = coreSettings.replace(/true/g, 'True').replace(/false/g, 'False');
  coreSettings = coreSettings.replace(/"/g, "'");
  // console.log(coreSettings);

  await fetchPutCoreSettings(rmtToken_, coreSettings);
  notificationMsg(0, 'Swarm core settings saved!');
}

function initCoreSettingValidation() {
  $.validator.addMethod("chkValidFormat", function (value, element) {
    let param = value;
    let paramKey = $(element).attr('data-id');
    var type = core_definition[`${paramKey}.valid_value.data_type`];
    var range = core_definition[`${paramKey}.valid_value.data_range`].replace(/[[\]]/g, '');
    if (range.split(":").length <= 1) return true;
    switch (type) {
      case 'double': {
        if (isEmptyString(param)) return true;
        var minVal = parseFloat(range.split(":")[0]);
        var maxVal = parseFloat(range.split(":")[1]);
        var doubleVal = parseFloat(param);
        if (isNaN(minVal) || isNaN(maxVal)) return true;
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
        if (!$.isNumeric(intVal)) {
          $.validator.messages.chkValidFormat = "value type should be " + type;
          return false;
        }
        if (intVal > maxVal || intVal < minVal) {
          $.validator.messages.chkValidFormat = "value must be in the range of " + range.replace(':', '~');
          return false;
        }
        return true;
      }
      default: {
        return true;
      }
    }

  }, $.validator.messages.chkValidFormat);

  $.validator.addMethod("compareThreshold", function (value, element) {
    const lowVal = parseInt(document.getElementById("save-core-settings-form").elements.namedItem("low_battery_threshold").value);
    const fullVal = parseInt(document.getElementById("save-core-settings-form").elements.namedItem("full_battery_threshold").value);
    // console.log(`low threshold: ${lowVal}, full threshold: ${fullVal}`)
    return lowVal < fullVal;
  });

  $save_core_form.validate({
    rules: {
      low_battery_threshold: {
        required: true,
        compareThreshold: true
      },
      full_battery_threshold: {
        required: true,
        compareThreshold: true
      }
    },
    messages: {
      low_battery_threshold: {
        required: 'Please enter low battery threshold.',
        compareThreshold: 'Greater than full battery threshold.'
      },
      full_battery_threshold: {
        required: 'Please enter full battery threshold',
        compareThreshold: 'Less than low battery threshold.'
      }
    },
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

// ===================
//   Event Callbacks 
// ===================
// const btnCoreFwUpdate = document.getElementById('core-firmware');
// btnCoreFwUpdate.addEventListener('click', updateCoreFw);
const btnDeploySw = document.getElementById('deploy-sw-btn');
btnDeploySw.addEventListener('click', updateCoreFw);

async function updateCoreFw() {
  // --- firmware update procedure ---
  // 1. --- get selected version ---
  var select = document.getElementById('core-firmware-ver');
  var value = select.options[select.selectedIndex].value;
  // console.log(value);

  console.log(fleetVer);
  let agentVer = fleetVer.agent;
  let artifactVer = fleetVer.artifact;
  var cmpAgentVer = compareVersion(value, agentVer);
  var cmpArtifactVer = compareVersion(value, artifactVer);

  // let currCoreVer = fleetVer.core.core_sw_version;
  // if (currCoreVer === value) {
  //   alert(`Your current version already is ${currCoreVer}!`);
  //   return;
  // }
  if ((!isEmptyString(agentVer) && cmpAgentVer !== 0) || (!isEmptyString(artifactVer) && cmpArtifactVer !== 0)) {
    alert('Agent and Artifact software need to be updated first!');
    return;
  }
  // --- pop-up modal ---
  if (confirm("Update software?")) {
    console.log("Software Updating!")

    // 2. apply the selected version to agent
    try {
      let data = await fetchPutCoreSwVer(rmtToken_, value);
      if (!data.ok) {
        var rtnMsg = await data.json();
        console.log(rtnMsg);
        notificationMsg(3, rtnMsg);
        return;
      }
      // 3. update the status on VIEW 
      updateCoreVerProgress();

      const badge = this.querySelector('.badge');
      badge.textContent = '';
    } catch (err) {
      console.error(err);
    }
  } else {
    console.log("Update Cancel!")
  }
}

async function updateCoreVerProgress() {
  const asyncUpdateProgress = async () => {
    var progress;
    // --- show the progress bar ---

    while (progress !== 'Succeeded') {
      try {
        var progressObj = await fetchGetCoreSwUpdateStatus(rmtToken_);
      } catch (err) {
        console.error(err);
      }

      progress = progressObj.core_sw_update_status;
      console.log('progress: ', progress);
      if (progress === "") break;

      // --- update on the view ---
      var barDom = $('#core-progress > div');
      if (progress.includes('%')) {
        var percent = progress.substring(progress.indexOf(":") + 1, progress.lastIndexOf("%")).trim();
        console.log(percent);
        $('#core-progress').show();
        barDom.css('width', `${percent}%`);
        // barDom.text(`${percent}%`);
      }

      barDom.text(progress);
      await sleep(1000); // sleep 1s
    }

    // --- hide the progress bar ---
    $('#core-progress').hide();
    notificationMsg(0, 'Core Software Updated!');
  };

  asyncUpdateProgress();
}

async function updateOtaSwProgress() {
  let progressBarDiv = document.getElementById('ota-progress-bar');
  const asyncOtaProgress = async () => {
    var progress;
    var progressArr = [];
    // TODO: still need to test multi progress bars
    while (progress !== 'Firmware update success') {
      try {
        var progressObj = await fetchOtaStatus(rmtToken_);
        progress = progressObj.ota_status;
        progressArr = progress.split(',');
        console.log('progressArr: ', progressArr);

        removeAllChildNodes(progressBarDiv);
        progressArr.forEach((prog, i) => {
          let barNode = createProgressBar(prog, i);
          progressBarDiv.append(barNode);
        });
      } catch (err) {
        console.error(err);
      }

      await sleep(1000); // sleep 1s
    }
    removeAllChildNodes(progressBarDiv);
  };

  asyncOtaProgress();
}

async function batchLocalUpdateProgress(_agents) {
  let agents = _agents;
  const asyncUpdateProgress = async () => {
    var progressArr = [];
    while (!progressArr.every(el => el === 'Succeeded') || progressArr.length === 0) {
      progressArr = [];
      try {
        let agents_string = JSON.stringify(agents);
        console.log(agents_string);
        var progressObj = await fetchPostAgentsSwUpdateStatus(rmtToken_, agents_string);
      } catch (err) {
        console.error(err);
      }

      var progress = progressObj.agent_sw_update_status;
      console.log('progress: ', progress);
      if (progress === undefined) break;
      for (let obj of Object.values(progress)) {
        let id = obj.robot_id;
        let status = obj.sw_update_status;

        let barNode = createProgressBar(status, id);
        let oriBarNode = document.getElementById(`progress_${id}`);
        let agentProgressTr = document.getElementById(`tr_${id}`);
        if (agentProgressTr !== null) {
          let agentProgressTd = document.getElementById(`td_${id}`);
          agentProgressTd.replaceChild(barNode, oriBarNode);
        } else {
          let tr = document.createElement('tr');
          tr.id = `tr_${id}`;
          let td = document.createElement('td');
          td.id = `td_${id}`;
          td.colSpan = "7";
          tr.appendChild(td).append(barNode);
          var $row = $("#agent-jsGrid").jsGrid("rowByItem", _.find(selAgents, { robot_id: id }));
          $row.after(tr);
        }

        if (status === 'Succeeded' || status.toLowerCase().indexOf('fail') >= 0) {
          agents.splice(agents.indexOf(id), 1);
          agentProgressTr.style.display = 'none';
        }
        progressArr.push(status);
      }

      await sleep(1000); // sleep 1s
    }
    // --- unlock modal buttons ---
    toggleSearchAgentsModalBtns(false);
    $('#search-agents-modal').modal('hide');
  };
  asyncUpdateProgress();
}

let agent_progress = {};
async function batchOtaUpdateProgress(_agents) {
  console.log(_agents);
  const asyncAgentOtaProgress = async () => {
    var agentProgObj = {};
    var progressArr = [];
    var startFetching = false;

    while (!startFetching || !isEmpty(agentProgObj)) {
      try {
        startFetching = true;
        var progressObj = await fetchAgentOtaStatus(rmtToken_, _agents);
        let progress = progressObj.agent_ota_update_status;
        for (let obj of Object.values(progress)) {
          let id = obj.robot_id;
          let status = obj.ota_update_status;
          progressArr = status.split(',');
          agentProgObj[id] = progressArr;
          console.log(`------------${id}------------`);
          // console.log('progressArr: ', progressArr);
          console.log(agentProgObj);

          // --- create progress bar container row ---
          let agentProgressTr = document.getElementById(`tr_${id}`);
          if (agentProgressTr === null) {
            let tr = document.createElement('tr');
            tr.id = `tr_${id}`;
            let td = document.createElement('td');
            td.id = `td_${id}`;
            td.colSpan = "10";
            tr.appendChild(td);
            var $row = $("#agent-jsGrid").jsGrid("rowByItem", _.find(selAgents, { robot_id: id }));
            $row.after(tr);
          }

          let agentProgressTd = document.getElementById(`td_${id}`);
          removeAllChildNodes(agentProgressTd);
          progressArr.forEach((prog, i) => {
            console.log(prog.trimStart());
            let rid = id.concat(i.toString());
            let barNode = createProgressBar(prog.trimStart(), rid);
            agentProgressTd.appendChild(barNode);
          });

          if (progressArr.every(el => el === 'Succeeded')) {
            delete agentProgObj[id];
            agentProgressTd.parentNode.style.display = 'none';
          }
        }

      } catch (err) {
        console.error(err);
      }

      await sleep(1000); // sleep 1s
    }
    // --- unlock modal buttons ---
    toggleSearchAgentsModalBtns(false, 'ota');
    $('#search-agents-modal').modal('hide');
  };

  asyncAgentOtaProgress();
}

function toggleSearchAgentsModalBtns(_disabled = true, _method = 'local') {
  document.getElementById(`batch_${_method}_deploy_btn`).disabled = _disabled;
  document.getElementById('close-search-agents').disabled = _disabled;
}

function compareVersion(version1, version2) {
  // console.log(`version1: ${version1}`);
  // console.log(`version2: ${version2}`);
  if (version1 === undefined || version2 === undefined) {
    console.log('version numbers not available');
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

// ========================
//     RESTful Requests 
// ========================

// function applyHeaderBgColorClass(selClass) {
//   var lastClass = $('.main-header').attr('class').split(' ').pop();
//   $('.main-header').removeClass(lastClass).addClass(selClass);
// }

// function applySidebarBgColorClass(selClass) {
//   var classList = $('.main-sidebar').attr('class').split(' ');
//   $.each(classList, function (index, item) {
//     if (item.indexOf("bg-") !== -1) {
//       $('.main-sidebar').removeClass(item);
//     }
//   });
//   $('.main-sidebar').addClass(selClass);

//   // change logo image
//   if (selClass !== 'bg-dark') {
//     $('.logo-xs').attr("src","dist/img/FARobotLogo_white.png");
//     $(".logo-xl").attr("src","dist/img/FARobotLogo_lg_white.png");
//   } else {
//     $('.logo-xs').attr("src","dist/img/FARobotLogo.png");
//     $(".logo-xl").attr("src","dist/img/FARobotLogo_lg.png");
//   }
// }

// function applySidebarActiveItemColorClass(selClass) {
//   var classList = $('.main-sidebar').attr('class').split(' ');
//   $.each(classList, function (index, item) {
//     if (item.indexOf("sidebar-") !== -1) {
//       $('.main-sidebar').removeClass(item);
//     }
//   });
//   $('.main-sidebar').addClass(selClass);
// }

// $("#nav-bg-color div").click(function () {
//   var bgClass = $(this).attr('class').split(' ')[0];
//   applyHeaderBgColorClass(bgClass);
// });

// $('#sidebar-bg-color div').click(function () {
//   var bgClass = $(this).attr('class').split(' ')[0];
//   applySidebarBgColorClass(bgClass);
// });

// $('#sidebar-item-color div').click(function () {
//   var itemColorClass = $(this).data('value');
//   applySidebarActiveItemColorClass(itemColorClass);
// });

$('#theme-color div').click(function () {
  var selTheme = $(this).data('value');
  console.log(selTheme);
  setLocalStorageByKey('theme', selTheme);

  var isDarkTheme = (selTheme === 'dark');
  toggleDarkTheme(isDarkTheme);
  autoSelectTheme(selTheme);
});

$('#font-select').on('click', function () {
  document.body.style.fontFamily = this.value;
});

$("input:radio[name=font-size-radio]").on('change', function () {
  const fontSizeOpt = $(this).val();
  applyFontSize(fontSizeOpt);
  // var per = getFontScalePercent(fontSizeOpt);
  // applyFontSizePercentage(per);
});

$("#sel-lang-env").on('change', async function () {
  const lng = $(this).val();
  // console.log(lng);

  await initLanguageSupport(lng);

  //--- update swarm core settings translation ---
  $save_core_form.empty();
  let coreSettings = await fetchGetCoreSettings(rmtToken_);
  let langMsg = await restGetLangMsg(lng);
  // console.log(langMsg);
  updateCoreSettings(coreSettings, langMsg);
});

$('#upload-conf-modal').on('hidden.bs.modal', function (e) {
  $("#conf-file-input").val('');
});

async function btnApplyUISettings() {
  console.log(settingsUI);
  var selFontSize = $('input[name="font-size-radio"]:checked').val();
  var selFontStyle = $('#font-select option:selected').val();
  var selLang = $('#sel-lang-env option:selected').val();

  settingsUI.theme = getSavedTheme();
  settingsUI.fontSize = selFontSize;
  settingsUI.fontStyle = selFontStyle;
  settingsUI.lang = selLang;

  // [lang] apply selected language 
  let lng = $('#sel-lang-env').val();
  setLocalStorageByKey('lang', lng);

  await restPostUISettings(settingsUI)
  notificationMsg(0, 'UI settings are saved');
}

function autoSelectTheme(theme) {
  $('#theme-color div').css('border', 'none')
  $('#theme-color div').filter(`[data-value='${theme}']`).css('border', '2px solid gray')
}


const cvtCriteriaTerms = {
  "gt": "above",
  "eq": "equal",
  "lt": "below",
  2: "ERROR",
  1: "WARN",
  0: "INFO",
};
async function getNotifyGroups() {
  var lineGroups = document.querySelector('#line-groups-block');
  var res = await restGetNotifyGroups();
  console.log(res);
  // --- clear all history ---
  $('#line-groups-block').contents().not('#notify-group-caption').remove();
  // --- render the notify groups criteria ---
  res.data.forEach((lg) => {
    const template = document.querySelector('#line-group-row');
    const node = document.importNode(template.content, true);

    var lineToken = node.querySelector('.line-token-col');
    lineToken.innerHTML = lg.line_token;

    var lvOp = node.querySelector('.level-op');
    lvOp.innerHTML = cvtCriteriaTerms[lg.level_op];
    var lvVal = node.querySelector('.level-val');
    lvVal.innerHTML = cvtCriteriaTerms[lg.level_val];

    var contOp = node.querySelector('.content-op');
    contOp.innerHTML = lg.content_op;
    var contVal = node.querySelector('.content-val');
    contVal.innerHTML = lg.content_val;

    lineGroups.append(node);
  });
}

async function updateLineGroups() {
  const lineToken = document.querySelector('#input-line-token').value;
  // console.log(lineToken);
  if (lineToken.length !== 43) {
    notificationMsg(3, 'Invalid token length!');
    return;
  }

  // --- get the message level and its operator ---
  const lvopOption = document.querySelector('#sel-lvop');
  var lvopSel = lvopOption.options[lvopOption.selectedIndex].value;

  const levelOption = document.querySelector('#sel-level');
  var levelSel = levelOption.options[levelOption.selectedIndex].value;

  // --- get the message content and its operator ---
  const msgOption = document.querySelector('#sel-contop');
  var msgopSel = msgOption.options[msgOption.selectedIndex].value;

  const msgInput = document.querySelector('#input-content').value;

  const notifyGroups = {
    lineToken: String(lineToken),
    logLevel: {
      operator: String(lvopSel),
      value: String(levelSel)
    },
    logContent: {
      operator: String(msgopSel),
      value: String(msgInput)
    }
  };
  console.log(JSON.stringify(notifyGroups));

  var res = await restPutNotifyGroups(notifyGroups);
  console.log(res);
  if (res.status_code !== 200) {
    notificationMsg(3, res.message);
    return;
  }
  notificationMsg(1, res.message);

  // --- update the row ---
  getNotifyGroups();
}

function parseUserCardInfo(_userInfo) {
  $('#profile-username').text(_userInfo.user_name);
  $('#user-account').val(_userInfo.user_id);
  $('#user-password').val('••••••••');
  $('#user-phone').val(_userInfo.user_phoneNum);
  $('#user-email').val(_userInfo.user_email);
}

function initUpdateUserForm() {
  $.validator.addMethod("regex", function (value, element, param) {
    return value.match(new RegExp("^" + param + "$"));
  });

  $.validator.addMethod("chkPasswordStrength", function (value, element) {
    let password = value;
    if (!(/^(?=.*[a-z])(?=.*[A-Z])(?=.*[0-9])(?=.*[~!@#$%^&*;:,.<>\[\]{}()\\|\-_])/.test(password))) {
      return false;
    }
    return true;
  }, function (value, element) {
    let password = $(element).val();
    if (!(/^(?=.*[A-Z])/.test(password))) {
      return 'Password must contain at least one uppercase';
    }
    else if (!(/^(?=.*[a-z])/.test(password))) {
      return 'Password must contain at least one lowercase';
    }
    else if (!(/^(?=.*[0-9])/.test(password))) {
      return 'Password must contain at least one number';
    }
    else if (!(/^(?=.*[~!@#$%^&*;:,.<>\[\]{}()\\|\-_])/.test(password))) {
      return "Password must contain at least one special character from ~!@#$%^&*;:,.<>[]{}()|-_";
    }
    return false;
  });

  $.validator.addMethod("chkConfirmPassword", function (value, element) {
    let confirmedPassword = value;
    if (confirmedPassword === $('#account-pwd').val()) {
      return true;
    }
    return false;
  });

  $form.validate({
    rules: {
      phoneNum: {
        required: true,
        minlength: 8,
        regex: "^([0-9\(\)\/\+ \-]{8,20})$"
      },
      email: {
        required: true,
        email: true
      }
    },
    messages: {
      phoneNum: {
        required: "Please enter contact number",
        minlength: "Phone number must be at least 8 numbers",
        regex: "Invalid contact number format"
      },
      email: {
        required: "Please enter email",
        email: "Invalid email address"
      }
    },
    errorElement: 'span',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.closest('.input-group').append(error);
    },
    highlight: function (element, errorClass, validClass) {
      $(element).addClass('is-invalid');
    },
    unhighlight: function (element, errorClass, validClass) {
      $(element).removeClass('is-invalid');
    }
  });
  $form.on('submit', submitHandler);

  $chg_pwd_form.validate({
    rules: {
      accountPwd: {
        required: true,
        minlength: 4,
        chkPasswordStrength: true
      },
      accountPwd2: {
        required: true,
        chkConfirmPassword: true
      }
    },
    messages: {
      accountPwd: {
        required: "Please enter password",
        minlength: "Your password must be at least 4 characters"
      },
      accountPwd2: {
        required: "Please enter confirm password",
        chkConfirmPassword: "Confirm password not match with the password"
      }
    },
    errorElement: 'span',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.closest('.input-group').append(error);
    },
    highlight: function (element, errorClass, validClass) {
      $(element).addClass('is-invalid');
    },
    unhighlight: function (element, errorClass, validClass) {
      $(element).removeClass('is-invalid');
    }
  });
  $chg_pwd_form.on('submit', submitChgPwdHandler);
}

function submitHandler(e) {

  if (!$form.valid()) { return; }

  e.preventDefault();

  var formArray = $form.serializeArray();
  // console.log(formArray);
  formArray.push(modifiedPwd);
  console.log(formArray);

  $.ajax({
    url: '/testdb/updateCurrentUserInfo',
    type: 'POST',
    data: formArray, //$form.serialize(),
    success: function (data) {
      notificationMsg(0, data);
      // toast(data);
    },
    error: function (e) {
      alert(e.responseText);
    }
  });
}

function submitChgPwdHandler(e) {
  if (!$chg_pwd_form.valid()) { return; }
  e.preventDefault();
  var input_password = $('#account-pwd').val();
  restPasswordExistence(input_password).done(function (isRepeat) {
    if (isRepeat) {
      alert('You have already used that password before, please try another.');
      return;
    }
    modifiedPwd.value = input_password;
    $('#change-pwd-modal').modal('hide');
  });
}

$('.switch-edit-mode').click(editButtonSwitch);
$('.switch-password-mode').click(pwdButtonSwitch);

function editButtonSwitch() {
  var readonly = $(this).prev('input').prop('readonly');
  $(this).prev('input').prop('readonly', !readonly);
  $(this).children().children('i').toggleClass('fa-pen');
  $(this).children().children('i').toggleClass('fa-eye');
}

function pwdButtonSwitch() {
  $(this).children().children('i').toggleClass("fa-eye fa-eye-slash");
  if ($(this).prev('input').attr('type') === 'text') {
    $(this).prev('input').attr('type', 'password');
  } else {
    $(this).prev('input').attr('type', 'text');
  }
}

// =============================================
//     License Upload and Signature Download 
// =============================================
// ------ license callbacks ------
// function showUploadZone() {
//   $('#drop-area').toggle();
// }

// async function removeLicense() {
//   console.log('remove license');
//   var res = await restDeleteLicense();
//   console.log(res);
//   if (res === 'OK') {
//     $('#lic-expiry-date').text('No License');
//     // $('.fa-medal').css('color', 'gray').prop('title', 'uncertificated');
//     notificationMsg(1, 'License is deactivated!');
//   } else {
//     notificationMsg(3, 'Fail to deactivate license!');
//   }

//   // await sysLicenseCheck();
// }

// async function exportSignature() {
//   window.location = `/signature`;
// }

// --- upload file by XHR object ---
// const url = "/upload-license";
// const form = document.querySelector('#upload-license');

// form.addEventListener('submit', async (e) => {
//   // disable default action
//   e.preventDefault();

//   // collect files
//   const files = document.querySelector('[name=fileElem]').files;
//   const formData = new FormData();
//   formData.append('license', files[0]);
//   console.log(files);

//   // POST form data
//   const xhr = new XMLHttpRequest();

//   // log response
//   xhr.onload = async () => {
//     console.log(xhr.responseText);
//     res = JSON.parse(xhr.responseText);
//     if (res.status) {
//       notificationMsg(1, 'License Uploaded!');

//       // ---license check ---
//       await sysLicenseCheck();
//     } else {
//       $('#lic-expiry-date').text('No License');
//       $('#hw-sig').text('No Hardware Signature');
//       // $('.fa-medal').css('color', 'gray').prop('title', 'uncertificated');
//       notificationMsg(3, 'Invalid license');
//     }
//   }

//   // create and send the request
//   xhr.open('POST', url);
//   xhr.send(formData);

// });

// To check these files, 1. if not ended with tar.gz, don't upload it
// 2. If ended with tar.gz and start with 'far', add to upload list.
// 要判斷這些檔案，1. 如果不是 tar.gz 結尾，就跳過。2. 是 tar.gz 結尾才加入 data.
// function getValidGzfiles(inputFiles) {
//   var rltArray = [];
//   let files = Array.from(inputFiles);
//   // console.log('files.length: ', files);
//   if (files.length > 4) {
//     alert('Must select at most 4 tar.gz files!');
//     return rltArray;
//   }
//   let names = [];
//   var data = new FormData();
//   for (let i = 0; i < files.length; i++) {
//     let filename = files[i].name;
//     let idxb = filename.indexOf('.tar.gz');
//     let name = filename.substring(0, idxb);
//     // let far = filename.indexOf('far');
//     // console.log(
//     //   'getNames, name: ',
//     //   name,
//     //   ', name.length: ',
//     //   name.length,
//     //   'far index',
//     //   far
//     // );
//     // if (name.length > 0 && far >= 0) {
//     if (name.length > 0) {
//       names.push(name);
//       data.append('zips', inputFiles[i]);
//     }
//   }
//   rltArray = [data, names];
//   // console.log('names: ', names);
//   return rltArray;
// }

// const checkUploadStatus = async (names) => {
//   checkCnt++;
//   console.log(`uploading ${names.join()} to /upload-zips path(${checkCnt})`);
//   if (uploadFailed) {
//     removeOverlay();
//     return;
//   }
//   let data = await restUploadStatus(names);
//   if (data.exists) {
//     console.log(`${names.join()} uploaded!!`);
//     const path = data.path || '';
//     if (path === '') {
//       notificationMsg(3, 'Upload files failed: path not found!');
//       removeOverlay();
//       return;
//     }
//     removeOverlay();
//     addDebianByNames(names, path);
//     return;
//   }
//   await sleep(2000); // sleep 2s
//   checkUploadStatus(names);
// }

// async function addDebianByNames(names, path) {
//   try {
//     for (let i in names) {
//       console.log('names[i]: ', names[i]);
//       displayOverlay(`Uploading ${names[i]} to local repository...`);
//       let data = await fetchPutAddDebianPackage(rmtToken_, names[i], path);
//       let msg = data.result;
//       let msgType = msg.toLowerCase().indexOf('success') > -1 ? 1 : 3;
//       notificationMsg(msgType, msg);
//       removeOverlay();

//       // --- remove package from upload-zips path ---
//       restDeleteUploadPackage(names[i]);
//     }
//     // -- refresh software list --
//     var swList = await fetchGetSwVerList(rmtToken_);
//     await genSoftwareVerObj();
//     updateFirmwareView(swList, fleetVer);
//   } catch (error) {
//     notificationMsg(3, 'Upload package to repo failed!');
//     removeOverlay();
//   }
// }

$('#sw-to-repo-btn').on('click', swToRepo);
async function swToRepo() {
  console.log('swToRepo');
  // checkCnt = 0;
  let input = document.createElement('input');
  input.type = 'file';
  input.accept = '.tar.gz';
  input.enctype = 'multipart/form-data';
  input.onchange = async (e) => {
    const files = input.files;
    // console.log(files);
    const formData = new FormData();
    formData.append('pkg', files[0]);

    displayOverlay('Uploading software to local repository...');
    try {
      let data = await fetchPutAddDebianPackage(rmtToken_, formData);
      const rtnMsg = await data.json();
      if (data.ok) {
        if (!rtnMsg.hasOwnProperty('result')) { return; }
        notificationMsg(rtnMsg.result.toLowerCase().includes('success') ? 1 : 2, rtnMsg.result);
      } else {
        notificationMsg(3, rtnMsg);
      }
    } catch (err) {
      console.error(err);
      notificationMsg(3, 'Upload failed!');
    }
    removeOverlay();

    // -- refresh software list --
    var swList = await fetchGetSwVerList(rmtToken_);
    await genSoftwareVerObj();
    updateFirmwareView(swList, fleetVer);

    // let gzfiles = getValidGzfiles(input.files);
    // if (gzfiles.length === 0) return;
    // let data = gzfiles[0];
    // let names = gzfiles[1];
    // console.log('data: ', data);
    // console.log('names: ', names);

    // if (names.length === 0) {
    //   notificationMsg(3, 'Invalid tar.gz file!');
    //   return;
    // }

    // displayOverlay('Uploading to upload-zips path...');
    // checkUploadStatus(names);
    // const path = '/upload-zips';
    // fetch(path, {
    //   method: 'POST',
    //   body: data,
    // })
    //   .then(function (response) {
    //     // console.log('response: ', response);
    //     if (response.ok) {
    //       console.log('response.ok ');
    //       return response.json();
    //     }
    //     return Promise.reject(response);
    //   })
    //   .then(function (data) {
    //     if (!data.status) {
    //       uploadFailed = true;
    //       notificationMsg(3, data.message);
    //       return;
    //     }
    //     uploadFailed = false;
    //   })
    //   .catch(function (error) {
    //     // ignore ERR_CONNECTION_RESET error
    //     uploadFailed = false;
    //     console.error('Something went wrong.', error);
    //     notificationMsg(3, 'Upload files failed!');
    //   });
  };

  input.click();
}

async function exportCoreSettings() {
  let data = await fetchCoreSettingsPackage(rmtToken_);
  // console.log(data);
  let blob = new Blob([data], { type: data.type });
  let blobURL = URL.createObjectURL(blob);

  var fileLink = document.createElement('a');
  fileLink.href = blobURL;
  fileLink.download = 'core-conf-' + getCurrentDateTime() + '.tar.gz';
  fileLink.click();

  notificationMsg(0, 'Core Settings Export!');
}

async function importCoreSettings() {
  const files = document.getElementById('conf-file-input').files;
  if (files.length === 0) return;
  // console.log(files[0]);

  var formData = new FormData();
  formData.append('core_settings', files[0]);

  displayOverlay('Importing core settings...');
  try {
    var data = await fetchPutCoreSettingsPackage(rmtToken_, formData);
    if (data.status === 200) {
      window.location.reload();
    } else {
      notificationMsg(3, 'Invalid import package!');
    }
  } catch (err) {
    console.error(err);
    notificationMsg(3, 'Import failed!');
  }
  $('#upload-conf-modal').modal('hide');
  removeOverlay();
}

async function resetCoreSettings() {
  displayOverlay('Resetting...');
  try {
    await fetchResetCoreSettings(rmtToken_);
    notificationMsg(0, 'All Settings Reset!');
    await sleep(1000);
    window.location.reload();
  } catch (err) {
    console.error(err);
    notificationMsg(3, 'Reset failed!');
  }
  removeOverlay();
}

function getCurrentDateTime() {
  var today = new Date();
  var todayString = today.toISOString().slice(0, 10).replace(/-/g, "");
  todayString = todayString.concat('-', today.getHours().toString().padStart(2, '0'));
  todayString = todayString.concat('-', today.getMinutes().toString().padStart(2, '0'));
  todayString = todayString.concat('-', today.getSeconds().toString().padStart(2, '0'));
  return todayString;
}

// --- add input values validator ---
function validateSettingInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element_comfirm_change = $('#confirm_change_pwd');
  const interaction_element_apply = $('#update-user-btn');
  const interaction_element_save = $('#system-settings-save-btn');

  const passwaordRule = new Rule('account-pwd', passwordValidation, interaction_element_comfirm_change);
  const passwaord2Rule = new Rule('account-pwd2', passwordValidation, interaction_element_comfirm_change);
  const phoneNameRule = new Rule('user-phone', phoneValidation, interaction_element_apply);
  const emailRule = new Rule('user-email', emailValidation, interaction_element_apply);

  const low_battery_thresholdRule = new Rule('plan_low_battery_threshold', settingNumberValueValidation, interaction_element_save);
  const full_battery_thresholdRule = new Rule('plan_full_battery_threshold', settingNumberValueValidation, interaction_element_save);
  const charging_due_timeRule = new Rule('plan_charging_due_time', settingNumberValueValidation, interaction_element_save);
  const battery_log_timeRule = new Rule('battery_log_battery_log_time', settingNumberValueValidation, interaction_element_save);
  const battery_log_thresholdRule = new Rule('battery_log_battery_log_threshold', settingNumberValueValidation, interaction_element_save);

  validatorManager.addValidator(passwaordRule);
  validatorManager.addValidator(passwaord2Rule);
  validatorManager.addValidator(phoneNameRule);
  validatorManager.addValidator(emailRule);

  validatorManager.addValidator(low_battery_thresholdRule);
  validatorManager.addValidator(full_battery_thresholdRule);
  validatorManager.addValidator(charging_due_timeRule);
  validatorManager.addValidator(battery_log_timeRule);
  validatorManager.addValidator(battery_log_thresholdRule);
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
    let checkAllRes = vm.checkAllRuleClear();
    console.log(checkAllRes)

    // --- [STYLING] reflect validation result ---

    // --- [UX] Interaction Logics ---
    let err_still_sc = [];
    let err_still_pwd = [];
    let err_still_user_setting = [];
    let case_choose = 0;
    for (const [key, value] of Object.entries(checkAllRes)) {
      if (key == 'plan_low_battery_threshold' || key == 'plan_full_battery_threshold' || key == 'plan_charging_due_time' || key == 'battery_log_battery_log_time'
        || key == 'battery_log_battery_log_threshold') {
        err_still_sc.push(value);
        case_choose = 1;
      } else if (key == 'user-phone' || key == 'user-email') {
        err_still_user_setting.push(value);
        case_choose = 2;
      } else if (key == 'account-pwd' || key == 'account-pwd2') {
        err_still_pwd.push(value);
        case_choose = 3;
      }
    }

    let err_still = undefined;
    let interaction_element = undefined;
    switch (case_choose) {
      case 1:
        err_still = err_still_sc;
        interaction_element = interaction_element_save;
        break;
      case 2:
        err_still = err_still_user_setting;
        interaction_element = interaction_element_apply;
        break;
      case 3:
        err_still = err_still_pwd;
        interaction_element = interaction_element_comfirm_change;
        break;
      default:
        break;
    }

    if (err_still.includes(false)) {
      interaction_element.prop('disabled', true);
    } else {
      interaction_element.prop('disabled', false);
    }

  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('settings-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

function settingNumberValueValidation(inputVal) {
  const reg1 = /^[0-9.]+$/;                // 0-9
  const reg2 = /\s/;                                      // space are not allow

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal);

  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid number or includes invalid charaters';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg2);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

const langAgentTableColumn = {
  'en': {
    'name': 'Agent name',
    'id': 'Agent ID',
    'ota_status': 'OTA status',
    'sw_ver': 'Software version',
    'update_ver': 'Update to',
    'check': 'Click to update'
  },
  'zht': {
    'name': '機器人名',
    'id': '機器人識別碼',
    'ota_status': 'OTA 狀態',
    'sw_ver': '軟體版本',
    'update_ver': '欲更新至',
    'check': '點擊更新'
  },
  'zh': {
    'name': '机器人名',
    'id': '机器人识别码',
    'ota_status': 'OTA 状态',
    'sw_ver': '软件版本',
    'update_ver': '欲更新至',
    'check': '点击更新'
  }
};

function openEulaModal() {
  document.getElementById('eula-content').innerHTML = eulaHtmlContent;
  $('#eula-modal').modal('show');
}

async function updateEulaStatus() {
  let slaData = await fetchGetSLAConfirmation(rmtToken_);
  if (!slaData.hasOwnProperty('confirmed')) return;
  const statusMsg = slaData.confirmed ? 'agreed by user' : 'waiting for user to agree';
  document.getElementById('eula-status').textContent = statusMsg;
}

// ==================================
//     Wibu License Authorization 
// ==================================

async function updateLicenseInfo() {
  let data = await fetchGetLicenseContainerInfo(rmtToken_);
  const statusMsg = data.retCode === 0 ? 'authorized' : 'unauthorized';
  const boxMask = data.boxMask || '';
  const serialNo = data.serialNo || '';
  document.getElementById('lic-status').textContent = statusMsg;
  document.getElementById('container-id').textContent = `${boxMask}-${serialNo}`;
  document.getElementById('lic-type').textContent = data.firmCode;

  data = await fetchGetLicenseInfo(rmtToken_);
  const expDateString = data.expirationTime;
  let expDate = new Date(expDateString);
  expDate = moment(expDate).format("YYYY-MM-DD hh:mm:ss");
  if (isNaN(expDate)) return;
  document.getElementById('lic-exp-date').textContent = expDate;
}

async function initLicenseStepper() {
  licenseStepper = new Stepper(document.querySelector('#license-stepper'));

  if (window.innerWidth > PHONE_MAX_WIDTH) {
    document.getElementById('step1Label').textContent = "Step 1: Footprint exporting";
    document.getElementById('step2Label').textContent = "Step 2: License activation";
    document.getElementById('step3Label').textContent = "Step 3: Receipt downloading";
  }

  const uploadLicForm = document.querySelector('#upload-license-form');
  uploadLicForm.addEventListener('submit', async (e) => {

    e.preventDefault();

    const files = document.querySelector('[name=license-file]').files;
    const formData = new FormData();
    formData.append('license', files[0]);

    let res = await fetchPostLicense(rmtToken_, formData);
    console.log(res);

    if (res.retCode === 0) {
      nextStep();
    } else {
      notificationMsg(3, res.message);
    }
  });

  if (!isMobile()) return;

  const licenseFileInput = document.querySelector('#license-file');
  licenseFileInput.addEventListener('change', (e) => {
    const files = e.target.files;
    if (!files.length) return;
    document.getElementById('license-label').textContent = files[0].name;
  });
}

function exportFootprint() {
  genFootprintReceiptLink('footprint');
}

function downloadReceipt() {
  genFootprintReceiptLink('receipt');
}

async function genFootprintReceiptLink(_type) {
  let data = await fetchGetLicenseFootprint(rmtToken_);
  // console.log(_type);
  // console.log(data);
  if (data.retCode === 0) {
    const context = data.buffer;
    const blob = new Blob([context], { type: 'application/octet-stream' });
    const blobUrl = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = blobUrl;
    a.download = 'context.WiBuRaC';
    document.getElementById(`${_type}-link`).appendChild(a);
    a.click();
    URL.revokeObjectURL(blobUrl);

    const isFootprint = _type === 'footprint';
    notificationMsg(1, `${_type.charAt(0).toUpperCase() + _type.slice(1)} is ${isFootprint ? 'exported' : 'downloaded'}!`);
    if (isFootprint) {
      nextStep();
    }

  } else {
    notificationMsg(3, `Failed to export ${_type}!`);
  }
}

function nextStep() {
  setTimeout(function () {
    licenseStepper.next();
  }, 1000);
}

function prevStep() {
  licenseStepper.previous();
}