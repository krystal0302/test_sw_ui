/*
 * Author: John Wu
 * Date: 15 July 21,
 * Description:
 **/
// ======================
//       Load Ready
// ======================

var chkOpFileArray = [];
var fleetFileArray = [];

$(function () {
  'use strict'
  initRundownAsync();
  fetchArtifactsServiceTypesCb();
  // modalValidateEvent('new-role-modal');
  validateRoleInputEvent()
});

// let rmtToken_;
let tooltips_ = {};
async function initRundownAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'role');

  bindTSTooltipEvts();

  // ====== role in Independent Page ======
  // --- fetch RMT token initially ---
  try {
    rmtToken_ = await fetchToken();
  } catch (err) {
    console.error(err);
  }

  // ====== load roles in the selected fleet ======
  let res = await fetchGetAllRoles(rmtToken_);
  let fltRoles = await res.json();
  fltRoles = fltRoles.roles;

  // --- load roles on overview card-deck ---
  loadRoles(fltRoles);

  // ====== role in Operation Page ======
  // // var roleString = $.cookie('role');
  // // // [protection] protect from empty strng
  // // if (roleString === "") return;
  // // var roleArray = roleString.split(",");

  // loadCurrentFleetRoles();

  // ====== roles check in all fleet configurations ======
  let fleets = [];
  try {
    // --- [SWARM API] fetch roles ---
    fleets = await fetchGetFleets(rmtToken_);
    // console.log(fleets);

    asyncReadFleetConfig(fleets);
  } catch (err) {
    console.error(err);
  }

  // ====== fetch tooltips ======
  tooltips_ = await restGetTooltips();
  console.log(tooltips_);
  let roleTooltips = tooltips_['role.roles.overview.title'] || '';
  $('#role-intro').attr('title', roleTooltips);

  // ------ language switch ------
  await initLanguageSupport();
}

let allFleetAgents_ = [];
let allFleetsName_ = [];
const asyncReadFleetConfig = async (fltArray) => {
  for (const flt in fltArray) {
    let fleetName = fltArray[flt].split('.').slice(0, -1).join('.');

    let fltSettings = {};
    // --- [SWARM API] fetch roles ---
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetName);
    fltSettings[fleetName] = fltConfigs;

    // console.log(fltSettings);

    allFleetsName_.push(fleetName);
    allFleetAgents_.push(fltSettings);
    await sleep(200); // sleep in ms
  }
  console.log(allFleetAgents_);
};

async function switchFleetCallback() {
  loadCurrentFleetRoles();
}

$('#hide-sidebar').on('click', async function () {
  $('.control-sidebar').ControlSidebar('toggle');
  var rolesDeckNode = document.querySelector('#roles-deck');
  var rolesCard = rolesDeckNode.getElementsByClassName('card');
  // console.log(rolesCard.length);
  var len = rolesCard.length;
  for (var i = 0; i < len; ++i) {
    rolesCard[i].setAttribute('style', 'background-color:lightgray;');
  }

  // --- update sidebar rename role ---
  let scanRoles;
  try {
    scanRoles = await fetchRoles(rmtToken_);
  } catch (err) {
    console.error(err);
  }

  scanRoles = scanRoles.roles.map(r => r.role_name);
  loadRoles(scanRoles);
});

// --- default role sample ---
async function removeRoleFromDeckCb(_roleName) {
  // pop-up modal
  // var txt;
  if (confirm("Remove the Role?")) {
    // txt = "You pressed OK!";

    // --- remove the role from fleet settings config. ---
    chkOpFileArray = [];
    fleetFileArray = [];

    // --- [SWARM API] fetch roles ---
    let chkFiles = await getRoleInUsedFiles(_roleName);
    chkOpFileArray = chkFiles.operations;
    fleetFileArray = chkFiles.fleets;

    // --- remove the operations which use the to-be removed role.
    chkRoleInUsedOperations(chkOpFileArray, $(this));

  } else {
    // txt = "You pressed Cancel!";
    console.log('press cancel')
  }
  // document.getElementById("confirm-modal").innerHTML = txt;
}

async function getRoleInUsedFiles(role_name) {
  let fileObj = {
    operations: [],
    fleets: []
  }
  // let res = await fetchGetFleets(rmtToken_);
  let fleetFiles = await fetchGetFleets(rmtToken_);
  fleetFiles = Object.keys(fleetFiles);
  // let fleetFiles = await res.json();
  console.log(fleetFiles);

  for (const fleetFile of fleetFiles) {
    var fleetName = fleetFile;

    let fltSettings = {};
    // --- [SWARM API] fetch roles ---
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetName);
    fltSettings[fleetName] = fltConfigs;
    console.log(fltSettings);

    var fltKey = Object.keys(fltSettings)[0]; // suppose only one key-value pair
    var index = fltSettings[fltKey].roles.indexOf(role_name);
    if (index >= 0) {
      restFleetOperations(fltKey, false).done(function (data) {
        fileObj.operations.push(...data);
      });
      // fltSettings[fltKey].roles.splice(index, 1);
      // var tmp_dict = {};
      // tmp_dict[fltKey] = fltSettings;
      // console.log(tmp_dict);
      fileObj.fleets.push(fltKey);
    }
  }

  return fileObj;
}
async function deleteTargetRole(_roleName) {
  // --- remove roles on back-end ---
  await restDeleteRoleData(_roleName + '.yaml');    // role (capability)
  await restDeleteBehaviorData(_roleName + '.xml'); // behvavior

  console.log("remove: " + _roleName);
  // --- remove the role from fleet config. ---
  console.log(fleetFileArray);
  for (let fleetFile of fleetFileArray) {
    let fltSettings = {};
    // --- [SWARM API] fetch roles ---
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetFile);
    fltSettings[fleetFile] = fltConfigs;
    console.log(fltSettings);

    var index = fltSettings[fleetFile].roles.indexOf(_roleName);
    fltSettings[fleetFile].roles.splice(index, 1);
    fltSettings = JSON.stringify(fltSettings);
    console.log(fltSettings);
    await restPostFleetSettings(fleetFile, fltSettings);
    fleetFileArray = fleetFileArray.filter(e => e !== fleetFile);
    console.log(fleetFileArray);
  }
}

function chkRoleInUsedOperations(_fileNameArray, _deletedTarget) {
  var _deletedRole = _deletedTarget.find(".role-name").text();
  restRoleInUsedOperations(_fileNameArray, _deletedRole, false).done(function (data) {
    if (data.length > 0) {
      let useFlowStr = '';
      data.forEach(flowFileName => {
        console.log(flowFileName)
        const splitFlowArray = flowFileName.split('-');
        const fleetName = splitFlowArray[0];
        const flowName = splitFlowArray[2].replace('.json', '');
        useFlowStr += `Fleet: ${fleetName}, Flow Name: ${flowName} \n`
      });
      if (confirm(`There are tasks or flows still using this role. Are you sure you want to delete? \n\n${useFlowStr}`)) {
        deleteOperations(data);
        deleteTargetRole(_deletedRole);
        // wsRemoveFleet("all", _deletedRole);
        //  --- remove role card from the deck ---
        $(`#rolebar-${_deletedRole}`).remove();
      }
    } else {
      deleteTargetRole(_deletedRole);
      //  --- remove agent card from the deck ---
      $(`#rolebar-${_deletedRole}`).remove();
    }

  });
}

function deleteOperations(deleteFileArray) {
  deleteFileArray.forEach((file) => {
    var operationType = file.indexOf("-task-") >= 0 ? "task" : "flow";
    if (operationType === 'task') {
      var fleetName = file.substring(0, file.indexOf("-task-"));
      var taskName = file.substring(file.indexOf("-task-") + 6).replace('.json', '');
      restDeleteTaskData(fleetName, taskName, false);
      notificationMsg(0, `${taskName} is deleted`);
    } else {
      var fleetName = file.substring(0, file.indexOf("-flow-"));
      var flowName = file.substring(file.indexOf("-flow-") + 6).replace('.json', '');
      restDeleteFlowData(fleetName, flowName, false);
      notificationMsg(0, `${flowName} is deleted`);
    }
  });
}

async function loadCurrentFleetRoles() {
  let res = await fetchGetAllRoles(rmtToken_);
  let fltRoles = await res.json();
  fltRoles = fltRoles.roles;

  // --- load roles on overview ---
  loadRoles(fltRoles);

  // toggle color theme again after DOM elements are all generated
  let isDarkMode = getSavedTheme() === 'dark';
  toggleContentDarkTheme(isDarkMode);
}

function loadRoles(_roles) {
  console.log(_roles);
  var $roleDeck = $('#roles-deck');
  $roleDeck.empty();
  _roles.sort();
  _roles.forEach((r) => {
    var node = createRoleCardView(r);
    $roleDeck.append(node);
  });
  applyFontSize(getSavedFontSize(), '#roles-deck');
}

async function cancelCreateRole() {
  $("#role-filename").css("border-color", '');
  $("#role-filename").val('');
  $('#new-role-modal').modal('hide');
}

async function createRole() {
  var $roleDeck = $('#roles-deck');
  var roleName = $('#role-filename').val();

  if (!inputCheck(roleName)) {
    notificationMsg(2, `Role name include invalid characters.`);
    return;
  }

  var roleFileExists = await restRoleFileExistence(roleName);
  if (roleFileExists) {
    notificationMsg(3, `Role(${roleName}) is already exists!`);
    return;
  }

  var node = createRoleCardView(roleName);
  $roleDeck.append(node);

  // --- create a role on back-end ---
  var btTemplate = { "elements": [{ "type": "element", "name": "root", "attributes": { "main_tree_to_execute": "BehaviorTree" }, "elements": [{ "type": "element", "name": "BehaviorTree", "attributes": { "ID": "BehaviorTree" }, "elements": [{ "type": "element", "name": "Sequence", "elements": [] }] }] }] };

  var behaviorFilename = roleName + ".xml";
  var roleFilename = roleName + ".yaml";
  var roleTemplate = {
    "Must": {
      "payload": -1,
      "behavior": `"${behaviorFilename}"`
    },
    "Prefer": {
      "speed": true
    }
  };
  // --- CRITICAL FORMAT! ---
  var roleTemplate = `Must:\n` +
    `  payload: -1 \n` +
    `  artifacts: [] \n` +
    `  behavior: "${behaviorFilename}"\n` +
    `  perceptions: [] \n` +
    `  behaviors: []\n` +
    `Prefer:\n` +
    `  speed: True\n`;

  // CRITICAL! Should not decorate it await
  restPostBehaviorData(behaviorFilename, btTemplate);
  restPostRoleData(roleFilename, roleTemplate);

  // --- add to fleet config. ---
  var fleetFilename = getSelectedFleet();

  let fltSettings = {};
  // --- [SWARM API] fetch roles ---
  let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetFilename);
  fltSettings[fleetFilename] = fltConfigs;
  console.log(fltSettings);

  fltSettings = JSON.stringify(fltSettings);
  await restPostFleetSettings(fleetFilename, fltSettings);

  notificationMsg(0, `${roleName} Role is created!`);
  modalInputSetting('default');
}

function createRoleCardView(_roleName) {
  const template = document.querySelector('#role-card');
  const node = document.importNode(template.content, true);

  let cardNode = node.querySelector('.card');
  cardNode.setAttribute('id', `rolebar-${_roleName}`);

  let titleNode = node.querySelector('.role-name');
  titleNode.textContent = _roleName;

  // [20220816 FAR-1968]
  // --- role capabilities edit event callback ---
  // var configNode = node.querySelector('.config-role');
  // configNode.setAttribute('title', 'Edit Capabilities');
  // configNode.addEventListener('click', loadSidebarAsyncCb.bind(cardNode, _roleName));

  // --- role behavior edit event callback ---
  var editNode = node.querySelector('.edit-role');
  editNode.setAttribute('title', 'Edit Role');
  editNode.setAttribute('id', `${_roleName}-edit-btn`);
  editNode.addEventListener('click', editRoleAsyncCb.bind(this, _roleName));

  // --- role remove event callback ---
  var removeNode = node.querySelector('.remove-role');
  removeNode.setAttribute('id', `${_roleName}-remove-btn`);
  removeNode.addEventListener('click', removeRoleFromDeckCb.bind(cardNode, _roleName));

  return node;
}

let selectedRoleDataCache_;
async function loadSidebarAsyncCb(_name) {
  this.setAttribute('style', 'background-color:white');

  // --- update role name ---
  $('#role-name').text(`${_name}`);
  // sbRoleName.val(_name);
  // sbRoleName.data('roleName', _name);
  // console.log(_name);

  // --- get role content (capabilities) from back-end. ---
  var roleData = await restGetRoleData(_name);
  // console.log(roleData);
  selectedRoleDataCache_ = flattenRoleData(roleData);
  var data = roleDataConversion(roleData);
  console.log(data);

  var $capDeck = $('#cap-deck');
  $capDeck.empty();

  // --- filter capability attributes ---
  data = data.filter(cap => capInputAttrMap_.hasOwnProperty(cap.name));
  console.log(data);
  // --- filter initialized capability attributes ---
  data = data.filter(cap => !((cap.name === 'payload') && (cap.value < 0)));
  data = data.filter(cap => !((cap.name === 'perceptions') && (cap.value.length === 0)));
  // --- render capability attributes ---
  data.forEach((cap) => {
    createCapRowViewByJSON(cap);
  })
}

function cvtBlackboard2JSON(_data) {
  console.log(_data);
  var json = {};
  _data.forEach(obj => {
    console.log(obj);
    var key = obj.attributes.output_key;
    var val = obj.attributes.value;
    json[key] = val;
  });
  return json;
}

$('#sb-role-edit').click(function () {
  roleEditButtonSwitch();
});

async function editRoleAsyncCb(_name) {
  // --- update editing role name ---
  $('#editing-role-name').html(_name);
  $('#hide_origin_role_name').html(_name);

  // --- switch to Role Editor ---
  document.getElementById("roles-showcase").style.display = "none";
  document.getElementById("role-editor").style.display = "inline";

  // --- Transform bt xsd to toolbar ---
  var btXsdContent = await restGetBtXsd();
  btXsdContent = JSON.parse(btXsdContent);

  var btXsdData = btXsdContent["xs:schema"]["xs:group"];
  console.log(btXsdData);

  var groups = [];
  btXsdData.forEach((obj) => {
    var gName = obj["_attributes"].name;
    // console.log(gName);
    if (gName !== "BuiltInMultipleTypes") {
      var groupObj = {};
      groupObj["name"] = gName;
      groupObj["members"] = [];
      // collect the group and its members.
      var membersObj = obj["xs:choice"]["xs:element"];
      // console.log(membersObj);
      if (Array.isArray(membersObj)) {
        membersObj.forEach((m) => {
          groupObj["members"].push(m["_attributes"].name);
        });
        groups.push(groupObj);
      }
      else {
        groupObj["members"].push(membersObj["_attributes"].name);
        groups.push(groupObj);
      }
    }
  });

  // --- fetch role behavior file from back-end ---
  var roleData = await restGetRoleData(_name);
  console.log(roleData);

  // -- case: CANNOT read the file --
  var fileReadFail = roleData.includes('ENOENT');
  var processedData;

  if (fileReadFail) {
    processedData = undefined;
    alert('fail to load role yaml file!');
  }
  else {
    // -- fetch and extract behavior filename --
    //  -- get role content (capabilities) --
    selectedRoleDataCache_ = flattenRoleData(roleData);
    console.log(selectedRoleDataCache_);
    selectedRoleDataCache_.name = _name;

    var data = roleDataConversion(roleData);

    //  -- get role behavior filename --
    var btFilename = "";
    data.forEach((d) => {
      if (d.name === "behavior") {
        btFilename = d.value;
      }
    })
    if (btFilename === "") {
      notificationMsg(3, 'No Behavior Found!')
      // toast("No Behavior Found!");
    }

    //  -- fetch behavior content --
    var btContent = await restGetBehaviorData(btFilename);
    console.log(btContent);
    btContent = JSON.parse(btContent);
    console.log(btContent);

    // --- data processing ---
    var root = btContent["elements"][0];
    var BehaviorTree = root["elements"][0];
    var sequence = BehaviorTree["elements"][0];

    processedData = cvtBtData2Editor(sequence);

    // --- filter out the top sequece ---
    console.log(JSON.stringify(processedData));
    processedData = processedData.children;
  }

  // TODO -- [to be confirmed] data preprocess - remove top sequence --
  // --- update role editor ---
  console.log(JSON.stringify(processedData));
  if (processedData === undefined) {
    // WorkAround: empty case
    processedData = [{}];
    renderRoleEditor(processedData);
    $('#nestable > .dd-list').empty();
  }
  else {
    renderRoleEditor(processedData);
  }

  // --- append the block by its attributes ---
  var brXsdData = await restGetBrXsd();
  brXsdData = JSON.parse(brXsdData);
  var brData3 = brXsdData["xs:schema"]["xs:complexType"];
  var typesAttrs = {};
  brData3.forEach((obj) => {
    var typeName = obj["_attributes"].name.replace(/Type/gi, '');
    var xsAttrArray = obj["xs:attribute"];
    var xsAttrs = [];

    if (Array.isArray(xsAttrArray)) {
      xsAttrArray.forEach((x) => {
        var obj = {};
        obj['name'] = x._attributes.name;
        obj['default'] = x._attributes.default;
        xsAttrs.push(obj);
      });
    }
    else if (xsAttrArray !== undefined) {
      var obj = {};
      obj['name'] = xsAttrArray._attributes.name;
      obj['default'] = xsAttrArray._attributes.default;
      xsAttrs.push(obj);
    }
    typesAttrs[typeName] = xsAttrs;
  });

  console.log(typesAttrs);

  // --- generate the toolbar ---
  var $toolbar = $('#bt-toolbar');
  $toolbar.empty();
  groups.forEach((g) => {
    var groupTab = createGroupTabView(g, typesAttrs);
    $toolbar.append(groupTab);
  });

  applyFontSize(getSavedFontSize(), '#bt-container');
}

function cvtBtData2File(_data) {
  console.log(_data);

  // --- convert to original data format ---
  if (_data.hasOwnProperty("children")) {
    // brach case (Sequence, Repeat)
    console.log(_data);
    var seqObj = { type: "element", name: _data.title, attributes: {}, elements: [] };

    for (key in _data) {
      if (key === 'id' || 'title' || 'children') continue;

      seqObj.attributes[key] = _data[key];
    }

    _data.children.forEach((el) => {
      console.log(el);
      var elObj = cvtBtData2File(el);
      if (elObj !== undefined) {
        seqObj.elements.push(elObj);
        console.log(elObj);
      }
    });
    console.log(seqObj);
    return seqObj;
  }
  else {
    // node case
    console.log(_data);
    var obj = { type: "element", name: _data.title, attributes: {} }

    for (key in _data) {
      if (key === 'id') continue;
      if (key === 'title') continue;

      obj.attributes[key] = _data[key];
    }
    console.log(obj);
    return obj;
  }
}

var blackBoardCache_ = [];

function cvtBtData2Editor(_data) {
  console.log(_data);
  if (_data.name === "SetBlackboard") {
    console.log(_data);
    blackBoardCache_.push(_data);
    return;
  }

  if (_data.hasOwnProperty("elements")) {
    // 'sequence' case
    console.log(_data.name);
    var seqObj = { id: makeId(), content: _data.name, title: _data.name, children: [] };
    // -- attributes --
    var attributes = _data.attributes;
    // console.log(attributes);
    for (key in attributes) {
      seqObj[key] = attributes[key];
    }

    // -- elements --
    _data.elements.forEach((el) => {
      console.log(el);
      var elObj = cvtBtData2Editor(el);
      if (elObj !== undefined) {
        seqObj.children.push(elObj);
        console.log(elObj);
      }
    });
    console.log(seqObj);
    return seqObj;
  }
  else {
    // non 'sequence case'
    // console.log(_data);
    // console.log(_data.name);
    var obj = { id: makeId(), content: _data.name, title: _data.name }
    var attributes = _data.attributes;
    // console.log(attributes);
    for (key in attributes) {
      obj[key] = attributes[key];
    }
    // console.log(obj);
    return obj;
  }
}

function expandAttrPaneView(_node) {
  var item_setting = $(_node).closest(".dd-item").find(".item-settings");
  if (item_setting.hasClass("d-none")) {
    item_setting.removeClass("d-none");
  } else {
    item_setting.addClass("d-none");
  }
}

function removeBrBlockView(_node) {
  // --- remove the behavior item view ---
  _node.parentNode.parentNode.remove();

  // --- remove attributes from black board ---
  var btAttrs = _node.parentNode.parentNode.attributes;
  // console.log(btAttrs);
  for (var i = 0; i < btAttrs.length; i++) {
    console.log("Name: " + btAttrs[i].name + " Value: " + btAttrs[i].value);

    if (!btAttrs[i].value.indexOf('{')) {
      var attrKey = btAttrs[i].value.replace(/[{}]/g, "");
      blackBoardCache_ = blackBoardCache_.filter(bb => bb.attributes.output_key !== attrKey);
    }
  }

  // --- [Debugging] update the data ---
  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

function attrFormatRectifier(_attrArr) {
  console.log(_attrArr);
  var retArr = [];
  var title = _attrArr['data-title'];
  var id = _attrArr['data-id'];
  for (key in _attrArr) {
    if (key === 'data-id') continue;
    if (key === 'data-title') continue;
    if (key === 'class') continue;

    var myObj = {};
    myObj["name"] = key.replace(/data-/g, '');
    myObj["default"] = _attrArr[key];
    retArr.push(myObj);
  }
  return { title: title, attr: retArr, id: id };
}

function renderRoleEditor(_json) {
  console.log(JSON.stringify(_json));
  // console.log(_json);
  console.log(blackBoardCache_);

  $("#nestable")
    .nestable({
      group: 1,
      json: _json,
      itemRenderer: function (item_attrs, content, children, options) {
        // console.log(item_attrs);

        var attrResult = attrFormatRectifier(item_attrs);
        console.log(attrResult);

        var block = genBtBlockPane(attrResult.title, attrResult.attr, attrResult.id);

        var attrBtn = block.btn;
        var attrData = block.attr;
        var attrPane = block.dom;
        var childAttr = (content === "Repeat") ? '' : 'dd-nochildren';
        childAttr = `<li class="dd-item dd3-item ${childAttr}" ${attrData}>`;

        // var title = dictUiTerms[content];
        // [protection]
        var title = (dictUiTerms.hasOwnProperty(content)) ? dictUiTerms[content] : content;

        var html =
          childAttr +
          `<div class="dd-handle dd3-handle"> Drag</div>` +
          `<div class="dd3-content"><span>${title}</span>` +
          `<div class="item-trash" onclick="removeBrBlockView(this);"><i class="fas fa-trash"></i></div>` +
          attrBtn +
          attrPane +
          children +
          `</li>`;

        return html;
      },
    })
    .on("change", updateOutput);

  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

let editedRole_;

function changeServiceType(_node) {
  var type = _node.options[_node.selectedIndex].text;
  console.log(type);

  var serviceNode = _node.parentNode.nextElementSibling;
  // console.log(serviceNode);

  // --- get service select ---
  var selectNode = serviceNode.querySelector('select');
  removeAllChildNodes(selectNode);
  services = service_options[type];
  // console.log(services);
  services.forEach((srv) => {
    var option = document.createElement("option");
    option.text = srv;
    selectNode.appendChild(option);
  });

}

function createGroupTabView(_btObj, _attrArr) {
  // console.log(_attrArr);
  // --- create tabs ---
  const template = document.querySelector('#bt-group-tab');
  const node = document.importNode(template.content, true);

  // --- create menus ---
  var nameNode = node.querySelector('.dropdown-toggle');
  nameNode.textContent = _btObj.name;

  // --- append the members into the tab iteratively ---
  var menuNode = node.querySelector('.dropdown-menu');
  _btObj.members.forEach((m) => {
    var aEl = document.createElement('a');
    aEl.setAttribute('class', 'dropdown-item');

    var iEl = document.createElement('i');
    iEl.setAttribute('class', 'fas fa-running');

    aEl.append(iEl);

    var labelEl = document.createElement('label');
    // var title = dictUiTerms[m];
    // [protection]
    var title = (dictUiTerms.hasOwnProperty(m)) ? dictUiTerms[m] : m;

    labelEl.innerHTML = '&ensp;' + title;
    aEl.append(labelEl);

    aEl.addEventListener('click', appendBtBarOnEditor.bind(this, m, _attrArr));

    menuNode.append(aEl);
  });

  return node
}

function updateUserEditVal(_node) {
  // -- get modified value ---
  var value = "";
  if ($(_node).is('select')) {
    value = _node.options[_node.selectedIndex].text;
  }
  else { // input case
    value = _node.value;
  }

  // -- get attr-name ---
  var attrName = _node.parentNode.querySelector('label').getAttribute('attr-name');

  // -- update data attr ---
  $(_node).closest(".dd-item").data(attrName, value);

  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

function updateSelectVal2(_node) {
  var value = _node.options[_node.selectedIndex].text;
  console.log(value);
  var attrName = _node.parentNode.querySelector('label').getAttribute('attr-name');
  console.log(attrName);
  // -- update data attr ---
  $(_node).closest(".dd-item").data(attrName, value);

  var nextDom = _node.parentNode.nextElementSibling;
  // console.log(nextDom);
  var srvDom = nextDom.querySelector('select');
  $(srvDom).empty();

  var opts = service_options[value];
  opts.forEach(o => {
    var option = document.createElement("option");
    option.text = o;
    option.value = 0;
    srvDom.append(option);
  });
  // -- update data attr ---
  var attrName = nextDom.querySelector('label').getAttribute('attr-name');
  console.log(attrName);
  // -- update data attr ---
  console.log(opts[0]);
  $(nextDom).closest(".dd-item").data(attrName, opts[0]);

  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

function updateSelectVal(_node) {
  // -- get modified value ---
  var value = "";
  if ($(_node).is('select')) {
    value = _node.options[_node.selectedIndex].text;
    // TODO: refactor
    // artifactSrvType_ = value;
    if (_node === 'docking' || 'undocking') {
      value = dictBtTerms[value];
    }
  }
  else { // input case
    // console.log((_node).is("input"));
    value = _node.value;
  }

  console.log(value);


  // -- get attr-name ---
  var attrName = _node.parentNode.querySelector('label').getAttribute('attr-name');

  // -- update data attr ---
  $(_node).closest(".dd-item").data(attrName, value);

  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

// --- available attributes for role editor ---
// 'duration',
let avAttrs = ['num_cycles', 'dock', 'type', 'service', 'retry_time'];

function compareStr(a, b) {
  if (a.name > b.name) {
    return -1;
  }
  if (a.name < b.name) {
    return 1;
  }
  return 0;
}

function genBtBlockPane(_title, _attr, _uuid) {
  console.log(_title);
  console.log(_attr);

  // --- edit on role editor case ---
  var attributes = _attr.filter(a => { return avAttrs.includes(a.name); });
  console.log(attributes);

  var retDom = '';
  var retAttr = '';

  // TODO: resolve the workaround
  // console.log(attributes);
  attributes.sort(compareStr);
  // console.log(attributes);

  // --- conditions by titles ---
  let srvType_ = '';
  if (_title === "Artifact") {
    attributes.forEach(attr => {
      if (attr.name === 'type') {
        console.log(service_options);
        var attrOptions = "";
        var typeOptions = Object.keys(service_options);
        console.log(attr.default);
        typeOptions.forEach((rm) => {
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="artifact_types" class="artifact_types" onchange="updateSelectVal2(this);">${attrOptions}</select></p>`;
        var types = Object.keys(service_options);
        var attrVal = (attr.default === undefined) ? types[0] : attr.default;
        console.log(attrVal);
        retAttr += ` data-${attr.name}=${attrVal}`;

        srvType_ = attrVal;
        console.log(srvType_);
      }
      else if (attr.name === 'service') {
        // TODO: --- refactor ---
        console.log(srvType_);
        var options = service_options[srvType_];
        console.log(options);
        // var options =service_options[];
        var attrOptions = "";
        options.forEach((rm) => {
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="artifact_services" class="artifact_services" onchange="updateUserEditVal(this);">${attrOptions}</select></p>`;
        // retAttr += ` data-${attr.name}="${options[0]}"`;
        var attrVal = (attr.default === undefined) ? options[0] : attr.default;
        console.log(attrVal);
        retAttr += ` data-${attr.name}=${attrVal}`;
      }
    });
  } else if (_title === "Docking") {
    attributes.forEach(attr => {
      if (attr.name === 'dock') {
        console.log(attr.default);
        var attrOptions = "";
        var dockOptions = dock_options;
        var val = attr.default;
        // attr.default = dictUiTerms[val];
        // [protection]
        attr.default = (dictUiTerms.hasOwnProperty(val)) ? dictUiTerms[val] : val;

        dockOptions.forEach((rm) => {
          console.log(rm);
          console.log(attr.default);
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        // retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select onclick="updateSelectVal(this);">${attrOptions}</select></p>`;
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="docking_types">${attrOptions}</select></p>`;
        retAttr += ` data-${attr.name}=${dictBtTerms[dockOptions[0]]}`;
      }
    });
  } else if (_title === "DockingArtifact") {
    attributes.forEach(attr => {
      if (attr.name === 'type') {
        console.log(service_options);
        var attrOptions = "";
        var typeOptions = Object.keys(service_options);
        console.log(attr.default);
        typeOptions.forEach((rm) => {
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="artifact_types" class="artifact_types" onchange="updateSelectVal2(this);">${attrOptions}</select></p>`;
        var types = Object.keys(service_options);
        var attrVal = (attr.default === undefined) ? types[0] : attr.default;
        console.log(attrVal);
        retAttr += ` data-${attr.name}=${attrVal}`;

        srvType_ = attrVal;
        console.log(srvType_);
      }
      else if (attr.name === 'service') {
        // TODO: --- refactor ---
        console.log(srvType_);
        var options = service_options[srvType_];
        console.log(options);
        // var options =service_options[];
        var attrOptions = "";
        options.forEach((rm) => {
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="artifact_services" class="artifact_services" onchange="updateUserEditVal(this);">${attrOptions}</select></p>`;
        // retAttr += ` data-${attr.name}="${options[0]}"`;
        var attrVal = (attr.default === undefined) ? options[0] : attr.default;
        console.log(attrVal);
        retAttr += ` data-${attr.name}=${attrVal}`;
      } else if (attr.name === 'dock') {
        console.log(attr.default);
        var attrOptions = "";
        var dockOptions = dock_options;
        var val = attr.default;
        // attr.default = dictUiTerms[val];
        // [protection]
        attr.default = (dictUiTerms.hasOwnProperty(val)) ? dictUiTerms[val] : val;

        dockOptions.forEach((rm) => {
          console.log(rm);
          console.log(attr.default);
          var tag = (attr.default === rm) ? "selected" : "";
          attrOptions += `<option ${tag}>${rm}</option>`;
        });
        // retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select onclick="updateSelectVal(this);">${attrOptions}</select></p>`;
        retDom += `<p style="display:none;"><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><select data-type="docking_types">${attrOptions}</select></p>`;
        retAttr += ` data-${attr.name}=${dictBtTerms[dockOptions[0]]}`;
      } else if (attr.name === 'retry_time') {
        retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><input type="text" value="${attr.default}" onkeyup="updateUserEditVal(this);"></p>`;
        retAttr += ` data-${attr.name}="${attr.default}"`;
      }
    });
  } else {
    attributes.forEach(attr => {
      retDom += `<p><label style="width:25%" attr-name="${attr.name}">${attr.name}: </label><input type="text" value="${attr.default}" onkeyup="updateUserEditVal(this);"></p>`;
      retAttr += ` data-${attr.name}="${attr.default}"`;
    });

  }

  retDom = (attributes.length) ? `<div class="item-settings" style="font-size:18px;">${retDom}</div>` : '';
  var attrBtn = (attributes.length) ? '<div class="item-edit2" onclick="expandAttrPaneView(this);"><i class="fas fa-cog"></i></div></div>' : '';

  // --- edit on plan editor case ---
  var planAttrs = _attr.filter(a => { return !avAttrs.includes(a.name); });
  // console.log(planAttrs);
  // console.log(blackBoardCache_);
  planAttrs.forEach(attr => {
    console.log(attr.default);
    if (attr.default !== undefined) {
      attr.default = attr.default.replace(/[{}]/g, '');
    }
    // console.log(attr);
    const result = blackBoardCache_.filter(attrObj => attrObj.attributes.output_key === attr.default);
    // console.log(result);
    // --- append new attributes for plan ----
    if (result.length === 0) {
      var bb = { "type": "element", "name": "SetBlackboard", "attributes": { "output_key": `${attr.name}_${_uuid}`, "value": `${attr.default}` } };
      blackBoardCache_.push(bb);

      retAttr += ` data-${attr.name}="{${attr.name}_${_uuid}}"`;
    }

    // --- reload existing attributes in role xml file ---
    retAttr += ` data-${attr.name}="{${attr.default}}"`;
  });
  retAttr = `data-id="${_uuid}" data-title="${_title}" ${retAttr}`;

  return { dom: retDom, attr: retAttr, btn: attrBtn };
}

function makeId(_len = 5) {
  let pool = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
  let poolSize = pool.length;
  let result = '';
  for (let i = 0; i < _len; ++i) {
    result += pool.charAt(Math.floor(Math.random() * poolSize));
  }
  return result;
}

function appendBtBarOnEditor(_member, _attrs) {
  // --- fill the attributes of each behavior by bt xsd ---
  var attrs = _attrs[_member];
  console.log(attrs);

  // --- [test] build attr pane ---
  console.log(service_options);
  var attrResult = genBtBlockPane(_member, _attrs[_member], makeId());
  var attrData = attrResult.attr;
  var attrPane = attrResult.dom;
  var attrBtn = attrResult.btn;

  // var title = dictUiTerms[_member];
  // [protection]
  var title = (dictUiTerms.hasOwnProperty(_member)) ? dictUiTerms[_member] : _member;

  var childAttr = (_member === "Repeat") ? '' : 'dd-nochildren';
  childAttr = `<li class="dd-item dd3-item ${childAttr}" ${attrData}>`;

  var html =
    childAttr +
    `<div class="dd-handle dd3-handle"> Drag</div>` +
    `<div class="dd3-content"><span>${title}</span>` +
    `<div class="item-trash" onclick="removeBrBlockView(this);"><i class="fas fa-trash"></i></div>` +
    attrBtn +
    attrPane +
    `</li>`;

  $('#nestable > .dd-list').append(html);

  updateOutput($("#nestable").data("output", $("#nestable-output")));
}

function addNewCapCb() {
  var capObj = {};

  // --- fetch the input data ---
  var name = $('#sbft-cap-name').val();
  var type = $('#sbft-cap-type').val();
  var value = $('#sbft-cap-value').val();

  // --- reset the input field ---
  // $('#sbft-cap-name').val('');
  $('#sbft-cap-name').prop('selectedIndex', 0);
  $('#sbft-cap-type').prop('selectedIndex', 0);
  // $('#sbft-cap-value').val('');
  // --- payload default options ---
  var elInput = document.createElement('input');
  elInput.setAttribute('class', 'form-control cap-value');
  elInput.setAttribute('id', 'sbft-cap-value');
  elInput.setAttribute('placeholder', 'unit: kg');
  var capNode = document.getElementById('sbft-cap-value');
  var capParentNode = document.getElementById('sbft-cap-value').parentNode;
  capNode.remove();
  capParentNode.append(elInput);

  capObj.name = name;
  capObj.type = (type === 'Required') ? 'Must' : 'Prefer';
  capObj.value = value;
  console.log(capObj);

  // --- update on view ---
  var $capDeck = $('#cap-deck');
  var capRow = createCapRowView(capObj);
  $capDeck.append(capRow);

  // --- update in cache ---
  var newCap = {};
  newCap['name'] = name;
  newCap['type'] = type;
  newCap['value'] = value;
  selectedRoleDataCache_.push(newCap);
}

function createCapRowViewByJSON(_cap) {
  var $capDeck = $('#cap-deck');
  var capRow = createCapRowView(_cap);
  $capDeck.append(capRow);
}

const capInputAttrMap_ = {
  'payload': function (_mountNode, _cap, options = { min: 0, max: 1000 }) {
    _mountNode.value = _cap.value;
    _mountNode.setAttribute('placeholder', `range: ${options.min}-${options.max}kg`);
    _mountNode.addEventListener('keyup', (e) => {
      if (!_.inRange(Number(_mountNode.value), options.min, options.max + 1)) {
        alert(`The payload is OUT-OF-RANGE!`);
        _mountNode.value = (_mountNode.value > options.max) ? options.max : options.min;
      }
    });
  },
  // 'artifacts': function (_mountNode, _cap) {
  //   _mountNode.value = _cap.value[0];
  //   var elSelect = document.createElement('select');
  //   elSelect.setAttribute('class', 'form-control cap-value');
  //   elSelect.setAttribute('disabled', true);
  //   ['fork', 'lift', 'conveyor', 'arm'].forEach((a) => {
  //     var elOption = document.createElement('option');
  //     elOption.value = elOption.text = a;
  //     elSelect.add(elOption);
  //   });
  //   elSelect.value = _cap.value[0];
  //   _mountNode.parentNode.replaceChild(elSelect, _mountNode);
  // },
  // 'behavior': function(_mountNode, _cap){},
  'perceptions': function (_mountNode, _cap) {
    _mountNode.value = _cap.value[0];
    var elSelect = document.createElement('select');
    elSelect.setAttribute('class', 'form-control cap-value');
    elSelect.setAttribute('disabled', true);
    [{ name: 'LiDAR', value: 'lidar' }].forEach((a) => {
      var elOption = document.createElement('option');
      elOption.value = a.value;
      elOption.text = a.name;
      elSelect.add(elOption);
    });
    // elSelect.value = _cap.value[0];
    elSelect.value = _cap.value;
    console.log(_cap.value);
    _mountNode.parentNode.replaceChild(elSelect, _mountNode);
  },
  // 'speed': function (_mountNode, _cap) {
  //   _mountNode.value = _cap.value[0];
  //   var elSelect = document.createElement('select');
  //   elSelect.setAttribute('class', 'form-control cap-value');
  //   elSelect.setAttribute('disabled', true);
  //   ['true', 'false'].forEach((a) => {
  //     var elOption = document.createElement('option');
  //     elOption.value = a;
  //     elOption.text = a;
  //     elSelect.add(elOption);
  //   });
  //   elSelect.value = _cap.value;
  //   _mountNode.parentNode.replaceChild(elSelect, _mountNode);
  // },
  'default': function (_mountNode, _cap) {
    _mountNode.value = _cap.value;
  }
};

const capNameHints_ = {
  'payload': 'the weight that the agent is affordable',
  'artifacts': 'will tell what ARTIFACTS is for',
  'behavior': 'will tell what BEHAVIOR is for',
  'perceptions': 'the sensor that the agent is equipped with',
  'speed': 'will tell what SPEED is for',
  'default': 'no one knows'
};

function createCapRowView(_cap) {
  const template = document.querySelector('#sb-capability-row');
  const node = document.importNode(template.content, true);

  var nameNode = node.querySelector('.cap-name');
  var token = capNameHints_.hasOwnProperty(_cap.name) ? _cap.name : 'default';
  nameNode.setAttribute('title', capNameHints_[token]);
  nameNode.value = (_cap.name === 'perceptions') ? _cap.name.slice(0, -1) : _cap.name;

  var typeNode = node.querySelector('.cap-type');
  var uiType = (_cap.type === 'Must') ? 'Required' : 'Preferred';
  typeNode.value = uiType;

  var valueNode = node.querySelector('.cap-value');
  token = capInputAttrMap_.hasOwnProperty(_cap.name) ? _cap.name : 'default';
  capInputAttrMap_[token](valueNode, _cap);

  var btnEditNode = node.querySelector('.cap-edit');
  var rowNode = node.querySelector('.card-header > .row');
  btnEditNode.addEventListener('click', capEditButtonSwitch.bind(rowNode));

  var cardNode = node.querySelector('.card');
  var rmNode = node.querySelector('.cap-remove');
  rmNode.addEventListener('click', removeCapFromDeckCb.bind(cardNode));

  return node
}

function removeCapFromDeckCb() {
  // pop-up modal
  // var txt;
  if (confirm("Remove the Capability?")) {
    // txt = "You pressed OK!";
    //  --- remove agent card from the deck ---
    $(this).CardWidget('remove');
    var capName = this.querySelector('.cap-name').value;
    var capType = this.querySelector('.cap-type').value;
    capType = (capType === 'Required') ? 'Must' : 'Prefer';
    var capValue = this.querySelector('.cap-value').value;
    if (capName === 'payload') {
      selectedRoleDataCache_ = selectedRoleDataCache_.filter(srd => srd.name !== capName);
    }
    if (capName === 'perception') {
      let perceptions = selectedRoleDataCache_.find(srd => srd.name === 'perceptions' && srd.type === capType);
      perceptions.value = perceptions.value.filter(p => p !== capValue);
    }

    console.log(selectedRoleDataCache_);
  } else {
    // txt = "You pressed Cancel!";
  }
  // document.getElementById("confirm-modal").innerHTML = txt;
  // selectedRoleDataCache_.forEach(srd => {
  //   console.log(srd);
  // })
}

function capEditButtonSwitch() {
  // --- cap. name, as an ID, SHOULD NOT be editable ---
  // var nameInputNode = this.querySelector('.cap-name');
  // console.log(nameInputNode);
  // nameInputNode.readOnly = !nameInputNode.readOnly;

  var typeInputNode = this.querySelector('.cap-type');
  typeInputNode.disabled = !typeInputNode.disabled;

  var valueInputNode = this.querySelector('.cap-value');
  console.log(valueInputNode.type);
  valueInputNode.readOnly = !valueInputNode.readOnly;
  if (valueInputNode.type !== 'text') { // select-one
    valueInputNode.disabled = !valueInputNode.readOnly;
  }

  var btnIconNode = this.querySelector('.cap-edit > .fas');

  btnIconNode.classList.toggle("fa-pen");
  btnIconNode.classList.toggle("fa-eye");
}

function roleEditButtonSwitch() {
  var target = $('.edit_role_name_btn .fa');
  var targetNode = document.getElementById('editing-role-name');

  if (targetNode.type === 'text' && !validate($(targetNode).val())[0]) {
    return;
  }

  if ($(target).hasClass('fa-pen')) {
    $(target).attr("class", "fa fa-eye");
    var target_width = targetNode.offsetWidth + 5;

    // console.log($(targetNode).text());
    var inputNode = document.createElement('input');
    inputNode.setAttribute("id", "editing-role-name");
    inputNode.setAttribute("class", 'col-12 editing-rolename-input role-input');
    inputNode.setAttribute("style", "font-size:24px");
    inputNode.setAttribute("style", `width: ${target_width + 40}px`);
    inputNode.value = $(targetNode).text();

    $(targetNode).replaceWith($(inputNode));

    validateEvent();
    return;
  }

  if ($(target).hasClass('fa-eye')) {
    $(target).attr("class", "fa fa-pen");
    var modifiedName = $(targetNode).val();

    var inputNode = document.createElement('span');
    inputNode.setAttribute("id", "editing-role-name");
    inputNode.setAttribute("class", 'editing-rolename-span');
    inputNode.setAttribute("style", "font-size:24px");
    inputNode.setAttribute("style", "word-break:break-all");
    inputNode.textContent = modifiedName;
    // $('#hide_origin_role_name').html(modifiedName);

    $(targetNode).replaceWith($(inputNode));

    return;
  }
}

function flattenRoleData(_data) {
  _data = JSON.parse(_data);
  var cvtData = [];
  for (var key in _data) {
    // console.log(key);
    var dataPair = _data[key];
    console.log(dataPair);
    for (var capName in dataPair) {
      var cap = {};
      console.log(`${capName}, ${key}, ${dataPair[capName]}`);
      // console.log(capName);
      cap.name = capName;
      cap.type = key;
      cap.value = dataPair[capName];
      var capValue = dataPair[capName];
      console.log(typeof capValue);
      cvtData.push(cap);
    }
  }

  return cvtData;
}

function roleDataConversion(_data) {
  _data = JSON.parse(_data);
  var cvtData = [];
  for (var key in _data) {
    // console.log(key);
    var dataPair = _data[key];
    console.log(dataPair);
    for (var capName in dataPair) {
      var cap = {};
      console.log(`${capName}, ${key}, ${dataPair[capName]}`);
      // console.log(capName);
      cap.name = capName;
      cap.type = key;
      cap.value = dataPair[capName];
      var capValue = dataPair[capName];
      console.log(typeof capValue);
      // if (typeof capValue !== 'object' || capName === 'artifacts') {
      if (capName !== 'behaviors') {
        cvtData.push(cap);
      }
    }
  }
  return cvtData;
}

async function saveRoleCapsCb() {
  var capDeck = document.querySelector('#cap-deck');
  var capRows = capDeck.querySelectorAll('.row');
  console.log(capRows);
  var updatedCap = [];
  capRows.forEach((cr) => {
    var capObj = {};

    capObj.name = cr.querySelector('.cap-name').value;
    capObj.type = cr.querySelector('.cap-type').value;
    capObj.value = cr.querySelector('.cap-value').value;

    // --- data-type justification ---
    if (capObj.name === 'payload') {
      capObj.value = Number(capObj.value);
    }
    if (capObj.name === 'speed') {
      capObj.value = capObj.value.toLowerCase();
      capObj.value = (capObj.value === 'true');
    }
    if (capObj.name === 'artifacts' || capObj.name === 'perceptions') {
      capObj.value = [capObj.value];
    }

    console.log(`${capObj.name}, ${capObj.type}, ${capObj.value}`);
    updatedCap.push(capObj);
  })
  console.log(updatedCap);

  // --- transform the perceptions data ---
  var tmpReqPcptRows = updatedCap.filter(uc => uc.name === 'perception' && uc.type === 'Required');
  var reqPerceptionArr = tmpReqPcptRows.map(p => p.value);
  var reqPcptRows = { name: 'perceptions', type: 'Required', value: reqPerceptionArr };

  var tmpPrefPcptRows = updatedCap.filter(uc => uc.name === 'perception' && uc.type === 'Preferred');
  var prefPerceptionArr = tmpPrefPcptRows.map(p => p.value);
  var prefPcptRows = { name: 'perceptions', type: 'Preferred', value: prefPerceptionArr };

  updatedCap = updatedCap.filter(uc => uc.name !== 'perception');
  updatedCap.push(reqPcptRows);
  updatedCap.push(prefPcptRows);

  // --- role capability types transformation ---
  for (cap of updatedCap) {
    console.log(cap)
    cap.type = (cap.type === 'Required') ? 'Must' : 'Prefer';
  }
  console.log(updatedCap);

  // --- update the edited data ---
  console.log(JSON.stringify(selectedRoleDataCache_));
  updatedCap.forEach((uc) => {
    selectedRoleDataCache_.forEach((rdc) => {
      if (uc.name === rdc.name && (uc.type === rdc.type)) {
        rdc.value = uc.value;
      }
    })
  })
  console.log(selectedRoleDataCache_);

  // --- unflatten the capabilities data  ---
  var restoredData = unflattenRoleData(selectedRoleDataCache_);
  console.log(restoredData);

  // --- write back to the back-end. ---
  var origRoleName = $('#sb-role-name').data('roleName');
  console.log(origRoleName);

  let modiRoleName = $('#role-name').text()
  // var modiRoleName = document.querySelector('#sb-role-name').value;
  var roleFilename = modiRoleName + '.yaml';
  console.log(roleFilename);

  // --- format conversion JSON > YAML ---
  var res = cvtRoleCapJson2Yaml(restoredData);
  console.log(res);
  await restPostRoleData(roleFilename, res);

  notificationMsg(0, 'Role Capabilities are saved!');
}

function cvtRoleCapJson2Yaml(_roleCap) {
  // --- Must capabilities conversion ---
  var must = _roleCap['Must'];
  var mustCap = JSON.stringify(must).replace(/[{}]/g, '');
  var mCap = mustCap.split(',');
  // console.log(mustCap);
  // console.log(mCap);

  var mCapStr = '';
  mCap.forEach(m => {
    m = m.split(':');
    m[0] = m[0].replace(/\"/g, '');
    m[1] = m[1].replace(/true/g, 'True').replace(/false/g, 'False');
    mCapStr += `  ${m[0]}: ${m[1]}\n`;
  })
  //
  // --- Prefer capabilities conversion ---
  var prefer = _roleCap['Prefer'];
  var preferCap = JSON.stringify(prefer).replace(/[{}]/g, '');
  var pCap = preferCap.split(',');
  // console.log(preferCap);
  // console.log(pCap);

  var pCapStr = '';
  pCap.forEach(p => {
    p = p.split(':');
    p[0] = p[0].replaceAll(/\"/g, '');
    p[1] = p[1].replace(/true/g, 'True').replace(/false/g, 'False');
    pCapStr += `  ${p[0]}: ${p[1]}\n`;
  })

  var content = `Must:\n` + mCapStr + `Prefer:\n` + pCapStr;

  return content;
}

function unflattenRoleData(_data) {
  console.log(_data);
  var retData = { Must: {}, Prefer: {} };
  _data.forEach((cap) => {
    if (cap.type === 'Must') {
      retData.Must[cap.name] = cap.value;
    }
    else {
      retData.Prefer[cap.name] = cap.value;
    }
  });
  return retData;
}

// ======================
//     Http Requests
// ======================

async function fetchArtifactsServiceTypesCb() {
  var artifactObj = {};
  var data = [];
  // --- Native API ---
  // data = await restArtifactsServiceTypes();
  // data.forEach(d => {
  //   console.log(Object.keys(d.services));
  //   artifactObj[d.type] = Object.keys(d.services);
  // })
  // console.error(artifactObj);
  // service_options = artifactObj;
  // // console.log(service_options);
  // extArtifacts_ = data.filter(d => d.category === "External").map(d => d.type);
  // // console.log(extArtifacts_);

  // --- Swarm Core API ---
  try {
    rmtToken_ = await fetchToken();
  } catch (err) {
    console.error(err);
  }
  data = await fetchGetArtifactTypes(rmtToken_);
  data.forEach(d => {
    // console.log(d.services.map(ds => ds.service));
    artifactObj[d.type] = d.services.map(ds => ds.service);
  })
  service_options = artifactObj;
  // console.log(service_options);
  extArtifacts_ = data.filter(d => d.category === "External").map(d => d.type);
  // console.log(extArtifacts_);

}

function toRolesOverview() {
  resetRenameElement();
  document.getElementById("roles-showcase").style.display = "inline";
  document.getElementById("role-editor").style.display = "none";

  // --- clean the previous combinations ---
  console.log('nestable destory');
  $('#nestable').nestable('destroy');
  $('#nestable').empty();

  // blackBoardCache_ = [];
  blackBoardCache_.length = 0;

  location.reload();
}

function resetRenameElement() {
  var target = $('.edit_role_name_btn .fa');
  var targetNode = document.getElementById('editing-role-name');
  console.log(target)

  if ($(target).hasClass('fa-eye')) {
    $(target).attr("class", "fa fa-pen");
    var modifiedName = $(targetNode).val();

    var inputNode = document.createElement('span');
    inputNode.setAttribute("id", "editing-role-name");
    inputNode.setAttribute("class", 'editing-rolename-span');
    inputNode.setAttribute("style", "font-size:24px");
    inputNode.setAttribute("style", "word-break:break-all");
    inputNode.textContent = modifiedName;
    // $('#hide_origin_role_name').html(modifiedName);

    $(targetNode).replaceWith($(inputNode));

    return;
  }
}

// ======================
//     Logic Blocks
// ======================

var updateOutput = function (e) {
  var list = e.length ? e : $(e.target);
  var output = list.data("output");
  if (output === undefined) return; // protect from error messages
  if (window.JSON) {
    // console.log(output);
    output.val(window.JSON.stringify(list.nestable("serialize")));
    editedRole_ = window.JSON.stringify(list.nestable("serialize"));
  } else {
    output.val("JSON browser support required for this demo.");
    editedRole_ = "none";
  }
  console.log(editedRole_);
};

async function saveRoleEditorCb() {
  var targetNode = document.getElementById('editing-role-name');
  var vidateRoleName = validate($(targetNode).val());

  if (targetNode.type === 'text' && !vidateRoleName[0]) {
    notificationMsg(3, vidateRoleName[2]);
    return;
  }

  // TODO: test
  // --- update the select value ---
  var rolesDeckNode = document.querySelector('#nestable');
  var selects = rolesDeckNode.querySelectorAll('select');
  console.log(selects);
  selects.forEach(function (sel) {
    updateSelectVal(sel);
  })

  // [protection] prevent from repeatedly save teh role combination
  updateOutput($("#nestable").data("output", $("#nestable-output")));

  // 1. --- parse edited result on editor in JSON ---
  var roleTemplate = { "id": 1, "title": "Sequence" };
  console.log(editedRole_);
  editedRole_ = JSON.parse(editedRole_);
  roleTemplate.children = editedRole_;
  editedRole_ = roleTemplate;
  console.log(JSON.stringify(editedRole_));

  // 2. --- convert and re-assemble by format schema ---
  var cvtResult = cvtBtData2File(editedRole_);
  var jsonResult = cvtResult;
  console.log(JSON.stringify(cvtResult));
  // 2.1 --- single docking protection ---
  let behaviorLen = cvtResult.elements?.length;
  // console.log(behaviorLen);
  if (behaviorLen === 1 && cvtResult.elements[0].name === "Docking") {
    alert('Single Docking Behavior is NOT allowed!\nPlease compose the behavior again!');
    return;
  }

  // 3. --- prepend the setblackboard ---
  console.log(blackBoardCache_);
  cvtResult.elements = blackBoardCache_.concat(cvtResult.elements);
  cvtResult = JSON.stringify(cvtResult);
  console.log(cvtResult);
  cvtResult = `{"elements":[{"type":"element","name":"root","attributes":{"main_tree_to_execute":"BehaviorTree"},"elements":[{"type":"element","name":"BehaviorTree","attributes":{"ID":"BehaviorTree"},"elements":[${cvtResult}]}]}]}`;

  // 4. --- update the selected role behavior file ---
  // console.log(cvtResult);
  var roleFilename = selectedRoleDataCache_.name + '.xml';
  restPostBehaviorData(roleFilename, cvtResult);

  // 4.5 --- update ---
  // --- save artifact types into capability file ---
  // -- check if have any types of Artifacts
  // console.log(jsonResult.elements);
  var els = jsonResult.elements;
  if (els.length) {
    // -- map these into an array
    els = els.map(el => el.attributes.type).filter(el => el !== undefined);
    els = [...new Set(els)];
    els = els.filter(artif => !extArtifacts_.includes(artif));
    console.log(els);

    var elStr = "[";
    if (els.length > 0) {
      els.forEach((el) => {
        elStr += `"${el}",`
      })
      elStr = elStr.slice(0, -1);
    }
    elStr += "]";

    // [20220816 FAR-1968]
    // // -- write back to the capability file
    // var capFilename = selectedRoleDataCache_.name + '.yaml';

    // // --- CRITICAL FORMAT! ---
    // var roleTemplate = `Must:\n` +
    //   `  payload: 250\n` +
    //   `  artifacts: ${elStr}\n` +
    //   `  behavior: "${roleFilename}"\n` +
    //   `  perceptions: []\n` +
    //   `  behaviors: []\n` +
    //   `Prefer:\n` +
    //   `  speed: True`;

    // // CRITICAL! Should not decorate it await
    // await restPostRoleData(capFilename, roleTemplate);
  }

  // 5. --- update new role to the fleet ---
  // // --- get the fleet name ---
  // var fleetName = getSelectedFleet();
  // var fleetFilename = fleetName;
  // alert(`Role : ${selectedRoleDataCache_.name} saved!`);

  // // --- fetch fleet_settings content ---
  // var fltSettings = await restGetFleetSettings(fleetFilename);
  // var fleetCache_ = fltSettings;
  // var fltKey = Object.keys(fltSettings)[0]; // suppose only one key-value pair

  // // --- update the role ---
  // // -- if the role is already in fleet config., then should not push again.
  // var isExist = fleetCache_[fltKey].roles.includes(selectedRoleDataCache_.name)
  // if (!isExist) {
  //   fleetCache_[fltKey].roles.push(selectedRoleDataCache_.name);
  // }
  // var result = JSON.stringify(fleetCache_);

  // // --- write back the to server ---
  // await restPostFleetSettings(fleetFilename, result);

  // --- rename role name ---
  let origRoleName = $('#hide_origin_role_name').text();
  let modiRoleName = origRoleName;
  var targetElement = document.getElementById('editing-role-name');

  if (targetElement.type === 'text') {
    modiRoleName = $(targetElement).val();
  } else {
    modiRoleName = $(targetElement).text();
  }

  console.log(origRoleName);
  console.log(modiRoleName);
  let resText = 'Role Saved!';
  let res_style = 0
  if (origRoleName !== modiRoleName) {
    console.log('--- start to rename the role ---');
    displayOverlay(`Renaming role and related data...`);
    let chkFiles = await getRoleInUsedFiles(origRoleName);
    console.log('BBBBBBBBBBBB')
    console.log(chkFiles)
    if (chkFiles.operations.length > 0) {
      var data = await restRoleInUsedOperations(chkFiles.operations, origRoleName);
      console.log(data);
      // rename role name in flows
      // await restRenameRoleInFlows(data, origRoleName, modiRoleName);
      res = await restRenameRoleConfig(data, origRoleName, modiRoleName);
    } else {
      // Role not be used in any fleet
      res = await restRenameRoleConfig(data, origRoleName, modiRoleName);
    }

    console.log('AAAAAAAAAAAAAAA')
    console.log(res)

    res = JSON.parse(res);
    if (res.statusCode === 200) {
      res_style = 1;
    } else {
      res_style = 3;
    }
    resText = res.message;
  }

  removeOverlay();
  notificationMsg(res_style, resText);
  resetRenameElement();
  // toast('Role Saved!');
}

function updateSbCapValue() {
  // var selVal = $('#sbft-cap-name').val();
  var selVal = document.getElementById('sbft-cap-name').value;
  console.log(selVal);
  // var capNode = document.getElementById('sbft-cap-value');
  // $('#sbft-cap-value').parent();
  var capNode = document.getElementById('sbft-cap-value');
  var capParentNode = document.getElementById('sbft-cap-value').parentNode;

  if (selVal === 'perceptions') {
    var elSelect = document.createElement('select');
    elSelect.setAttribute('class', 'form-control cap-value');
    elSelect.setAttribute('id', 'sbft-cap-value');
    [{ name: 'LiDAR', value: 'lidar' }].forEach((a) => {
      var elOption = document.createElement('option');
      elOption.value = a.value;
      elOption.text = a.name;
      elSelect.add(elOption);
    });
    elSelect.value = 'lidar';
    capNode.remove();
    capParentNode.append(elSelect);
    return;
  }
  if (selVal === 'payload') {
    var elInput = document.createElement('input');
    elInput.setAttribute('class', 'form-control cap-value');
    elInput.setAttribute('id', 'sbft-cap-value');
    elInput.setAttribute('placeholder', 'unit: kg');
    capNode.remove();
    capParentNode.append(elInput);
    return;
  }
}

// function checkInvalidParam() {
//   $('#save-settings-form').data('validator').element($(`input[name="${this.name}"]`));
// }

function validateEvent() {
  validateRoleInputEvent()
  // $('.editing-rolename-input').on('keyup', function () {
  //   let param_val = $(this).val();
  //   let vali = validate(param_val);

  //   $(this).css('border-color', vali[1]);
  // });
}

function validate(param_val) {
  let css_border_color = '';
  let valid_msg = '';

  ret = inputCheck(param_val);
  valid_msg = (ret) ? '' : 'Input Role name is invalid.';
  css_border_color = (ret) ? 'black' : 'red';

  return [ret, css_border_color, valid_msg];
}

// --- add input values validator ---
function validateRoleInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules
  //                  * add rules into validatorManager   -
  **/
  const role_validatorManager = new ValidatorMananger();

  const roleNameRule = new Rule('role-filename', textNameValidation, $('#confirm-create-role'));
  const renameroleNameRule = new Rule('editing-role-name', textNameValidation, $('#save-role-btn'));

  role_validatorManager.addValidator(roleNameRule);
  role_validatorManager.addValidator(renameroleNameRule);
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
  const targets = document.getElementsByClassName('role-input');
  for (let el of targets) {
    console.log('AVASV')
    console.log(el)
    el.addEventListener("keyup", validationFlow.bind(el, role_validatorManager));
  }
}

// async function rmtTokenCheck() {
//   if (rmtToken_ === undefined) {
//     try {
//       rmtToken_ = await fetchToken();
//       notificationMsg(1, 'RMT ONLINE!');
//       // promptBox('success', 'Success: RMT ONLINE!');
//     } catch (err) {
//       console.error(err);
//       notificationMsg(3, 'RMT OFFLINE!');
//       // promptBox('error', 'Error: RMT OFFLINE!');
//       console.log(rmtToken_);
//       await sleep(5000);
//       rmtTokenCheck();
//     }
//   }
// }


// ======================
//       Test Data
// ======================
const dock_options = ["docking", "undocking"];
let service_options = {};
let extArtifacts_ = []; // external artifacts

const dictUiTerms = {
  Sequence: "Sequence",
  Patrol: "Patrol",
  Docking: "Docking",
  Rotate: "Rotate",
  // InitialPose: "InitialPose",
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
  // InitialPose: "InitialPose",
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