/*
 * Author: Angela Kao
 * Date: 29 Sep 2021
 * Description: Public file for every pages
 **/

// ------------------------------
//  Data Source Common Utilities
// ------------------------------

const IPAD_PORTRAIT_WIDTH = 767;
const PHONE_MAX_WIDTH = 504;
let AssetsDict = restAsset();
let executingFlow = { "using_map": [], "using_fleets": {} };
let settingsUI = { theme: 'dark', fontSize: 'medium', fontStyle: '', lang: 'en' };

// ------------------------------
//      UI Common Utilities
// ------------------------------
const langRmtState = {
  'en': {
    'online': 'RMT ONLINE',
    'offline': 'RMT OFFLINE',
  },
  'zht': {
    'online': 'RMT 上線',
    'offline': 'RMT 下線',
  },
  'zh': {
    'online': 'RMT 在线',
    'offline': 'RMT 离线',
  },
};
let rmtToken_ = undefined;
async function sbRmtTokenCheck() {
  if (rmtToken_ === undefined || !rmtToken_.hasOwnProperty('access_token')) {
    let lang = getSetLang() || 'en';
    try {
      rmtToken_ = await fetchToken();
      notificationMsg(1, langRmtState[lang].online);
    } catch (err) {
      console.error(err);
      notificationMsg(3, langRmtState[lang].offline);
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

// ------------------------------
//      UI Protection check
// ------------------------------
// setInterval(checkProtection, 100);
async function checkProtection() {
  if (rmtToken_ === undefined || !rmtToken_.hasOwnProperty('access_token')) {
    // try {
    //   rmtToken_ = await fetchToken();
    //   notificationMsg(1, 'RMT ONLINE!');
    // } catch (err) {
    //   console.error(err);
    //   notificationMsg(3, 'RMT OFFLINE!');
    //   console.log(rmtToken_);
    //   await sleep(5000);
    //   rmtTokenCheck();
    // }
  } else {
    var res = await fetchFleetStates(rmtToken_, '', 'all');
    if (!res.ok) { return; }

    var queryData = await res.json();

    if (queryData.fleet_state.length == 0) {
      executingFlow = { "using_map": [], "using_fleets": {} }
    } else {
      executingFlow = { "using_map": [], "using_fleets": {} }
      for (const fleet_data of queryData.fleet_state) {
        let need_to_add = [];
        let fleet_name = fleet_data.fleet_name;
        let fleet_robots = fleet_data.robots.map(function (item, index, array) {
          if (item.task_id == undefined || item.task_id.length == 0) {
            need_to_add.push(false);
          } else {
            if (!executingFlow["using_map"].includes(item.map)) {
              executingFlow["using_map"].push(item.map);
            }
            need_to_add.push(true);
            return item.robot_id
          }
        });

        console.log(fleet_data);
        if (!fleet_data.hasOwnProperty("artifacts")) { continue; }
        let fleet_artifacts = fleet_data.artifacts.map((art) => art.id);

        if (fleet_robots.length != 0) {
          let fleet_data = { 'robots': fleet_robots, 'artifacts': fleet_artifacts };
          console.log(fleet_name, need_to_add)
          if (need_to_add.includes(true)) {
            executingFlow["using_fleets"][fleet_name] = fleet_data;
          }
        }
      }
    }
  }
  console.log(executingFlow)
  return executingFlow;
}

// If name of robot/artifact is none, append '-' + id to it.
function avoidNoneName(titleNode, name, id) {
  let adjust_text_dict = adjustShowRobotName(name, id);
  titleNode.textContent = adjust_text_dict["show_text"];
  titleNode.title = adjust_text_dict["title_text"];
}

function avoidNoneVal(titleNode, name, id) {
  let adjust_text_dict = adjustShowRobotName(name, id);
  titleNode.val(adjust_text_dict["show_text"]);
}

function adjustShowRobotName(name, id) {
  let show_text = "";
  let title_text = "";
  let text_length = 20;
  if (name != undefined && name.length > 0) {
    show_text = name;
  } else {
    show_text = `ID-${id}`;
  }
  title_text = show_text;
  if (show_text.length > text_length) {
    show_text = `${show_text.slice(0, text_length)}...`;
  }
  return { "show_text": show_text, "title_text": title_text }
}

function genSideBarMenu2(role, selPage, selPageLink) {
  createBrandLogo();
  for (const [roleKey, roleVal] of Object.entries(userAuthFunctions_)) {
    // if (roleKey === role) {
    //   // for (const [funcKey, funcVal] of Object.entries(roleVal)) {
    //   //   // console.log(funcVal);
    //   // createSideBarItem(funcKey, funcVal, selPage, selPageLink);
    //   // }
    //   // createSideBarItem(funcKey, funcVal, selPage, selPageLink);
    //   createSidebarFooter();
    //   chkTriggerListVisible();
    // }
    createSidebarFooter();
    chkTriggerListVisible();

  }
  sbRmtTokenCheck();
}

function genSideBarMenu(role, selPage, selPageLink) {
  createBrandLogo();
  for (const [roleKey, roleVal] of Object.entries(userAuthFunctions_)) {
    if (roleKey === role) {
      for (const [funcKey, funcVal] of Object.entries(roleVal)) {
        // console.log(funcVal);
        createSideBarItem(funcKey, funcVal, selPage, selPageLink);
      }
      createSidebarFooter();
      chkTriggerListVisible();
    }
  }
  sbRmtTokenCheck();
}

function createBrandLogo() {
  var logoLink =
    `<a href="index.html" class="brand-link logo-switch">
      <img src="${getImagePath()}/FARobotLogo.png" alt="FARobot Logo" class="brand-image img-circle elevation-3 logo-xs">
      <img src="${getImagePath()}/FARobotLogo_lg.png" alt="FARobot Logo" class="brand-image logo-xl">
   </a>`;
  $('.sidebar').before(logoLink);
}

async function createSidebarFooter() {
  var version = await restGetUIVersion();
  // console.log(version);
  var footerDiv = `<div class="p-3 sidebar-footer"><span>Version</span> ${version}</div>`;
  $('.sidebar').after(footerDiv);
  $('.sidebar-footer > span').attr('data-i18n', "nav.footer.version")
}

function createSideBarItem(key, obj, selPage, selPageLink) {
  let isSelectedPage = key === selPage;
  let isHelpPage = key === "help";
  let isTriggerList = key === 'taskTrigger';
  const bIsWrapperSdk = key === 'wrapperSdk';

  var $li = $('<li>');
  if (obj.isTreeView) {
    $li = createTreeViewTitle(isSelectedPage || isTriggerList);
  } else {
    $li.addClass('nav-item function-link');
  }
  // fleet group function class
  if (typeof obj.lastItemUnderFleetHrchy !== 'undefined') {
    $li.addClass('fleet-group');
  }
  // trigger function add id for recognize
  if (isTriggerList) {
    $li.addClass('function-link');
    $li.attr('id', 'trigger-list');
  }
  $li.append(createSideBarItemLink(obj, isSelectedPage, isHelpPage, bIsWrapperSdk));
  if (obj.isTreeView) {
    $li.append(createTreeViewItem(obj.treeItem, selPageLink));
  }
  $('.nav-sidebar').append($li);

  // fleet group functions
  if (typeof obj.lastItemUnderFleetHrchy !== 'undefined' && obj.lastItemUnderFleetHrchy) {
    var $div = $('<div>').addClass('user-panel').css('width', '100%');
    var fleet_li = `<li class="nav-header" style="width: 100%;">
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
  if (needOpenMenu) {
    $li.addClass('menu-open');
  } else {
    $li.addClass('menu-close');
  }
  return $li;
}

function createSideBarItemLink(obj, isActiveLink = true, isHelpLink = false, isWrapperSdk = false) {
  var $a = $('<a>');
  if (isHelpLink) {
    let path = window.location.protocol + "//" + window.location.hostname + ":5000/docs";
    $a.attr('href', path).addClass('nav-link');
  } else if (isActiveLink) {
    $a.attr('href', obj.dirPath).addClass('nav-link active');
  } else if (isWrapperSdk) {
    $a.attr('href', obj.dirPath).attr('target', "_blank").addClass('nav-link');
  } else {
    $a.attr('href', obj.dirPath).addClass('nav-link');
  }

  var $p = $('<p>').text(obj.functionName);
  $p.attr('data-i18n', `nav.${obj.functionName.replace(' ', '_').toLowerCase()}`);
  // console.log(obj);

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
    var $li = $('<li>').addClass('nav-item function-link');
    var $a = $('<a>').attr('href', itemVal).addClass('nav-link');
    if (itemVal === selPageLink) {
      $a.addClass('active');
    }
    var $i = $('<i>').addClass('far fa-circle nav-icon');
    var $p = $('<p>').text(itemKey);
    $p.attr('data-i18n', `nav.${itemKey.replace(' ', '_').toLowerCase()}`);

    $a.append($i).append($p);
    $li.append($a);
    $ul.append($li);
  }
  return $ul;
}

function applyPieChartFontStyle() {
  let spanEl = document.querySelector(".card-header > h3.card-title > span");
  console.log(spanEl);
  if (spanEl == null) { return; }
  let fontSize = parseFloat(window.getComputedStyle(spanEl).getPropertyValue("font-size"));
  console.log(fontSize);
  pieLegendTextFontSize_ = fontSize;
  pieCenterTextFontSize_ = fontSize * 2;
}

async function setUIlocalStorage() {
  var settingsData = await restGetSettings();
  var settingsUIData = settingsData['settings.UI'];
  if (settingsUIData === undefined) {
    // save default value
    delete settingsUI['fontStyle'];
    await restPostUISettings(settingsUI)
    settingsData = await restGetSettings();
    settingsUIData = settingsData['settings.UI'];
  }
  Object.keys(settingsUIData).forEach(function (key) {
    var value = settingsUIData[key];
    setLocalStorageByKey(key, value);
    // console.log(`UI settings key: ${key}, value: ${value}`);
  });
}

async function applyUISettings() {
  applyFontSpecs();
  await setUIlocalStorage();
  Object.keys(localStorage).forEach(function (key) {
    var value = localStorage.getItem(key);
    switch (key) {
      case 'theme': {
        var isDarkTheme = (value === 'dark');
        toggleDarkTheme(isDarkTheme);
        // --- select UI settings appearance property ---
        $('#theme-color div').filter(`[data-value='${value}']`).css('border', '2px solid gray')
        break;
      }
      case 'fontSize': {
        applyFontSize(value);
        // var per = getFontScalePercent(value);
        // applyFontSizePercentage(per);
        // --- select UI settings font size property ---
        $('input:radio[name="font-size-radio"]').filter(`[value='${value}']`).attr('checked', true);
        // TODO: agent mode adaptor
        // applyPieChartFontStyle();
        break;
      }
      case 'fontStyle': {
        document.body.style.fontFamily = value;
        // --- select UI settings font style property ---
        $('#font-select').val(value);
        break;
      }
      case 'lang': {
        // --- select UI settings lang property ---
        $("#sel-lang-env").val(value);
      }
      default: {
        break;
      }
    }
  });
}

function toggleDarkTheme(on) {
  toggleNavBarDarkTheme(on);
  toggleSidebarDarkTheme(on);
  toggleContentDarkTheme(on);
}

function toggleSidebarDarkTheme(darkOn) {
  if (darkOn) {
    $('.main-sidebar').addClass('sidebar-dark-primary').removeClass('sidebar-light-primary');
  } else {
    $('.main-sidebar').addClass('sidebar-light-primary').removeClass('sidebar-dark-primary');
  }
}

function toggleNavBarDarkTheme(darkOn) {
  if (darkOn) {
    $('.main-header').addClass('navbar-dark').removeClass('navbar-light');
    $('.main-header').addClass('farobot-dark-mode').removeClass('farobot-light-mode');
  } else {
    $('.main-header').addClass('navbar-light').removeClass('navbar-dark');
    $('.main-header').addClass('farobot-light-mode').removeClass('farobot-dark-mode');
  }
}

function toggleContentDarkTheme(darkOn) {
  if (darkOn) {
    $("body").addClass('farobot-dark-mode').removeClass('farobot-light-mode');
    $('.content-wrapper').addClass('farobot-dark-mode').removeClass('farobot-light-mode');
    if (isMobile()) {
      $('.farobot-md-sidemenu').addClass('control-sidebar-dark').removeClass('control-sidebar-light');
      $('.farobot-lg-sidemenu').addClass('control-sidebar-dark').removeClass('control-sidebar-light');
    } else {
      $('.farobot-sidemenu').addClass('control-sidebar-dark').removeClass('control-sidebar-light');
    }
    $('#confirm-modal').addClass('farobot-dark-mode').removeClass('farobot-light-mode'); // role page only
    $('#edit-tools-card').addClass('bg-gradient-dark').removeClass('bg-gradient-secondary');

    $(document).find('.card').each(function (i, el) {
      // console.log(el)
      if (el.id === 'edit-tools-card') return;
      $(el).addClass('card-dark').removeClass('card-light');
    });

    $(document).find('#operation-deck').each(function (i, el) {
      // console.log(el)
      $(el).addClass('farobot-card-deck-darkshadow').removeClass('farobot-card-deck-lightshadow');
    });
  } else {
    $("body").addClass('farobot-light-mode').removeClass('farobot-dark-mode');
    $('.content-wrapper').addClass('farobot-light-mode').removeClass('farobot-dark-mode');
    if (isMobile()) {
      $('.farobot-md-sidemenu').addClass('control-sidebar-light').removeClass('control-sidebar-dark');
      $('.farobot-lg-sidemenu').addClass('control-sidebar-light').removeClass('control-sidebar-dark');
    } else {
      $('.farobot-sidemenu').addClass('control-sidebar-light').removeClass('control-sidebar-dark');
    }
    $('#confirm-modal').addClass('farobot-light-mode').removeClass('farobot-dark-mode'); // role page only
    $('#edit-tools-card').addClass('bg-gradient-secondary').removeClass('bg-gradient-dark');

    $(document).find('.card').each(function (i, el) {
      // console.log(el)
      if (el.id === 'edit-tools-card') return;
      $(el).addClass('card-light').removeClass('card-dark');
    });

    $(document).find('#operation-deck').each(function (i, el) {
      // console.log(el)
      $(el).addClass('farobot-card-deck-lightshadow').removeClass('farobot-card-deck-darkshadow');
    });
  }
}

function getFontScalePercent(sizeOpt) {
  switch (sizeOpt) {
    case 'small':
      return '90%';
    case 'medium':
      return '100%';
    case 'large':
      return '110%';
    default:
      return '100%';
  }
}

function getFontScaleSize(_basedFontSize) {
  let per = getFontScalePercent(getSavedFontSize());
  let scaleFontSize = (_basedFontSize * (parseFloat(per) / 100)).toFixed(1);
  return scaleFontSize;
}

// function applyFontSizePercentage(percent) {
//   $('*').each(function () {
//     $(this).css('font-size', percent);
//   });
// }

function getImagePath() {
  return isMobile() ? "public/dist/img" : "dist/img";
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
      let flow_type = jsonData.Event.event === undefined ? jsonData.Event.type : jsonData.Event.event;
      if (flow_type.includes("Manual")) {
        createTaskTriggerItem(selFleet, jsonData.flow_name, 2);
      }
    });
  } catch (err) {
    console.error("read flow error: ");
    console.error(err);
  }

  chkTriggerListVisible();
  applyFontSize(getSavedFontSize(), '#trigger-list');
}

function createTaskTriggerItem(fleet, taskName, mode) {
  var event = "";
  if (mode == 1) {
    event = "t";
  } else if (mode == 2) {
    event = "f";
  }

  let viewPath = isMobile() ? 'public/views' : 'views';
  $("#trigger-list .nav-treeview").append(`<li class='nav-item trigger-item' id='lsb-${taskName}'>
                                             <a href='${viewPath}/manual_trigger_modal.html' class='nav-link' id='${fleet};${taskName}_link_${event}'>
                                               <p>${taskName}</p>
                                             </a>
                                           </li>`);
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
const langConfirmTaskDispatch = {
  'en': {
    'body': "Are you sure to trigger",
    'cancel': "Cancel",
    'confirm': "Confirm"
  },
  'zht': {
    'body': "確認觸發",
    'cancel': "取消",
    'confirm': "確認"
  },
  'zh': {
    'body': "确认触发",
    'cancel': "取消",
    'confirm': "确认"
  }
};

$(document).on('click', '.trigger-item a', function (e) {
  var task = this.id.split('_link_');
  var fleet = task[0].split(';')[0];
  var taskName = task[0].split(';')[1];
  trigger_event = task[1];
  trigger_name = taskName;

  let lang = getSetLang();
  e.preventDefault();
  $("#manual-trigger-confirm-modal").load($(this).attr('href'), function () {
    // priority text
    let p_text = $("#task_priority_select option:selected").text()

    // $('#manual-trigger-confirm-modal .modal-body').html(`Are you sure to trigger <a title="click to edit" href='operation.html?flow_name=${taskName}'>${taskName}</a>?`);
    $('#manual-trigger-confirm-modal .modal-body').html(`${langConfirmTaskDispatch[lang].body} <a title="click to edit" href='operation.html?flow_name=${taskName}'>${taskName}</a> with priority: <span id="task_priority_span">${p_text}</span> ?`);
    $('#manual-trigger-confirm-modal .modal-footer [data-i18n="general.btn_Cancel"]').text(langConfirmTaskDispatch[lang].cancel);
    $('#manual-trigger-confirm-modal .modal-footer [data-i18n="general.btn_Confirm"]').text(langConfirmTaskDispatch[lang].confirm);
  });

  $("#add-manual").attr('data-i18n', 'btn_Confirm');
  $("#manual-trigger-confirm-modal").modal('show');
});

$(document).on('click', '#add-manual', function (e) {
  if (trigger_event == "t") {
    // send_ros_taskReq(getSavedFleet(), trigger_name);
    send_ros_taskReq(getSelectedFleet(), trigger_name);
  } else if (trigger_event == "f") {
    // send_ros_flowReq(trigger_name);
    sendFlowTask(trigger_name, false);
  }
});

$(window).on('load', function () {
  if (!isMobile()) return;
  let pathArray = window.location.pathname.split('/');
  let firstPath = pathArray[1];
  if (firstPath === 'public') {
    if (pathArray.length > 2) {
      let secondPath = pathArray[2];
      // redirect to mobile page
      window.location.href = window.location.protocol + "//" + window.location.hostname + ":3000/" + secondPath;
    }
  }
});

$(document).on('change', "#task_priority_select", function (e) {
  $('#task_priority_span').text($("#task_priority_select option:selected").text())
});

function bindTSTooltipEvts() {
  if (!isMobile()) return;
  $(document).on('click', '.custom-tooltip', function () {
    var checkAttr = $(this).attr('data-toggle');
    if (checkAttr == "tooltip") {
      $('.tooltip').not(this).tooltip('hide');
      $(this).tooltip('show');
    }
  });

  $(window).scroll(function () {
    $('.tooltip').tooltip('hide');
  });

  var dashboard_flow_list = $('.flow-list-panel');
  if (dashboard_flow_list.length) {
    dashboard_flow_list.scroll(function () {
      $('.tooltip').tooltip('hide');
    });
  }
}

// ------------------------------
//         Fleet events
// ------------------------------

var _switchFleetCallback = function () { };

function setSwitchFleetCallback(_callback) {
  _switchFleetCallback = _callback;
}

function toggleSideBarMenu() {
  const btn = document.querySelector('a[data-widget=pushmenu]');
  btn.addEventListener('click', function () {
    const isMenuFold = getIsMenuFold() === 'true';
    sessionStorage.setItem("isMenuFold", !isMenuFold);
  });

  const isMenuFold = getIsMenuFold() === 'true';
  if (isMenuFold) {
    if (document.body.classList.contains('sidebar-collapse')) { return; }
    document.body.classList.add("sidebar-collapse");
  } else {
    if (!document.body.classList.contains('sidebar-collapse')) { return; }
    document.body.classList.remove("sidebar-collapse");
  }
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
  // setLocalStorageByKey("fleet", getSavedFleet(), firstFleet);
  // var savedFleet = getSavedFleet();
  setSessionStorageByKey("fleet", getSelectedFleet(), firstFleet);
  var savedFleet = getSelectedFleet();
  genTaskTrigger(savedFleet);
  $('#fleet-select').val(savedFleet);
  console.log("========fleet: " + savedFleet + "========");
}

function genFleetSelectOptionsView(_fleets) {
  $('#fleet-select').find('option').remove();
  if (!_fleets.length) {
    $('#fleet-select').prop('disabled', true).append('<option disabled selected hidden>No Fleets</option>');
    removeSelectedFleet();
    return;
  }

  for (var i = 0; i < _fleets.length; i++) {
    _fleets[i] = _fleets[i].split('.').slice(0, -1).join('.');
    $('#fleet-select').append(`<option value='${_fleets[i]}'>${_fleets[i]}</option>`);
  }

  // remove saved fleet if config file is removed
  // var savedFleet = getSavedFleet();
  var savedFleet = getSelectedFleet();
  if ($.inArray(savedFleet, _fleets) == -1) {
    removeSelectedFleet();
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
  // setLocalStorageByKey("fleet", opt);
  setSessionStorageByKey("fleet", opt);
  genTaskTrigger(opt);

  if ($("#flow_edit_dashboard").is(':visible')) {
    window.location.href = "operation.html";
  }
}

function resetSavedFleet(_behaviour, _fleetName) {
  // var savedFleet = getSavedFleet();
  // var savedFleet = getSelectedFleet();
  let savedFleet = $('#fleet-select').val();
  console.log('!!! reset saved fleet !!!')
  console.log(savedFleet);
  console.log(_fleetName);
  switch (_behaviour) {
    case 'delete':
      if (savedFleet === _fleetName) {
        var firstFleet = $('#fleet-select').prop("selectedIndex", 0).val();
        // setLocalStorageByKey("fleet", firstFleet !== null ? firstFleet : "");
        setSessionStorageByKey("fleet", firstFleet !== null ? firstFleet : "");
        // remove fleet option & refresh task trigger list
        $('#fleet-select').find('option[value=' + _fleetName + ']').remove();
        genTaskTrigger(firstFleet);
      }
      break
    default:
      break
  }
}

function setLocalStorageByKey(keyName, keyValue, keyDefaultValue = '') {
  if (isEmptyString(keyDefaultValue)) {
    keyDefaultValue = keyValue;
  }
  if (localStorage.hasOwnProperty(keyName)) {
    localStorage.setItem(keyName, keyValue);
  } else {
    localStorage.setItem(keyName, keyDefaultValue);
  }
}

function getSavedTheme() {
  return localStorage.hasOwnProperty('theme') ? localStorage.getItem("theme") : "";
}

function getSavedFontSize() {
  return localStorage.hasOwnProperty('fontSize') ? localStorage.getItem("fontSize") : "";
}

function getSavedFleet() {
  return localStorage.hasOwnProperty('fleet') ? localStorage.getItem("fleet") : "";
}

// function removeSavedFleet() {
//   localStorage.removeItem('fleet');
// }

function getSavedCellRelation() {
  return localStorage.hasOwnProperty('connCells') ? localStorage.getItem("connCells") : "";
}
function getSetLang() {
  return localStorage.hasOwnProperty('lang') ? localStorage.getItem("lang") : "";
}

function setSessionStorageByKey(keyName, keyValue, keyDefaultValue = '') {
  if (sessionStorage.hasOwnProperty(keyName)) {
    sessionStorage.setItem(keyName, keyValue);
  } else {
    sessionStorage.setItem(keyName, keyDefaultValue);
  }
}

function getSelectedFleet() {
  return sessionStorage.hasOwnProperty('fleet') ? sessionStorage.getItem("fleet") : "";
}

function getIsMenuFold() {
  return sessionStorage.hasOwnProperty('isMenuFold') ? sessionStorage.getItem("isMenuFold") : false;
}

function removeSelectedFleet() {
  sessionStorage.removeItem('fleet');
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
      // for (const [role_key, role_value] of Object.entries(role_item)) {
      // console.log(role_key);
      // console.log(role_value);
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

  wsPubTaskReq(send_msgs);
}

// async function send_ros_taskReq0(fleet, taskName, isTimer = false) {
//   var new_task = new ROSLIB.Message({
//     task_id: '',
//     task_name: '',
//     task_type: '',
//     task_params: '',
//     priority: 2,
//     robot_id: '',
//     fleet_name: '',
//     timer_scheduled: isTimer,
//     task_time: {
//       sec: 0,
//       nanosec: 0
//     }
//   });
//   var send_msgs = new ROSLIB.Message({
//     tasks: null
//   });

//   var data = await restGetTaskData(fleet, taskName);
//   var task_param = [];

//   for (const role_item of data.task_info.role.role_params) {
//     for (const [role_key, role_value] of Object.entries(role_item.content)) {
//       if ((role_key.includes("goal_") && !role_key.includes("goal_label_")) || role_key.includes("cell_id_")) {
//         var valArray = role_value.split(";");
//         if (valArray.length >= 3) {
//           var map = valArray[0];
//           var area = valArray[1];
//           var cell = valArray[2];

//           var mapCells = await restGetMapCells(map + ".json");
//           var jsonObj = JSON.parse(mapCells);
//           var cellArray = jsonObj[`${area}`];
//           var cellObj = Object.values(cellArray).filter(function (elements) {
//             return elements.cell_id === cell;
//           });
//           if (cellObj.length > 0) {
//             var cellCoord = cellObj[0].cell_coordinate.toString().replaceAll(',', ';');
//             task_param.push(`${role_key}:${cellCoord}`);
//           }
//         }
//       } else if (role_key.includes("mode_")) {
//         task_param.push(`${role_key}:coord`);
//       } else {
//         task_param.push(`${role_key}:${role_value}`);
//       }
//     }
//   }

//   var robot_id = ""
//   if (data.task_info.assigned_robot == "auto") {

//   } else {
//     robot_id = data.task_info.assigned_robot;
//   }

//   // timer trigger
//   if (typeof data.task_info.task_time_info !== 'undefined') {
//     var timeInfo = data.task_info.task_time_info.split(";");
//     if (timeInfo.length == 1 && data.task_info.task_time_info !== "") {
//       var startDateSec = new Date(timeInfo[0]).getTime() / 1000;
//       new_task.task_time = {
//         sec: startDateSec,
//         nanosec: 0
//       };
//     } else if (timeInfo.length == 3) {
//       var startTime = new Date(timeInfo[0]).getTime() / 1000;
//       var duration = getSeconds(timeInfo[1]);
//       var endTime = new Date(timeInfo[2]).getTime() / 1000;
//       var times = Math.floor((endTime - startTime) / duration);
//       startTime += duration * timerTriggerTimes_;
//       timerTriggerTimes_ += 1;
//       new_task.task_time = {
//         sec: startTime,
//         nanosec: 0
//       };
//       if (timerTriggerTimes_ <= times) {
//         send_ros_taskReq(fleet, taskName, true);
//       }
//     }
//   }

//   new_task.task_id = genUuid();
//   new_task.task_type = data.task_info.role.role_value;
//   new_task.task_name = data.task;
//   new_task.robot_id = robot_id;
//   new_task.task_params = task_param.join(',');
//   new_task.fleet_name = fleet;

//   // Fill send_msgs
//   send_msgs.tasks = [new_task];
//   console.log("============ send task =================")
//   console.log(new_task)
//   // task_request_pub.publish(send_msgs);
//   wsPubTaskReq(send_msgs);
// }

function resetTimerTriggerTimes() {
  timerTriggerTimes_ = 0;
}

async function sendFlowTask(_flowName, isTimer, timer_arg = undefined) {
  var options = {
    args: {
      "start_time": "",
      "end_time": "",
      "interval": "",
      "priority": "",
      "params": {}
    }
  };

  if (isTimer) {
    options['args']['start_time'] = timer_arg['start_time'];
    options['args']['end_time'] = timer_arg['end_time'];
    options['args']['interval'] = timer_arg['interval'];
  }

  let task_priority_val = $("#task_priority_select").val();
  options['args']['priority'] = (task_priority_val !== undefined && task_priority_val !== '0') ? task_priority_val : "";
  // TODO: refactor the flow id generation
  console.log(rmtToken_);
  console.log(' ********* send flow ********');
  console.log(options);
  const res = await fetchPostFlow(rmtToken_, _flowName, options);
  const data = await res.json();
  // --- [Connection] health check ---
  if (!res.ok) {
    notificationMsg(3, `[CONN.] ${data}`);
    return;
  }
  // --- [Swarm System] health check ---
  const sysStateStyle = (data.system_status_code === 200) ? 1 : 3;
  notificationMsg(sysStateStyle, `[SWARM] Send Flow ${data.system_message}`);
  return data.swarm_data["flow_id"]
}

// function send_ros_flowReq(flowName) {
//   // var new_flow_msg = new ROSLIB.Message({
//   //   event: '',
//   //   event_type: '',
//   //   params: 'ui',
//   //   timer_scheduled: false,
//   //   plan_time: {
//   //     sec: 0,
//   //     nanosec: 0
//   //   }
//   // });

//   // new_flow_msg.event = genUuid();
//   // new_flow_msg.event_type = flowName;
//   // console.log("============ send flow =================")
//   // console.log(new_flow_msg);
//   // // flow_request_pub.publish(new_flow_msg);

//   var new_flow_msg = {
//     event: genUuid(),
//     event_type: flowName,
//     params: 'ui',
//     timer_scheduled: false,
//     plan_time: {
//       sec: 0,
//       nanosec: 0
//     }
//   };
//   console.log("============ send flow =================")
//   console.log(new_flow_msg)
//   wsPubFlowReq(new_flow_msg);
// }


// ------------------------------
//      Login Status
// ------------------------------
function setCookie(cname, cvalue, exdays) {
  const d = new Date();
  d.setTime(d.getTime() + (exdays * 24 * 60 * 60 * 1000));
  let expires = "expires=" + d.toUTCString();
  document.cookie = cname + "=" + cvalue + ";" + expires + ";path=/";
}

function getCookie(cname) {
  let name = cname + "=";
  let decodedCookie = decodeURIComponent(document.cookie);
  let ca = decodedCookie.split(';');
  for (let i = 0; i < ca.length; i++) {
    let c = ca[i];
    while (c.charAt(0) == ' ') {
      c = c.substring(1);
    }
    if (c.indexOf(name) == 0) {
      return c.substring(name.length, c.length);
    }
  }
  return "";
}

function setLoginStatus(islogin) {
  console.log('ASDASDASDASD')
  console.log(islogin)

  setCookie('currenlogin', islogin, 1)
}

function loginState() {
  let currentstatusData = getCookie('currenlogin');
  let currentPath = window.location.pathname.replace('/', '');

  if (currentstatusData.length == 0) {
    return;
  } else {
    if (currentPath === "login.html") {
      // window.location.href = "login.html";
      return;
    } else {
      // let prevloginStatus = JSON.parse(prevstatusData);
      let currentloginStatus = JSON.parse(currentstatusData);
      if (currentloginStatus.loginStatus == false && currentloginStatus.userID == 'startLogin') {
        window.location.href = "login.html";
      } else if (currentloginStatus.loginStatus == false && currentloginStatus.userID == 'logout') {
        window.location.href = "login.html";
      }
    }
  }
}

setInterval(() => {
  loginState();
}, 500);


async function getLoginStatus(status, selPage, selPageLink = "") {
  console.log(status);
  console.log(selPageLink);
  if (status === "logout" && selPageLink === "settings_lic.html") {
    genSideBarMenu2(status.role, selPage, selPageLink);
    toggleSideBarMenu();
    applyUISettings();
    return;
  }

  if (status === "logout") {
    // alert("Please login first!");
    window.location.href = "login.html";
    return;
  } else {
    console.log(window.history);
    // TODO: need to block all pages except license page if no valid license
    let authPages_ = userAuthFunctions_[status.role];
    authPages_ = Object.keys(authPages_);
    let isAuthPage = authPages_.includes(selPage);
    if (!isAuthPage) {
      // alert("You can't access this page!");
      if (window.history.length < 3) {
        window.location.href = "index.html";
      } else {
        window.history.back();
      }
      return;
    }
    $('#user-login-status').html(status.greetingMsg);
    genSideBarMenu(status.role, selPage, selPageLink);
    toggleSideBarMenu();
    await genFleets();
    applyUISettings();
  }
}

async function initLanguageSupport(lng = 'null') {
  console.log(lng);
  if (lng === 'null') {
    lng = getSetLang() || 'en';
  }
  let langObj = await restGetLangObj(lng);

  i18next.init(langObj, function (err, t) {
    jqueryI18next.init(i18next, $);
    $('[data-i18n]').localize();
  });

  const convert = {
    'en': 'US',
    'zht': 'TW',
    'zh': 'CN',
  };
  $('.country_flags').attr('country', convert[lng]);
}

function adjustShowtext(text) {
  let show_text = text;
  let text_length = 20;

  if (show_text.length > text_length) {
    show_text = `${show_text.slice(0, text_length)}...`;
  }
  return show_text
}

function userActivityDetector() {
  var secsSinceLastActivity = 0;
  var maxInactivitySec = 30 * 60;

  // setInterval(function () {
  //   secsSinceLastActivity++;
  //   console.log(secsSinceLastActivity + ' seconds since the user was last active');
  //   if (secsSinceLastActivity >= maxInactivitySec) {
  //     console.log('User has been inactive for more than ' + maxInactivitySec + ' seconds');
  //     // alert("Time out. Please login again!");
  //     // log out
  //     restLogout();
  //   }
  // }, 1000);
  setTimeout(function durationCheck() {
    secsSinceLastActivity++;
    // console.log(secsSinceLastActivity + ' seconds since the user was last active ---');
    if (secsSinceLastActivity >= maxInactivitySec) {
      console.log('User has been inactive for more than ' + maxInactivitySec + ' seconds');
      // alert("Time out. Please login again!");
      // log out
      restLogout();
    }
    setTimeout(durationCheck, 1000);
  }, 1000);

  function activity(evt) {
    // console.log(evt.type + ' reset trigger! ---');
    secsSinceLastActivity = 0;
  }

  var activityEvents = [
    'mousedown', 'mousemove', 'keydown',
    'scroll', 'touchstart'
  ];

  activityEvents.forEach(function (eventName) {
    document.addEventListener(eventName, activity.bind(eventName), true);
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

function notificationMsg(typeNum, msg) {
  const popup = Notification({
    position: 'bottom-right',
    duration: 5000
  });

  let lang = getSetLang() || 'en';
  console.log(lang);
  var type2 = notificationHeader[lang][typeNum];
  var type = notificationType[typeNum];
  if (!popup[type]) return;

  popup[type]({
    // title: type.charAt(0).toUpperCase() + type.slice(1),
    title: type2,
    message: msg,
    callback: null
  });
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

function displayOverlay(text, background_color = "rgba(0,0,0,.5)") {
  $("<table id='overlay'><tbody><tr><td>" + text + "</td></tr></tbody></table>").css({
    "position": "fixed",
    "top": "0px",
    "left": "0px",
    "width": "100%",
    "height": "100%",
    "background-color": background_color,
    "z-index": "10000",
    "vertical-align": "middle",
    "text-align": "center",
    "color": "#fff",
    "font-size": "40px",
    "font-weight": "bold",
    "cursor": "wait"
  }).appendTo("body");
}

function removeOverlay() {
  $("#overlay").remove();
}

function isNumeric(str) {
  if (typeof str != "string") return false // we only process strings!
  return !isNaN(str) && // use type coercion to parse the _entirety_ of the string (`parseFloat` alone does not do this)...
    !isNaN(parseFloat(str)) // ...and ensure strings of whitespace fail
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

const notificationType = {
  0: 'info',
  1: 'success',
  2: 'warning',
  3: 'error'
};

const notificationHeader = {
  'en': {
    0: 'Info',
    1: 'Success',
    2: 'Warning',
    3: 'Error'
  },
  'zht': {
    0: '訊息',
    1: '成功',
    2: '警告',
    3: '錯誤'
  },
  'zh': {
    0: '资讯',
    1: '成功',
    2: '警告',
    3: '错误'
  }
};

// const avatarMap_ = {
//   bbr: `${getImagePath()}/sprites/bbr-75x75.png`,
//   br: `${getImagePath()}/sprites/smr1000_300mm-75x75.png`,
//   hik600: `${getImagePath()}/sprites/hik-75x75.png`,
//   smr250: `${getImagePath()}/sprites/smr250-75x75.png`,
//   smr250black: `${getImagePath()}/sprites/smr250black-75x75.png`,
//   smr1000: `${getImagePath()}/sprites/smr1000-75x75.png`,
//   mir250: `${getImagePath()}/sprites/mir-75x75.png`,
//   none: `${getImagePath()}/sprites/no_image-75x75.png`,
// }

const eulaHtmlContent = `<h4><b>Licensing Agreement for Mobile Robot Management Software</b></h4><br>
<p>This License Agreement is a binding agreement between FARobot Inc. and its affiliates (together, “FARobot”) and you (“Licensee”) governing your use of Mobile Robot Management Software (“FARobot Software”).</p>
<p>FAROBOT IS WILLING TO LICENSE FAROBOT SOFTWARE TO YOU ONLY UPON THE CONDITION THAT YOU ACCEPT ALL OF THE TERMS AND CONDITIONS CONTAINED IN THIS LICENSE AGREEMENT. PLEASE READ THIS AGREEMENT CAREFULLY AND CONSULT WITH FAROBOT TECHNICAL PARTNERS OR ITS AUTHORIZED TECHNICAL PARTNERS BEFORE USING FAROBOT SOFTWARE, AS DOING SO WILL INDICATE YOUR ASSENT TO THE TERMS AND CONDITIONS. IF YOU DO NOT AGREE TO THESE TERMS AND CONDITIONS, OR IF YOU DO NOT HAVE AUTHORITY TO ENTER INTO THIS AGREEMENT, THEN FAROBOT WILL NOT AND DOES NOT LICENSE THE SOFTWARE TO YOU, IN WHICH EVENT YOU MUST REFRAIN FROM USING FAROBOT SOFTWARE.</p>
<ol>
  <li>Terminology
    <ol>
      <li>“Certified Partners” means business partners contracted with FARobot.</li>
      <li>“End User” means all users other than Certified Partners.</li>
      <li>“FARobot Software” means Mobile Robot Management Software.</li>
      <li>“Hardware” means the device associated with FARobot Software, which shall be prepared by User.</li>
      <li>“License” means one unit of usage rights granted to you and which can only be associated to one and only one device.</li>
      <li>“License Activation Date” means the date on which a License commences, and thereby begins the licensing term.</li>
      <li>“License Key” means a unique license file that a subscriber receives after the order is placed.</li>
      <li>“License Period” means the valid license term starting from License Activation Date.</li>
      <li>“User” means Certified Partners and End Users.</li>
    </ol>
  </li>
  <li>Scope of License and Support
    <ol>
      <li>Grant of License<br>
Subject to the terms and conditions of this Agreement, FARobot hereby grants you a non-exclusive, non-transferable License to use FARobot Software in accordance with the terms and conditions herein. You are obligated to prepare your own Hardware device and each Device License may only be associated with one single Hardware device (i.e., a single unit). You shall not exceed the scope of the license granted hereunder. Any rights not expressly granted by FARobot to you are reserved by FARobot, and all implied licenses are disclaimed. The User is granted a License commence on License Activation Date and the License Term continues until the expiry date specified on the purchase order based on applicable subscription fees paid. The condition of the software update service will also be provided on a need basis and will be charged at then-current price specified on the purchase order.</il>
      <li>Scope of License for Users<br>
The scope of license varies by the user types.
        <ol>
          <li>Certified Partner<br>
The Certified Partner shall enter into an independent contract, in which the scope of license for Certified Partner will be specified.</li>
          <li>End User<br>
The End User is granted the rights to use all the function in the user interface. The End User is NOT granted the rights to sell and rights to reproduce.</li>
        </ol>
      </li>
      <li>Subscription Renewal<br>
The FARobot Software is an on-premise license, your subscription to FARobot Software will NOT be automatically renew. You are obligated to renew the subscription term to avoid any possible systematic problem due to the subscription termination.</li>
      <li>Restrictions<br>
You may not copy, publish, display, disclose, sell, rent, lease, modify, store, loan, distribute, or create derivative works or imitations of FARobot Software, or any part thereof. You may not assign, sublicense, convey or otherwise transfer, pledge as security or otherwise encumber the rights and licenses granted hereunder. You may not copy, reverse engineer, decompile, reverse compile, translate, adapt, or disassemble FARobot Software, or any part thereof, nor shall you attempt to obtain the source code for FARobot Software. Except as and only to the extent expressly permitted in this License or in a separate written agreement between you and FARobot, you may not market, co-brand, or private label FARobot Software or any part thereof. You may not use FARobot Software, or any part thereof, in the operation of a service bureau or for the benefit of any other person or entity. You may not cause, assist or permit any third party to do any of the foregoing. Portions of FARobot Software may utilize or include third party software and other copyrighted material (“Third Party Material”), and your use of FARobot Software is also subject to any terms and conditions for such Third-Party Material. FARobot has no express or implied obligation to provide any technical or other support for Third Party Material other than compliance with the applicable license terms, and makes no warranty (express, implied or statutory) whatsoever with respect thereto.</li>
    </ol>
  </li>
  <li>Payment</li>
The licensing price is specified in the quotation. The license of FARobot Software will be granted only after a purchase order is accepted and the payment is received; or else, FARobt reserves the rights to withdraw the license.
  <li>Obligations
    <ol>
      <li>Use with Third Party Products<br>
If you use FARobot Software together with third-party products or services, such use is at your risk. You are responsible for complying with any terms and conditions required by the third-party. FARobot does not provide support or guarantee ongoing integration support for products or services that are not a native part of FARobot Software.</li>
      <li>No Misuse<br>
You shall not (a) interfere with other customers’ access to or use of FARobot Software, or with its security, (b) facilitate attacks on or disruption of FARobot Software, including denial of service attacks, unauthorized access, penetration testing, crawling, or distribution of malware, (c) cause an unusual spike or increase in your use of FARobot Software that negatively impacts its operation, or (d) use FARobot Software for any purpose other than its contemplated purpose.</li>
      <li>Account Access<br>
You shall keep all account information up to date, use reasonable means to protect your account information, passwords and other login credentials, and promptly notify FARobot of any known or suspected unauthorized use of or access to your account.</li>
      <li>Compliance<br>
If you fail to comply with applicable law or this Agreement, including its restrictions on acceptable use of FARobot Software, FARobot shall reserve the right to immediately remove your devices from FARobot Software or immediately terminate your access to FARobot Software.</li>
    </ol>
  </li>
  <li>Intellectual Property Rights
    <ol>
      <li>Ownership<br>
As between you and FARobot, FARobot Software is solely owned by FARobot, and it is licensed (not sold) to you. You shall not have any right, title, or interest to FARobot Software except as provided in this Agreement, and title to FARobot Software and all copies of it remains with FARobot.</li>
      <li>Intellectual Property Rights<br>
FARobot Software is protected under the intellectual property laws of countries around the world. You agree to respect and not to remove, obliterate, or cancel from view any copyright, trademark, confidentiality or other proprietary notice, mark, or legend appearing on any aspect of FARobot Software or output generated by FARobot Software, and to reproduce and include same on each copy of such. Except as otherwise expressly provided herein, FARobot grants no express or implied right under any patents, copyrights, trademarks, or other intellectual property rights. Any feedback you provide to FARobot in connection with your use of FARobot Software shall become the exclusive property of FARobot and may be used by FARobot in the course of its business.</li>
      <li>No reverse-engineering<br>
As you use FARobot Software, you warrant that you would not conduct reverse-engineering of any kind. You shall be obligated to bear the liquidated damage if such conduct occurred and shall compensate FARobot in full for the direct and indirect damages due to the reverse-engineering.</li>
    </ol>
  </li>
  <li>Confidential Information; Ownership and Use of Data
    <ol>
      <li>Confidentiality<br>
You acknowledge that FARobot Software contains proprietary trade secrets of FARobot and you hereby agree to maintain the confidentiality of FARobot Software using at least as great a degree of care as you use to maintain the confidentiality of your own most confidential information.</li>
      <li>Collection and Ownership of Data
        <ol>
         <li>By using FARobot Software to manage your devices, you understand and agree that you are (1) collecting data regarding the devices that connect to your network and how your network is being used, including the types of data described below, including data that may contain personally identifiable information of your network users (collectively, “Customer Data”) and (2) transferring that data to FARobot Software for processing and storage.</li>
         <li>The FARobot Software includes functionality that allows you to limit or restrict the types of information collected, and you are solely responsible for ensuring that you have the authority to collect any data that you collect. You retain all right, title, and interest in and to your Customer Data.</li>
        </ol>
      <li>Traffic Information Data Collected<br>
“Traffic Information” includes information about devices that connect to your network, such as MAC address, device name, device type, operating system, geolocation information, and information transmitted by devices when attempting to access or download data or content (e.g., host names, protocols, port numbers, and IP addresses). FARobot processes and stores Traffic Information on your behalf so you can monitor the use and performance of your network and exercise control over the traffic on your network.</li>
      <li>FARobot’s Use of Your Data<br>
FARobot shall use Customer Data only as required to provide FARobot Software and, only to the extent necessary, to protect its rights in any dispute with you or as required by law. FARobot will access, process and use data in connection with your use of FARobot Software in accordance with applicable privacy and data protection laws. For further detail, please visit https://www.farobottech.com/ Subject to applicable laws requiring otherwise, FARobot shall be under no obligation to provide you with any of your data that may be stored in FARobot Software after the termination of your subscription.</li>
    </ol>
  </li>
  <li>Warranty and Liability
    <ol>
      <li>Limitation of Liability<br>
IN NO EVENT WILL FAROBOT BE LIABLE TO YOU OR ANY THIRD PARTY FOR ANY INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, WITHOUT LIMITATION, INDIRECT, SPECIAL, PUNITIVE, OR EXEMPLARY DAMAGES FOR LOSS OF BUSINESS, LOSS OF PROFITS, BUSINESS INTERRUPTION, OR LOSS OF BUSINESS INFORMATION) ARISING OUT OF THE USE OF OR INABILITY TO USE FAROBOT SOFTWARE, OR FOR ANY CLAIM BY ANY OTHER PARTY, EVEN IF FAROBOT HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. FAROBOT’S TOTAL AGGREGATE LIABILITY WITH RESPECT TO ITS OBLIGATIONS UNDER THIS AGREEMENT OR OTHERWISE WITH RESPECT TO FAROBOT SOFTWARE SHALL BE EQUAL TO THE SUBSCRIPTION FEES YOU HAVE PAID TO FAROBOT FOR FAROBOT SOFTWARE.</li>
      <li>No Warranty<br>
FAROBOT SOFTWARE IS PROVIDED “AS IS.” TO THE MAXIMUM EXTENT PERMITTED BY LAW, FAROBOT DISCLAIMS ALL WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, WITHOUT LIMITATION, IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. FAROBOT DOES NOT WARRANT THAT FAROBOT SOFTWARE WILL MEET ANY REQUIREMENTS OR NEEDS YOU MAY HAVE, OR THAT FAROBOT SOFTWARE WILL OPERATE WITHOUT ERROR, OPERATE IN AN UNINTERRUPTED FASHION, OR BE ACCESSIBLE AT ANY GIVEN TIME. FAROBOT DOES NOT WARRANT THAT FAROBOT SOFTWARE WILL BE FREE OF ANY DEFECTS OR ERRORS (“BUGS”), THAT ANY BUGS WILL BE CORRECTED, OR THAT THE FAROBOT SOFTWARE IS COMPATIBLE WITH ANY PARTICULAR PLATFORM. SOME JURISDICTIONS DO NOT ALLOW THE WAIVER OR EXCLUSION OF IMPLIED WARRANTIES SO THEY MAY NOT APPLY TO YOU. IF THIS EXCLUSION IS HELD TO BE UNENFORCEABLE BY A COURT OF COMPETENT JURISDICTION, THEN ALL WARRANTIES THAT CANNOT BE DISCLAIMED SHALL BE LIMITED IN DURATION TO THE SHORTER OF (1) A PERIOD OF THIRTY (30) DAYS FROM THE DATE OF YOUR INITIAL PURCHASE OF A PAID SUBSCRIPTION TO FAROBOT SOFTWARE AND (2) THE MINIMUM PERIOD ALLOWED BY LAW.</li>
    </ol>
  </li>
  <li>Indemnification
Licensee shall defend, indemnify and hold FARobots and its affiliates, employees and agents harmless, from and against all sums, claims, costs, duties, liabilities, directly or indirectly losses, obligations, suits, actions, damages, penalties, awards, fines, interest, and other expenses (including investigation expenses and attorneys’ fees) that FARobots may incur or be obligated to pay as a result of Licensee’s unauthorized use, modification, resale, transfer, shipment, or export of the FARobot Software.</li>
  <li>Other General Term
    <ol>
      <li>Governing Law and Venue<br>
Unless prohibited by local law, all disputes hereunder shall be resolved in the courts of New Taipei City, Taiwan. All claims arising under this Agreement shall be governed by the laws of Taiwan, excluding its principles of conflict of laws and the United Nations Convention on Contracts for the Sale of Goods.</li>
      <li>Termination<br>
This Agreement is effective until it is terminated. You may terminate this Agreement at any time by cancelling your subscription to FARobot Software and deleting your account. FARobot may terminate this Agreement for any reason, including but not limited to your violation of any terms of this Agreement. All provisions relating to confidentiality, proprietary rights, and non-disclosure shall survive the termination of this Agreement.</li>
      <li>Agreement Binding on Successors<br>
This Agreement shall be binding upon and shall inure to the benefit of the parties hereto, their heirs, administrators, successors, and assigns.</li>
      <li>No Waiver<br>
No waiver by FARobot any default shall be deemed as a waiver of any prior or subsequent default of the same or other provisions of this Agreement.</li>
      <li>Integration<br>
This Agreement constitutes the entire understanding of the parties concerning the subject matter herein, and revokes and supersedes all prior agreements between the parties and is intended as a final expression of their agreement. FARobot may modify this Agreement at any time.</li>
      <li>Severability<br>
If any provision hereof is held invalid or unenforceable by a court of competent jurisdiction, such invalidity shall not affect the validity or operation of any other provision and such invalid provision shall be deemed to be severed from the Agreement.</li>
    </ol>
  </li>
</ol>`;

// let langTemplateObj_ =
// {
// 	"lng": "en",
// 	"resources": {
// 		"en": {
// 			"translation": {
// 				"general": {
// 					"btn_Cancel": "Cancel",
// 					"btn_Confirm": "Confirm"
// 				},
// 				"nav": {
// 					"dashboard": "Dashboard",
// 					"live_view": "Live Map",
// 					"fleet_configuration": "Fleet configuration",
// 					"flow_configuration": "Flow configuration",
// 					"flow_trigger": "Flow trigger",
// 					"map": "Map",
// 					"create_map": "Create Map",
// 					"edit_map": "Edit Map",
// 					"role": "Role",
// 					"system_settings": "System settings",
// 					"log": "Log",
// 					"user_management": "User management",
// 					"api_info": "API info",
// 					"sign_out": "Sign out",
// 					"footer": {
// 						"version": "Version"
// 					}
// 				},
// 				"dashboard": {
// 					"ttl_AgntStcs": "Agents Statistics",
// 					"ttl_ArtfStcs": "Artifacts Statistics",
// 					"ttl_FlowStcs": "Flows Statistics",
// 					"ttl_AgntList": "Flows List",
// 					"ttl_ArtfList": "Agents List",
// 					"ttl_FlowList": "Flows List",
// 					"tabs": {
// 						"flow": "Flow",
// 						"task": "Task"
// 					},
// 					"modal": {
// 						"ttl_AgntDetail": "Agent Details",
// 						"col_AgntId": "ID",
// 						"col_AgntName": "Name",
// 						"col_AgntStts": "Status",
// 						"col_AgntMode": "Mode",
// 						"col_AgntModel": "Model",
// 						"col_AgntVers": "Version",
// 						"col_AgntIp": "IP",
// 						"col_AgntMac": "Mac",
// 						"col_AgntBatt": "Battery"
// 					},
// 					"tmpl": {
// 						"col_AgntName": "Agent Name",
// 						"col_AgntTask": "Task",
// 						"col_AgntBatt": "Battery",
// 						"col_ArtfName": "Artifact Name",
// 						"col_NoFlows": "No Flows in this fleet",
// 						"chart": {
// 							"arr_AgntStates": [
// 								"Idle",
// 								"Charging",
// 								"Active",
// 								"Paused",
// 								"Waiting",
// 								"Emergency",
// 								"Unintialized",
// 								"Disconnected"
// 							],
// 							"arr_ArtfStates": [
// 								"In service",
// 								"No data received",
// 								"Error",
// 								"Disconnected"
// 							],
// 							"arr_FlowStates": [
// 								"Queued",
// 								"Active",
// 								"Completed",
// 								"Failed",
// 								"Paused"
// 							]
// 						},
// 						"list": {
// 							"agnt": {
// 								"lbl_Name": "Agent Name",
// 								"lbl_Task": "Task",
// 								"lbl_Batt": "Battery",
// 								"row_Empty": "No Agents in this fleet"
// 							},
// 							"artf": {
// 								"lbl_Name": "Artifact Name",
// 								"row_Empty": "No Artifacts in this fleet"
// 							},
// 							"flow": {
// 								"lbl_Name": "Name",
// 								"lbl_Id": "ID",
// 								"lbl_Assignee": "Assigned to",
// 								"lbl_Status": "Status",
// 								"lbl_Progress": "Progress",
// 								"lbl_Operation": "Operation",
// 								"row_Empty": "No Flows in this fleet"
// 							}
// 						}
// 					}
// 				},
// 				"live_view": {
// 					"ttl_liveView": "Live View",
// 					"lbl_Map": "Map",
// 					"lbl_MapFileName": "file name"
// 				},
// 				"fleet_config": {
// 					"btn_CreateNewFleet": "Create New Fleet",
// 					"btn_DeleteFleet": "Delete Fleet",
// 					"btn_SaveFleet": "Save",
// 					"ttl_AgntFltStts": "Agent Fleet Status",
// 					"ttl_ArtfFltStts": "Artifact Fleet Status",
// 					"ttl_Maps": "Maps",
// 					"ttl_Roles": "Roles",
// 					"sidebar": {
// 						"lbl_AgntArtfName": "Name",
// 						"lbl_AgntArtfId": "ID",
// 						"lbl_AgntArtfMode": "Mode",
// 						"lbl_AgntArtfModel": "Model",
// 						"lbl_AgntArtfSw": "Version",
// 						"lbl_AgntArtfUl": "Upgrade list",
// 						"btn_AgntLink": "Link to AMR",
// 						"btn_AgntInitP": "Initial Pose",
// 						"btn_AgntReboot": "Reboot",
// 						"tab_PortData": "Import/Export",
// 						"tab_FactorySet": "Factory Settings"
// 					},
// 					"tmpl": {
// 						"btn_SidebarImport": "Import",
// 						"btn_SidebarExport": "Export",
// 						"agnt": {
// 							"lbl_SidebarName": "Agent Name",
// 							"lbl_AvailTitle": "Available Agents",
// 							"ttp_SidebarDesc": "Description",
// 							"ttp_SidebarRange": "Range",
// 							"ttp_SidebarUnit": "Unit",
// 							"btn_Load": "Loading",
// 							"btn_Ready": "Ready",
// 							"btn_Init": "Confirm Initial Pose"
// 						},
// 						"artf": {
// 							"lbl_SidebarName": "Artifact Name",
// 							"lbl_AvailTitle": "Available Artifacts",
// 							"lbl_SidebarDesc": "Description",
// 							"lbl_SidebarRange": "Range",
// 							"lbl_SidebarUnit": "Unit"
// 						},
// 						"maps": {
// 							"lbl_AvailTitle": "Available Maps"
// 						},
// 						"role": {
// 							"lbl_AvailTitle": "Available Roles"
// 						}
// 					}
// 				},
// 				"flow_config": {
// 					"ttl_Flows": "Flow",
// 					"ttl_PlanArtfStatus": "Plan Artifact Status",
// 					"btn_createFlow": "flow",
// 					"btn_ToRoleEditor": "Go to Role Overview",
// 					"editor": {
// 						"btn_Back": "Back",
// 						"btn_SaveFlow": "Save",
// 						"lbl_FlowTrig": "Flow trigger",
// 						"tab_Role": "Role",
// 						"btn_Run": "Run",
// 						"btn_ClearPanel": "Clear Panel"
// 					},
// 					"tmpl": {
// 						"list": {
// 							"artf": {
// 								"lbl_Title": "Available Artifacts"
// 							}
// 						}
// 					}
// 				},
// 				"create_map": {
// 					"ttl_CreateMap": "Create Map",
// 					"btn_SaveMap": "Save Map",
// 					"editor": {
// 						"btn_SelAgnt": "Choose Agent",
// 						"tab_CreateMap": "Create",
// 						"lbl_StartMap": "Start Mapping",
// 						"col_Param": "Parameters",
// 						"col_Value": "Value",
// 						"col_ParamRes": "Resolution",
// 						"col_ParamOrigin": "Origin",
// 						"col_ParamHeight": "Height",
// 						"col_ParamWidth": "Wdith",
// 						"lbl_RotateMap": "rotate map"
// 					}
// 				},
// 				"edit_map": {
// 					"ttl_MapEditor": "Map Editor",
// 					"ttl_PropView": "Properties View",
// 					"opt_MapImg": "Map",
// 					"opt_MapOrig": "Origin",
// 					"opt_Routes": "Route",
// 					"opt_Cells": "Cells",
// 					"tgl_EditProp": "Edit Properties",
// 					"tgl_MapImg": "Map",
// 					"tgl_Routes": "Route",
// 					"tgl_Cells": "Cells",
// 					"modal": {
// 						"ttl_RenameMap": "Rename the Map",
// 						"btn_Cancel": "Cancel",
// 						"btn_Confirm": "Confirm",
// 						"ttl_DeleteMap": "Delete",
// 						"lbl_ConfirmMsg": "Confirm"
// 					},
// 					"editor": {
// 						"ttl_MapEditPropTools": "Edit Properties Tool",
// 						"ttl_LineWidth": "Line Width",
// 						"btn_MapView": "View",
// 						"btn_MapLine": "Line",
// 						"btn_MapEraser": "Eraser",
// 						"btn_MapEditOrigin": "Edit Origin",
// 						"btn_SetOrigin": "Set Origin",
// 						"btn_CancelSetOrigin": "Cancel Set Origin",
// 						"btn_FitView": "Fit",
// 						"btn_AddVertex": "Add Vertex",
// 						"btn_EditVertex": "Edit Vertex",
// 						"btn_AddEdge": "Add Edge",
// 						"btn_EditEdge": "Edit Edge",
// 						"btn_AddCell": "Add Cell",
// 						"btn_EditCell": "Edit Cell",
// 						"btn_DeleteCell": "Delete Cell",
// 						"btn_FunctionType": "function-type Menu",
// 						"btn_Create": "Create",
// 						"btn_Cancel": "Cancel",
// 						"btn_Undo": "Undo",
// 						"btn_Save": "Save",
// 						"btn_Delete": "Delete",
// 						"lbl_Label": "label",
// 						"lbl_Weight": "weight",
// 						"lbl_FunctionType": "function type",
// 						"lbl_Area": "area",
// 						"lbl_Direction": "direction",
// 						"opt_Forward": "Forward",
// 						"opt_Backward": "Backward",
// 						"opt_Left": "Left",
// 						"opt_Right": "Right",
// 						"lbl_Load": "Load",
// 						"opt_Empty": "Empty",
// 						"opt_Occupied": "Occupied",
// 						"lbl_Markers": "Markers",
// 						"opt_Pattern": "Pattern",
// 						"opt_None": "None",
// 						"lbl_MarkerOffset": "Marker offset",
// 						"lbl_CellSize": "Cell size"
// 					}
// 				},
// 				"role": {
// 					"ttl_RolesOverview": "Roles Overview",
// 					"btn_CreateNewRole": "Create New Role",
// 					"btn_BackToPrev": "Back",
// 					"btn_SaveRole": "Save"
// 				},
// 				"sys_set": {
// 					"tab_swarm_core": "Swarm Core",
// 					"tab_UsrSetting": "User Settings",
// 					"tab_UiSetting": "Ui Settings",
// 					"tab_Others": "Others",
// 					"swarm_core": {
// 						"ttl_LowBatteryThresh": "Low battery threshold",
// 						"txt_LowBatteryThresh": "When the AMR is not in the charging mode, it will be assigned the charging task if its battery level is lower than this threshold",
// 						"btn_SaveSettings": "Save"
// 					},
// 					"usr_set": {
// 						"lbl_Account": "Account",
// 						"lbl_Pwd": "Password",
// 						"lbl_ContactNumber": "Contact Number",
// 						"lbl_Email": "E-mail",
// 						"btn_Apply": "Apply",
// 						"ttl_ChangePwd": "Change Password",
// 						"txt_Pwd": "Password",
// 						"txt_ConfirmPwd": "Confirm Password",
// 						"btn_Cancel": "Cancel",
// 						"btn_Confirm": "Confirm"
// 					},
// 					"ui_set": {
// 						"lbl_Theme": "Appereance",
// 						"opt_Dark": "Dark",
// 						"opt_Light": "Light",
// 						"lbl_FontSize": "Font Size",
// 						"opt_Small": "Small",
// 						"opt_Medium": "Medium",
// 						"opt_Large": "Large",
// 						"lbl_LangEnv": "Language Environment",
// 						"btn_Apply": "Apply"
// 					},
// 					"others": {
// 						"lbl_SwVer": "Software Version",
// 						"lbl_CurrVer": "Current software version",
// 						"lbl_DepolyVer": "Select intent software version and click to deploy",
// 						"lbl_LocoRepo": "Software to local repo",
// 						"btn_Sw2Loco": "Software to local",
// 						"lbl_LGN": "LINE Groups Notification",
// 						"lbl_LineToken": "Line Token",
// 						"lbl_LogLevel": "Log Level",
// 						"lbl_LogContent": "Log Content",
// 						"plh_LineToken": "input token here...",
// 						"plh_LogKeywords": "filter content here...",
// 						"btn_Update": "Update",
// 						"lbl_LicAuth": "License Authorization",
// 						"lbl_LicExpire": "License Authorization",
// 						"lbl_HwSig": "Hardware Signature",
// 						"btn_Activate": "Activate",
// 						"btn_Remove": "Remove",
// 						"btn_ExportHwSig": "Export",
// 						"lbl_FacSet": "Data Import/Export - Factory Reset",
// 						"btn_ImportData": "Import",
// 						"btn_ExportData": "Export",
// 						"lbl_EraseData": "Earase all data",
// 						"btn_Delete": "Delete"
// 					}
// 				},
// 				"log": {
// 					"ttl_BatteryLog": "Battery Log",
// 					"btn_BatteryLogExport": "Export",
// 					"ttl_DebugLog": "Debug Log",
// 					"btn_DebugLogExport": "Export",
// 					"ttl_EventLog": "Event Log",
// 					"btn_AdvancedOptions": "Advanced option",
// 					"btn_DownloadCSV": "Download CSV",
// 					"col_Time": "Time",
// 					"col_EventType": "Event Type",
// 					"col_EventId": "Event ID",
// 					"col_ModPub": "Pub Module",
// 					"col_ModSub": "Sub Module",
// 					"col_Level": "Level",
// 					"col_Msg": "Message Content",
// 					"col_DeviceId": "Device ID",
// 					"editor": {
// 						"plh_Keywords": "Search for any keywords",
// 						"lbl_Event": "Event",
// 						"lbl_Module": "Module",
// 						"lbl_Others": "Others",
// 						"lbl_DateTime": "Date & Time",
// 						"btn_Cancel": "Clear",
// 						"btn_Clear": "Clear",
// 						"btn_Apply": "Apply"
// 					}
// 				},
// 				"usr_man": {
// 					"ttl_ManageUser": "Manage User",
// 					"btn_CreateNewUser": "Create New User",
// 					"col_Edit": "Edit",
// 					"col_UserName": "User Name",
// 					"col_UserAccount": "User Account",
// 					"col_Admin": "Admin",
// 					"col_PhoneNumber": "Phone Number",
// 					"col_Email": "E-mail",
// 					"col_Membership": "Memeber Since",
// 					"col_Options": "Options",
// 					"btn_DeleteUser": "Delete",
// 					"btn_SaveUser": "Save",
// 					"editor": {
// 						"ttl_AddUser": "Add a new user",
// 						"lbl_Account": "Account",
// 						"lbl_PhoneNumber": "Contack Number",
// 						"lbl_Pwd": "Password",
// 						"lbl_Email": "E-mail",
// 						"lbl_Name": "Name",
// 						"lbl_Auth": "Authority",
// 						"btn_Cancel": "Cancel",
// 						"btn_Create": "Create"
// 					}
// 				},
// 				"api": {
// 					"agent_setttings": {
// 						"version": "1.0.0",
// 						"category": {
// 							"system": "system",
// 							"control": "control",
// 							"localization": "localization",
// 							"perception": "perception",
// 							"planning": "planning"
// 						},
// 						"system": {
// 							"map": {
// 								"title": "Map",
// 								"description": "The current map that robot is using."
// 							},
// 							"fleet_name": {
// 								"title": "Fleet Name",
// 								"description": "The current fleet that robot is joined."
// 							},
// 							"agent_name": {
// 								"title": "Agent Name",
// 								"description": "User defined agent name."
// 							},
// 							"initial_pose_x": {
// 								"title": "Initial Pose X",
// 								"description": ""
// 							},
// 							"initial_pose_y": {
// 								"title": "Initial Pose Y",
// 								"description": ""
// 							},
// 							"initial_pose_yaw": {
// 								"title": "Initial Pose Yaw",
// 								"description": ""
// 							},
// 							"initial_pose_confirmed": {
// 								"title": "Initial Pose Confirmed",
// 								"description": ""
// 							},
// 							"mapping_mode": {
// 								"title": "Mapping Mode",
// 								"description": "True: Enable SLAM, False: disable SLAM"
// 							}
// 						},
// 						"control": {
// 							"[move_mode] max_velocity": {
// 								"title": "Max Velocity",
// 								"description": "The maximum velocity for linear direction"
// 							},
// 							"[move_mode] goal_tolerance_distance": {
// 								"title": "Goal Tolerance Distance",
// 								"description": "The control error tolerance for longitude"
// 							},
// 							"[move_mode] goal_tolerance_angle": {
// 								"title": "Gaol Tolerance Angle",
// 								"description": "The control error tolerance for rotation"
// 							},
// 							"[move_mode] max_angular_velocity": {
// 								"title": "Max Angular Velocity",
// 								"description": "The maximum velocity for angular direction"
// 							},
// 							"[move_mode] distance_threshold_high_velocity": {
// 								"title": "Distance Threshold High Velocity",
// 								"description": "The distance for calculating the limitation of maximum velocity, over this value max velocity would use higher speed"
// 							},
// 							"[move_mode] distance_threshold_low_velocity": {
// 								"title": "Distance Threshold Low Velocity",
// 								"description": "The distance for calculating the limitation of maximum velocity, lower this value max velocity would use lower speed"
// 							},
// 							"[docking_mode] abort_distance": {
// 								"title": "Abort Distance",
// 								"description": "The distance which starts to check the control error tolerance"
// 							},
// 							"[docking_mode] num_of_retries": {
// 								"title": "Num. of Retries",
// 								"description": "The number of docking process retry when control error is too large"
// 							},
// 							"[docking_mode] docking_time_limit": {
// 								"title": "Docking Time Limit",
// 								"description": "The time limit for the docking process"
// 							},
// 							"[docking_mode] detect_time_limit": {
// 								"title": "Delete Time Limit",
// 								"description": "The time limit for the detection pattern"
// 							},
// 							"[docking_mode] max_velocity": {
// 								"title": "Max Velocity",
// 								"description": "The docking mode maximum velocity in linear direction"
// 							},
// 							"[docking_mode] max_angular_velocity": {
// 								"title": "Max Angular Velocity",
// 								"description": "The docking mode maximum velocity in rotation"
// 							},
// 							"[docking_mode] tolerance_longitudinal_distance": {
// 								"title": "Toloerance Logitudinal Distance",
// 								"description": "The control error tolerance for longitude"
// 							},
// 							"[docking_mode] tolerance_lateral_distance": {
// 								"title": "Tolerance Lateral Distance",
// 								"description": "The control error tolerance for latitude"
// 							},
// 							"[docking_mode] tolerance_rotation": {
// 								"title": "Tolerance Rotation",
// 								"description": "The control error tolerance for rotation"
// 							},
// 							"[rotate_mode] max_angular_velocity": {
// 								"title": "Max Angular Velocity",
// 								"description": "The maximum velocity for angular direction"
// 							},
// 							"[rotate_mode] rotation_goal_reach_tolerance": {
// 								"title": "Rotation Goal Reach Tolerance",
// 								"description": "The control error tolerance for rotation"
// 							}
// 						},
// 						"localization": {
// 							"localization_max_reference_range": {
// 								"title": "Localization Max Reference Range",
// 								"description": "The laser range is used for lidar localization."
// 							}
// 						},
// 						"planning": {
// 							"abort_stop_time": {
// 								"title": "Abort Stop Time",
// 								"description": "The time for trigger rerouting or navigation failed when obstacles exist"
// 							},
// 							"safety_time_for_move": {
// 								"title": "Safety Time for Move",
// 								"description": "The time for the robot restarts to move when stopping"
// 							},
// 							"footprint": {
// 								"title": "Footprint",
// 								"description": "The size of the robot. format: [[m,m],[m,m],[m,m],[m,m]]"
// 							},
// 							"[safety_zone] stop_longitudinal_distance": {
// 								"title": "Stop Longitudinal Distance",
// 								"description": "The distance for creating a stop area in the longitudinal direction"
// 							},
// 							"[safety_zone] stop_lateral_distance": {
// 								"title": "Stop Lateral Distance",
// 								"description": "The distance for creating a stop area in the latitudinal direction"
// 							},
// 							"[safety_zone] slow_lateral_distance": {
// 								"title": "Slow Lateral Distance",
// 								"description": "The distance for creating a slow area in the latitudinal direction"
// 							}
// 						},
// 						"perception": {
// 							"[rack] detect_tolerance": {
// 								"title": "Detect Tolerance",
// 								"description": "Tolerance between detect position and cell position"
// 							},
// 							"[rack] distance_tolerance": {
// 								"title": "Distance Tolerance",
// 								"description": "Tolerance for filtering point which is far from cell position"
// 							},
// 							"[rack] check_size_tolerance": {
// 								"title": "Check Size Tolerance",
// 								"description": "Comparison between real rack size and detect rack size."
// 							}
// 						}
// 					},
// 					"swarm_core_settings": {
// 						"version": "1.0.0",
// 						"plan": {
// 							"low_battery_threshold": {
// 								"title": "Low battery threshold",
// 								"description": "When the AMR is not in the charging mode, it will be assigned the charging task if its battery level is lower than this threshold."
// 							},
// 							"full_battery_threshold": {
// 								"title": "Full battery threshold",
// 								"description": "When the AMR is in the charging mode, the charging task will be canceled if its battery level is above this threshold."
// 							},
// 							"auto_charging": {
// 								"title": "Auto charging",
// 								"description": "Enable or disable auto charging function"
// 							},
// 							"charging_due_time": {
// 								"title": "Charging due time",
// 								"description": "When the AMR is in the charging mode, the charging task will be canceled if exceeding its battery charging time."
// 							}
// 						},
// 						"battery_log": {
// 							"battery_log_time": {
// 								"title": "Battery log time",
// 								"description": "Time Period to record battery conditions of each SMR."
// 							},
// 							"battery_log_threshold": {
// 								"title": "Battery log threshold",
// 								"description": "Time limit to store battery log each SMR."
// 							}
// 						},
// 						"system": {
// 							"system_restart": {
// 								"title": "System restart",
// 								"description": "Restart swarm core system"
// 							}
// 						},
// 						"btn_SystemRestart": "restart",
// 						"btn_SaveSettings": "Save"
// 					}
// 				}
// 			}
// 		}
// 	}
// };