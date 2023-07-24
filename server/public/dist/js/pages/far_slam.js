/*
 * Author: John Wu
 * Date: 5 May 2021
 * Description:
 *   1. virtual joystick
 *   2. save map to server
 **/
// ======================
//       Load Ready
// ======================
const updateRate = 1000;

var agent_ip_dict = {};
let select_amr_ip = undefined;
let isUpload = false;
let rotateAngle = 0;

let robot_x_canvas = 0;
let robot_y_canvas = 0;
let robot_yaw = 0;

let global_lin = 0;
let global_ang = 0;
let move_somewhere = false;

let canvasIsClean = true;

var gSprite_ = new Image();
gSprite_.src = `${getImagePath()}/sprites/slam_agent.png`;

setInterval(function () {
  if (move_somewhere) {
    moveSomeWhere(global_lin, global_ang);
    console.log(`======= global linear: ${global_lin}, global angular: ${global_ang} =======`);
    if (global_lin == 0 && global_ang == 0) {
      move_somewhere = false;
    }
  }
}, 200);

$(function () {
  displayOverlay('Data Loading...');
  showTextInVisCanvas("No mapping data receive.");
  initTimeSync();
  initDataAsync();
  initSaveMapForm();

  scanManualModeAgents();

  initJoystick();
  initMobileJoystickEvts();

  //-- buttons for map rotation
  $('.btn-plus, .btn-minus').on('click', function (e) {
    const isNegative = $(e.target).closest('.btn-minus').is('.btn-minus');
    const input = $(e.target).closest('.input-group').find('input');
    if (input.is('input')) {
      input[0][isNegative ? 'stepDown' : 'stepUp']()
      if (isNegative)
        rotateAngle -= 1;
      else
        rotateAngle += 1;
    }
  })

  validateMapNameInputEvent();
});

var prevAgents = {};
setInterval(async function () {
  if (isUpload) {
    return;
  }

  try {
    scanData_ = await fetchScanRobots2(rmtToken_);
  } catch (err) {
    console.error(err);
  }

  var agents = scanData_.robots.filter(robot => robot.mode === "manual");
  if (JSON.stringify(agents) === JSON.stringify(prevAgents)) return;
  prevAgents = agents;
  console.log(`---manual agent---`);
  console.log(agents);

  agents.forEach(element => agent_ip_dict[element.robot_id] = element.ip);

  // --- initialized the widgets ---
  $("#agent-select").empty();
  var canDiv = document.getElementById('hidden-map');
  $(canDiv).empty();
  document.getElementById('customSwitch1').checked = false;
  document.getElementById('customSwitch1').disabled = true;

  var mapDiv = document.getElementById('far-slam-map');
  if (agents.length === 0) {
    $(mapDiv).hide();
    notificationMsg(2, 'No manual mode agents!');
    // toast('No manual mode agents!!');
    removeOverlay();
    return;
  }

  $(mapDiv).show();

  document.getElementById('customSwitch1').disabled = false;
  // console.log('fetched robots');
  document.getElementById("scan-switch").style.display = "inline";
  updateAvailableAgents(agents);
  // --- update the start gmapping switch ---
  updateAgentsMappingData(agents[0].robot_id);

  updateAgentUploadMaps();
}, 10000);


let ts;
function initTimeSync() {
  ts = timesync.create({
    server: 'swarmsync',
  });

  ts.on('sync', function (state) {
    console.log('sync ' + state);
  });

  ts.on('change', function (offset) {
    console.log('changed offset: ' + offset + ' ms');
    $('#time-offset').text(`offset: ${offset} ms`);
    $('#synched-time').text(`sync time: ${Date.now()} ms`);
  });

  ts.send = function (to, data, timeout) {
    return new Promise(function (resolve, reject) {
      $.ajax({
        url: to,
        type: 'POST',
        data: JSON.stringify(data),
        contentType: 'application/json',
        dataType: 'json', // response type
        timeout: timeout
      })
        .done(function (data) {
          //console.log('receive', data);
          ts.receive(to, data);
          resolve();
        })
        .fail(function (err) {
          console.log('Error', err);
          reject(err);
        });
    });
  };
}

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'map', 'slam.html');

  // ------ language switch ------
  await initLanguageSupport();
}

function initSaveMapForm() {
  $.validator.addMethod("chkDupMapName", function (value, element) {
    if (isEmpty(value)) return true;
    let data = restMapFileNameExistence(value);
    let mapNameExists = data.responseJSON;
    if (mapNameExists === undefined) {
      return false;
    }
    return !mapNameExists;
  });

  $('#save-map-form').validate({
    rules: {
      mapFileName: {
        chkInputVal: true,
        chkDupMapName: true
      }
    },
    messages: {
      mapFileName: {
        chkInputVal: 'Map Name include invalid characters',
        chkDupMapName: 'Map name already exists'
      }
    },
    errorElement: 'div',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.after(error);
    }
  });

  $('#save-map-form').on('submit', saveScannedMap);
}

// -------- Navigation and Map START --------
let slamAgent_ = 'fb_0'; // Dummy value
let msgInfo_;
let gMapMeta_ = {
  imageData: undefined,
  w: 0,
  h: 0,
  resolution: 0,
  origin: {
    x: 0,
    y: 0
  },
  fetched: false
};

let updateInterval_;
let slamPose_;

function resetGMapMeta () {
  gMapMeta_ = {
    imageData: undefined,
    w: 0,
    h: 0,
    resolution: 0,
    origin: {
      x: 0,
      y: 0
    },
    fetched: false
  }

  var canvas = document.getElementById('map_canvas');
  if (canvas !== null) {
    canvas.remove();
  }
}

function initRosMessages(_agent) {
  // wsSlamTopicsInit(_agent);

  // updateInterval_ = setInterval(updateSlamMessages, 100);
  updateSlamMessages(updateRate);
}

async function updateSlamMessages(interval) {
  if (isUpload) {
    return;
  }

  let slamObj = {
    info: {
      mapName: 'default',
      resolution: 0,
      height: 0,
      width: 0,
      origin: {
        position: {
          x: 0,
          y: 0,
          z: 0,
        },
        orientation: 0
      },
      rawImage: ''
    },
    msg: ''
  };
  const startMapping = document.getElementById("customSwitch1").checked;

  if (startMapping) {
    await fetchGmappingStatus(select_amr_ip)
    .then(response => response.json())
    .then(data => {
      const { gmapping_status, rtn_code } = {...data};
      if (rtn_code.code === 0) {
        const {
          data,
          gmapping_pose_x,
          gmapping_pose_y,
          gmapping_pose_yaw,
          height,
          width,
          resolution,
          origin_pose_x,
          origin_pose_y
        } = {...gmapping_status};

        slamObj.info.resolution = resolution;
        slamObj.info.height = height;
        slamObj.info.width = width;
        slamObj.info.origin.position = {
          x: origin_pose_x,
          y: origin_pose_y,
          z: 0,
        };
        slamObj.info.origin.orientation = [gmapping_pose_x, gmapping_pose_y, gmapping_pose_yaw];
        slamObj.info.rawImage = `data:image/png;base64, ${data}`;
      }
    })
    .catch(error => {
      console.error(error);
    });

    await fetchCreateMapImage(select_amr_ip)
    .then(response => response.json())
    .then(data => {
      const { create_map, rtn_code } = {...data};
      if (rtn_code.code === 0 && create_map.data !== undefined) {
        const mapBase64Str = `data:image/png;base64, ${create_map.data}`;
        slamObj.msg = mapBase64Str;
      }
      // console.log(create_map);
      // console.log(rtn_code);
    })
    .catch(error => {
      console.error(error);
      clearVisCanvas();
      showTextInVisCanvas("No mapping data receive.");
    });
  } else {
    clearVisCanvas();
    showTextInVisCanvas("No mapping data receive.");
  }

  if (slamObj.msg.length !== 0) {
    clearVisCanvas();
    renderMapOnCanvas(slamObj);
  }

  updateMapInfo(slamObj);

  setTimeout(function () {
    updateSlamMessages(interval);
  }, interval);
}

function updateMapInfo (slamObj) {
  const mapInfo = slamObj.info;
  msgInfo_ = mapInfo;

  $("#map-resolution").html(mapInfo.resolution.toFixed(4));
  $("#map-origin").html(`[${mapInfo.origin.position.x.toFixed(2)}, ${mapInfo.origin.position.y.toFixed(2)}, 0]`);
  if (rotateAngle === 0) {
    $("#map-width").html(mapInfo.width);
    $("#map-height").html(mapInfo.height);
  }
  gMapMeta_ = {
    imageData: mapInfo.rawImage,
    w: mapInfo.width,
    h: mapInfo.height,
    resolution: mapInfo.resolution,
    origin: { x: mapInfo.origin.position.x, y: mapInfo.origin.position.y },
    fetched: true
  };
}

function formatAsYaml(info) {
  var content = `image: ${info.mapName}.png\n` +
    `mode: trinary\n` +
    `resolution: ${info.resolution.toFixed(4)}\n` +
    `origin: [${info.origin.position.x}, ${info.origin.position.y}, 0]\n` +
    `negate: 0\n` +
    `occupied_thresh: 0.65\n` +
    `free_thresh: 0.25\n` +
    `nav_graph: ${info.mapName}.dot\n` +
    `cell: ${info.mapName}.json\n` +
    `triton: ${info.mapName}.amf`;

  return content;
}

function moveSomeWhere(linVel, angVel) {
  var jsonMove = {
    velCmd: {
      linear: {
        x: linVel,
        y: 0.0,
        z: 0.0
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: angVel
      }
    },
    slamAgent: slamAgent_,
    issuedTime: ts.now()
  };

  let x = linVel.toFixed(1);
  let z = +angVel.toFixed(1);

  if (Math.abs(z) === 0) {
    z = 0;
  }

  // console.log(jsonMove);
  console.log('========================');
  console.log(x, z);
  console.log('========================');

  if (select_amr_ip != undefined) {
    putOpenLoop(select_amr_ip, x, z);
  } else {
    console.log('No seleted AMR!');
  }
  // wsMoveSomeWhere(jsonMove)
}

// -------- joystick functions --------
var manager;
var lin;
var ang;
var joystick_timeout;
var velocity_repeat_delay = 800; // [ms]
var max_joy_pos = 0;

function initJoystick() {
  var tab_height = $('#tab_1 > .row > .col-lg-3').height();
  var tab_width = $('#tab_1 > .row > .col-lg-3').width();
  setJoystickView(tab_width, tab_height);
}

function removeJoystick() {
  if (!isMobile()) {
    joystickContainer = document.getElementById('joystick');
  } else if (window.innerWidth <= 768) {
    joystickContainer = document.getElementById('header-joystick');
  } else {
    joystickContainer = document.getElementById('tab-joystick');
  }
  while (joystickContainer.hasChildNodes()) {
    joystickContainer.removeChild(joystickContainer.childNodes[0]);
  }
  if (!jQuery.isEmptyObject(manager)) {
    manager.destroy();
  }
}

function setJoystickView(_w, _h) {
  // _w: parent DOM width, _h: parent DOM height.
  removeJoystick();
  joySize = 128;
  if (joySize > $(window).height()) {
    joySize = $(window).height();
  }
  if (joySize > $(window).width()) {
    joySize = $(window).width();
  }
  max_joy_pos = joySize / 3;
  createJoystick(_w * 0.6, _h - (joySize * 0.6), 128);
}

function createJoystick(x, y, d) {
  let dtJoystick = document.getElementById('joystick');
  let topJoystick = document.getElementById('header-joystick');
  let bottomJoystick = document.getElementById('tab-joystick');
  if (!isMobile()) {
    joystickContainer = dtJoystick;
  } else if (window.innerWidth <= IPAD_PORTRAIT_WIDTH) {
    joystickContainer = topJoystick;
    topJoystick.style.display = 'block';
    bottomJoystick.style.display = 'none';
  } else {
    joystickContainer = bottomJoystick;
    bottomJoystick.style.display = 'block';
    topJoystick.style.display = 'none';
  }
  var options = {
    zone: joystickContainer,
    position: { left: x + 'px', top: y + 'px' },
    mode: 'static',
    size: d,
    restOpacity: 1.0,
    tJoystick: true
  };
  manager = nipplejs.create(options);
  manager.on('move', function (evt, nipple) {
    var direction = nipple.angle.degree - 90;
    if (direction > 180) {
      direction = -(450 - nipple.angle.degree);
    }

    // Max_Linear_Velocity: 0.5 m/s, Max_Angular_Speed: 0.3 rad/s
    lin = Math.cos(direction / 57.29) * 0.5 * nipple.distance / max_joy_pos;
    ang = Math.sin(direction / 57.29) * 0.3 * nipple.distance / max_joy_pos;
    console.log(nipple.position)
    console.log(`linear: ${lin}, angular: ${ang}`);

    global_lin = lin;
    global_ang = ang;
    move_somewhere = true;
    // clearInterval(joystick_timeout);
    // moveSomeWhere(lin, ang);
    // joystick_timeout = setInterval(function () { moveSomeWhere(lin, ang); }, velocity_repeat_delay);
    // setTimeout(function () { moveSomeWhere(lin, ang); }, 100);
  });
  manager.on('end', function () {
    // clearInterval(joystick_timeout);
    // moveSomeWhere(0, 0);
    // setTimeout(function () { moveSomeWhere(lin, ang); }, 100);
    global_lin = 0;
    global_ang = 0;
  });
  manager[0].ui.front.style.opacity = 1.0;
  manager[0].ui.back.style.opacity = 1.0;
  manager[0].ui.front.style.background = `url(./${getImagePath()}/joystick-red.png)`;
  manager[0].ui.back.style.background = `url(./${getImagePath()}/joystick-base.png)`;
}

function normalizeAngle(angle) {
  angle = angle % 360;
  if (angle < 0) {
    angle = angle + 360;
  }
  return angle;
}

async function uploadDataAfterSave (mapName) {
  let seleted_agent_id = $('#agent-select').val();
  let upload_res = await fetchUploadAgentMap(rmtToken_, seleted_agent_id, mapName);
  if (upload_res) {
    if (document.getElementById('overlay') !== null) {
      removeOverlay();
    }
    notificationMsg(0, 'The scanned map Saved!');
  } else {
    // due to need pull from smr, but smr save map may have delay
    uploadDataAfterSave(mapName);
  }
}

async function saveScannedMap(e) {
  e.preventDefault();
  if (!$('#save-map-form').valid()) { return; }
  $('#myModal').modal('hide');
  var mapName = document.getElementById('map-filename').value || "default";

  displayOverlay('Save Map ...');
  await postSaveSlamMap(select_amr_ip, mapName, normalizeAngle($('#rotateAngle').val()))
  .then(function (response) {
    return response.json();
  })
  .then(async function(data) {
    console.log(data);
    if (data.rtn_code.code === 0) {
      uploadDataAfterSave(mapName)
    }
  })
  .catch(function(error) {
    console.log('error:', error);
    if (document.getElementById('overlay') !== null) {
      removeOverlay();
    }
  });
}

async function switchSlamAgent() {
  console.log(`selected agent section`);
  var sel = document.getElementById('agent-select');
  var opt = sel.options[sel.selectedIndex].text;
  // console.log(`selected agent: ${opt}`);
  if (opt === "") return;
  console.log(`selected agent: ${opt}`);
  slamAgent_ = opt;

  displayOverlay('Data Loading...');
  // --- update the start gmapping switch ---
  updateAgentsMappingData(opt);
  await updateAgentUploadMaps();
  showSelectedImage();
}


let scanData_;
// let rmtToken_ = {};
async function scanManualModeAgents() {
  if (rmtToken_ == undefined) {
    return;
  }

  if (Object.keys(rmtToken_).length === 0) {
    try {
      rmtToken_ = await fetchToken();
      notificationMsg(1, 'RMT ONLINE!');
    } catch (err) {
      console.error(err);
      notificationMsg(3, 'RMT OFFLINE!');
      // toast('Error: Fail to reach RMT!', 'rgba(255,0,0,0.6)');
    }
  }
}

async function updateAgentsMappingData(_currAgent) {
  select_amr_ip = agent_ip_dict[_currAgent];
  resetGMapMeta()
  // console.log(select_amr_ip);

  // 1. get settings info
  var agentSettings;
  try {
    agentSettings = await fetchAgentSettings(rmtToken_, _currAgent);
  } catch (err) {
    console.error(err);
  }
  // 2. get gmapping mode
  if (agentSettings.agent_settings[0] === undefined) {
    return
  }

  var configData = agentSettings.agent_settings[0].nav_conf;
  console.log(configData);
  // -- rectify data --
  configData = configData.replace(/'/g, '"');
  configData = configData.replace(/True/g, 'true').replace(/False/g, 'false');
  configData = JSON.parse(configData);

  console.log(configData);
  var gmappingEnable = configData['system']['mapping_mode'];

  // 3. update to switch toggle
  // document.getElementById("customSwitch1").checked = gmappingEnable;
  await fetchRobotStatus(select_amr_ip)
  .then(response => response.json())
  .then(data => {
    const { robot_status, rtn_code } = {...data};
    if (rtn_code.code === 0) {
      const { SLAM } = {...robot_status};
      document.getElementById("customSwitch1").checked = SLAM;
    }
  })
  .catch(error => {
    console.error(error);
    document.getElementById("customSwitch1").checked = gmappingEnable;
  });

  // --- create new ROS message Instances ---
  initRosMessages(slamAgent_);

  if (document.getElementById('overlay') !== null) {
    removeOverlay();
  }
}

function updateAvailableAgents(_agents) {
  console.log(_agents);
  $("#agent-select").find('option').remove();
  // console.log(`agent number: ${agents.length}`);

  slamAgent_ = _agents[0].robot_id;
  _agents.forEach(function (agentId) {
    $("#agent-select").append(`<option value='${agentId.robot_id}'>${agentId.robot_id}</option>`);
  });
}

function switchGmapping(_node) {
  console.log(_node.checked);
  // --- get current robot id ---
  var opt = document.getElementById('agent-select').value;
  if (opt === "") {
    notificationMsg(2, 'No manual mode agents!');
    // toast("No manual mode agents!!");
  }

  var enableScan = document.getElementById('customSwitch1').checked;
  const lockStatus = !enableScan;
  if (enableScan) {
    putEnableSLAM(select_amr_ip);
    putSetManualLock(select_amr_ip, lockStatus);
  } else {
    putDisnableSLAM(select_amr_ip);
    putSetManualLock(select_amr_ip, lockStatus);
    resetGMapMeta()
  }
  // var stat = (enableScan) ? "True" : "False";
  // var enableCmd = `{'system': {'mapping_mode': ${stat}}}`;
  // fetchPutAgentSettings(rmtToken_, opt, enableCmd);
}

// ===========================
//     VIS-NETWORK DRAWING
// ===========================
// --- vis network initialization ---
let container = $("#far-slam-map");

// --- Configuration ---
let options = {
  autoResize: true,
  layout: { hierarchical: { enabled: false } },
  // --- dragView: pan ---
  interaction: { dragNodes: false, selectable: false, selectConnectedEdges: false, navigationButtons: false, hover: false, multiselect: false, dragView: true },
  manipulation: { enabled: false },
};

// --- create a network instance ---
let network = new vis.Network(container[0], { nodes: [], edges: [] }, options);

var mapViewCenterX = 1984 / 2;
var mapViewCenterY = 1984 / 2;
var wRatio = network.canvas.canvasViewCenter.x / mapViewCenterX;
var hRatio = network.canvas.canvasViewCenter.y / mapViewCenterY;
var viewScale = (wRatio > hRatio) ? hRatio : wRatio;

const visCanvas = container[0].querySelector('.vis-network');
visCanvas.style.backgroundColor = "#7F7F7F";

network.moveTo({
  position: { x: mapViewCenterX, y: mapViewCenterY },
  scale: viewScale
});

// network.moveTo({
//   position: {
//     x: 1984 / 2,
//     y: 1984 / 2
//   },
//   scale: (720 / 1984)
// });

network.on("beforeDrawing", function (ctx) {
  drawVisMap(ctx);
  // drawVisSlamAgent(ctx, gMapMeta_, slamPose_, gSprite_);
});

network.on("afterDrawing", function (ctx) {
  // console.log('AAAAAAAAAAAAAAAAAAAA')
  // console.log(gMapMeta_)
  // if (gMapMeta_ == undefined){
  //   return;
  // }
  // if (gMapMeta_.fetched) {
  //     drawVisRobot(ctx);
  // }
});

// ---
let upload_container = $("#far-upload-map");

// --- create a network instance ---
let upload_network = new vis.Network(upload_container[0], { nodes: [], edges: [] }, options);

upload_network.on("beforeDrawing", function (ctx) {
  drawVisLoadMap(ctx);
});

upload_network.on("afterDrawing", function (ctx) {
  var upload_canvas = document.getElementById('upload_map_canvas');
  if (upload_canvas === null) {
    var mapViewCenterX = 1984 / 2;
    var mapViewCenterY = 1984 / 2;
    var wRatio = network.canvas.canvasViewCenter.x / mapViewCenterX;
    var hRatio = network.canvas.canvasViewCenter.y / mapViewCenterY;
    var viewScale = (wRatio > hRatio) ? hRatio : wRatio;

    upload_network.moveTo({
      position: { x: mapViewCenterX, y: mapViewCenterY },
      scale: viewScale
    });

    // upload_network.moveTo({
    //   position: {
    //     x: 1984 / 2,
    //     y: 1984 / 2
    //   },
    //   scale: (720 / 1984)
    // });
  }
});

function drawVisLoadMap(ctx) {
  // --- draw from hidden canvas
  var upload_canvas = document.getElementById('upload_map_canvas');

  if (upload_canvas === null) { return; }

  var width = upload_canvas.width;
  var height = upload_canvas.height;
  // --- [protection] ---
  if (width === 0 || height === 0) { return; }

  ctx.drawImage(upload_canvas, 0, 0, width, height);
}

function drawVisMap(ctx) {
  // --- draw from hidden canvas
  var canvas = document.getElementById('map_canvas');
  // console.log(canvas);
  if (canvas === null) { return; }

  var width = canvas.width;
  var height = canvas.height;
  // --- [protection] ---
  if (width === 0 || height === 0) { return; }

  ctx.drawImage(canvas, 0, 0, width, height);
}


var rotateInputField = document.getElementById("rotateAngle");
rotateInputField.onchange = function () {
  rotateAngle = (this.value) * 1;
}

// =============================
//     CANVAS MANIPULATIONS
// =============================
// --- overview map drawings ---
var image = new Image();
var lastRotateAngle = 0;
var lastWidth = 0;
var lastHeight = 0;
function renderMapOnCanvas(_data) {
  // --- canvas drawing ---
  var canvas = document.getElementById('map_canvas');

  if (canvas === null || rotateAngle !== lastRotateAngle || _data.info.width !== lastWidth || _data.info.height !== lastHeight) {
    // Create new canvas
    canvas = document.createElement('canvas');
    canvas.setAttribute("id", "map_canvas")
    canvas.setAttribute("style", "box-sizing: content-box; border: solid 2px #A9A9A9;");
    canvas.setAttribute("style", "display:none;");
    lastRotateAngle = rotateAngle;
    lastWidth = _data.info.width;
    lastHeight = _data.info.height;

    // Resize the canvas if map has been rotated or the map size has been increased
    if (rotateAngle != 0) {
      let { newWidth, newHeight } = getNewCanvasSize(rotateAngle, lastWidth, lastHeight);
      canvas.width = newWidth;
      canvas.height = newHeight;
      $("#map-width").html(Math.round(newWidth));
      $("#map-height").html(Math.round(newHeight));
    } else {
      canvas.width = lastWidth;
      canvas.height = lastHeight;
    }

    // Clear old canvas and append a new one
    var canDiv = document.getElementById('hidden-map');
    $(canDiv).empty();
    canDiv.appendChild(canvas);
  }

  image.onload = function () {
    let canvas = document.getElementById('map_canvas');
    let context = canvas.getContext('2d');

    context.save();

    // pick the gray(#cdcdcd) color to fill outside the image
    context.rect(0, 0, canvas.width, canvas.height);
    context.fillStyle = '#7F7F7F';
    context.fill();

    // rotate based on canvas center
    context.translate(canvas.width / 2, canvas.height / 2);
    context.rotate(rotateAngle * Math.PI / 180);

    // draw image
    context.drawImage(image, -lastWidth / 2, -lastHeight / 2, lastWidth, lastHeight);

    context.restore();
  };
  image.src = _data.msg;

  network.redraw();
}

function getNewCanvasSize(angle, oldWidth, oldHeight) {
  //
  // (x2,y2)     (x1,y1)
  //     -----------
  //     |         |
  //     |         |
  //     -----------
  // (x3,y3)     (x4,y4)
  //

  let radians = (Math.PI / 180) * angle;
  let cos = Math.cos(radians);
  let sin = Math.sin(radians);

  let newX1 = (oldWidth / 2.0) * cos - (oldHeight / 2.0) * sin;
  let newX2 = (-oldWidth / 2.0) * cos - (oldHeight / 2.0) * sin;
  let newX3 = (-oldWidth / 2.0) * cos - (-oldHeight / 2.0) * sin;
  let newX4 = (oldWidth / 2.0) * cos - (-oldHeight / 2.0) * sin;
  let newY1 = (oldWidth / 2.0) * sin + (oldHeight / 2.0) * cos;
  let newY2 = (-oldWidth / 2.0) * sin + (oldHeight / 2.0) * cos;
  let newY3 = (-oldWidth / 2.0) * sin + (-oldHeight / 2.0) * cos;
  let newY4 = (oldWidth / 2.0) * sin + (-oldHeight / 2.0) * cos;

  // find the proper width and height for the new canvas to cover the rotated image.
  let newWidth = Math.max(newX1, newX2, newX3, newX4) - Math.min(newX1, newX2, newX3, newX4);
  let newHeight = Math.max(newY1, newY2, newY3, newY4) - Math.min(newY1, newY2, newY3, newY4);
  return { newWidth, newHeight };
}

function drawVisSlamAgent(_ctx, _mapDesc, _agentPose, _sprite) {
  // --- [protection] ---
  if (_mapDesc === undefined) { return; }
  if (_agentPose === undefined) { return; }

  var pos = tfROS2Canvas(_mapDesc, { x: _agentPose.x, y: _agentPose.y });

  // --- draw agent as a sprite
  _ctx.save();
  _ctx.translate(pos.x, pos.y);
  _ctx.rotate(-_agentPose.yaw);
  _ctx.drawImage(_sprite, -16, -16, 32, 32);
  _ctx.restore();
}

$('#nav-create-tab').on('click', function () {
  $('#far-upload-map').hide();
  $('#far-slam-map').show();
  $('#far-upload-map').hide()
  $('#slam-submit').show();
  isUpload = false;
})

$('#nav-upload-tab').on('click', function () {
  $('#far-upload-map').show();
  $('#far-slam-map').hide();
  $('#far-upload-map').show()
  $('#slam-submit').hide();
  isUpload = true;
  showSelectedImage();
})

$('#upload_map').on('click', async function () {
  let seleted_agent_id = $('#agent-select').val();
  let map_name = $('#agent-map-upload option:selected').text();
  let upload_res = await fetchUploadAgentMap(rmtToken_, seleted_agent_id, map_name);
  if (upload_res) {
    notificationMsg(1, `${map_name} upload success.`);
  } else {
    notificationMsg(3, `${map_name} upload fail.`);
  }
})

$("#agent-map-upload").on('change', function () {
  showSelectedImage();
});

async function showSelectedImage() {
  if (!isUpload) {
    return;
  }

  let map_name = $('#agent-map-upload option:selected').text();
  let image_bsae64 = '';

  await fetchSelectedMap(select_amr_ip, map_name)
  .then(response => response.json())
  .then(data => {
    console.log(data)
    if (data.rtn_code.code === 0 && data.data !== undefined) {
      image_bsae64 = `data:image/png;base64, ${data.data}`;
    }
  })
  .catch(error => {
    console.error(error);
  });

  var img = new Image();
  img.src = image_bsae64;

  var upload_canDiv = document.getElementById('hidden-upload-map');
  $(upload_canDiv).empty();

  var upload_canvas = document.getElementById('upload_map_canvas');

  img.onload = function () {
    upload_canvas = document.createElement('canvas');
    upload_canvas.setAttribute("id", "upload_map_canvas")
    upload_canvas.setAttribute("style", "box-sizing: content-box; border: solid 2px #A9A9A9;");
    upload_canvas.setAttribute("style", "display:none;");

    upload_canvas.width = img.width;
    upload_canvas.height = img.height;

    upload_canDiv.appendChild(upload_canvas);

    let upload_context = upload_canvas.getContext('2d');

    // draw image
    upload_context.drawImage(img, 0, 0, img.width, img.height);
  };

  upload_network.redraw();
}

async function updateAgentUploadMaps() {
  $('#agent-map-upload').empty();
  $("#agent-map-upload").append(new Option('map loading', 'map_loading'));

  let seleted_agent_id = $('#agent-select').val();
  var res = await fetchRobotMaps(rmtToken_, seleted_agent_id);
  if (res.ok) {
    $('#agent-map-upload').empty();
    let map_info_list = await res.json();
    console.log(map_info_list)
    map_info_list['maps'].forEach(function (map_obj) {
      console.log(map_obj['map_name'])
      $("#agent-map-upload").append(new Option(map_obj['map_name'], map_obj['map_name']));
    });
  } else {
    $('#agent-map-upload').empty();
    $("#agent-map-upload").append(new Option('no map available', 'no_map'));
  }
  $('#nav-upload-tab').show();
}

function getRobotState(robot_mapping_pose) {
  if (robot_mapping_pose === undefined) return;
  console.log(robot_mapping_pose)
  // For Sacne
  robot_yaw = (robot_mapping_pose.yaw);
  robot_x = robot_mapping_pose.pose_x;
  robot_y = robot_mapping_pose.pose_y;
  console.log(robot_x, robot_y, robot_yaw)

  // For Draw robot
  if (gMapMeta_.fetched) {
    var pos = tfROS2Canvas(gMapMeta_, {
      x: robot_x,
      y: robot_y
    });
    robot_x_canvas = pos.x
    robot_y_canvas = pos.y
  }
}

function drawVisRobot(ctx) {
  // 666.6008 453.3980
  ctx.save();
  ctx.translate(robot_x_canvas, robot_y_canvas);
  ctx.rotate(-robot_yaw);
  ctx.drawImage(gSprite_, -16, -16, 32, 32);
  ctx.restore();
}

function showTextInVisCanvas(show_text) {
  var vis_canvas = $('#far-slam-map div canvas')[0];
  var ctx = vis_canvas.getContext("2d");
  // ctx.font = "30px Arial";
  // ctx.fillText(show_text, vis_canvas.width / 2 - 175, vis_canvas.height / 2);

  var textFontSize;
  if (window.innerWidth <= PHONE_MAX_WIDTH || window.outerWidth <= PHONE_MAX_WIDTH) {
    textFontSize = "15px Arial";
  } else {
    textFontSize = "30px Arial";
  }
  ctx.font = textFontSize;

  var textWidth = 0;
  var text_canvas = document.createElement("canvas");
  var text_ctx = text_canvas.getContext("2d");
  text_ctx.font = textFontSize
  textWidth = text_ctx.measureText(show_text).width;

  ctx.fillText(show_text, container.innerWidth() / 2 - (textWidth / 2), container.innerHeight() / 2);
  // if (canvasIsClean){
  //   var vis_canvas = $('#far-slam-map div canvas')[0];
  //   var ctx = vis_canvas.getContext("2d");
  //   ctx.font = "30px Arial";
  //   ctx.fillText(show_text, vis_canvas.width/2 - 175, vis_canvas.height/2);
  //   canvasIsClean = false;
  // }
}

function clearVisCanvas() {
  var vis_canvas = $('#far-slam-map div canvas')[0];
  var ctx = vis_canvas.getContext("2d");
  ctx.clearRect(0, 0, vis_canvas.width, vis_canvas.height);
  // if (!canvasIsClean){
  //   var vis_canvas = $('#far-slam-map div canvas')[0];
  //   var ctx = vis_canvas.getContext("2d");
  //   ctx.clearRect(0, 0, vis_canvas.width, vis_canvas.height);
  //   canvasIsClean = true;
  // }
}

// --- add input values validator ---
function validateMapNameInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();

  const mapNameRule = new Rule('map-filename', textNameValidation, $('#save_map_comfirm'));
  const mapAngleRule = new Rule('rotateAngle', digitValueValidation, $('#slam-submit'));

  validatorManager.addValidator(mapNameRule);
  validatorManager.addValidator(mapAngleRule);
  // console.log(validatorManager);

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
  const targets = document.getElementsByClassName('map-name-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

function initMobileJoystickEvts() {
  if (!isMobile()) return;
  $('a[data-toggle="tab"]').on('shown.bs.tab', function (e) {
    var targetId = $(e.target).attr("id");
    if (window.innerWidth > IPAD_PORTRAIT_WIDTH || window.outerWidth > IPAD_PORTRAIT_WIDTH) return;
    let topJoystick = document.getElementById('header-joystick');
    topJoystick.style.display = targetId === 'nav-upload-tab' ? 'none' : 'block';
  });

  window.addEventListener('resize', initJoystick);
}
