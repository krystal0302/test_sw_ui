/*
 * Author: John Wu
 * Date: 9 Mar 2021
 * Description: 
 *   1. Display swarm system infomation on Main Page
 *   2. Add pop-up sidebar features
 **/

// --- agents asset  ---

const avatar_map_ = {
  bbr: "dist/img/sprites/bbr-440x220.png",
  br: "dist/img/sprites/smr1000_300mm-440x220.png",
  hik600: "dist/img/sprites/hik-440x220.png",
  smr250: "dist/img/sprites/smr250-440x220.png",
  smr250black: "dist/img/sprites/smr250black-440x220.png",
  smr1000: "dist/img/sprites/smr1000-440x220.png",
  mir250: "dist/img/sprites/mir-440x220.png",
  none: "dist/img/sprites/noimage-440x220.png"
}

const mode_map_ = {
  0: "IDLE",
  1: "CHARGING",
  2: "MOVING",
  3: "PAUSED",
  4: "WAITING",
  5: "EMERGENCY",
  6: "GOING_HOME",
  7: "DOCKING",
  8: "UNINITIALIZED",
  404: "DISCONNECTED"
};

// ============================
//   Event Loop for Dashboard 
// ============================
setInterval(function () {
  // console.log(tasks_obj_);
  // if (map_data_.fetched) {
  //   drawMapDataOnUI();
  // } else {
  //   return;
  // }

  // if (tasks_obj_.updated) {
  //   tasks_obj_.updated = false;
  // taskStateOnUI(tasks_obj_);
  // }
  if (tasks_obj_ !== undefined) {
    taskStateOnUI(tasks_obj_);
  }

  // if (fleet_obj_.updated) {
  //   // console.log(fleet_obj_);
  //   fleet_obj_.updated = false;
  //   fleetStateOnUI(fleet_obj_);
  // }
  if (fleet_obj_ !== undefined) {
    fleetStateOnUI(fleet_obj_);
  }

  // if (role_obj_.updated) {
  //   role_obj_.updated = false;
  //   roleStateOnUI(role_obj_);
  // }
  if (role_obj_ !== undefined) {
    roleStateOnUI(role_obj_);
  }

}, 300)

setInterval(() => {
  if (tasks_obj_ !== undefined) {
    subscribe_tasks_state();
  }
}, 1000);

// --- Touring Performance --- 
// setInterval(function () {
//   startPlay();
// }, 100);


var finished_tasks_num_ = 0;
var finished_cargo_ = 0;
// var robot_list = [];
var role_dict = {}; // role:params

// --- Map Service for ROS2 ---
const canvas_width = 720;  // [CFG] frame width
const canvas_height = 600; // [CFG] frame height

// -- the paramenters are supposed for the perspective view(ppv) --
const ppvWidth = 200;   // [CFG]
const ppvHeight = 200;  // [CFG]

var tempTaskMsgArr = [];
var tempFleetMsgArr = [];
var fleet_maps = [];
let rmtToken;

// ======================
//       Load Ready 
// ======================

// ------ Render Widgets ------
let deliveryChart;

let deliveryChartData = {
  labels: [],
  datasets: [
    {
      label: 'Digital Goods',
      fill: true,
      borderWidth: 2,
      lineTension: 0,
      spanGaps: true,
      borderColor: '#00A0FF',
      pointRadius: 3,
      pointHoverRadius: 7,
      pointColor: '#00A0FF',
      pointBackgroundColor: '#00A0FF',
      data: []
    }
  ]
}

let deliveryChartOptions = {
  maintainAspectRatio: false,
  responsive: true,
  legend: {
    display: false,
  },
  scales: {
    xAxes: [{
      ticks: {
        fontColor: '#ffffff',
      },
      gridLines: {
        display: false,
        color: '#ffffff',
        drawBorder: false,
      }
    }],
    yAxes: [{
      ticks: {
        stepSize: 2,
        fontColor: '#ffffff',
        beginAtZero: true,
      },
      gridLines: {
        display: true,
        color: '#ffffff',
        zeroLineColor: '#ffffff',
        drawBorder: false,
      }
    }]
  }
}

$(function () {

  'use strict'
  initDataAsync();

  /* jQueryKnob */
  $('.knob').knob({
    "readOnly": true,
    'fgColor': '#00A0FF'
  });

  // ------ init the overview map ------
  try {
    loadBundledMapDataAsync().then(function () {
      fleet_maps = maps;
      addAgentCards(agents);

      // toggle color theme again after DOM elements are all generated
      var isDarkMode = getSavedTheme() === 'dark';
      toggleContentDarkTheme(isDarkMode);
    });
  } catch (err) {
    console.error("loadBundledMapDataAsync error: " + err);
  }

  // subscribe_tasks_state();
  // Delivery Items graph chart
  var deliveryChartCanvas = $('#delivery-chart').get(0).getContext('2d');

  // This will get the first returned node in the jQuery collection.
  deliveryChart = new Chart(deliveryChartCanvas, {
    type: 'line',
    data: deliveryChartData,
    options: deliveryChartOptions
  })

  var canvas = document.createElement('canvas');
  canvas.setAttribute("id", "map_canvas")
  canvas.setAttribute("style", "box-sizing: content-box; border: solid 3px #A9A9A9;");
  // canvas.addEventListener("mousedown", startPan);

  // fixed canvas size
  // canvas.width = canvas_width;
  // canvas.height = canvas_height;

  var canDiv = document.getElementById('overview-map');
  canDiv.hidden = true;
  canDiv.appendChild(canvas);

  resetVisNetwork(options_, network_);

  // $('#smr-card-0').click(sidebar_cb);
  // $('#smr-card-1').click(sidebar_cb);

})

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'dashboard');
}

// ------ another part ------

// let sb_update;
// var $sidebar = $('.control-sidebar')

// function sidebar_cb() {
//   // ------ start to trigger the sidebar ------
//   console.log("first title text: " + $(this).find('.info-box-number').html());

//   // empty all children nodes
//   $sidebar.empty();

//   // append the content
//   var $container = $('<div />', {
//     class: 'p-3 control-sidebar-content'
//   })

//   $sidebar.append($container)

//   // --- sidebar title ---
//   var selected_one = $(this);
//   var model = $(this).find('.info-box-number').html();
//   var sidebar_model = '<h5>' + model + '</h5>Map: <select class="form-control" id="map-select" onchange="liveSwitchMap();"></select><hr class="mb-2"/>';

//   // var icon = $('#smr-img-0').attr('src');
//   var icon = $(this).find('.card-img-top').attr('src');
//   // console.log('image src: ' + icon);

//   var role = $(this).find('.info-box-text').next().html();
//   // console.log('role: ' + role);

//   var mode = 'IDLE';

//   var pos_x = $(this).find('.smr-pos-x').html();
//   var pos_y = $(this).find('.smr-pos-y').html();
//   console.log("position, x: " + pos_x + ", y: " + pos_y);

//   // var power = $('#smr-battery-0').html();
//   var power = $(this).find('.battery-percent').html();

//   // row1: agent image and ppv
//   var sidebar_row1 = '\
//     <div class="row card-deck">\
//       <!-- agent image -->\
//       <div class="card col-lg-6 p-3" data-widget="control-sidebar" style="justify-content:center;">\
//         <img class="card-img-top" src="'+ icon + '">\
//       </div>\
//       <!-- delimiter -->\
//       <!--<div class="card col-lg-1" data-widget="control-sidebar" style="justify-content:center;visibility:hidden"></div>-->\
//       <!-- agent ppv -->\
//       <div class="card col-lg-6 p-3 nav2dmapCanvas" id="agent-view" data-widget="control-sidebar">\
//       </div>\
//     <div>\
//   ';

//   // row2: agent role, status and position
//   var sidebar_row2 = '\
//     <div class="row card-deck mt-4" style="min-height:15%">\
//       <!-- agent role -->\
//       <div class="card col-lg-4 farobot-view-bg text-center" data-widget="control-sidebar">\
//         <div class="card-header farobot-sidemenu-card-header">Role:</div>\
//         <div class="card-body"><span id="sb-role";"></span></div>\
//       </div>\
//       <!-- agent status -->\
//       <div class="card col-lg-3 farobot-view-bg text-center" data-widget="control-sidebar">\
//         <div class="card-header farobot-sidemenu-card-header">Status:</div>\
//         <div class="card-body"><span id="sb-mode";"></span></div>\
//       </div>\
//       <!-- agent position -->\
//       <div class="card col-lg-5 farobot-view-bg text-center" data-widget="control-sidebar">\
//         <div class="card-header farobot-sidemenu-card-header">Position:</div>\
//         <div class="card-body"><span id="sb-pos";"></span></div>\
//       </div>\
//     <div>\
//   ';

//   // row3: agent executing task 
//   // TODO: trench the message from the the task info
//   var sidebar_row3 = '\
//     <div class="row card-deck mt-4">\
//       <div class="card col-lg-12 col-sm-6 col-12 farobot-view-bg">\
//         <div class="info-box-content">\
//         <div class="card-header farobot-sidemenu-card-header text-center">Task</div>\
//         <marquee direction="left" scrollamount="2" behavior="scroll" id="sb-task-name"> no task yet </marquee>\
//         <span class="info-box-number" id="sb-task-percent">0%</span>\
//         <div class="progress"> <div class="progress-bar" id="sb-task-percent-bar" style="width: 0%"></div> </div>\
//         </div>\
//       </div>\
//     <div>\
//   ';

//   // row4: agent firmware version and power 
//   var sidebar_row4 = '\
//     <div class="row card-deck mt-4" style="min-height:10%">\
//       <div class="card col-lg-6 farobot-view-bg text-center" data-widget="control-sidebar">\
//         <div class="card-header farobot-sidemenu-card-header">Firmware Version: </div>\
//         <div class="card-body"><span>1.0.0</span></div>\
//       </div>\
//       <div class="card col-lg-6 farobot-view-bg text-center" data-widget="control-sidebar">\
//         <div class="card-header farobot-sidemenu-card-header">Battery Percentage: </div>\
//         <div class="card-body"><span>'+ power + '</span></div>\
//       </div>\
//     <div>\
//   ';


//   $container.append(sidebar_model);
//   $container.append(sidebar_row1);
//   $container.append(sidebar_row2);
//   $container.append(sidebar_row3);
//   $container.append(sidebar_row4);


//   // --- perspective view of the map for each agent ---
//   var ppvDiv = document.getElementById('agent-view');
//   var ppv_cvs;
//   // --- portion view of the map for agent 0 ---
//   ppv_cvs = document.createElement('canvas');
//   ppv_cvs.setAttribute("style", "box-sizing: content-box; border: dashed 5px #808080; margin-left:20px");

//   ppv_cvs.width = ppvWidth;
//   ppv_cvs.height = ppvHeight;

//   ppvDiv.appendChild(ppv_cvs);


//   // --- start to render perspective view map ---
//   $('.vis-network').find('canvas').attr('id', 'test');
//   var overview = document.getElementById('test');
//   // var overview = $('.vis-network').find('canvas')
//   // drawPPVMap(canvas, ppv_cvs, agent0_x, agent0_y);
//   // drawPPVMap(overview, ppv_cvs, 250, 250);

//   var x = 250;
//   var y = 250;

//   clearInterval(sb_update);
//   sb_update = setInterval(function () {
//     role = selected_one.find('.info-box-text').next().html();
//     mode = selected_one.find('.info-box-mode').next().html();
//     var pos_xx = selected_one.find('.smr-pos-x').html();
//     var pos_yy = selected_one.find('.smr-pos-y').html();
//     // console.log(pos_xx + ',' +pos_yy);

//     var my_pos = coordTransform({ x: pos_xx, y: pos_yy });
//     var task_name = selected_one.find('.task-name-span').html();
//     var raw_progress = selected_one.find('.progress-bar').css('width');
//     var progress = raw_progress.substr(0, 6) * 100 / 277.5;
//     if (isNaN(progress)) {
//       progress = 0;
//     }

//     $('#sb-role').html(role);
//     $('#sb-mode').html(mode);
//     $('#sb-pos').html('(' + pos_xx + ', ' + pos_yy + ')');
//     $('#sb-task-name').html(task_name);
//     $('#sb-task-percent').html(progress.toFixed(2) + '%');
//     $('#sb-task-percent-bar').css("width", progress + '%');
//     if ($('#map-select > option').length == 0) {
//       fleet_maps.forEach((map) => {
//         $('#map-select').append(`<option value='${map}'>${map}</option>`);
//       });
//       liveSwitchMap();
//     }
//     drawPPVMap(overview, ppv_cvs, pos_xx, pos_yy);
//   }, 100)

// }

// ====================================
//         OPERATION WORKFLOW 
// ====================================
// --- Render the play sequentially --- 
function startPlay() {
  // - protection -
  if (!map_data_.fetched)
    return;

  var canvas = document.getElementById('map_canvas');
  if (canvas === null) { return; }
  var context = canvas.getContext('2d');

  // --- start to render overview map layer by layer ---
  // -- render the map --
  drawMapImg(context);

  // -- render the WMS --
  drawWMS(context, wms_obj_.msg);

  // -- render the Graph --
  drawGraph(context, graph_obj_.msg);

  // -- render the Path -- 
  drawPath(context, path_obj_.msg);

  // -- render the agents --
  drawFleet(context, fleet_obj_.msg);
}



// =============================
//     CANVAS MANIPULATIONS 
// =============================
// --- overview map drawings ---
let mapData = null;
let mapImg = new Image();

function drawMapDataOnUI() {
  // canvas drawing 
  var canvas = document.getElementById('map_canvas');
  var context = canvas.getContext('2d');

  var width = map_data_.w;
  var height = map_data_.h;

  mapData = context.createImageData(width, height);

  for (var row = 0; row < height; row++) {
    for (var col = 0; col < width; col++) {
      // determine the index into the map data
      var mapI = col + ((height - row - 1) * width);
      // determine the value
      var data = map_data_.d[mapI];
      var val;
      if (data === 100) {
        val = 0;
      } else if (data === 0) {
        val = 255 - 30;
      } else {
        val = 127;
      }

      // determine the index into the image data array
      var i = (col + (row * width)) * 4;
      mapData.data[i] = val;   // r
      mapData.data[++i] = val; // g
      mapData.data[++i] = val; // b
      mapData.data[++i] = 255; // a
    }
  }

  context.putImageData(mapData, -X_OFFSET_canvas, -Y_OFFSET_canvas);
  // console.log(mapData);

  mapImg.src = canvas.toDataURL();
}

function drawMapImg(ctx) {
  var width = ctx.canvas.clientWidth;
  var height = ctx.canvas.clientHeight;

  ctx.clearRect(0, 0, width, height);
  ctx.drawImage(mapImg, 0, 0);
}

function drawPPVMap(srcCVS, dstCVS, agent_x, agent_y) {
  // -- find out the portion coordinates. (center, width, height) --
  gMapMeta_.w = gMapData_.w;
  gMapMeta_.h = gMapData_.h;
  var pos = tfROS2Canvas(gMapMeta_, { x: agent_x, y: agent_y });

  var Ox = pos.x;
  var Oy = pos.y;

  network_.moveTo({
    position: { x: Ox, y: Oy },
    scale: (720 / gMapData_.h) * 1.5
  });

  // -- draw it on the canvas --
  if (dstCVS === null) {
    return;
  }

  var dstCTX = dstCVS.getContext('2d');
  // dstCTX.clearRect(0, 0, ppvWidth, ppvHeight);
  dstCTX.drawImage(srcCVS, canvas_width / 2 - ppvWidth / 2, canvas_height / 2 - ppvHeight / 2, ppvWidth, ppvHeight, 0, 0, ppvWidth, ppvHeight);
  // console.log("viewImg Ox: " + Ox + " , Oy: "+ Oy);
  // console.log("vwCanvas width: " + ppvWidth + " , height: "+ ppvHeight);
}


let agent0_x = 0.0;
let agent0_y = 0.0;

let sprite0 = new Image();
// sprite0.src = './dist/img/sprites/br250.png';
// let sprite0_src = './dist/img/sprites/br250.png';
// sprite0.src = './dist/img/sprites/smr1000_300mm-150x150.png';
// let sprite0_src = './dist/img/sprites/smr1000_300mm-440x220.png';
sprite0.src = './dist/img/sprites/smr250-150x150.png';
let sprite0_src = './dist/img/sprites/smr250-440x220.png';

let agent1_x = 0.0;
let agent1_y = 0.0;

let sprite1 = new Image();
// sprite1.src = './dist/img/sprites/mir250.png';
// let sprite1_src = './dist/img/sprites/mir250.png';
// sprite1.src = './dist/img/sprites/smr1000_300mm-150x150.png';
// let sprite1_src = './dist/img/sprites/smr1000_300mm-440x220.png';
sprite1.src = './dist/img/sprites/smr250black-150x150.png';
let sprite1_src = './dist/img/sprites/smr250black-440x220.png';

function drawFleet(ctx, msg) {
  // var cap = "";
  for (i in msg) {
    // console.log(markers[index]);
    // + fleet_obj_.msg[i].location.x.toFixed(2) + ', ' + fleet_obj_.msg[i].location.y.toFixed(2) + ']' 
    var pos = coordTransform({ x: msg[i].location.x, y: msg[i].location.y });

    // update the values of agent coordinates
    if (i == 0) {
      agent0_x = pos.x;
      agent0_y = pos.y;
    } else {
      agent1_x = pos.x;
      agent1_y = pos.y;
    }

    // --- draw agent as a circle 
    // ctx.beginPath();
    // ctx.arc(pos.x, pos.y, 10, 0, 2 * Math.PI);
    // ctx.fillStyle = 'orange'; // agent_color
    // ctx.fill();
    // ctx.stroke();

    // --- draw agent as a sprite 
    // - draw by id -
    // if(i==0){
    //     ctx.drawImage(sprite0, pos.x-32, pos.y-19, 64, 38);
    // }else{
    //     ctx.drawImage(sprite1, pos.x-32, pos.y-19, 64, 38);
    // }

    // - draw by model -
    if (msg[i].model == "BR250") {
      ctx.drawImage(sprite0, pos.x - 32, pos.y - 19, 64, 38);
    } else {
      ctx.drawImage(sprite1, pos.x - 32, pos.y - 19, 64, 38);
    }


    // --- draw initial capital of agent role 
    // if (msg[i].role !== "") {
    //     agent_color = 'red';
    //     cap = msg[i].role.charAt(0).toUpperCase();
    // }

    // cap = msg[i].role[0];
    // console.log("cap: " + msg[i].role);

    // ctx.fillStyle = "#380DFF";
    // ctx.textAlign = "center";
    // ctx.font = "25px Arial";
    // ctx.fillText(cap, pos.x, pos.y);
  }
}

function drawWMS(ctx, msg) {
  var pos;
  for (i in msg) {
    pos = coordTransform({ x: msg[i].pose.position.x, y: msg[i].pose.position.y });

    ctx.beginPath();
    ctx.rect(pos.x - 20, pos.y - 20, 40, 40);
    ctx.fillStyle = RGBToHex(msg[i].color.r, msg[i].color.g, msg[i].color.b);
    ctx.fill();
    ctx.stroke();
  }
}

function drawGraph(ctx, msg) {
  var pos;
  for (i in msg) {
    pos = coordTransform({ x: msg[i].pose.position.x, y: msg[i].pose.position.y });

    if (graph_obj_.msg[i].points.length > 0) {
      // drawEdge(from_pos.x, from_pos.y, to_pos.x, to_pos.y);
      var from_pos = coordTransform({ x: graph_obj_.msg[i].points[0].x, y: graph_obj_.msg[i].points[0].y });
      var to_pos = coordTransform({ x: graph_obj_.msg[i].points[1].x, y: graph_obj_.msg[i].points[1].y });

      // Edge
      var headlen = 12; // length of arrow head in pixels
      var dx = to_pos.x - from_pos.x;
      var dy = to_pos.y - from_pos.y;
      var angle = Math.atan2(dy, dx);

      ctx.beginPath();
      ctx.moveTo(from_pos.x, from_pos.y);
      ctx.lineTo(to_pos.x, to_pos.y);
      ctx.lineTo(to_pos.x - headlen * Math.cos(angle - Math.PI / 8), to_pos.y - headlen * Math.sin(angle - Math.PI / 8));
      ctx.moveTo(to_pos.x, to_pos.y);
      ctx.lineTo(to_pos.x - headlen * Math.cos(angle + Math.PI / 8), to_pos.y - headlen * Math.sin(angle + Math.PI / 8));
      ctx.strokeStyle = "#B20000";
      ctx.stroke();
      ctx.strokeStyle = "black";
    } else {
      // drawVertex(scene_pos.x, scene_pos.y);
      ctx.beginPath();
      ctx.arc(pos.x, pos.y, 5, 0, 2 * Math.PI);
      ctx.fillStyle = "gray";
      ctx.fill();
      ctx.stroke();
    }
  }
}

function drawPath(ctx, msg) {
  var pos;
  for (i in msg) {
    pos = coordTransform({ x: msg[i].pose.position.x, y: msg[i].pose.position.y });

    if (path_obj_.msg[i].points.length > 0) {
      // console.log(path_obj_.msg[i].points);
      var from_pos = coordTransform({ x: path_obj_.msg[i].points[0].x, y: path_obj_.msg[i].points[0].y });
      var to_pos = coordTransform({ x: path_obj_.msg[i].points[1].x, y: path_obj_.msg[i].points[1].y });

      // Edge
      var headlen = 10; // length of head in pixels
      var dx = to_pos.x - from_pos.x;
      var dy = to_pos.y - from_pos.y;
      var angle = Math.atan2(dy, dx);

      ctx.beginPath();
      ctx.moveTo(from_pos.x, from_pos.y);
      ctx.lineTo(to_pos.x, to_pos.y);
      ctx.moveTo(to_pos.x, to_pos.y);
      ctx.strokeStyle = "#66FF00";
      ctx.lineWidth = 5;
      ctx.stroke();
      ctx.strokeStyle = "black";
      ctx.lineWidth = 1;
    }
  }
}



// ==============================
//    RELATED VARIABLES 
// ==============================
tasks_list_ = {}; // declared on ros_comm.js
let chart_labels = [];
let chart_data = [];
const total_tasks = 2; // TODO: load the value from back-end 
let prev_task_num = 0;
let prev_finished_cargo = 0;

let X_OFFSET_canvas = 150; // 400
let Y_OFFSET_canvas = 150; // 350 


// ==============================
//    ROS MESSAGES ON UI 
// ==============================
function taskStateOnUI(_tasksState) {
  // -------- Plan Progress  --------
  // status           : _tasksState.msg[i].status
  // progress rate    : (running / total) * 100%

  // -------- Delivered Items  --------
  // delivered items  : with timestamp

  // -------- Task Schedule Overview --------
  // status           : _tasksState.msg[i].status            [id: #task-status-X]
  // task name          : _tasksState.msg[i].task_name       [id: #task-name-X]
  // complete_percent : _tasksState.msg[i].complete_percent  [id: #task-percent-X]

  let it = 0;
  finished_cargo_ = 0;
  finished_tasks_num_ = 0
  // console.log(_tasksState.msg);

  for (let key in tasks_list_) {
    let value = tasks_list_[key];
    // console.log(key, value);
    if (key.substr(0, 6) == "undock" || key.substr(0, 4) == "home") {
      continue;
    }
    // --- Update on UI ---
    $('#task-status-' + it).removeClass(function (index, className) {
      return (className.match(/(^|\s)bg-\S+/g) || []).join(' ');
    });

    // console.log('tasks_list_[key].state: '+tasks_list_[key].state);
    if (tasks_list_[key].state == 2) {
      $('#task-status-' + it).addClass('bg-success');
      // TODO: reflect real data
      $('#task-percent-' + it).html('100%');
      if (tasks_list_[key].task_name.substr(0, 3) == "tra") {
        finished_cargo_++;
      }
      finished_tasks_num_++;
      // console.log('task finished');
    }
    else {
      $('#task-status-' + it).addClass('bg-warning');
      // TODO: reflect real data
      $('#task-percent-' + it).html('50%');
      // console.log('task executing');
    }

    $('#task-name-' + it).html(tasks_list_[key].task_name);
    // $('#task-percent-'+it).html(tasks_list_[key].complete_percent);
    // $('#smr-task-name-'+it).html(tasks_list_[key].task_name);
    it++;
  }

  if (finished_cargo_ > prev_finished_cargo) {
    var currTime = new Date();
    var datetime = currTime.getHours() + ":" + currTime.getMinutes() + ":" + currTime.getSeconds();

    // console.log("finished_tasks: " + finished_tasks_num_);

    deliveryChart.data.labels.push(datetime);
    deliveryChart.data.datasets[0].data.push(finished_cargo_);

    deliveryChart.update();
    prev_finished_cargo = finished_cargo_;
  }

  // --- Plan Progress Widget ---
  var rate = (finished_tasks_num_ / total_tasks) * 100;
  $('#plan-progress').val(rate).trigger('change');
  // console.log("count finished tasks: " + finished_tasks_num_);
  // console.log(task_state);

}

function subscribe_tasks_state() {
  // if (tasks_obj_ === undefined) return;

  finished_cargo_ = 0;
  finished_tasks_num_ = 0;
  // var taskFleet = tasks_obj_.fleet_name;
  // var tasks = tasks_obj_.tasks;
  var taskFleet = tasks_obj_.fleet;
  var tasks = tasks_obj_.msg;
  // console.log(_message.fleet_name);

  var div = "";

  if (taskFleet !== $('#fleet-select').val() && !$('#no-data-div').length && $('.trigger-div').length == 0) {
    $('#task-schedule-div').append('<div class="card col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>');
  }
  if (taskFleet !== $('#fleet-select').val()) { return };

  tasks = tasks.sort(compare).slice(0, 10);
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
            <button type="button" class="delete-trigger-btn" id='${task_item.task_id}'>×</button>
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

  if (finished_cargo_ > prev_finished_cargo) {
    var currTime = new Date();
    var datetime = currTime.getHours() + ":" + currTime.getMinutes() + ":" + currTime.getSeconds();

    // console.log("finished_tasks: " + finished_tasks_num_);

    deliveryChart.data.labels.push(datetime);
    deliveryChart.data.datasets[0].data.push(finished_cargo_);

    deliveryChart.update();
    prev_finished_cargo = finished_cargo_;
  }

  // --- Plan Progress Widget ---
  var rate = (finished_tasks_num_ / total_tasks) * 100;
  $('#plan-progress').val(rate).trigger('change');

}

$(document).on('click', '.delete-trigger-btn', function (e) {
  send_remove_task("id", this.id);
  $(this).parent().parent().remove();
});

function send_remove_task(type, task) {
  // let remove_task_msg = new ROSLIB.Message({
  //   data: `${type}:${task}`
  // });
  // remove_task_pub.publish(remove_task_msg);

  var jsonRemoveTask = { data: `${type}:${task}` };
  wsRemoveTask(jsonRemoveTask);
}

function compare(a, b) {
  var a_sec = a.start_time.sec + a.start_time.nanosec / 1000000000;
  var b_sec = b.start_time.sec + b.start_time.nanosec / 1000000000;
  if (a_sec < b_sec) {
    return -1;
  }
  if (a_sec > b_sec) {
    return 1;
  }
  return 0;
}

async function switchFleetCallback() {
  resetUI();

  try {
    var selFleet = getSavedFleet();
    var fltSettings = await restGetFleetSettings(selFleet);
    var fltKey = Object.keys(fltSettings)[0];
    fleet_maps = fltSettings[fltKey].maps;
    console.log(fleet_maps);
    addAgentCards(fltSettings[fltKey].agents);
  }
  catch (err) {
    console.error("Get Fleet Settings error: " + err);
  }
}

function resetUI() {
  var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
  $('#robot-util').val(0).trigger('change');
  $('#plan-progress').val(0).trigger('change');
  resetDeliveryChart();
  var $emptyDiv = $('#smr-card-0').detach();
  $('#agent-card-deck-db').empty();
  $('#agent-card-deck-db').append($emptyDiv);
  // $sidebar.empty();
  // $sidebar.hide();
  $('#task-schedule-div').html('');
  $('#task-schedule-div').append(`<div class="card ${cardClass} col-12" id="no-data-div" style="margin: 10px;"><div class="card-body">No task yet.</div></div>`);
  tempTaskMsgArr = [];
  tempFleetMsgArr = [];
}

function resetDeliveryChart() {
  let totalData = deliveryChart.data.labels.length;
  while (totalData >= 0) {
    deliveryChart.data.labels.pop();
    deliveryChart.data.datasets[0].data.pop();
    totalData--;
  }
  deliveryChart.update();
}

function addAgentCards(fleet_agents) {
  var emptyAgentData = {
    model: "",
    robot_id: "",
    role: "",
    location: { x: 0, y: 0 },
    mode: 404,
    battery_percent: "--"
  };

  var cardDeck = document.getElementById('agent-card-deck-db');
  fleet_agents.forEach((agent) => {
    console.log(agent);
    emptyAgentData.robot_id = agent;
    const template = document.querySelector('#agent-card-db');
    const node = document.importNode(template.content, true);

    var cardClass = getSavedTheme() === 'dark' ? 'card-dark' : 'card-light';
    var endtryNode = node.querySelector('.card-entry');
    endtryNode.id = agent;
    endtryNode.classList.add(cardClass);
    // endtryNode.addEventListener('click', sidebar_cb);

    updateAgentCardView(node, emptyAgentData);
    cardDeck.prepend(node);
  });
}

function fleetStateOnUI(_fleetState) {
  if (JSON.stringify(tempFleetMsgArr) !== JSON.stringify(_fleetState.msg)) {
    console.log("========fleet state msg========");
    console.log(_fleetState.msg);
  }

  // -------- Robots Utilization Widget --------
  var workingAgents = _fleetState.msg.filter((agent) => { return agent.mode !== 0; }); // 0: IDLE
  // console.log(workingAgents.length);

  var rate = (workingAgents.length / _fleetState.msg.length) * 100;

  // -------- Fleet Status Overview  --------
  // image        :                                    [id: .smr-card-X > img.src]
  // model        : _fleetState.msg[i].model           [id: .smr-model-X]
  // role         : _fleetState.msg[i].role            [id: .smr-role-X]
  // task         : _fleetState.msg[i].task_name       [id: .smr-task-name-X]
  // battery      : _fleetState.msg[i].battery_percent [id: .smr-battery-X]
  // TODO: task percent needed

  _fleetState.msg.forEach((agent) => {
    // console.log(agent);
    if (agent.fleet_name === $('#fleet-select').val()) {

      $('#robot-util').val(rate).trigger('change');

      var targetEl = document.getElementById(agent.robot_id);
      // console.log(targetEl);

      if (targetEl !== null) {
        updateAgentCardView(targetEl, agent);
      }
    }
  });
  tempFleetMsgArr = _fleetState.msg;


  // -------- pop-up detailed info of a agent --------
  // model        : _fleetState.msg[i].model
  // role         : _fleetState.msg[i].role
  // task         : ref. task_state
  // mode(status) : _fleetState.msg[i].mode
  // position.x   : _fleetState.msg[i].location.x.toFixed(2)
  // position.y   : _fleetState.msg[i].location.y.toFixed(2)
  // firmware ver.: ref. to fixed value temporarily
  // battery      : _fleetState.msg[i].battery_percent

}

function updateAgentCardView(_domNode, _agentData) {

  var imgNode = _domNode.querySelector('.card-img-top');
  var token = _agentData.model.toLowerCase();
  if (typeof avatar_map_[token] === 'undefined') {
    imgNode.src = avatar_map_['none'];
  } else {
    imgNode.src = avatar_map_[token];
  }

  // --- agent name ---
  var roleNode = _domNode.querySelector('.smr-name');
  roleNode.textContent = _agentData.robot_id;

  // --- agent role ---
  if (_agentData.role.substr(0, 3) == "tra") {
    _agentData.role = "transport";
  }
  if (_agentData.role.substr(0, 4) == "home" || _agentData.role.substr(0, 6) == "undock") {
    _agentData.role = ""
  }
  var roleNode = _domNode.querySelector('.smr-role');
  roleNode.textContent = _agentData.role;

  // --- agent location ---
  var smrXNode = _domNode.querySelector('.smr-pos-x');
  smrXNode.textContent = _agentData.location.x.toFixed(2);
  var smrYNode = _domNode.querySelector('.smr-pos-y');
  smrYNode.textContent = _agentData.location.y.toFixed(2);

  var statusNode = _domNode.querySelector('.smr-mode');
  statusNode.textContent = mode_map_[_agentData.mode];

  // // --- agent executing task ---
  // var task_complete_percent = 0;
  // var task_name = "";
  // console.log(tasks_obj_);
  // if (tasks_obj_.msg.hasOwnProperty('length') && tasks_obj_.msg.length > 0) {
  //   for (const t of _tasksState.msg) {
  //     if (t.task_id == _fleetState.msg[i].task_id) {
  //       task_complete_percent = t.complete_percent;
  //       task_name = t.task_name;
  //     }
  //   }
  // }
  // var taskNode = _domNode.querySelector('.smr-task-name');
  // taskNode.textContent = task_name;

  // var taskPercentNode = _domNode.querySelector('.smr-task-percent');
  // taskPercentNode.textContent = task_complete_percent;

  // --- battery percent ---
  var powerNode = _domNode.querySelector('.battery-percent');
  powerNode.textContent = _agentData.battery_percent + '%';

}

function roleStateOnUI(_roleState) {
  // var role_param_list = _roleState.msg.split('&');
  var role_param_list = _roleState.split('&');
  // var role_dict = {}; // role:params move to global
  for (i = 0; i < role_param_list.length; i++) {
    role_dict[role_param_list[i].split("@")[0]] = role_param_list[i].split("@")[1];
  }
  var role_list = Object.keys(role_dict);
  var x = document.getElementById("task_type");
  var option = document.createElement("option");
  var task_type_list = [];
  // for (i = 0; i < x.length; i++) {
  //     task_type_list.push(x.options[i].text);
  // }
  for (var i = 0; i < role_list.length; i++) {
    if (task_type_list.find(element => element == role_list[i]) == undefined) {
      option.text = role_list[i];
      // x.add(option);
    }
  }
}

// ===========================
//         UTILITIES 
// ===========================

function RGBToHex(r, g, b) {

  r = parseInt(r * 255);
  g = parseInt(g * 255);
  b = parseInt(b * 255);

  r = r.toString(16);
  g = g.toString(16);
  b = b.toString(16);

  if (r.length == 1)
    r = "0" + r;
  if (g.length == 1)
    g = "0" + g;
  if (b.length == 1)
    b = "0" + b;

  return "#" + r + g + b;
}

function readTextFile(file) {
  var result = '';
  var rawFile = new XMLHttpRequest();
  rawFile.open("GET", file, false);
  rawFile.onreadystatechange = function () {
    if (rawFile.readyState === 4) {
      if (rawFile.status === 200 || rawFile.status == 0) {
        result = rawFile.responseText;
        //alert(result);
      }
    }
  }
  rawFile.send(null);
  return result;
}

function coordTransform(pos) {
  var x_transformed = (pos.x * SCALE_ - X_OFFSET_ros - X_OFFSET_canvas).toFixed(2);
  var y_transformed = ((map_data_.h - Y_OFFSET_canvas) - (pos.y * SCALE_ - Y_OFFSET_ros)).toFixed(2);
  return { x: x_transformed, y: y_transformed };
}