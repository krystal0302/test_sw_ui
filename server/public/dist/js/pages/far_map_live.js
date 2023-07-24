/*
 * Author: John Wu
 * Date: 24 Mar 2021
 * Description:
 *   1. View and edit map
 *   2. View and edit graph
 *   3. View and edit WMS
 **/

// ============================
//     Global Variables
// ============================
let apiQueryData_ = {};
const dataPullFrequency = 500;

// ==================
//       Models
// ==================
let agentAssets_ = AssetsDict["agentAssets"];

const zoneTypeNames_ = {
  1: 'config',
  2: 'bypass'
};

const zonePriorityNames_ = {
  0: 'normal',
  1: 'high'
};

// ============================
//     Map Global Variables
// ============================
let gMapImg_ = undefined;
let gMapData_ = undefined;
let gMapMeta_ = undefined;
let bCells_ = false;

// ======================
//       Load Ready
// ======================
$(function () {
  'use strict'
  init();
});

let currFleet_ = "";
let currMap_ = "";
let mapAlias_ = "";
let cellAssets_;
let cellDetectionTypes_;
let bakCanvas_;
let zoneQueryData_ = [];

async function init() {
  // ------ authentication ------
  try {
    await rmtTokenCheck();
    await initAPIConfig(rmtToken_);

    let scannedAgents = await faApiGetScanRobot();
  } catch (err) {
    console.error(err);
  }

  // ------ register user activity detector ------
  userActivityDetector();

  // -------- Data Sourcing by Polling --------
  // pollWmsStates(apiQueryData_);
  // pollFleetStates(apiQueryData_, 800);
  // pollPaths(apiQueryData_);
  // pollHikForbiddenArea(apiQueryData_);

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'liveMap');

  // ------ load cell types ------
  var settingsData = await restGetSettings();
  // console.log(settingsData);
  cellDetectionTypes_ = settingsData['cell.detectionTypes'];
  // console.log(cellDetectionTypes_);

  // ---- load map file list ----
  loadBundledMapDataAsync();

  // -------- canvas for map --------
  bakCanvas_ = document.createElement('canvas');
  bakCanvas_.setAttribute("style", "box-sizing: content-box; border: solid 2px #A9A9A9;");

  // resetVisNetwork(options_, network_);
  // function test(){
  //   console.log(gMapMeta_)
  //   if (gMapMeta_ !== undefined){
  //     network_.redraw();
  //   }
  //   window.requestAnimationFrame(test());
  // }

  // ------ language switch ------
  await initLanguageSupport();
  // test();
  draw();
}

// redraw
let init_fetch_api_data = false;

var fps = 1;
var now;
var then = Date.now();
var interval = 1000 / fps;
var delta;

function draw() {

  now = Date.now();
  delta = now - then;

  if (delta > interval) {
    // update time stuffs
    then = now - (delta % interval);

    if (currMap_.length != 0 && !init_fetch_api_data) {
      init_fetch_api_data = true;
      initFetchMapRelateData();
    }

    if (gMapMeta_ != undefined) {
      network_.redraw();
    }
  }

  requestAnimationFrame(draw);
}

// update api data
function initFetchMapRelateData() {
  pollInternalLiveView(dataPullFrequency);
  // pollWmsStates2(500);
  // pollFleetStates2(500);
  // pollPaths2(500);
  // pollHikForbiddenArea2(500);
}


// =======================
//     Animation Ticks
// =======================
setInterval(function () {
  // if (fleet_fleet_obj_ !== undefined) {
  //   updateFleetSprites(fleet_fleet_obj_);
  // }
  // if (queryData_ !== undefined && queryData_.hasOwnProperty('agentListView')) {
  //   updateFleetSprites2(queryData_.agentListView);
  // }
  // if (apiQueryData_ !== undefined && apiQueryData_.hasOwnProperty('fleet_state')) {
  //   updateFleetSprites2(apiQueryData_.fleet_state);
  // }

  // if (wms_obj_ !== undefined) {
  //   updateVisCellStatus(wms_obj_.msg);
  // }
  // if (apiQueryData_ !== undefined && apiQueryData_.hasOwnProperty('wms')) {
  //   updateVisCellStatus2(apiQueryData_.wms);
  // }

  if (apiQueryData_ !== undefined) {
    updateFleetSprites2(apiQueryData_);
    updateVisCellStatus2(apiQueryData_);
  }


}, 450)

function pollInternalLiveView(_interval = 1000) {
  faApiGetInternalLiveMapData(currFleet_, currMap_)
  .then(function (response) {
    // console.log(response);
    apiQueryData_ = response.data;
  })
  .catch(function (error) {
    console.log(error);
  })
  .finally(function () {
    // always executed
    setTimeout(function () {
      pollInternalLiveView(_interval);
    }, _interval);
  })
}


async function rmtTokenCheck() {
  if (rmtToken_ === undefined) {
    try {
      rmtToken_ = await fetchToken();
      notificationMsg(1, 'RMT ONLINE!');
    } catch (err) {
      console.error(err);
      notificationMsg(3, 'RMT OFFLINE!');
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}


// Set select options or set `not available` msg.
async function setSelectAndMsg() {
  // -- fetch available fleet configurations --
  var selFleet = getSelectedFleet();
  currFleet_ = selFleet;

  // -- fetch fleet_settings --
  var fltSettings = await restGetFleetSettings(selFleet);

  // -- load the map in the fleet --
  var fltKey = Object.keys(fltSettings)[0];
  var maps = fltSettings[fltKey].maps;
  console.log(maps);

  // --- generate map alias list ---
  const insMaps = _.intersectionWith(mapAlias_, maps,
    (o, mapname) => o.name == mapname);
  console.log(insMaps);
  for (let map of insMaps) {
    $('#map-select').append(`<option value='${map.name}'>${map.alias_name}</option>`);
  }
  let map_section = document.getElementById('map_section');
  let emptyDiv = document.getElementById('no-map-caption');
  let far_net = document.getElementById('far-network');
  let liveview = document.getElementById('liveview');
  if (0 === insMaps.length) {
    let scaleFontSize = getFontScaleSize(55);
    emptyDiv.style.cssText =
      `text-align: center; font-size: ${scaleFontSize}px; border: 2px; color: white; height: 20px;`;
    emptyDiv.innerText = 'No available map in fleet';
    map_section.style.display = 'none';
    liveview.style.display = 'none';
    far_net.style.visibility = 'hidden';
  } else {
    emptyDiv.innerText = '';
    map_section.style.display = 'block';
    liveview.style.display = 'block';
    far_net.style.visibility = 'visible';
  }
  $('#liveview > i').attr('class', 'fas fa-eye-slash');
  $('#liveview > i').attr('style', 'color:rgba(128, 128, 128, 0.6)');
}

async function loadBundledMapDataAsync() {
  // --- generate map alias list ---
  var data = await restGetAllMapData();
  var objArray = JSON.parse(data);
  mapAlias_ = objArray;

  await setSelectAndMsg();

  // -- fetch the graph and WMS content --
  // --- select first map option ---
  var el = document.getElementById('map-select');
  if (el.options.length === 0) return;
  var mapName = el.options[el.selectedIndex].value;
  console.log(`========map: ${mapName}========`);
  currMap_ = mapName;

  // --- update map bundled data async ---
  updateMapAsync(mapName);
}

// ============================
//     Level 1 functions
// ============================
async function updateMapAsync(_fn) {
  updateMapNameHintText(_fn);
  // [protection] if map name is null, clean all.
  if (_fn === null) {
    return;
  }

  // --- update map image data ---
  var mapPNG = _fn + '.png';
  gMapData_ = await restGetMapImg(mapPNG);
  gMapImg_ = await globalMapImageBackup(gMapData_);
  console.log(gMapData_);
  gMapMeta_ = { w: gMapData_.w, h: gMapData_.h };

  var mapViewCenterX = gMapData_.w / 2;
  var mapViewCenterY = gMapData_.h / 2;
  var wRatio = network_.canvas.canvasViewCenter.x / mapViewCenterX;
  var hRatio = network_.canvas.canvasViewCenter.y / mapViewCenterY;
  var viewScale = (wRatio > hRatio) ? hRatio : wRatio;

  network_.moveTo({
    position: { x: mapViewCenterX, y: mapViewCenterY },
    scale: viewScale
  });

  // --- update map meta data ---
  console.log(gMapMeta_);
  var data = await restGetMapMetaInfo(_fn);

  data = data.split(/\r?\n/);
  // --- update gMapMeta_ ---
  for (i in data) {
    var el = data[i].split(':');
    var key = el[0];
    if (key === 'image') {
      gMapMeta_[key] = el[1].trim();
      continue;
    }
    if (key === 'resolution') {
      gMapMeta_[key] = Number(el[1]);
      continue;
    }
    if (key === 'origin') {
      console.log(`el[1]: ${el[1]}`);
      el[1] = el[1].replace(/[\[\]']+/g, '');
      var arrPos = el[1].split(',');

      var coord = {};
      coord['x'] = Number(arrPos[0]);
      coord['y'] = Number(arrPos[1]);
      coord['z'] = Number(arrPos[2]);

      gMapMeta_[key] = coord;

      continue;
    }
    if (key === 'occupied_thresh') {
      gMapMeta_[key] = Number(el[1]);
      continue;
    }
    if (key === 'nav_graph') {
      gMapMeta_[key] = el[1].trim();
      continue;
    }
    if (key === 'cell') {
      gMapMeta_[key] = el[1].trim();
      continue;
    }
    if (key === 'triton') {
      gMapMeta_[key] = el[1].trim();
      continue;
    }
  }

  if (!gMapMeta_.hasOwnProperty('nav_graph')) {
    var filename = gMapMeta_.image.split('.').slice(0, -1).join('.');
    gMapMeta_['nav_graph'] = filename + '.dot';

  }
  if (!gMapMeta_.hasOwnProperty('cell')) {
    var filename = gMapMeta_.image.split('.').slice(0, -1).join('.');
    gMapMeta_['cell'] = filename + '.json';

  }
  if (!gMapMeta_.hasOwnProperty('triton')) {
    var filename = gMapMeta_.image.split('.').slice(0, -1).join('.');
    gMapMeta_['triton'] = filename + '.amf';

  }

  var cellData = await restGetMapCells(gMapMeta_.cell);
  bCells_ = false;
  jsonWMS2_ = JSON.parse(cellData);
  visDrawCells();

  var graphData = await restGetMapGraph(gMapMeta_.nav_graph);
  bNavGraph_ = false;
  strDOT2_ = graphData;
  visDrawGraph();

  var data = await fetchGetZoneConfig(rmtToken_, _fn);
  data = await data.json();
  zoneQueryData_ = data;
}

function updateMapNameHintText(_name = null) {
  // $('#map-file-name').text(_name === null ? '' : 'file name: ' + _name);
  $('#map-file-name').text(_name === null ? '' : _name);
}

// ============================
//     Level 2 functions
// ============================
// --- updateMapImageAsync ---
function globalMapImageBackup(_mapData) {
  var mapDataURL = `data:image/png;base64,${_mapData.data}`;
  var mapImg = new Image();
  mapImg.onload = function () {
    loadMapImageView(_mapData);
  };
  mapImg.src = mapDataURL;
  // console.log(mapImg)
  return mapImg;
}

function loadMapImageView(_data) {
  bakCanvas_.width = _data.w;
  bakCanvas_.height = _data.h;
  var ctx = bakCanvas_.getContext('2d');
  ctx.drawImage(gMapImg_, 0, 0);
}

// -- draw storage cells from json file --
let jsonWMS2_ = {};
let wmsCellStatusAssets_ = {};

function visDrawCells() {
  // -- [protection] confirm the data is updated --
  if (bCells_) return;
  bCells_ = true;

  // --- read the data from WMS json file ---
  var cellNodes = [];
  for (key in jsonWMS2_) {
    var area = jsonWMS2_[key];

    for (let i = 0; i < area.length; i++) {
      var cell_id = area[i].cell_id;
      // var status = area[i].if_occupied;
      var x = area[i].cell_coordinate[0];
      var y = area[i].cell_coordinate[1];
      var pos = tfROS2Canvas(gMapMeta_, { x: x, y: y });

      var ang = -cvtRad2Deg(area[i].cell_coordinate[2]).toFixed(2);
      var svgHtml =
        `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
          <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#00ff00" fill-opacity="0.4"/>
         </svg>`;
      var svgUrl = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtml);

      var status = {};
      status['empty'] = svgUrl

      var svgHtmlOccupied =
        `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
          <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#ff0000" fill-opacity="0.4"/>
         </svg>`;
      var svgUrlOccupied = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtmlOccupied);
      status['occupied'] = svgUrlOccupied

      var svgHtmlLoaded =
        `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
          <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#625B57" fill-opacity="0.4"/>
         </svg>`;
      var svgUrlLoaded = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtmlLoaded);
      status['loaded'] = svgUrlLoaded

      var svgHtmlUnknown =
        `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
          <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#ffff00" fill-opacity="0.4"/>
         </svg>`;
      var svgUrlUnknown = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtmlUnknown);
      status['unknown'] = svgUrlUnknown

      wmsCellStatusAssets_[cell_id] = status


      console.log(svgUrl);

      var wmsNode = {
        title: `<div style="color: ${getColor(cellDetectionTypes_, area[i].type, 'tooltipColor')}">${cell_id}</div>`,
        id: cell_id,
        x: pos.x,
        y: pos.y,
        color: {
          background: getColor(cellDetectionTypes_, area[i].type, 'bgColor'),
          border: getColor(cellDetectionTypes_, area[i].type, 'borderColor')
        }
      };
      wmsNode.label = cell_id.slice(0, 5) + (cell_id.length > 5 ? ".." : "");
      wmsNode.shape = 'circle';
      wmsNode.widthConstraint = 25;
      wmsNode.borderWidth = 2;
      // wmsNode.shape = 'image';
      // wmsNode.image = svgUrl;
      // wmsNode.size = 20; // default: 25
      wmsNode.font = {
        size: 7,
        color: getColor(cellDetectionTypes_, area[i].type, 'color'),
        bold: true
      };
      wmsNode.group = 'wms';
      wmsNode.area = key;
      // network_.body.data.nodes.update(wmsNode);
      cellNodes.push(wmsNode);
    }
  }
  network_.body.data.nodes.update(cellNodes);
  // console.log("node number: "+network.body.data.nodes.length);
};

let strDOT2_ = "";
function visDrawGraph() {
  // -- [protection] confirm the data is updated --
  if (bNavGraph_) return;
  bNavGraph_ = true;

  var graph = cvtDot2Json(strDOT2_);
  // var txtArea = document.getElementById('ta-graph');

  var graphNodes = [];
  for (i in graph.nodes) {
    var pos = tfROS2Canvas(gMapMeta_, { x: graph.nodes[i].x, y: graph.nodes[i].y });
    graph.nodes[i].x = pos.x;
    graph.nodes[i].y = pos.y;
    graph.nodes[i].group = 'navnode';

    // network_.body.data.nodes.update(graph.nodes[i]);
    graphNodes.push(graph.nodes[i]);
  }
  network_.body.data.nodes.update(graphNodes);

  // --- render edges ---
  // var graphEdges = []
  // for (i in graph.edges) {
  //   // network_.body.data.edges.update(graph.edges[i]);
  //   graphEdges.push(graph.edges[i]);
  // }
  // network_.body.data.edges.update(graphEdges);
  network_.body.data.edges.update(graph.edges);

  var nodes = objectToArray(network_.getPositions());
  nodes.forEach(addConnections);
};

// ========================
//   Vis-Network Data I/O
// ========================
var currJsonGraph = {};
// ------ export network ------
function addConnections(elem, index) {
  elem.connections = network_.getConnectedNodes(elem.id);
}

function objectToArray(obj) {
  // console.log(obj);
  return Object.keys(obj).map(function (key) {
    obj[key].id = key;
    return obj[key];
  });
}


// ===========================
//     VIS-NETWORK DRAWING
// ===========================
// --- network initialization ---
let nodes = new vis.DataSet([]);
let edges = new vis.DataSet([]);
let container_ = $("#far-network");

// --- Configuration ---
let options_ = {
  autoResize: true,
  layout: { hierarchical: { enabled: false } },
  physics: { enabled: false },
  nodes: {
    shape: 'dot',
    // color: 'gray',
    size: 5,
    font: {
      size: 12,
      color: 'rgba(255,80,0,0)' // hide the label of vertices
    },
    fixed: {
      x: true,
      y: true,
    }
  },
  edges: {
    // color: "red",
    color: "rgba(87, 87, 87, 0.9)",
    value: 1,
    scaling: { min: 1, max: 2 },
    shadow: true,
    dashes: true,
    smooth: { enabled: true },
    arrows: {
      to: { enabled: true, scaleFactor: 0.3, type: "arrow" }
    },
    background: { enabled: true, color: "white" }
  },
  // dragView: pan function
  interaction: { dragNodes: false, selectable: false, selectConnectedEdges: false, navigationButtons: true, hover: false, multiselect: false, dragView: true },
  manipulation: { enabled: false },
  groups: {
    navnode: {
      color: { background: "gray", border: "black" }
    },
    wms: {
      color: { background: "gray", border: "black" }
    }
  }
};


// --- create a network instance ---
let network_ = new vis.Network(container_[0], { nodes: nodes, edges: edges }, options_);
let canvas_ = null;

network_.on("beforeDrawing", function (ctx) {
  if (bakCanvas_ === undefined) { return; }
  var w = bakCanvas_.width;
  var h = bakCanvas_.height;
  // --- clean previous drawing ---
  ctx.clearRect(0, 0, w, h);
  ctx.drawImage(bakCanvas_, 0, 0, w, h);
});


// let forbidden_area_obj2_ = {
//   "STK2": { "points": [[14.517, -2.633974552154541], [14.517, 3.8767], [17.75243377685547, 3.8767], [17.75243377685547, -2.633974552154541]], "unlock": [] },
//   "TWIST": { "points": [[35.45661544799805, -12.430886268615723], [35.45661544799805, -8.54386043548584], [41.65913391113281, -8.54386043548584], [41.65913391113281, -12.430886268615723]], "unlock": ['fb_3'] },
// };
network_.on("afterDrawing", function (ctx) {
  // TODO: confirm with multiple maps and graphs
  if (gMapMeta_ === {} || gMapMeta_ === undefined) return; // when gMapMeta_ is not ready, no transformation

  // ======== Markers ========
  // --- DataSource: WebSocket (Red) ---
  // if (forbidden_area_obj_ !== undefined) {
  //   drawVisForbiddenZone(ctx, forbidden_area_obj_);
  // }

  // --- DataSource: Swarm API (Orange) ---
  // if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('forbidden_area')) {
  //   drawVisForbiddenZone(ctx, apiQueryData_.forbidden_area);
  // }

  // --- DataSource: WebSocket (Red) ---
  // if (path_obj_ !== undefined && path_obj_.msg !== undefined) {
  //   drawVisPath(ctx, path_obj_.msg);
  // }

  // --- DataSource: Swarm API (Orange) ---
  // if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('paths')) {
  //   drawVisPath2(ctx, apiQueryData_.paths);
  // }

  // ======== Fleet State ========
  // --- DataSource: WebSocket (Red) ---
  // console.log(et_fleet_obj_);
  // if ((fleet_fleet_obj_ !== undefined) && (currFleet_ in fleet_fleet_obj_)) {
  //   drawVisFleet(ctx, fleet_fleet_obj_[currFleet_]);
  // }

  // --- DataSource: Logging Core DB (Blue) ---
  // console.log(queryData_ );
  // if ((queryData_ !== undefined) && queryData_.hasOwnProperty('agentListView')) {
  //   drawVisFleet2(ctx, queryData_.agentListView);
  // }

  // --- DataSource: Swarm API (Orange) ---
  // if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('fleet_state')) {
  //   drawVisFleet3(ctx, apiQueryData_.fleet_state);
  // }

  // if ((apiQueryData_ !== undefined) && apiQueryData_.hasOwnProperty('zone')) {
  //   drawVisFunctionalZone(ctx, apiQueryData_.zone);
  // }


  if (apiQueryData_ !== undefined) {
    drawVisForbiddenZone(ctx, apiQueryData_);
    drawVisPath2(ctx, apiQueryData_);
    drawVisFleet3(ctx, apiQueryData_);
  }

  drawVisFunctionalZone(ctx, zoneQueryData_);

});

// --- update WMS status ---
const wmsStyles_ = {
  // 'occupied': { border: 'rgba(255,0,0,0.4)' },
  'rack': { border: 'rgba(255,0,0,0.4)' },
  'empty': { border: 'rgba(0,255,0,0.4)' },
  'loaded': { border: 'rgba(98,91,87,0.4)' }
};
function updateVisCellStatus2(_msgs) {
  if (!_msgs.hasOwnProperty('cells')) return;
  _msgs = _msgs.cells;

  $('#liveview > i').attr('class', 'fas fa-eye');
  if (wmsCellStatusAssets_ === {}) {
    $('#liveview > i').attr('style', 'color:rgba(128, 128, 128, 0.6)');
    return;
  }

  var cellNodes = [];
  for (i in _msgs) {
    if (_msgs[i].map !== currMap_) continue;
    if (!wmsCellStatusAssets_.hasOwnProperty(_msgs[i].cell_id)) { continue; }

    var wmsNode = {};
    wmsNode.id = _msgs[i].cell_id; // cell-id
    wmsNode.shape = 'circle';
    wmsNode.size = 20; // default: 25
    wmsNode.color = wmsStyles_[_msgs[i].load] || { border: 'rgba(255,255,0,0.4)' };

    cellNodes.push(wmsNode);
  }

  network_.body.data.nodes.update(cellNodes);
  $('#liveview > i').attr('style', 'color:rgba(0, 255, 0, 0.6)');
};

function updateVisCellStatus(_msgs) {
  // console.log(_msgs);
  // console.log(wmsCellStatusAssets_);
  if (wmsCellStatusAssets_ === {}) return;

  var cellNodes = [];
  for (i in _msgs) {

    if (!wmsCellStatusAssets_.hasOwnProperty(_msgs[i].text)) continue;

    var wmsNode = {};
    wmsNode.id = _msgs[i].text; // cell-id

    // console.log(_msgs[i].text);
    var color;
    if (Math.abs(_msgs[i].color.r - 0.3) < 0.01 && _msgs[i].color.g === 0 && _msgs[i].color.b === 0) {
      color = {
        border: 'rgba(255,0,0,0.4)'
      };
    }
    else if (_msgs[i].color.r === 0 && Math.abs(_msgs[i].color.g - 0.3) < 0.01 && _msgs[i].color.b === 0) {
      color = {
        border: 'rgba(0,255,0,0.4)'
      };
    }
    else if (Math.abs(_msgs[i].color.r - 1.0) > 0.01 && Math.abs(_msgs[i].color.g - 1.0) < 0.01 && Math.abs(_msgs[i].color.b - 1.0) < 0.01) {
      color = {
        border: 'rgba(98,91,87,0.4)'
      };
    }
    else {
      color = {
        border: 'rgba(255,255,0,0.4)'
      };
    }
    // console.log(status);

    // wmsNode.shape = 'image';
    // wmsNode.image = wmsCellStatusAssets_[_msgs[i].text][status];
    wmsNode.shape = 'circle';
    wmsNode.size = 20; // default: 25
    // wmsNode.color = 'rgba(4, 75, 148, 0.4)'
    wmsNode.color = color

    cellNodes.push(wmsNode);
  }

  network_.body.data.nodes.update(cellNodes);
};


function drawVisPath(ctx, _msg) {
  for (i in _msg) {
    // console.log(path_obj_.msg[i].points);
    if (_msg[i].points.length === 0) continue;

    // --- draw chasing route ---
    var from_pos = tfROS2Canvas(gMapMeta_, { x: _msg[i].points[0].x, y: _msg[i].points[0].y });
    var to_pos = tfROS2Canvas(gMapMeta_, { x: _msg[i].points[1].x, y: _msg[i].points[1].y });

    // Edge
    ctx.beginPath();
    ctx.moveTo(from_pos.x, from_pos.y);
    ctx.lineTo(to_pos.x, to_pos.y);
    ctx.moveTo(to_pos.x, to_pos.y);
    ctx.strokeStyle = "#00A0FF";
    ctx.lineWidth = 5;
    ctx.stroke();
    ctx.strokeStyle = "black";
    ctx.lineWidth = 1;
  }
}

function drawVisPath2(ctx, _msg) {
  if (!_msg.hasOwnProperty('markers')) return;
  _msg = _msg.markers;
  // console.log(_msg);
  for (i in _msg) {
    // console.log(path_obj_.msg[i].points);
    if (_msg[i].points.length === 0) { continue; }

    for (let j = 0; j < _msg[i].points.length - 1; j++) {
      // --- draw chasing route ---
      var from_pos = tfROS2Canvas(gMapMeta_, { x: _msg[i].points[j].x, y: _msg[i].points[j].y });
      var to_pos = tfROS2Canvas(gMapMeta_, { x: _msg[i].points[j + 1].x, y: _msg[i].points[j + 1].y });

      // Edge
      ctx.beginPath();
      ctx.moveTo(from_pos.x, from_pos.y);
      ctx.lineTo(to_pos.x, to_pos.y);
      ctx.moveTo(to_pos.x, to_pos.y);
      ctx.strokeStyle = "#00A0FF";
      ctx.lineWidth = 5;
      ctx.stroke();
      ctx.strokeStyle = "black";
      ctx.lineWidth = 1;
    }
  }
}

// ex. agentSprites={robotid: image}
let agentSprites_ = {};
let agentIdTag_ = {};

function updateFleetSprites(_obj) {
  // --- protections ---
  if (currFleet_ === "") return;
  if (_obj === undefined) {
    console.log('No fleet data received !!');
    return;
  }
  if (_obj[currFleet_] === undefined) {
    console.log('fleet name is not matched');
    return;
  }

  // --- start to work ---
  // console.log(_obj[currFleet_]);
  var msg = _obj[currFleet_].robots;
  console.log(msg);

  // --- update the sprites from the fleet fleet obj ---
  for (i in msg) {
    if (agentSprites_.hasOwnProperty(msg[i].robot_id)) continue;

    var sprite = new Image();
    var model = msg[i].model.toLowerCase();
    sprite.src = agentAssets_[model].topview; // avatar
    agentSprites_[msg[i].robot_id] = sprite;

    var tImg = new Image();
    var tCanvas = document.createElement('canvas')
    var robotId = msg[i].robot_id;

    tCanvas.height = 32;
    tCanvas.width = 16 * (robotId.length);

    var tCtx = tCanvas.getContext('2d');
    tCtx.font = "24px Arial"; // CRITICAL: define font size first.
    tCtx.fillText(robotId, 0, 24);
    tImg.src = tCanvas.toDataURL();
    agentIdTag_[msg[i].robot_id] = tImg;
  }

}

function updateFleetSprites2(_obj) {
  // console.log(_obj)
  // --- protections ---
  if (!_obj.hasOwnProperty('fleet_state') || _obj.fleet_state === undefined) {
    console.log('No fleet data received !!');
    return;
  }

  // --- start to work ---
  var msg = _obj.fleet_state[0].robots;
  msg = msg.filter(robot => robot.connection_status == 0);

  // --- update the sprites from the fleet fleet obj ---
  for (i in msg) {
    if (agentSprites_.hasOwnProperty(msg[i].robot_id)) continue;

    var sprite = new Image();
    var model = msg[i].model.toLowerCase() || 'smr250';
    sprite.src = agentAssets_[model].topview; // avatar
    agentSprites_[msg[i].robot_id] = sprite;

    var tImg = new Image();
    var tCanvas = document.createElement('canvas')
    var robotName = msg[i].robot_name;

    tCanvas.height = 32;
    tCanvas.width = 16 * (robotName.length);

    var tCtx = tCanvas.getContext('2d');
    tCtx.font = "24px Arial"; // CRITICAL: define font size first.
    tCtx.fillText(robotName, 0, 24);
    tImg.src = tCanvas.toDataURL();
    agentIdTag_[msg[i].robot_id] = tImg;
  }
}

function drawVisFleet(_ctx, _obj) {
  // --- protection ---
  if (agentSprites_ === {}) return;

  // var sprite;
  // console.log(_obj);
  var msg = _obj.robots;
  // console.log(msg);

  for (i in msg) {
    // [debugging]
    // console.log(`curr fleet: ${currFleet_}, curr map: ${currMap_}`)
    // console.log(`fleet compare: ${msg[i].fleet_name !== currFleet_}`)
    // console.log(`map compare:   ${msg[i].map !== currMap_}`)

    // [protection] agents show when fleet and map names are matched.
    if (msg[i].map !== currMap_) {
      // console.log('map is not matched');
      return;
    }

    var pos = tfROS2Canvas(gMapMeta_, { x: Number(msg[i].location.x), y: Number(msg[i].location.y) });
    if (Object.keys(pos).length === 0 && pos.constructor === Object) {
      console.warn("map meta-data is not loaded yet!");
      return;
    }
    // console.log(pos);

    // --- draw agent as a sprite
    if (!msg[i].hasOwnProperty('robot_id')) { continue; }
    let agentId = msg[i].robot_id;
    if (agentId === undefined) { continue; }
    // console.log(agentId);
    var sprite = agentSprites_[agentId];
    if (sprite === undefined) { continue; }
    // console.log(sprite);
    var tag = agentIdTag_[agentId];
    if (tag === undefined) { continue; }
    // console.log(sprite);

    // --- draw agent as a sprite
    _ctx.save();
    _ctx.translate(Number(pos.x), Number(pos.y));
    _ctx.rotate(- msg[i].location.yaw);
    _ctx.drawImage(sprite, -16, -16, 32, 32);
    if (bAgentTagSwitch) {
      _ctx.drawImage(tag, -10, -30, 16 * (tag.width / tag.height), 16);
    }
    _ctx.restore();

  }
}

function drawVisFleet3(_ctx, _obj) {
  // --- protection ---
  // console.log(_obj);
  if (isEmpty(agentSprites_) || !_obj.hasOwnProperty('fleet_state')) return;

  let msg = _obj.fleet_state[0].robots;
  msg = msg.filter(robot => robot.connection_status == 0);
  // console.log(msg);

  for (i in msg) {
    // [protection] agents show when fleet and map names are matched.
    if (msg[i].map !== currMap_) {
      // console.log('map is not matched');
      continue;
    }

    var pos = tfROS2Canvas(gMapMeta_, { x: Number(msg[i].location.x), y: Number(msg[i].location.y) });
    if (Object.keys(pos).length === 0 && pos.constructor === Object) {
      console.warn("map meta-data is not loaded yet!");
      continue;
    }
    // console.log(pos);

    // --- draw agent as a sprite
    if (!msg[i].hasOwnProperty('robot_id')) { continue; }
    let agentId = msg[i].robot_id;
    if (agentId === undefined) { continue; }
    // console.log(agentId);
    var sprite = agentSprites_[agentId];
    if (sprite === undefined) { continue; }
    // console.log(sprite);
    var tag = agentIdTag_[agentId];
    if (tag === undefined) { continue; }
    // console.log(sprite);

    // --- draw agent as a sprite
    _ctx.save();
    _ctx.translate(Math.round(Number(pos.x)), Math.round(Number(pos.y)));
    _ctx.rotate(- msg[i].location.yaw);
    _ctx.drawImage(sprite, -16, -16, 32, 32);
    if (bAgentTagSwitch) {
      _ctx.drawImage(tag, -10, -30, 16 * (tag.width / tag.height), 16);
    }
    _ctx.restore();

  }
}

function drawVisFleet2(_ctx, _obj) {
  // --- protection ---
  console.log(_obj);
  if (agentSprites_ === {}) return;

  var msg = _obj;
  console.log(msg);
  console.log(agentSprites_);

  for (i in msg) {
    // [debugging]
    // console.log(`curr fleet: ${currFleet_}, curr map: ${currMap_}`)
    // console.log(`fleet compare: ${msg[i].fleet_name !== currFleet_}`)
    // console.log(`map compare:   ${msg[i].map !== currMap_}`)

    // [protection] agents show when fleet and map names are matched.
    if (msg[i].map !== currMap_) {
      // console.log('map is not matched');
      return;
    }

    var pos = tfROS2Canvas(gMapMeta_, { x: Number(msg[i].pose_x), y: Number(msg[i].pose_y) });
    if (Object.keys(pos).length === 0 && pos.constructor === Object) {
      console.warn("map meta-data is not loaded yet!");
      return;
    }
    // console.log(pos);

    // --- draw agent as a sprite
    if (!msg[i].hasOwnProperty('robot_id')) { continue; }
    let agentId = msg[i].robot_id;
    if (agentId === undefined) { continue; }
    // console.log(agentId);
    var sprite = agentSprites_[agentId];
    if (sprite === undefined) { continue; }
    // console.log(sprite);
    var tag = agentIdTag_[agentId];
    if (tag === undefined) { continue; }
    // console.log(sprite);

    // --- draw agent as a sprite
    _ctx.save();
    _ctx.translate(Number(pos.x), Number(pos.y));
    _ctx.rotate(- msg[i].pose_a);
    _ctx.drawImage(sprite, -16, -16, 32, 32);
    if (bAgentTagSwitch) {
      _ctx.drawImage(tag, -10, -30, 16 * (tag.width / tag.height), 16);
    }
    _ctx.restore();

  }
}

function drawVisForbiddenZone(_ctx, _obj) {
  if (!_obj.hasOwnProperty('hik') || isEmpty(_obj.hik)) return;
  _obj = _obj.hik;

  let rgba = 'rgba(255, 0, 0, 0.2)';
  for (zoneKey in _obj) {
    if (_obj[zoneKey].area_type == 'safty_area') {
      return;
    }

    _ctx.save();
    _ctx.beginPath();

    var vertices = _obj[zoneKey].points;
    var num = vertices.length;

    var v0 = tfROS2Canvas(gMapMeta_, { x: Number(vertices[0][0]), y: Number(vertices[0][1]) });
    _ctx.moveTo(Number(v0.x), Number(v0.y));
    var minX = Number(v0.x);
    var minY = Number(v0.y);

    for (i = 1; i < num; ++i) {
      var Vi = tfROS2Canvas(gMapMeta_, { x: Number(vertices[i][0]), y: Number(vertices[i][1]) });
      Vi.x = Number(Vi.x);
      Vi.y = Number(Vi.y);
      _ctx.lineTo(Vi.x, Vi.y);
      if (minX > Vi.x) { minX = Vi.x; }
      if (minY > Vi.y) { minY = Vi.y; }
    }

    var unlockLen = _obj[zoneKey].unlock.length || 0;
    rgba = (unlockLen > 0) ? 'rgba(0, 255, 0, 0.2)' : 'rgba(255, 0, 0, 0)';

    _ctx.fillStyle = rgba;
    _ctx.closePath();
    _ctx.fill();
    // --- append the agent tags on the polygon ---
    if (unlockLen === 0) { continue; }

    _ctx.fillStyle = "Black";
    _ctx.font = '24px Arial';
    minX += 5;
    minY += 5;
    for (var i in _obj[zoneKey].unlock) {
      _ctx.fillText(_obj[zoneKey].unlock[i], minX, minY + (30 * i));
    }
    _ctx.restore();
  }
}

function drawVisFunctionalZone(_ctx, _obj) {
  if (typeof _obj !== 'object' || _obj.length === 0) return;
  for (key in _obj) {
    _ctx.save();
    _ctx.beginPath();

    const vertices = _obj[key].vertices;
    const num = vertices.length;
    const opacity = _obj[key].priority === 0 ? 0.2 : 0.5;
    const typeColor = _obj[key].zone_type === 1 ? `rgba(144, 160, 196, ${opacity})` : `rgba(211, 170, 126, ${opacity})`;

    if (!_obj[key].activate) {
      _ctx.setLineDash([5, 3]);
    }

    var V0 = tfROS2Canvas(gMapMeta_, { x: Number(vertices[0].x), y: Number(vertices[0].y) });
    _ctx.moveTo(Number(V0.x), Number(V0.y));

    for (i = 0; i < num; ++i) {
      let Vi = tfROS2Canvas(gMapMeta_, { x: Number(vertices[i].x), y: Number(vertices[i].y) });
      Vi.x = Number(Vi.x);
      Vi.y = Number(Vi.y);
      _ctx.lineTo(Vi.x, Vi.y);
      _ctx.strokeStyle = typeColor;
      _ctx.stroke();
    }
    _ctx.lineTo(V0.x, V0.y);
    _ctx.stroke();

    _ctx.fillStyle = typeColor;
    _ctx.closePath();
    _ctx.setLineDash([0]);
    _ctx.fill();
  }
}

let bAgentTagSwitch = false;
network_.on('click', function (params) {
  bAgentTagSwitch = !bAgentTagSwitch;
  handleZoneMouseClicked(params);
})

function handleZoneMouseClicked(e) {
  if (typeof zoneQueryData_ !== 'object' || zoneQueryData_.length === 0) return;
  let insideZones = [];
  const obj = zoneQueryData_;
  for (key in obj) {
    // check if the mouse is click inside the polygon
    const vertices = obj[key].vertices;
    const poly = tfPolygonPoints2Canvas(vertices);
    const num = vertices.length;
    const mouseX = e.pointer.canvas.x;
    const mouseY = e.pointer.canvas.y;
    const geometry = new Geometry();
    const p = new Point2D(mouseX, mouseY);
    if (geometry.pointInPolygon(poly, num, p)) {
      insideZones.push(obj[key].zone_name);
    }
  }
  if (insideZones.length === 0) {
    document.getElementById('zone-desc').style.display = 'none';
  } else if (insideZones.length === 1) {
    const zoneName = insideZones[0];
    const zone = _.find(obj, { zone_name: zoneName });
    let descString =
      `<b>Zone type</b>: ${zoneTypeNames_[zone.zone_type]}<br>
         <b>Zone name</b>: ${zone.zone_name}<br>
         <b>Zone priority</b>: ${zonePriorityNames_[zone.priority]}<br>
         <b>Zone setting</b>:<br><br>`;
    const props = _.pickBy(zone.properties, (value, key) => value !== null);
    for (const [key, value] of Object.entries(props)) {
      descString += `${key}: ${value}<br>`;
    }
    document.getElementById('zone-desc').style.display = 'block';
    document.getElementById('zone-desc').innerHTML = descString;
  } else {
    document.getElementById('zone-desc').style.display = 'block';
    document.getElementById('zone-desc').textContent = `${insideZones.join()} overlapping region`;
  }

}

function tfPolygonPoints2Canvas(_points) {
  let cpPoints = JSON.parse(JSON.stringify(_points));
  cpPoints.map((point) => {
    var visPos = tfROS2Canvas(gMapMeta_, {
      x: Number(point.x),
      y: Number(point.y)
    });
    point.x = Number(visPos.x);
    point.y = Number(visPos.y);
    return point;
  });
  return cpPoints;
}


// ================================
//        DOM Callbacks
// ================================
async function switchFleetCallback() {
  $("#map-select").empty();

  await setSelectAndMsg();

  liveSwitchMap();
}

async function liveSwitchMap() {
  resetVisNetwork(options_, network_);

  var selMapName = $("#map-select option:selected").val();
  if (selMapName === undefined) return;
  console.log(`========map: ${selMapName}========`);
  currMap_ = selMapName;

  updateMapAsync(selMapName);
}

function resetVisNetwork(_options, _network) {
  // --- vis-network options ---
  _options.nodes.fixed = { x: true, y: true };
  _options.manipulation.enabled = false;
  _options.interaction.dragNodes = false;
  _options.interaction.selectable = false;
  _options.interaction.selectConnectedEdges = false;
  _options.interaction.navigationButtons = true;

  _network.setOptions(_options);
  _network.fit();

  // --- vis dataset reset ---
  _network.body.data.nodes.clear();
  _network.body.data.edges.clear();

  // --- clean vis-network background ---
  const context = bakCanvas_.getContext('2d');
  context.clearRect(0, 0, bakCanvas_.width, bakCanvas_.height);
}


