/*
 * Author: John Wu
 * Date: 24 Mar 2021
 * Description:
 *   1. View and edit map
 *   2. View and edit graph
 *   3. View and edit WMS 
 **/

// ==================
//       Models
// ==================
let agentAssets_ = {
  br: {
    type: 'bbr',
    title: 'BBR',
    model: 'FAR-BBR-250',
    avatar: 'dist/img/sprites/bbr-440x220.png',
    thumbnail: 'dist/img/sprites/bbr-150x150.png',
  },
  smr1000: {
    type: 'smr1000',
    title: 'SMR 1000',
    model: 'FAR-SMR-1000',
    avatar: 'dist/img/sprites/smr1000-440x220.png',
    thumbnail: 'dist/img/sprites/smr1000-150x150.png',
  },
  smr1000_300mm: {
    type: 'smr1000',
    title: 'SMR 1000',
    model: 'FAR-SMR-1000',
    avatar: 'dist/img/sprites/smr1000_300mm-440x220.png',
    thumbnail: 'dist/img/sprites/smr1000_300mm-150x150.png',
  },
  smr250: {
    type: 'smr250',
    title: 'SMR 250',
    model: 'FAR-SMR-250',
    avatar: 'dist/img/sprites/smr250-440x220.png',
    thumbnail: 'dist/img/sprites/smr250-150x150.png',
  },
  smr250black: {
    type: 'smr250',
    title: 'SMR 250',
    model: 'FAR-SMR-250',
    avatar: 'dist/img/sprites/smr250black-440x220.png',
    thumbnail: 'dist/img/sprites/smr250black-150x150.png',
  },
  hik600: {
    type: 'hik',
    title: 'HIK',
    model: 'FAR-HIK',
    avatar: 'dist/img/sprites/hik-440x220.png',
    thumbnail: 'dist/img/sprites/hik-150x150.png',
  },
  mir250: {
    type: 'mir',
    title: 'MiR 250',
    model: 'FAR-MiR-250',
    avatar: 'dist/img/sprites/mir-440x220.png',
    thumbnail: 'dist/img/sprites/mir-150x150.png',
  },
  none: {
    type: 'smr1000',
    title: 'SMR 1000',
    model: 'FAR-SMR-1000',
    avatar: 'dist/img/sprites/smr1000_300mm-440x220.png',
    thumbnail: 'dist/img/sprites/smr1000_300mm-150x150.png',
  },
};

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
});

let maps = [];
let agents = [];
let currFleet_ = "";
let currMap_ = "";
let cellAssets_;

// =======================
//     Animation Ticks 
// =======================
setInterval(function () {
  network_.redraw();
}, 200);


// ============================
//     Map Global Variables 
// ============================
let gMapImg_;
let gMapData_;
let gMapMeta_;
let bCells_ = false;
let cellDetectionTypes_;

async function loadBundledMapDataAsync() {
  // wait until sidebar fleet is generated
  await sleep(1000); // sleep 1s

  var selFleet = getSavedFleet();
  console.log("========fleet: " + selFleet + "========");
  currFleet_ = selFleet;

  // -- fetch fleet_settings --
  var fltSettings = await restGetFleetSettings(selFleet);

  // ------ load cell types ------
  var settingsData = await restGetSettings();
  cellDetectionTypes_ = settingsData['cell.detectionTypes'];

  // -- load the map in the fleet --
  var fltKey = Object.keys(fltSettings)[0];
  maps = fltSettings[fltKey].maps;
  agents = fltSettings[fltKey].agents;

  // --- update map bundled data async ---
  console.log(maps);
  var mapName = maps[0];
  updateMapAsync(mapName);
}

// ============================
//     Level 1 functions 
// ============================
async function updateMapAsync(_fn) {
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

  network_.moveTo({
    position: { x: gMapData_.w / 2, y: gMapData_.h / 2 },
    scale: (720 / gMapData_.h) * 3.5
  });

  // --- update map meta data ---
  var mapYAML = _fn + '.yaml';
  console.log(_fn);
  // var mapYAML = _fn;

  gMapMeta_ = {};
  console.log(gMapMeta_);
  var data = await restGetMapMetaInfo(mapYAML);
  console.log(data);

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
  var cvs = document.getElementById('map_canvas');
  // [protection]
  if (cvs === null) { return; }

  cvs.width = _data.w;
  cvs.height = _data.h;

  var ctx = cvs.getContext('2d');
  ctx.drawImage(gMapImg_, 0, 0);
}

function cvtRad2Deg(_rad) {
  var deg = _rad * (180.0 / Math.PI);
  deg = Math.round(deg * 1000) / 1000; // CRITICAL! ensure the precision
  return deg;
}

function cvtDeg2Rad(_deg) {
  var rad = _deg * (Math.PI / 180.0);
  rad = Math.round(rad * 1000) / 1000; // CRITICAL! ensure the precision
  return rad;
}

// -- draw storage cells from json file --
let jsonWMS2_ = {};
function visDrawCells() {
  // -- [protection] confirm the data is updated --
  if (bCells_) return;
  bCells_ = true;

  // --- read the data from WMS json file ---
  for (key in jsonWMS2_) {
    var area = jsonWMS2_[key];

    for (let i = 0; i < area.length; i++) {
      var cell_id = area[i].cell_id;
      // var status = area[i].if_occupied;
      var x = area[i].cell_coordinate[0];
      var y = area[i].cell_coordinate[1];
      var pos = tfROS2Network_file({ x: x, y: y });

      var ang = -cvtRad2Deg(area[i].cell_coordinate[2]).toFixed(2);
      var svgHtml =
        `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
          <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#044B94" fill-opacity="0.4"/>
         </svg>`;
      var svgUrl = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtml);

      var wmsNode = {
        id: cell_id,
        x: pos.x,
        y: pos.y,
        color: {
          background: getColor(cellDetectionTypes_, area[i].type, 'bgColor'),
          border: getColor(cellDetectionTypes_, area[i].type, 'borderColor')
        }
      };
      wmsNode.label = cell_id;
      wmsNode.shape = 'dot';
      wmsNode.borderWidth = 2;
      wmsNode.size = 15; // default: 25
      wmsNode.font = {
        size: 8,
        color: getColor(cellDetectionTypes_, area[i].type, 'borderColor'),
        bold: true
      };
      wmsNode.group = 'wms';
      wmsNode.area = key;
      network_.body.data.nodes.update(wmsNode);
    }
  }
  // console.log("node number: "+network.body.data.nodes.length);
};

let strDOT2_ = "";
function visDrawGraph() {
  // -- [protection] confirm the data is updated --
  if (bNavGraph_) return;
  bNavGraph_ = true;

  var graph = cvtDot2Json(strDOT2_);
  // var txtArea = document.getElementById('ta-graph');

  for (i in graph.nodes) {
    var pos = tfROS2Network_file({ x: graph.nodes[i].x, y: graph.nodes[i].y });
    graph.nodes[i].x = pos.x;
    graph.nodes[i].y = pos.y;
    graph.nodes[i].group = 'navnode';

    network_.body.data.nodes.update(graph.nodes[i]);
  }

  // --- render edges ---
  for (i in graph.edges) {
    network_.body.data.edges.update(graph.edges[i]);
  }

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
  console.log(obj);
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
  width: '720px',
  height: '600px',
  layout: { hierarchical: { enabled: false } },
  physics: { enabled: false },
  nodes: {
    shape: 'dot',
    // color: 'gray',
    size: 5,
    font: {
      size: 12,
      color: 'rgba(255,80,0,0)' // hide therlabel of vertices
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

network_.on("beforeDrawing", function (ctx) {
  // console.log('--- before drawing ---');
  // --- draw image ---
  var canvas = document.getElementById('map_canvas');
  if (canvas === null) {
    // console.log('canvas is null or not updated');
    return;
  }

  var width = canvas.width;
  var height = canvas.height;

  // --- clean before drawing ---
  ctx.clearRect(0, 0, width, height);

  // --- draw board ---
  drawBoard(ctx, { w: width, h: height });

  if (canvas === null) { return; }
  // ctx.globalAlpha = 0.6;
  ctx.drawImage(canvas, 0, 0, width, height);
  // ctx.globalAlpha = 1.0;

});


network_.on("afterDrawing", function (ctx) {
  // TODO: confirm with multiple maps and graphs
  if (gMapMeta_ === {}) return; // when gMapMeta_ is empty, cannot do transformation
  // console.log(path_obj_);
  if (path_obj_ !== undefined) {
    drawVisPath(ctx, path_obj_.msg);
  }
  // drawVisFleet(ctx, fleet_obj_.msg);
  // console.log(fleet_fleet_obj_);
  // console.log(`${currFleet_}`);
  if (fleet_fleet_obj_ === undefined || fleet_fleet_obj_[currFleet_] === undefined) return;
  drawVisFleet(ctx, fleet_fleet_obj_[currFleet_]);
});

function drawBoard(_ctx, _dim, _span = 50) {
  var bw = _dim.w;
  var bh = _dim.h;

  // --- vertical line ---
  for (var x = 0; x <= bw; x += _span) {
    _ctx.moveTo(0.5 + x, 0);
    _ctx.lineTo(0.5 + x, bh);
  }

  // --- horizontal line ---
  for (var y = 0; y <= bh; y += _span) {
    _ctx.moveTo(0, 0.5 + y);
    _ctx.lineTo(bw, 0.5 + y);
  }
  _ctx.strokeStyle = "transparent";
  _ctx.stroke();
}


function drawVisPath(ctx, msg) {
  for (i in msg) {
    // console.log(path_obj_.msg[i].points);
    if (path_obj_.msg[i].points.length === 0) continue;

    // --- draw chasing route ---
    var from_pos = tfROS2Canvas(gMapMeta_, { x: path_obj_.msg[i].points[0].x, y: path_obj_.msg[i].points[0].y });
    var to_pos = tfROS2Canvas(gMapMeta_, { x: path_obj_.msg[i].points[1].x, y: path_obj_.msg[i].points[1].y });

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

function drawVisFleet(ctx, _obj) {
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
    if (msg[i].map !== currMap_) return;

    var pos = tfROS2Canvas(gMapMeta_, { x: msg[i].location.x, y: msg[i].location.y });

    // --- draw agent as a circle 
    // ctx.beginPath();
    // ctx.arc(pos.x, pos.y, 10, 0, 2 * Math.PI);
    // ctx.fillStyle = 'orange'; // agent_color
    // ctx.fill();
    // ctx.stroke();

    // --- draw agent as a sprite 
    // sprite = (i == 0) ? sprite0 : sprite1;
    var model = (msg[i].model || 'smr250').toLowerCase();
    let sprite = new Image();
    sprite.src = agentAssets_[model].avatar;
    ctx.drawImage(sprite, pos.x - 32, pos.y - 19, 64, 38);
  }
}


// ================================
//        DOM Callbacks    
// ================================

async function liveSwitchMap() {
  resetVisNetwork(options_, network_);

  var selMapName = $("#map-select option:selected").text();
  currMap_ = selMapName;
  if (selMapName === "") return;
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
  var canvas = document.getElementById('map_canvas');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);
}

// =================================
//     Coordinates Transfomation     
// =================================
function tfROS2Network_file(_pos) {
  var x_network, y_network;
  var xRosOffset = gMapMeta_.origin.x / gMapMeta_.resolution;
  var yRosOffset = gMapMeta_.origin.y / gMapMeta_.resolution;

  x_network = (_pos.x / gMapMeta_.resolution - xRosOffset).toFixed(4);
  y_network = (_pos.y / gMapMeta_.resolution - yRosOffset);
  y_network = (gMapData_.h - y_network).toFixed(4);

  return { x: x_network, y: y_network };
}

