/*
 * Author: John Wu
 * Date: 24 Mar 20,
 * Description:
 *   1. View and edit map
 *   2. View and edit graph
 *   3. View and edit WMS 
 **/
class DiffNode {
  constructor(_key, _value) {
    this.key = _key;
    this.value = _value;
    this.next = null;
    this.prev = null;
  }
}

class FarCache {
  constructor() {
    this.head = null;
    this.tail = null;
    this.size = 0;
    this.maxSize = 4;
    this.cache = {};
  }

  put(_key, _value) {
    let newNode;
    // --- if the key not present in cache ---
    if (this.cache[_key] === undefined) {
      newNode = new DiffNode(_key, _value)
    }

    // --- initial list case ---
    if (this.size === 0) {
      this.head = newNode;
      this.tail = newNode;
      this.cache[_key] = newNode;
      this.size++;
      return this;
    }

    if (this.size === this.maxSize) {
      this.#dropOneFromTail();
    }

    // --- add an item to the head 
    this.head.next = newNode;
    newNode.prev = this.head;
    this.head = newNode;
    this.size++;

    // --- add to cache ---
    this.cache[_key] = newNode;
    return this;
  }

  get(key) {
    if (!this.cache[key]) {
      return undefined;
    }

    let foundNode = this.cache[key];
    if (foundNode === this.head) return foundNode;

    let previous = foundNode.prev;
    let next = foundNode.next;

    if (foundNode === this.tail) {
      previous.next = null;
      this.tail = previous;
    } else {
      previous.next = next;
      next.prev = previous;
    }

    this.head.prev = foundNode;
    foundNode.next = this.head;
    foundNode.prev = null;
    this.head = foundNode;

    return foundNode;
  }

  popHead() {
    if (!this.head) {
      return undefined;
    }

    let foundNode = this.head;

    // --- last node case ---
    if (!this.head.prev) {
      this.head = null;
      this.tail = null;
      this.size = 0;
      delete this.cache[foundNode.key]
      return foundNode;
    }

    // --- regular case ---
    this.head = foundNode.prev;
    this.head.next = null;
    this.size--;
    console.log(this.size);
    delete this.cache[foundNode.key]
    return foundNode;
  }

  #dropOneFromTail() {
    // --- update the tail ---
    let newTail = this.tail.next;
    newTail.prev = null;
    // --- delete the dropping tail node ---
    delete this.cache[this.tail.key];
    this.size--;
  }

  getHead() {
    return this.head;
  }

  flush() {
    this.head = null;
    this.tail = null;
    this.size = 0;
    this.maxSize = 4;
    this.cache = {};
  }

}

// ==================
//       Models
// ==================
let zoneTypes_ = {
  1: {
    'imgName': 'config',
    'pointColor': '#5477e9',
    'fillColor': '#90a0c4'
  },
  2: {
    'imgName': 'bypass',
    'pointColor': '#ef9e4b',
    'fillColor': '#d3aa7e'
  }
};

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  init();
  initMobileResizeEvts();
  bindTSPinch2Zoom();
});

const ZONE_POINT_PREFIX = 'p-';
const ZONE_LINE_PREFIX = 'l-';
const ZONE_POLYGON_PREFIX = 'z-';
// let cellTypes_;
let cellDetectionTypes_;

let functionTypes_ = {};
let functionDetectionTypes_;

let fleetModels_ = {};
let apiQueryData_ = {};

var fleetFileArray = [];
let jsonZoneArray = [];
let jsonZoneCache_ = {};

async function init() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'map', 'map.html');

  // ------ language switch ------
  await initLanguageSupport();
  let lng = getSetLang() || 'en';
  langTemplateObj_ = await restGetTemplateLang(lng, 'edit_map');

  initRenameMapNameForm();

  // -------- Data Sourcing by Polling --------
  pollWmsStates(apiQueryData_);

  // ------ load cell types ------
  var settingsData = await restGetSettings();
  console.log(settingsData);

  cellDetectionTypes_ = settingsData['cell.detectionTypes'];
  console.log(cellDetectionTypes_);
  // cellTypes_ = settingsData['cell.types'];
  // console.log(cellTypes_);

  // var cellTypesData = await restGetCellTypes();
  // cellTypesData = JSON.parse(cellTypesData);
  // cellTypes_ = cellTypesData;
  // console.log(cellTypes_);

  // ------ load function types ------
  functionTypes_ = await restGetFunctionTypes();
  console.log(functionTypes_);
  // TODO: refector
  if (functionTypes_.includes('ENOENT')) {
    // await restPostFunctionTypes();
    await restPostFunctionTypes(FUNCTION_TYPES_);
    functionTypes_ = await restGetFunctionTypes();
  }
  functionTypes_ = JSON.parse(functionTypes_);
  console.log(functionTypes_);

  // ------ load models in the fleet ------
  let selFleet = $('#fleet-select').val();
  let token = await rmtTokenCheck2();
  let res;
  try {
    // res = await fetchFleetStates(token, selFleet);
    res = await fetchRobotsTemplates2(token);
  } catch (err) {
    console.error(err);
  }

  if (res.ok) {
    fleetModels_ = await res.json();
    fleetModels_ = Object.keys(fleetModels_);
  }
  // console.log(fleetModels_);
  // if (fleetModels_.hasOwnProperty('fleet_state')) {
  //   fleetModels_ = fleetModels_.fleet_state.find(fs => fs.fleet_name === selFleet);
  //   console.log(fleetModels_);
  // }
  // if (fleetModels_ !== null && fleetModels_.hasOwnProperty('robots')) {
  //   fleetModels_ = _.uniq(fleetModels_.robots.map(r => r.model));
  // }
  console.log(fleetModels_);

  // ------ load map file list ------
  loadBundledMapDataAsync();

  // ------ canvas for map ------
  var canvas2 = document.createElement('canvas');
  canvas2.setAttribute("id", "map_canvas2")
  canvas2.setAttribute("style", "box-sizing: content-box; border: solid 2px #A9A9A9;");
  canvas2.setAttribute("style", "display:none;");

  var canDiv2 = document.getElementById('hidden-map2');
  canDiv2.appendChild(canvas2);

  // ------ vis-network options ------
  visOptions_.nodes.fixed = {
    x: true,
    y: true
  };
  visOptions_.manipulation.enabled = false;
  visOptions_.interaction.dragNodes = false;
  visOptions_.interaction.selectable = false;
  visOptions_.interaction.selectConnectedEdges = false;
  visOptions_.interaction.navigationButtons = true;

  visNetwork_.setOptions(visOptions_);
  visNetwork_.fit();

  // --- vis dataset reset ---
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  // -- Edit Properties --
  toggleEdit(false);
  initZoneEditProperties();

  // -- Properties View --
  toggleNavGraph(true);
  toggleCells(true);

  // ------ calculate canvas DOM width & height ------
  let visNetworkRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
  let viewHeight = visNetworkRect.height;
  let viewWidth = visNetworkRect.width;
  mapViewSize = Math.min(viewWidth, viewHeight);
  console.log(`map view size: ${mapViewSize}`);
  $('.cont').css('height', `${mapViewSize}px`);

  var upperCanvases = document.getElementsByClassName('upper-canvas');
  for (var i = 0; i < upperCanvases.length; i++) {
    upperCanvases[i].style.left = `calc(50% - ${mapViewSize / 2}px)`;
  }

  // $('#grid').css({
  //   'width': `${mapViewSize}px`,
  //   'height': `${mapViewSize}px`
  // });
  // $('#c').css({
  //   'width': `${mapViewSize}px`,
  //   'height': `${mapViewSize}px`
  // });

  // $('#cont > .canvas-container > canvas').prop('width', `${mapViewSize}px`);
  // .css({
  //   'width': `${mapViewSize}px !important`,
  //   'height': `${mapViewSize}px !important`,
  //   'left': `calc(50% ${mapViewSize/2}px) !important`
  // });

  validateVertexInputEvent();
  validateCellInputEvent();
}

async function rmtTokenCheck2() {
  var token;
  try {
    token = await fetchToken();
  } catch (err) {
    console.error(err);
    await sleep(5000);
    token = rmtTokenCheck2();
  }
  return token;
}

function pollWmsStates(_dataObj, _inteval = 1000) {
  setInterval(async function () {
    try {
      let res = await fetchWmsStates(rmtToken_, currSelectedMap_);
      if (res.ok) {
        // console.log(' request OK ');
        _dataObj['wms'] = await res.json();
      }
    }
    catch (e) {
      console.log(' request NOT OK ');
      _dataObj['wms'] = undefined;
    }
    // if (res.ok) {
    //   console.log(' request OK ');
    //   _dataObj['wms'] = await res.json();
    // } else {
    //   console.log(' request NOT OK ');
    //   _dataObj['wms'] = undefined;
    // }
    // console.log(apiQueryData_)
  }, _inteval);
};

function initRenameMapNameForm() {
  $.validator.addMethod("chkDupMapName", function (value, element) {
    let data = restMapAliasExistence(value);
    let mapAliasExists = data.responseJSON;
    if (mapAliasExists === undefined) {
      return false;
    }
    return !mapAliasExists;
  });

  // $.validator.addMethod("chkUnchangedMapName", function (value, element) {
  //   var mapName = $('#map-select option:selected').text();
  //   let new_mapname = value;
  //   if (new_mapname !== mapName) {
  //     return true;
  //   }
  //   return false;
  // });

  $('#rename-map-form').validate({
    rules: {
      newMapName: {
        required: true,
        // chkUnchangedMapName: true,
        chkInputVal: true,
        chkDupMapName: true
      }
    },
    messages: {
      newMapName: {
        required: 'Please enter map name',
        // chkUnchangedMapName: 'Map name not changed',
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

  $('#rename-map-form').on('submit', renameMapAlias);
}

$("#rename_map").on("click", async function () {
  executingFlow = await checkProtection();
  let is_map_using = executingFlow["using_map"].includes($('#map-select').val());

  if (!is_map_using) {
    $($(this).attr("data-target")).modal("show");
  } else {
    notificationMsg(3, 'Map has executing task, cannot rename.');
    return;
  }
});

$("#delete_map").on("click", async function () {
  executingFlow = await checkProtection();
  let is_map_using = executingFlow["using_map"].includes($('#map-select').val());

  if (!is_map_using) {
    $($(this).attr("data-target")).modal("show");
  } else {
    notificationMsg(3, 'Map has executing task, cannot delete.');
    return;
  }
});

async function renameMapAlias(e) {
  e.preventDefault();
  if (!$('#rename-map-form').valid()) { return; }
  $('#rename-map-modal').modal('hide');
  let mapFileName = $('#map-select').val();
  let newMapName = $('#new-mapname').val();
  // console.log(mapFileName)
  // console.log(newMapName)

  // --- Native APIs ---
  // let data = await restPostMapAlias(mapFileName, newMapName);
  // if (data.status_code === 200) {
  //   notificationMsg(1, data.message);
  //   $(`#map-select option[value="${mapFileName}"]`).text(`${newMapName}`);
  // } else {
  //   notificationMsg(3, data.message);
  // }

  // --- Swarm Core APIs ---
  let res = await fetchPutMapNameAlias(rmtToken_, mapFileName, newMapName);
  let statusCode = res.status;
  let statusText = await res.json();
  // console.log(res);
  if (statusCode === 200) {
    $(`#map-select option[value="${mapFileName}"]`).text(`${newMapName}`);
    notificationMsg(1, statusText);
  } else {
    notificationMsg(3, statusText);
  }
}

// =======================
//     Animation Ticks 
// =======================
let liveOn;
function toggleCellStatus() {
  console.log('live cell status');
  if (!liveOn) {
    // liveOn = setInterval(updateVisCellStatus(wms_obj_.msg), 400);
    liveOn = setInterval(function () {
      if (apiQueryData_ !== undefined && apiQueryData_.hasOwnProperty('wms')) {
        console.log('----- live on ------');
        updateVisCellStatus2(apiQueryData_.wms);
      }

      visNetwork_.redraw();
    }, 450);
    $('#live-btn-text').text('Live Status On');
    $('#live-btn-text').siblings('.fas').attr('class', 'fas fa-eye');
    $('#live-status-text').val('on');
    $('#set-live-btn').text('Set Off');

    rtSetCellConfig(true);
  } else {
    clearInterval(liveOn)
    liveOn = null;
    updateVisCellStatus();
    // updateVisCellStatus2();
    $('#live-btn-text').text('Live Status Off');
    $('#live-btn-text').siblings('.fas').attr('class', 'fas fa-eye-slash');
    $('#live-btn-text').siblings('.fas').attr('style', 'color:darkgray');
    $('#live-status-text').val('off');
    $('#set-live-btn').text('Set On');

    rtSetCellConfig(false);
  }
}

function rtSetCellConfig(_isOn) {
  // ------ switch labels ------
  document.getElementById("cell-operation").innerText = (_isOn) ? "Set Cell Load" : "Edit Cell";
  // document.getElementById("cell-createButton").textContent = (_isOn) ? "Apply" : "Create"; // "Save"

  // ------ styling ------
  let elTable = document.getElementById('cell-popUp');
  elTable.classList.toggle('rt-config', _isOn);

  let container = document.getElementById('cell-popUp');
  let elLoad = container.querySelector('.tr-cell-load');
  elLoad.style.display = (_isOn) ? '' : 'none';
  let elApplyBtn = container.querySelector('#cell-applyButton');
  elApplyBtn.style.display = (_isOn) ? '' : 'none';
  let elCreateBtn = container.querySelector('#cell-createButton');
  elCreateBtn.style.display = (_isOn) ? 'none' : '';

  // ------ hide/show RT element ------
  const persistEls = ['tr-cell-label', 'tr-cell-area'];
  const rtEl = (_isOn) ? '' : 'none';
  const nonRtEl = (_isOn) ? 'none' : '';
  // let container = document.querySelector('#cell-popUp');
  let matches = container.querySelectorAll('table > tbody > tr');
  matches.forEach((el) => {
    console.log(el.className);
    el.style.display = (el.className === 'tr-cell-load') ? rtEl : nonRtEl;
    // if (el.className === 'tr-cell-load') { el.style.display = rtEl; }
    console.log(persistEls.includes(el.className));
    if (persistEls.includes(el.className)) {
      el.querySelector('input').disabled = (_isOn) ? true : false;
      el.style.display = '';
    }
  });
}

// --- update WMS status ---
const wmsStyles_ = {
  // 'occupied': { border: 'rgba(255,0,0,0.4)' },
  'rack': { border: 'rgba(255,0,0,0.4)' },
  'empty': { border: 'rgba(0,255,0,0.4)' },
  'loaded': { border: 'rgba(98,91,87,0.4)' }
};
function updateVisCellStatus2(_msgs) {
  _msgs = _msgs?.cells;
  // if (wmsCellStatusAssets_ === {}) return;
  // console.log(_msgs);
  let cellNodes = [];

  // --- LIVE-OFF cell status ---
  if (_msgs === undefined) {
    _msgs = visNetwork_.body.nodes;
    console.log(_msgs);
    $('#live-btn-text').siblings('.fas').attr('style', 'color:rgba(0,0,0,0.6)');

    for (i in _msgs) {
      if (_msgs[i].options.group !== 'wms') { continue; }

      var wmsNode = {};
      wmsNode.id = _msgs[i].id; // cell-id
      var color = {
        // border: getLiveStatusColor()
        border: "rgba(0,0,0,0.6)"
      };

      wmsNode.shape = 'circle';
      wmsNode.size = 20; // default: 25
      wmsNode.color = color

      cellNodes.push(wmsNode);
    }

    visNetwork_.body.data.nodes.update(cellNodes);
    return;
  }

  // --- LIVE-ON cell status ---  
  console.log(jsonCellCache_);
  var cells = _.flatten(Object.values(jsonCellCache_));
  // console.log(cells);
  // var cellNodes = [];
  // console.log(_msgs);
  for (i in _msgs) {
    console.log(_msgs[i]);
    // console.log(currSelectedMap_);
    if (_msgs[i].map !== currSelectedMap_) continue;
    // if (!wmsCellStatusAssets_.hasOwnProperty(_msgs[i].cell_id)) { continue; }
    let target = cells.find(c => c.cell_id === _msgs[i].cell_id);
    // console.log(target)

    var wmsNode = {};
    // wmsNode.id = _msgs[i].cell_id; // cell-id
    wmsNode.id = target.cell_uuid; // cell-id
    wmsNode.shape = 'circle';
    wmsNode.size = 20; // default: 25
    wmsNode.color = wmsStyles_[_msgs[i].load] || { border: 'rgba(255,255,0,0.4)' };

    cellNodes.push(wmsNode);
  }

  // console.log(cellNodes);
  visNetwork_.body.data.nodes.update(cellNodes);
  $('#live-btn-text').siblings('.fas').attr('style', 'color:rgba(0,255,0,0.6)');
};

function updateVisCellStatus(_msgs) {
  console.log(_msgs);
  // _msgs = _msgs?.wms;
  var cellNodes = [];

  // --- LIVE-OFF cell status ---
  if (_msgs === undefined) {
    _msgs = visNetwork_.body.nodes;
    console.log(_msgs);
    for (i in _msgs) {
      if (_msgs[i].options.group !== 'wms') { continue; }

      var wmsNode = {};
      wmsNode.id = _msgs[i].id; // cell-id
      var color = {
        border: getLiveStatusColor()
      };

      wmsNode.shape = 'circle';
      wmsNode.size = 20; // default: 25
      wmsNode.color = color

      cellNodes.push(wmsNode);
    }

    visNetwork_.body.data.nodes.update(cellNodes);
    $('#live-btn-text').siblings('.fas').attr('style', 'color:rgba(0,0,0,0.6)');
    return;
  }

  // --- LIVE-ON cell status ---
  var cells = _.flatten(Object.values(jsonCellCache_));
  console.log(cells);
  for (i in _msgs) {
    console.log(_msgs[i].text);
    let target = cells.find(c => c.cell_id === _msgs[i].text);
    console.log(target)
    if (target === undefined) { continue; }
    var wmsNode = {};
    // wmsNode.id = _msgs[i].text; // cell-id
    wmsNode.id = target.cell_uuid; // cell-id

    var color = { border: getLiveStatusColor(_msgs[i].color) };

    wmsNode.shape = 'circle';
    wmsNode.size = 20; // default: 25
    wmsNode.color = color

    cellNodes.push(wmsNode);
  }

  visNetwork_.body.data.nodes.update(cellNodes);
};

function getLiveStatusColor(_colorCode) {
  // --- color of cell live-off status ---
  if (_colorCode === undefined)
    return 'rgba(0,0,0,0.2)'

  let color;
  if (Math.abs(_colorCode.r - 0.3) < 0.01 && _colorCode.g === 0 && _colorCode.b === 0) {
    // --- status: 'occupied' --- 
    color = 'rgba(255,0,0,0.4)';
  }
  else if (_colorCode.r === 0 && Math.abs(_colorCode.g - 0.3) < 0.01 && _colorCode.b === 0) {
    // --- status: 'empty' ---
    color = 'rgba(0,255,0,0.4)';
  }
  else if (Math.abs(_colorCode.r - 1.0) > 0.01 && Math.abs(_colorCode.g - 1.0) < 0.01 && Math.abs(_colorCode.b - 1.0) < 0.01) {
    // --- status: 'loaded' ---
    color = 'rgba(98,91,87,0.4)';
  }
  else {
    // --- status: 'unknown' ---
    color = 'rgba(255,255,0,0.4)';
  }
  // console.log(status);

  return color;
}

// ======================
//       Event Loop 
// ======================
let bCells_ = false;
let bUpdateWMS2_ = false;
let bNavGraph_ = false;
let visNetwork_;

// let gridCanvas = new fabric.Canvas('grid', {
//   selection: false
// });

// ======================
//       Events
// ======================
let canvasScale_ = 1.0;
// let dragScale_ = 1.0;
let mapViewSize = 720;
function drawFabricMap() {
  var mapDataURL = `data:image/png;base64,${gMapData_.data}`;

  var img = new Image();
  img.onload = function () {
    var finalDim = Math.max(img.width, img.height);
    // dragScale_ = finalDim >= mapViewSize ? Math.floor(finalDim / mapViewSize) : 1.0;
    // console.log(`drag scale: ${dragScale_}`);
    canvasScale_ = finalDim >= mapViewSize ? finalDim / mapViewSize : 1.0;
    console.log(`canvas scale: ${canvasScale_}`);

    // var leftest = (finalDim - img.width) / 2;
    console.log(finalDim);
    console.log(img.width);
    // console.log(leftest);
    // var f_img = new fabric.Image(img, { left: leftest, top: 0 });
    var f_img = new fabric.Image(img);
    currentImage = f_img;

    // fCanvas.setWidth(f_img.width);
    // fCanvas.setHeight(f_img.height);

    // fCanvas.setWidth(finalDim);
    // fCanvas.setHeight(finalDim);

    fCanvas.setWidth(mapViewSize);
    fCanvas.setHeight(mapViewSize);
    fCanvasZoomIn(fCanvas, 1 / canvasScale_);

    var fCanvasElement = fCanvas.getElement();
    fCanvasElement.style.left = `calc(50% - ${mapViewSize / 2}px)`;

    fCanvas.setBackgroundImage(f_img).renderAll();
  };
  img.src = mapDataURL;
}

function drawFabricZoneMap() {
  var mapDataURL = `data:image/png;base64,${gMapData_.data}`;

  var img = new Image();
  img.onload = function () {
    var finalDim = Math.max(img.width, img.height);
    canvasScale_ = finalDim >= mapViewSize ? finalDim / mapViewSize : 1.0;
    console.log(`canvas scale: ${canvasScale_}`);

    var f_img = new fabric.Image(img);
    fZoneCanvas.setWidth(mapViewSize);
    fZoneCanvas.setHeight(mapViewSize);
    fCanvasZoomIn(fZoneCanvas, 1 / canvasScale_);

    var fCanvasElement = fZoneCanvas.getElement();
    fCanvasElement.style.left = `calc(50% - ${mapViewSize / 2}px)`;

    fZoneCanvas.setBackgroundImage(f_img).renderAll();
  };
  img.src = mapDataURL;

  // let zCtx = document.querySelector('#far-network2 > div > canvas').getContext("2d");
  // let dataURL = zCtx.canvas.toDataURL();
  // let visCanvasImg = document.getElementById('zoneCanvasImg');
  // visCanvasImg.src = dataURL;
}

function drawFabricZones() {
  let cpZoneArr = JSON.parse(JSON.stringify(jsonZoneArray));
  cpZoneArr.forEach(zone => {
    if (zone.vertices.length === 0) return;
    let type = zoneTypes_[zone.zone_type] || 'none';
    let color = type.fillColor || 'gray';
    let points = zone.vertices;

    points.map((obj) => {
      var visPos = tfROS2Canvas(gMapMeta_, {
        x: Number(obj.x),
        y: Number(obj.y)
      });
      obj.x = Number(visPos.x);
      obj.y = Number(visPos.y);
      return obj;
    });

    var polygon = new fabric.Polygon(points, {
      id: ZONE_POLYGON_PREFIX + zone.uuid,
      stroke: color,
      fill: color,
      opacity: zone.priority === 0 ? 0.3 : 0.8,
      objectCaching: false,
      moveable: false,
      selectable: false
    });
    fZoneCanvas.add(polygon);
  });

  fZoneCanvas.renderAll();
}

function drawFabricSelectedZonePoints(_selObj, _selMode = "none", _selCoord = {}) {
  // console.log(_selObj);
  let modeColor;
  switch (_selMode) {
    case 'delete':
      modeColor = 'red';
      break;
    case 'edit':
      modeColor = 'green';
      break;
    case 'add':
      modeColor = 'black';
      break;
  }

  let points = _selObj.points;
  points.forEach(point => {
    let fillColor = _selObj.fill;
    if (!isEmpty(_selCoord)) {
      fillColor = _selCoord.x === point.x && _selCoord.y === point.y ? modeColor : _selObj.fill;
    }
    const pointOption = {
      id: _selObj.id.replace(ZONE_POLYGON_PREFIX, ZONE_POINT_PREFIX),
      radius: 7 / fZoneCanvas.getZoom(),
      fill: fillColor,
      stroke: _selObj.stroke,
      strokeWidth: 1,
      left: point.x,
      top: point.y,
      selectable: false,
      hasBorders: false,
      hasControls: false,
      originX: 'center',
      originY: 'center',
      objectCaching: false
    };
    var circle = new fabric.Circle(pointOption);
    fZoneCanvas.add(circle);
  });
}

function drawFabricEditingZonePoints() {
  fZoneCanvas.discardActiveObject().renderAll();
  let obj = getFabricZoneObject(jsonZoneCache_.uuid);
  fZoneCanvas.bringToFront(obj);
  drawFabricSelectedZonePoints(obj);
  selShape = obj;
}

function drawFabricEditingZone(_point) {
  let id = selShape.id.replace(ZONE_POLYGON_PREFIX, '');
  const pointOption = {
    id: ZONE_POINT_PREFIX + id,
    radius: 7 / fZoneCanvas.getZoom(),
    fill: '#adadad',
    stroke: '#adadad',
    strokeWidth: 1,
    left: _point.x,
    top: _point.y,
    selectable: false,
    hasBorders: false,
    hasControls: false,
    originX: 'center',
    originY: 'center',
    objectCaching: false
  };
  var point = new fabric.Circle(pointOption);
  fZoneCanvas.add(point);

  selShape.set({
    fill: '#adadad',
    opacity: 0.3
  });
  removeFabricZoneObjects(selShape.id);
  drawFabricZone(id);
}

function drawFabricZone(_id) {
  var polygon = new fabric.Polygon(selShape.points, {
    id: ZONE_POLYGON_PREFIX + _id,
    stroke: selShape.stroke,
    fill: selShape.fill,
    opacity: selShape.opacity,
    objectCaching: false,
    moveable: false,
    selectable: false
  });
  fZoneCanvas.add(polygon);
}

function activeSelectedZone(_selZone) {
  disableFabricObjResizing(_selZone);
  disableFabricObjDragging(_selZone);
  fZoneCanvas.setActiveObject(_selZone);
  fZoneCanvas.bringToFront(_selZone);
}

function getFabricZoneObject(_id) {
  let obj;
  _id = ZONE_POLYGON_PREFIX + _id;
  fZoneCanvas.forEachObject(function (o) {
    if (o.id && o.id === _id) {
      obj = o;
    }
  });
  return obj;
}

function restoreUnsavedFabricZone() {
  if (selShape === undefined || oldPoints.length === 0) return;
  let id = selShape.id.replace(ZONE_POLYGON_PREFIX, '');
  let index = _.findIndex(jsonZoneArray, { uuid: id });
  if (index !== -1) {
    let type = jsonZoneArray[index].zone_type;
    let color = zoneTypes_[type].fillColor;
    let priority = jsonZoneArray[index].priority;
    selShape.set({
      fill: color,
      opacity: priority === 0 ? 0.3 : 0.8,
      points: oldPoints
    });
  }
  removeFabricZoneObjects(selShape.id);
  drawFabricZone(id);
}

function removeFabricZoneObjects(_removeId) {
  fZoneCanvas.getObjects().forEach(async function (o) {
    if (!o.hasOwnProperty('id')) return;
    if (o.id.includes(_removeId)) {
      fZoneCanvas.remove(o);
    }
  });
}

function removeAllFabricZones() {
  fZoneCanvas.getObjects().forEach(async function (o) {
    fZoneCanvas.remove(o);
  });
}

function removeUnsavedFabricZones() {
  fZoneCanvas.getObjects().forEach(async function (o) {
    let filteredZone = _.filter(jsonZoneArray, zone => o.id.includes(zone.uuid));
    if (filteredZone.length > 0) return;
    fZoneCanvas.remove(o);
  });
}

function disableFabricObjResizing(el) {
  el.setControlsVisibility({
    mt: false,
    mb: false,
    ml: false,
    mr: false,
    bl: false,
    br: false,
    tl: false,
    tr: false,
    mtr: false,
  });
}

function disableFabricObjDragging(el) {
  el.lockMovementX = true;
  el.lockMovementY = true;
}

function btnEditPolygonPoint() {
  let id = this.id.replace(ZONE_POINT_PREFIX, '');
  let nodeX = document.getElementById("zone-node-x").value;
  let nodeY = document.getElementById("zone-node-y").value;
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(nodeX),
    y: Number(nodeY)
  });

  var index = _.findIndex(selShape.points, { x: this.left, y: this.top });
  selShape.points.splice(index, 1, { x: Number(visPos.x), y: Number(visPos.y), theta: 0 });

  // --- re-render zone on canvas ---
  removeFabricZoneObjects(selShape.id);
  drawFabricZone(id);
  removeFabricZoneObjects(ZONE_POINT_PREFIX);
  drawFabricSelectedZonePoints(selShape);

  // --- update zone cache data ---
  updateZoneVertices(id, selShape.points);

  clearNodePopUp();
}

// function drawFabricBoard() {
//   // ------ build grid ------
//   var canvasWidth = mapViewSize; // [CFG] canvas DOM width
//   var canvasHeight = mapViewSize; // [CFG] canvas DOM height 
//   gridCanvas.clear();

//   // --- transform formula ---
//   var meterPerGrid = Number($('#grid-span').val());
//   var grid = meterPerGrid * (canvasHeight / gMapData_.h) / gMapMeta_.resolution;

//   for (var i = 0; i < (canvasWidth / grid); i++) {
//     gridCanvas.add(new fabric.Line([i * grid, 0, i * grid, canvasHeight], {
//       type: 'line',
//       stroke: '#ccc',
//       selectable: false
//     }));
//     gridCanvas.add(new fabric.Line([0, i * grid, canvasWidth, i * grid], {
//       type: 'line',
//       stroke: '#ccc',
//       selectable: false
//     }))
//   };
// }

// --- object manipulation on fabric ---
function setSelectedZones() {
  var opt = document.getElementById('zone-tool').value;
  console.log(opt);

  if (opt === 'forbidden') {
    var prohib_rect = new fabric.Rect({
      left: 200,
      top: 150,
      width: 200,
      height: 50,
      fill: 'red',
      rx: 5,
      ry: 5,
      opacity: 0.5,
      objectCaching: false
    });

    fCanvas.add(prohib_rect);

    return;
  }

  if (opt === 'preferred') {
    var prefer_rect = new fabric.Rect({
      left: 300,
      top: 250,
      width: 200,
      height: 50,
      fill: 'lime',
      rx: 5,
      ry: 5,
      opacity: 0.5,
      objectCaching: false
    });

    fCanvas.add(prefer_rect);

    return;
  }
}

function btnShowCropTool() {
  console.log(`--- show crop tool ---`);
  var btnCrop = document.createElement('crop-map');
  btnCrop.setAttribute("style", "display:none;");

  // --- create and add select rect ---
  addSelectRect();
  fCanvas.renderAll();
  // --- show crop button ---
  var btnCrop = document.getElementById('crop-map');
  btnCrop.removeAttribute('style');
}

let selectionRect;
let currentImage;

function addSelectRect() {
  selectionRect = new fabric.Rect({
    fill: "rgba(0,0,0,0.2)",
    originX: "left",
    originY: "top",
    stroke: "black",
    opacity: 1,
    width: fCanvas.width,
    height: fCanvas.height,
    hasRotatingPoint: false,
    transparentCorners: false,
    cornerColor: "white",
    cornerStrokeColor: "black",
    borderColor: "black",
    cornerSize: 12,
    padding: 0,
    cornerStyle: "circle",
    borderDashArray: [5, 5],
    borderScaleFactor: 1.3,
  });

  // -- set object dimension --
  selectionRect.left = mapRoi_.left;
  selectionRect.top = mapRoi_.top;
  selectionRect.width = mapRoi_.width;
  selectionRect.height = mapRoi_.height;

  // -- set control property --
  selectionRect.setControlVisible('mt', false);
  selectionRect.setControlVisible('mb', false);
  selectionRect.setControlVisible('ml', false);
  selectionRect.setControlVisible('mr', false);

  // selectionRect.scaleToWidth(200);
  fCanvas.add(selectionRect)
  // --- set active object ---
  fCanvas.setActiveObject(selectionRect)
}

let mapRoi_ = {
  left: 5,
  top: 5,
  width: 710,
  height: 710
};

function btnGetRoiTool() {
  // --- create and add select rect ---
  addSelectRect();
  fCanvas.renderAll();

  // show crop button
  // var btnCrop = document.getElementById('crop-map');
  // btnCrop.removeAttribute('style');

  // --- dom render logic ---
  document.getElementById("get-roi").style.display = "none";
  document.getElementById("set-roi").style.display = "inline";
  document.getElementById("cancel-roi").style.display = "inline";
}

function btnSetRoiView() {
  mapRoi_.left = selectionRect.left;
  mapRoi_.top = selectionRect.top;
  mapRoi_.width = selectionRect.width;
  mapRoi_.height = selectionRect.height;

  fCanvas.remove(selectionRect);

  // --- dom render logic ---
  document.getElementById("get-roi").style.display = "inline";
  document.getElementById("set-roi").style.display = "none";
  document.getElementById("cancel-roi").style.display = "none";
}

function btnCancelRoiView() {
  fCanvas.remove(selectionRect);
  // --- dom render logic ---
  document.getElementById("get-roi").style.display = "inline";
  document.getElementById("set-roi").style.display = "none";
  document.getElementById("cancel-roi").style.display = "none";
}

let originSprite;
let isOriginSet_ = true;

function btnEditOrigin() {
  isOriginSet_ = false;

  // --- switch off drawing mode ---
  fCanvas.isDrawingMode = false;

  fabric.Image.fromURL(`${getImagePath()}/ucs.png`, function (myImg) {

    // --- get origin position ---
    console.log(`Pos: ${gMapMeta_.origin.x}, ${gMapMeta_.origin.y}`);
    var currPos = tfROS2Canvas(gMapMeta_, {
      x: 0.0,
      y: 0.0
    });
    // console.log(`curr Pos: ${currPos.x}, ${currPos.y}`);

    var currLeft = (currPos.x - (mapOrigin_.width / 2));
    var currTop = (currPos.y - (mapOrigin_.height / 2));

    console.log(`currLeft: ${currLeft}, currTop: ${currTop}`);

    originSprite = myImg.set({
      id: 'mapOrigin',
      left: currLeft,
      top: currTop,
      width: mapOrigin_.width,
      height: mapOrigin_.height,
      hasControls: false
    });

    fCanvas.add(originSprite);
  });

  // --- dom render logic ---
  document.getElementById("get-origin").style.display = "none";
  document.getElementById("set-origin").style.display = "inline";
  document.getElementById("cancel-origin").style.display = "inline";
  document.getElementById("undo-draw").style.display = "none";
}

let mapOrigin_ = {
  left: 0,
  top: 0,
  width: 128,
  height: 128
};

async function btnSetOrigin() {
  isOriginSet_ = true;
  fCanvas.getObjects().forEach(async function (o) {
    if (o.id === 'mapOrigin') {
      console.log('get active object map origin');
      // fCanvas.setActiveObject(o);
      console.log(o.aCoords.tl);
      mapOrigin_.left = o.aCoords.tl.x;
      mapOrigin_.top = o.aCoords.tl.y;
      mapOrigin_.width = o.aCoords.br.x - o.aCoords.tl.x;
      mapOrigin_.height = o.aCoords.br.y - o.aCoords.tl.y;

      var Xcenter = mapOrigin_.left + mapOrigin_.width / 2;
      var Ycenter = mapOrigin_.top + mapOrigin_.height / 2;
      // console.log(`center: (${Xcenter}, ${Ycenter})`);

      var rosPos = tfCanvas2ROS(gMapMeta_, {
        x: Xcenter,
        y: Ycenter
      });
      gMapMeta_.origin.x -= Number(rosPos.x);
      gMapMeta_.origin.y -= Number(rosPos.y);

      // --- redraw routes and cells ---
      visNetwork_.body.data.nodes.clear();
      visNetwork_.body.data.edges.clear();

      visUpdateMapOrigin();

      bCells_ = false;
      visDrawCells(visNetwork_, jsonCellCache_);

      try {
        // --- load graph data from back-end ---
        let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
        let graphData = await res.text();

        // --- rendering on UI ---
        var graph = cvtDot2Json(graphData);
        bNavGraph_ = false;
        visDrawGraph(visNetwork_, graph);
      } catch (err) {
        console.error(err);
      }

    }
  });

  fCanvas.remove(originSprite);

  // --- DOM render logic ---
  document.getElementById("get-origin").style.display = "inline";
  document.getElementById("set-origin").style.display = "none";
  document.getElementById("cancel-origin").style.display = "none";
  document.getElementById("undo-draw").style.display = "inline";

  await updateMapMetaToServer(currSelectedMap_, 'farobot');
  // // --- fetch data from server ---
  // var data = await restGetMapMetaInfo(currSelectedMap_);
  // // console.log(data);

  // // --- data process (origin update) ---
  // data = data.split(/\r?\n/);

  // var updatedData = '';
  // let newUpdatedData = {};
  // for (var i in data) {
  //   // console.log(data[i]);
  //   if (data[i] === "") continue;

  //   var el = data[i].split(':');
  //   var key = el[0];
  //   if (key === 'origin') {
  //     updatedData += `origin: [${gMapMeta_.origin.x}, ${gMapMeta_.origin.y}, 0]` + '\n';
  //     newUpdatedData["origin_x"] = gMapMeta_.origin.x;
  //     newUpdatedData["origin_y"] = gMapMeta_.origin.y;
  //   }
  //   else {
  //     updatedData += data[i] + '\n';
  //     newUpdatedData[key] = data[i];
  //   }
  // }
  // console.log(updatedData)

  // // --- save updated data back to server ---
  // var mapName = currSelectedMap_;
  // // await restPostMapMetaInfo2(mapName, updatedData);
  // let res = await fetchPutMapMeta(rmtToken_, mapName, newUpdatedData);
  // res = await res.json();
  // console.e.log(res);
}

async function updateMapMetaToServer(_mapName, _mode = "farobot") {
  // === 1. update data by swarm APIs ===
  if (_mode === 'farobot') {
    // --- fetch data from server ---
    let res = await fetchGetMapMeta(rmtToken_, _mapName);
    let mapMeta = await res.json()
    console.log(mapMeta);
    mapMeta['origin_x'] = gMapMeta_.origin.x;
    mapMeta['origin_y'] = gMapMeta_.origin.y;

    // --- save updated data back to server ---
    res = await fetchPutMapMeta(rmtToken_, _mapName, mapMeta);
    res = await res.json();
    console.log(res);
    return;
  }

  // === 2. update data by native APIs ===
  // --- fetch data from server ---
  let data = await restGetMapMetaInfo(_mapName);
  // console.log(data);

  // --- data processing (origin update) ---
  data = data.split(/\r?\n/);

  var updatedData = '';
  for (var i in data) {
    // console.log(data[i]);
    if (data[i] === "") continue;

    var el = data[i].split(':');
    var key = el[0];
    if (key === 'origin') {
      updatedData += `origin: [${gMapMeta_.origin.x}, ${gMapMeta_.origin.y}, 0]` + '\n';
    }
    else {
      updatedData += data[i] + '\n';
    }
  }
  console.log(updatedData)

  // --- write back data to server ---
  await restPostMapMetaInfo2(_mapName, updatedData);
}

function btnCancelSetOrigin() {
  cancelOriginEditingMode();
}

function cancelOriginEditingMode() {
  isOriginSet_ = true;
  fCanvas.remove(originSprite);

  // --- dom render logic ---
  document.getElementById("get-origin").style.display = "inline";
  document.getElementById("set-origin").style.display = "none";
  document.getElementById("cancel-origin").style.display = "none";
  document.getElementById("undo-draw").style.display = "inline";
}

async function btnSaveMapImage() {
  if (!isOriginSet_) {
    alert('Orgin is NOT Set Yet!');
    return;
  }

  fCanvasfitToViewport()
  var map_data = fCanvas.toDataURL({
    format: 'png',
    top: 0,
    left: 0,
    width: gMapData_.w,
    height: gMapData_.h
  });

  // --- by Native APIs ---
  // await restPostMapImage(currSelectedMap_, map_data);
  // mapImageChanged = false;
  // notificationMsg(0, 'Map saved successfully!');

  // --- by Swarm APIs ---
  map_data = map_data.split(',')[1];
  console.log(map_data)
  let res = await fetchPutMapImage(rmtToken_, currSelectedMap_, map_data);
  let statusCode = res.status;
  let statusText = await res.json();
  if (statusCode === 200) {
    notificationMsg(1, statusText);
    mapImageChanged = false;
  } else {
    notificationMsg(3, statusText);
  }

}

function btnCropMap() {
  // create a mask rectangle for crop
  let rect = new fabric.Rect({
    left: selectionRect.left,
    top: selectionRect.top,
    width: selectionRect.getScaledWidth(),
    height: selectionRect.getScaledHeight(),
    absolutePositioned: true,
  })

  // --- add the current image clipPath property ---
  currentImage.clipPath = rect;

  // --- remove the mask layer ---
  fCanvas.remove(selectionRect);

  // --- init new image instance ---
  var cropped = new Image();

  // --- set src value of canvas cropped area as toDataURL ---
  cropped.src = fCanvas.toDataURL({
    left: rect.left,
    top: rect.top,
    width: rect.width,
    height: rect.height,
  });

  // --- after onload clear the canvas and add cropped image to the canvas ---
  cropped.onload = function () {
    fCanvas.clear();
    image = new fabric.Image(cropped);
    image.left = rect.left;
    image.top = rect.top;
    image.setCoords();
    fCanvas.add(image);
    fCanvas.renderAll();
  };
}


// ==========================
//     Draw Editing Tools     
// ==========================
// ====== Fabric JS Drawing ======    
let fCanvas = new fabric.Canvas('c', {
  isDrawingMode: false,
  borderColor: 'gray',
  enableRetinaScaling: false
});

let fZoneCanvas = new fabric.Canvas('zoneCanvas', {
  fireRightClick: true,
  stopContextMenu: true
});

let fTempPolygonOption = {
  fill: '#adadad',
  stroke: '#adadad',
  strokeWidth: 1,
  opacity: 0.3,
  selectable: false,
  hasBorders: false,
  hasControls: false,
  evented: false,
  objectCaching: false
}

// $("#edit-mode").click(function () {
//   fCanvas.isDrawingMode = !fCanvas.isDrawingMode;
// if (fCanvas.isDrawingMode) {
//   $("#tool-panel").show();
//   $("#tool-panel2").hide();
//   $("#edit-mode").val("Drawing mode");
// } else {
//   $("#tool-panel").hide();
//   $("#tool-panel2").show();
//   $("#edit-mode").val("Selection mode");
// }
// });

var isRedoing = false;
var firstUndo = true;
var stack_ = [];
var mapGraphQueue_ = [];
var mapImageChanged = false;

fCanvas.on('object:added', function () {
  mapImageChanged = true;
  if (!isRedoing) {
    stack_ = [];
  }
})


$("#undo-draw").click(function () {
  mapImageChanged = (fCanvas._objects.length - 1) > 0;
  if (fCanvas._objects.length > 0) {
    stack_.push(fCanvas._objects.pop());
    fCanvas.renderAll();
  }
});

// $("#redo-draw").click(function () {
//   if(h.length>0){
//     isRedoing = true;
//     fCanvas.add(h.pop());
//   }
// });

$("#save-png").click(function () {
  fCanvas.isDrawingMode = false;
  var content = document.getElementById("canvas-content");
  content.innerHTML = fCanvas.toDataURL('png');
});

$("#save-json").click(function () {
  fCanvas.isDrawingMode = false;
  var content = document.getElementById("canvas-content");
  content.innerHTML = JSON.stringify(fCanvas);
});

$("#add-object").click(function () {
  setSelectedZones();
});

$("#remove-object").click(function () {
  var activeObject = fCanvas.getActiveObject();
  if (activeObject) {
    fCanvas.remove(activeObject);
  }
});

fCanvas.on('mouse:wheel', function (opt) {
  var delta = opt.e.deltaY;
  // var zoom = fCanvas.getZoom();
  var zoom = fCanvas.getZoom() * canvasScale_;
  zoom *= 0.999 ** delta;
  if (zoom > 30) zoom = 30;
  // if (zoom < 0.01) zoom = 0.01;
  if (zoom < 1) zoom = 1;
  // fCanvas.zoomToPoint({
  //   x: opt.e.offsetX,
  //   y: opt.e.offsetY
  // }, zoom);
  // fCanvas.setZoom(zoom);
  fCanvas.setZoom(zoom * (1 / canvasScale_));
  opt.e.preventDefault();
  opt.e.stopPropagation();

  // // --- grid canvas ---
  // zoom = gridCanvas.getZoom();
  // zoom *= 0.999 ** delta;
  // if (zoom > 10) zoom = 10;
  // if (zoom < 1) zoom = 1;
  // gridCanvas.zoomToPoint({
  //   x: opt.e.offsetX,
  //   y: opt.e.offsetY
  // }, zoom);
  // gridCanvas.setZoom(zoom);
  // opt.e.preventDefault();
  // opt.e.stopPropagation();
})

fCanvas.on('mouse:down', function (opt) {
  var evt = opt.e;
  this.isDragging = true;
  this.selection = false;
  if (evt instanceof TouchEvent) {
    const { clientX, clientY } = evt.touches[0]
    this.lastPosX = clientX
    this.lastPosY = clientY
  } else {
    this.lastPosX = evt.clientX;
    this.lastPosY = evt.clientY;
  }
  console.log(fCanvas.item(0));
  console.log(opt);
  if (opt.target === null) return;
  if (opt.target.id === 'mapOrigin') {
    this.isDragging = false;
  }
});

fCanvas.on('mouse:move', function (opt) {
  if (this.isDragging) {
    var e = opt.e;
    var vpt = this.viewportTransform;
    if (e instanceof TouchEvent) {
      const { clientX, clientY } = e.touches[0];
      // vpt[4] += (clientX - this.lastPosX) * dragScale_;
      // vpt[5] += (clientY - this.lastPosY) * dragScale_;
      vpt[4] += (clientX - this.lastPosX);
      vpt[5] += (clientY - this.lastPosY);
      this.requestRenderAll();
      this.lastPosX = clientX;
      this.lastPosY = clientY;
    } else {
      // vpt[4] += (e.clientX - this.lastPosX) * dragScale_;
      // vpt[5] += (e.clientY - this.lastPosY) * dragScale_;
      vpt[4] += (e.clientX - this.lastPosX);
      vpt[5] += (e.clientY - this.lastPosY);
      this.requestRenderAll();
      this.lastPosX = e.clientX;
      this.lastPosY = e.clientY;
    }
  }
});

fCanvas.on('mouse:up', function (opt) {
  this.setViewportTransform(this.viewportTransform);
  this.isDragging = false;
  this.selection = true;
  if (!isTouchDevice()) return;
  if (pinchZoom_ && originSprite !== undefined) {
    fCanvas.getObjects().forEach(async function (o) {
      if (o.id === 'mapOrigin') {
        fCanvas.remove(o);
      }
    });
    fCanvas.add(originSprite);
  }
});

let selShape;
fZoneCanvas.on('mouse:down', function (opt) {
  if (opt.button !== 1 || opt.target === null || !opt.target.id.includes(ZONE_POLYGON_PREFIX)) return;
  // --- click on polygon ---

  // --- open zone editor ---
  let zone_uuid = opt.target.id.replace(ZONE_POLYGON_PREFIX, '');
  let switchNode = document.querySelector(`#zone-container-${zone_uuid} .activation-switch`);
  if (switchNode.checked) return;
  document.getElementById(`zone-container-${zone_uuid}`).click();
  fZoneCanvas.selection = false;
  activeSelectedZone(opt.target);
  selShape = opt.target;
});

function resetSelectedPolygonPoints() {
  if (selShape === undefined) return;
  fZoneCanvas.discardActiveObject().renderAll();
  removeFabricZoneObjects(ZONE_POINT_PREFIX + selShape.id.replace(ZONE_POLYGON_PREFIX, ''));
  selShape = undefined;
}

function addPoint(opt) {
  toggleToolbarEditing(false);
  let selType = document.getElementById('zone-type-select').selectedIndex;
  let type = zoneTypes_[selType] || 'none';
  let color = type.pointColor || 'black';
  let zoneId = jsonZoneCache_.uuid;
  let zoom = fZoneCanvas.getZoom();
  const pointOption = {
    id: ZONE_POINT_PREFIX + zoneId,
    radius: 5 / zoom,
    fill: color,
    stroke: color,
    strokeWidth: 1,
    left: opt.e.layerX / zoom,
    top: opt.e.layerY / zoom,
    selectable: false,
    hasBorders: false,
    hasControls: false,
    originX: 'center',
    originY: 'center',
    objectCaching: false
  };
  var point = new fabric.Circle(pointOption);

  // --- fill first point with highlight color ---
  if (pointArray.length === 0) {
    point.set({
      radius: 7 / zoom,
      fill: 'red'
    });
  } else {
    // --- check for overlapping zones(boundary level) ---
    let p1 = pointArray[pointArray.length - 1];
    let p2 = point;
    let zoneIds = getLineIntersectZones(p1, p2);
    let isOverlapping = isIntersectSameZone(zoneIds);
    if (isOverlapping) {
      alert('Same zone type and priority can NOT overlap!');
      fZoneCanvas.selection = false;
      return;
    }
  }
  fZoneCanvas.add(point);
  pointArray.push(point);

  // --- add a new line with the same start and end point ---
  const linePoints = [
    opt.e.layerX / zoom,
    opt.e.layerY / zoom,
    opt.e.layerX / zoom,
    opt.e.layerY / zoom,
  ];
  const lineOption = {
    id: ZONE_LINE_PREFIX + zoneId,
    fill: color,
    stroke: color,
    strokeWidth: 5 / zoom,
    selectable: false,
    hasBorders: false,
    hasControls: false,
    originX: 'center',
    originY: 'center',
    evented: false,
    objectCaching: false
  };
  var line = new fabric.Line(linePoints, lineOption);
  line.class = 'line';
  fZoneCanvas.add(line);
  activeLine = line;
  lineArray.push(line);

  if (activeShape) {
    const points = activeShape.get('points');
    points.push({
      x: opt.e.layerX / zoom,
      y: opt.e.layerY / zoom
    });
    var polygon = new fabric.Polygon(points, fTempPolygonOption);
    fZoneCanvas.remove(activeShape);
    fZoneCanvas.add(polygon);
    fZoneCanvas.renderAll();
    activeShape = polygon;
  } else {
    const polyPoint = [{
      x: opt.e.layerX / zoom,
      y: opt.e.layerY / zoom,
    },];
    var polygon = new fabric.Polygon(polyPoint, fTempPolygonOption);
    fZoneCanvas.add(polygon);
    activeShape = polygon;
  }
}

function generatePolygonZone() {
  var points = [];
  for (const point of pointArray) {
    points.push({
      x: point.left,
      y: point.top,
    });
    fZoneCanvas.remove(point);
  }

  // --- remove lines from canvas ---
  for (const line of lineArray) {
    fZoneCanvas.remove(line);
  }

  // --- remove selected shape ---
  fZoneCanvas.remove(activeShape);

  // --- create polygon from collected points ---
  let zoneId = jsonZoneCache_.uuid;
  let selType = document.getElementById('zone-type-select').selectedIndex;
  let selPrio = document.getElementById('zone-priority-select').selectedIndex;
  let type = zoneTypes_[selType] || 'none';
  let color = type.fillColor || 'gray';
  var polygon = new fabric.Polygon(points, {
    id: ZONE_POLYGON_PREFIX + zoneId,
    stroke: color,
    fill: color,
    opacity: selPrio === 0 ? 0.3 : 0.8,
    objectCaching: false,
    moveable: false,
    selectable: false
  });
  fZoneCanvas.add(polygon);

  // --- draw text inside polygon ---
  // var text = new fabric.Text("Zone", {
  //   left: getPolygonCenterCoords(points).x,
  //   top: getPolygonCenterCoords(points).y,
  //   fill: color,
  //   fontSize: 20
  // });
  // fZoneCanvas.add(text);

  // --- change back first point color ---
  // if (pointArray.length > 0) {
  //   pointArray[0].set({
  //     fill: color
  //   });
  // }

  // --- save vertices to zone cache ---(
  updateZoneVertices(zoneId, points);

  toggleZoneEditTabButtons();

  // --- close draw mode ---
  lineArray = [];
  pointArray = [];
  activeLine = null;
  activeShape = null;
  isDrawMode = false;
  fZoneCanvas.selection = true;
}

function getPolygonIntersectZones(id, pol) {
  let overlapZoneIds = [];
  let cpPoints = JSON.parse(JSON.stringify(pol));
  cpPoints.map((point) => {
    var visPos = tfROS2Canvas(gMapMeta_, {
      x: Number(point.x),
      y: Number(point.y)
    });
    point.x = Number(visPos.x);
    point.y = Number(visPos.y);
    return point;
  });

  fZoneCanvas.forEachObject(function (o) {
    if (!o.hasOwnProperty('id')) return;
    if (!o.id.includes(ZONE_POLYGON_PREFIX)) return;
    let zoneId = o.id.replace(ZONE_POLYGON_PREFIX, '');
    if (id === zoneId) return;
    let intPols = intersectPolygons(cpPoints, o.points);
    if (intPols.length > 0) {
      overlapZoneIds.push(zoneId);
    }
  });
  return overlapZoneIds;
}

function getLineIntersectZones(p1, p2) {
  let overlapZoneIds = [];
  let isIntersect = false;
  fZoneCanvas.forEachObject(function (o) {
    if (!o.hasOwnProperty('id')) return;
    if (!o.id.includes(ZONE_POLYGON_PREFIX)) return;
    isIntersect = false;
    let edges = geometry.polygonLines(o.points);
    let drawFromPoint = new Point2D(p1.left, p1.top);
    let drawToPoint = new Point2D(p2.left, p2.top);
    let drawLine = new Line2D(drawFromPoint, drawToPoint);
    for (let i = 0; i < edges.length; i++) {
      isIntersect = geometry.lineIntersectLine(drawLine, edges[i]);
      if (isIntersect) break;
    }
    if (isIntersect) {
      overlapZoneIds.push(o.id.replace(ZONE_POLYGON_PREFIX, ''));
    }
  });
  return overlapZoneIds;
}

function getVertexIntersectZones(p) {
  let overlapZoneIds = [];
  fZoneCanvas.forEachObject(function (o) {
    if (!o.hasOwnProperty('id')) return;
    if (!o.id.includes(ZONE_POLYGON_PREFIX) || !geometry.pointInPolygon(o.points, o.points.length, p)) return;
    overlapZoneIds.push(o.id.replace(ZONE_POLYGON_PREFIX, ''));
  });
  return overlapZoneIds;
}

function isIntersectSameZone(_overlapIds) {
  let selType = document.getElementById('zone-type-select').selectedIndex;
  let selPrio = document.getElementById('zone-priority-select').selectedIndex;
  let isIntersect = false;
  if (_overlapIds.length > 0) {
    _overlapIds.forEach(id => {
      let overlapZone = _.find(jsonZoneArray, { uuid: id });
      let isDupType = (overlapZone.zone_type === selType);
      let isDupPriority = (overlapZone.priority === selPrio);
      if (isDupType && isDupPriority) {
        isIntersect = true;
        return;
      }
    });
    return isIntersect;
  }
  return false;
}

function getSameZones(_overlapIds, _type, _priority) {
  let overlapZoneNames = [];
  _overlapIds.forEach(id => {
    let overlapZone = _.find(jsonZoneArray, { uuid: id });
    let isDupType = (overlapZone.zone_type === _type);
    let isDupPriority = (overlapZone.priority === _priority);
    if (isDupType && isDupPriority) {
      overlapZoneNames.push(overlapZone.zone_name);
    }
  });
  return overlapZoneNames;
}

function tfPolygonPoints2ROS(_points) {
  let cpPoints = JSON.parse(JSON.stringify(_points));
  cpPoints.map((obj) => {
    var rosPos = tfCanvas2ROS(gMapMeta_, { x: obj.x, y: obj.y });
    obj.x = Number(rosPos.x);
    obj.y = Number(rosPos.y);
    obj.theta = 0;
    return obj;
  });
  return cpPoints;
}

// function getPolygonCenterCoords(points) {
//   let pointsCnt = points.length;
//   const x = points.reduce((sum, dot) => sum += dot.x, 0) / pointsCnt;
//   const y = points.reduce((sum, dot) => sum += dot.y, 0) / pointsCnt;

//   return { x, y };
// }

function toggleToolbarEditing(_status = true) {
  document.getElementById('zone-type-select').disabled = !_status;
  document.getElementById('zone-priority-select').disabled = !_status;
  document.getElementById('delete-zone-polygon').disabled = !_status;
  document.getElementById('save-changes').disabled = !_status;
}

function toggleZoneEditTabButtons() {
  let zoneExists = jsonZoneCache_.hasOwnProperty('vertices');
  document.getElementById('add-zone-polygon').style.display = zoneExists ? "none" : "block";
  document.getElementById('edit-zone-polygon').style.display = zoneExists ? "block" : "none";
  document.getElementById('add-zone-vertex').style.display = zoneExists ? "block" : "none";
  document.getElementById('end-zone-vertex').style.display = "none";
  document.getElementById('delete-zone-vertex').style.display = zoneExists ? "block" : "none";

  resetAddZoneVertexParams();
  toggleZoneEditButtonState();
}

function toggleZoneEditButtonState(_mode = 'none') {
  document.getElementById('cancel-zone-edit').style.display = "block";
  document.getElementById('cancel-zone-edit').disabled = false;
  switch (_mode) {
    case 'delete':
      document.getElementById('edit-zone-polygon').disabled = true;
      document.getElementById('delete-zone-vertex').disabled = false;
      document.getElementById('add-zone-vertex').disabled = true;
      break;
    case 'edit':
      document.getElementById('edit-zone-polygon').disabled = false;
      document.getElementById('delete-zone-vertex').disabled = true;
      document.getElementById('add-zone-vertex').disabled = true;
      break;
    case 'add':
      document.getElementById('edit-zone-polygon').disabled = true;
      document.getElementById('delete-zone-vertex').disabled = true;
      document.getElementById('add-zone-vertex').disabled = false;
      break;
    case 'none':
      document.getElementById('edit-zone-polygon').disabled = false;
      document.getElementById('delete-zone-vertex').disabled = false;
      document.getElementById('add-zone-vertex').disabled = false;
      document.getElementById('cancel-zone-edit').style.display = "none";
      break;
  }
}

function fitToViewport() {
  switch (selectedEditType) {
    case 'rdMapImg':
      fCanvasfitToViewport();
      fCanvasZoomIn(fCanvas, 1 / canvasScale_);
      break;
    case 'rdZone':
      break;
    default:
      visFitToViewport();
  }
}

function fCanvasfitToViewport() {
  // --- canvas.setViewportTransform([zoom, 0, 0, zoom, panX, panY]) ---
  fCanvas.setViewportTransform([1, 0, 0, 1, 0, 0]);
  // gridCanvas.setViewportTransform([1, 0, 0, 1, 0, 0]);
}

function fCanvasZoomIn(cvs_, ratio_) {
  cvs_.setZoom(ratio_);
}

function visFitToViewport() {
  visNetwork_.moveTo({
    position: {
      x: gMapData_.w / 2,
      y: gMapData_.h / 2
    },
    scale: (mapViewSize / gMapData_.h)
  });
}

function viewMode() {
  // --- additional tools invisible ---
  hideAdditionalTools();

  // --- reset origin related buttons
  cancelOriginEditingMode();

  // --- unbind All Event ---
  removeCanvasEvents();
  changeCanvasProperty(false, false);

  // --- recover to default mode ---
  fCanvas.on('mouse:down', function (opt) {
    var evt = opt.e;
    this.isDragging = true;
    this.selection = false;
    if (evt instanceof TouchEvent) {
      const { clientX, clientY } = evt.touches[0]
      this.lastPosX = clientX
      this.lastPosY = clientY
    } else {
      this.lastPosX = evt.clientX;
      this.lastPosY = evt.clientY;
    }
    console.log(fCanvas.item(0));
    console.log(opt);
    if (opt.target === null) return;
    if (opt.target.id === 'mapOrigin') {
      this.isDragging = false;
    }
  });

  fCanvas.on('mouse:move', function (opt) {
    if (this.isDragging) {
      var e = opt.e;
      var vpt = this.viewportTransform;
      if (e instanceof TouchEvent) {
        const { clientX, clientY } = e.touches[0];
        // vpt[4] += (clientX - this.lastPosX) * dragScale_;
        // vpt[5] += (clientY - this.lastPosY) * dragScale_;
        vpt[4] += (clientX - this.lastPosX);
        vpt[5] += (clientY - this.lastPosY);
        this.requestRenderAll();
        this.lastPosX = clientX;
        this.lastPosY = clientY;
      } else {
        // vpt[4] += (e.clientX - this.lastPosX) * dragScale_;
        // vpt[5] += (e.clientY - this.lastPosY) * dragScale_;
        vpt[4] += (e.clientX - this.lastPosX);
        vpt[5] += (e.clientY - this.lastPosY);
        this.requestRenderAll();
        this.lastPosX = e.clientX;
        this.lastPosY = e.clientY;
      }
    }
  });

  fCanvas.on('mouse:up', function (opt) {
    this.setViewportTransform(this.viewportTransform);
    this.isDragging = false;
    this.selection = true;
  });
}

// --- shape properties can be specified in JSON format ---
var prohib_rect = new fabric.Rect({
  left: 200,
  top: 150,
  fill: 'red',
  width: 200,
  height: 50,
  opacity: 0.5,
  stroke: 'black',
  strokeWidth: 1
});

// --- add objects on the canvas ---
var prefer_rect = new fabric.Rect({
  left: 300,
  top: 250,
  width: 200,
  height: 50,
  fill: 'lime',
  rx: 10,
  ry: 10,
  opacity: 0.5,
  objectCaching: false
});

prefer_rect.on('scaling', function () {
  this.set({
    width: this.width * this.scaleX,
    height: this.height * this.scaleY,
    scaleX: 1,
    scaleY: 1
  })
})

$("#font-size").click(function () {
  var context = fCanvas.getContext("2d");
  context.lineWidth = $("#font-size").val();
  console.log(`change lineWidth ${context.lineWidth}`);
})

// $("#pen-draw").click(function () {
//   removeCanvasEvents();
//   var val = fCanvas.isDrawingMode;
//   val = !val;
//   changeSelectableStatus(false);
//   changeCanvasProperty(false, val);
//   var width = $("#font-size").val();
//   // console.log("brush width: "+width);
//   fCanvas.freeDrawingBrush.color = "#000000";
//   fCanvas.freeDrawingBrush.width = width;
// })

$("#erase-draw").click(function () {
  hideAdditionalTools();
  removeCanvasEvents();

  changeSelectableStatus(false);
  changeCanvasProperty(false, true);
  var width = 10;
  fCanvas.freeDrawingBrush.color = "#FFFFFF";
  fCanvas.freeDrawingBrush.width = width;
})

let line;
$("#line-draw").click(function () {
  showAdditionalTools();
  removeCanvasEvents();

  changeSelectableStatus(false);
  changeCanvasProperty(false, false);
  var lineWidth = $('#widthNum').val() || 5;
  line = new Line(fCanvas);
  line.setWidth(lineWidth);
})

$('#line-width-slider').change(function () {
  var lineWidth = $('#widthNum').val() || 5;
  console.log(`change line width to ${lineWidth}!`);
  line = new Line(fCanvas);
  line.setWidth(lineWidth);
})

// $("#rect-draw").click(function () {
//   removeCanvasEvents();
//   changeSelectableStatus(false);
//   changeCanvasProperty(false, false);
//   var squrect = new Rectangle(fCanvas);
// });

function removeCanvasEvents() {
  fCanvas.off('mouse:down');
  fCanvas.off('mouse:move');
  fCanvas.off('mouse:up');
  fCanvas.off('object:moving');
}

function removeZoneCanvasEvents() {
  fZoneCanvas.off('mouse:down');
  fZoneCanvas.off('mouse:move');
  fZoneCanvas.off('mouse:up');
}

function changeSelectableStatus(val) {
  fCanvas.forEachObject(function (obj) {
    obj.selectable = val;
  })
  fCanvas.renderAll();
}

function changeCanvasProperty(selValue, drawingVal) {
  fCanvas.selection = selValue;
  fCanvas.isDrawingMode = drawingVal;
}


// =========================
//       Drawing Tools    
// =========================
// ------ Line ------ 
var Line = (function () {
  function Line(canvas) {
    this.canvas = canvas;
    this.isDrawing = false;
    this.bindEvents();
    this.strokeWidth_ = 2;
  }

  Line.prototype.bindEvents = function () {
    var inst = this;
    inst.canvas.on('mouse:down', function (o) {
      inst.onMouseDown(o);
    });
    inst.canvas.on('mouse:move', function (o) {
      inst.onMouseMove(o);
    });
    inst.canvas.on('mouse:up', function (o) {
      inst.onMouseUp(o);
    });
    inst.canvas.on('object:moving', function (o) {
      inst.disable();
    })
  }

  Line.prototype.onMouseUp = function (o) {
    console.log('---- on mouse up ----');
    var inst = this;
    if (inst.isEnable()) {
      inst.disable();
    }
  };

  Line.prototype.onMouseMove = function (o) {
    console.log('---- on mouse move ----');
    var inst = this;
    if (!inst.isEnable()) { return; }

    console.log('--- on mouse move 2 ---');
    var pointer = inst.canvas.getPointer(o.e);
    var activeObj = inst.canvas.getActiveObject();

    activeObj.set({
      x2: pointer.x,
      y2: pointer.y
    });
    console.log('--- on mouse move: set Coords ---');
    activeObj.setCoords();
    inst.canvas.renderAll();
  };

  Line.prototype.onMouseDown = function (o) {
    console.log('---- on mouse down ----');
    var inst = this;
    inst.enable();

    var pointer = inst.canvas.getPointer(o.e);
    origX = pointer.x;
    origY = pointer.y;

    var points = [pointer.x, pointer.y, pointer.x, pointer.y];
    var line = new fabric.Line(points, {
      strokeWidth: this.strokeWidth_,
      stroke: 'black',
      fill: 'black',
      originX: 'center',
      originY: 'center',
      selectable: false,
      hasBorders: false,
      hasControls: false
    });
    console.log('--- test point ---');
    inst.canvas.add(line).setActiveObject(line);
  };

  Line.prototype.isEnable = function () {
    return this.isDrawing;
  }

  Line.prototype.enable = function () {
    this.isDrawing = true;
  }

  Line.prototype.disable = function () {
    this.isDrawing = false;
  }

  Line.prototype.setWidth = function (w) {
    this.strokeWidth_ = w;
  }

  return Line;
}());

// ------ Rectangle ------ 
// var Rectangle = (function () {
//   function Rectangle(canvas) {
//     var inst = this;
//     this.canvas = canvas;
//     this.className = 'Rectangle';
//     this.isDrawing = false;
//     this.bindEvents();
//   }

//   Rectangle.prototype.bindEvents = function () {
//     var inst = this;
//     inst.canvas.on('mouse:down', function (o) {
//       inst.onMouseDown(o);
//     });
//     inst.canvas.on('mouse:move', function (o) {
//       inst.onMouseMove(o);
//     });
//     inst.canvas.on('mouse:up', function (o) {
//       inst.onMouseUp(o);
//     });
//     inst.canvas.on('object:moving', function (o) {
//       inst.disable();
//     })
//   }

//   Rectangle.prototype.onMouseUp = function (o) {
//     var inst = this;
//     inst.disable();
//   };

//   Rectangle.prototype.onMouseMove = function (o) {
//     var inst = this;

//     if (!inst.isEnable()) {
//       return;
//     }
//     console.log("mouse move rectange");
//     var pointer = inst.canvas.getPointer(o.e);
//     var activeObj = inst.canvas.getActiveObject();

//     activeObj.stroke = 'black',
//       activeObj.strokeWidth = 5;
//     activeObj.fill = 'transparent';

//     if (origX > pointer.x) {
//       activeObj.set({
//         left: Math.abs(pointer.x)
//       });
//     }
//     if (origY > pointer.y) {
//       activeObj.set({
//         top: Math.abs(pointer.y)
//       });
//     }

//     activeObj.set({
//       width: Math.abs(origX - pointer.x)
//     });
//     activeObj.set({
//       height: Math.abs(origY - pointer.y)
//     });

//     activeObj.setCoords();
//     inst.canvas.renderAll();

//   };

//   Rectangle.prototype.onMouseDown = function (o) {
//     var inst = this;
//     inst.enable();

//     var pointer = inst.canvas.getPointer(o.e);
//     origX = pointer.x;
//     origY = pointer.y;

//     var rect = new fabric.Rect({
//       left: origX,
//       top: origY,
//       originX: 'left',
//       originY: 'top',
//       width: pointer.x - origX,
//       height: pointer.y - origY,
//       selectable: false,
//       hasBorders: false,
//       hasControls: false
//     });

//     inst.canvas.add(rect).setActiveObject(rect);
//   };

//   Rectangle.prototype.isEnable = function () {
//     return this.isDrawing;
//   }

//   Rectangle.prototype.enable = function () {
//     this.isDrawing = true;
//   }

//   Rectangle.prototype.disable = function () {
//     this.isDrawing = false;
//   }

//   return Rectangle;
// }());


// ===============================
//       VIS-NETWORK DRAWING 
// ===============================
// --- network initialization ---
let visNodes_ = new vis.DataSet([]);
let visEdges_ = new vis.DataSet([]);
let visContainer_ = $("#far-network2");

// --- Configuration ---
let visOptions_ = {
  autoResize: true,
  layout: {
    hierarchical: {
      enabled: false
    }
  },
  physics: {
    enabled: false
  },
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
    color: "rgba(87, 87, 87, 0.9)",
    value: 1,
    scaling: {
      min: 1,
      max: 2
    },
    shadow: true,
    dashes: false, // smooth enabled false will affect dash lines
    smooth: {
      enabled: false
    },
    arrows: {
      to: {
        enabled: true,
        scaleFactor: 0.3,
        type: "arrow"
      }
    },
    background: {
      enabled: true,
      color: "white"
    }
  },
  // dragView: pan function
  interaction: {
    hover: false,
    multiselect: false,
    dragView: true
  },
  manipulation: {},
  groups: {
    navnode: {
      color: {
        background: "gray",
        border: "black"
      }
    },
    wms: {
      color: {
        background: "gray",
        border: "black"
      }
    }
  }
};


// --- create a network instance ---
visNetwork_ = new vis.Network(visContainer_[0], {
  nodes: visNodes_,
  edges: visEdges_
}, visOptions_);

visNetwork_.on("beforeDrawing", function (ctx) {
  // console.log('--- before drawing ---');
  // --- draw map image ---
  var canvas = document.getElementById('map_canvas2');
  if (canvas === null) { return; }
  var width = canvas.width;
  var height = canvas.height;

  // --- clean before drawing ---
  ctx.clearRect(0, 0, width, height);

  if (canvas === null) { return; }
  ctx.drawImage(canvas, 0, 0, width, height);
  // ctx.globalAlpha = 1.0;
});

visNetwork_.on("afterDrawing", function (ctx) {
  // console.log('--- after drawing ---');

  ctx.beginPath();
  let startX = 0;
  let startY = 0;
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group === 'zonenode') {
      let visX = visNetwork_.body.nodes[i].options.x;
      let visY = visNetwork_.body.nodes[i].options.y;
      if (visNetwork_.body.nodes[i].options.id.includes('start')) {
        startX = Number(visX);
        startY = Number(visY);
      }
      ctx.lineTo(visX, visY);
    }
  }

  if (isEmptyString(last_node_id)) {
    ctx.fillStyle = 'rgba(148, 167, 245, 0.8)';
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = 'Blue';
    ctx.font = '16px Arial';
    ctx.fillText('Zone name', startX, startY);
  }
});

// --- click with selecting edge on `Route` layer ---
visNetwork_.on("selectEdge", function (props) {
  let manipulationMode = visNetwork_.manipulation.inMode;
  console.log("check manipulation mode when select edge on map: " + manipulationMode);

  // let selectedEditType = $('.custom-radio input[type=radio]:checked').val();
  if (props.nodes.length == 0 && selectedEditType === 'rdRoute') {
    visNetwork_.editEdgeMode();
    showManipulationModeText('Edit Edge');
  }

  hideDeleteButtons();
  // document.getElementById("edit-edge").style.display = "inline";
  document.getElementById("delete-node").style.display = "inline";
});

visNetwork_.on("doubleClick", function (params) {
  if (!isTouchDevice()) { return; }
  editNodeHandler(params);
});

visNetwork_.on("oncontext", function (params) {
  if (isTouchDevice()) { return; }
  editNodeHandler(params);
});

visNetwork_.on("selectNode", function (props) {
  if (props.nodes[0] !== last_node_id && props.nodes[0].includes('start')) {
    // --- add an end connection edge ---
    var dir_edges = [];
    var dir_edge = {
      from: last_node_id,
      to: props.nodes[0],
      arrows: {
        to: {
          enabled: false
        }
      }
    }
    dir_edges.push(dir_edge);
    visNetwork_.body.data.edges.update(dir_edges);
    last_node_id = '';
  }
});

// --- dummy click without selecting anything ---
visNetwork_.on("click", function (props) {
  let manipulationMode = visNetwork_.manipulation.inMode;
  console.log("check manipulation mode when click on map: " + manipulationMode);

  if (isEmptyString(last_node_id)) {
    var polygon = [];
    for (i in visNetwork_.body.nodes) {
      if (visNetwork_.body.nodes[i].options.group === 'zonenode') {
        let visX = visNetwork_.body.nodes[i].options.x;
        let visY = visNetwork_.body.nodes[i].options.y;
        polygon.push([Number(visX), Number(visY)]);
      }
    }
    if (polygon.length === 0) return;
    let inside = coordIsInsidePolygon([props.pointer.canvas.x, props.pointer.canvas.y], polygon);
    console.log('point is inside: ' + inside);
  }

  clearNodePopUp();

  if (manipulationMode === 'editEdge') { return; }

  hideDeleteButtons();

  if (selectedEditType === 'rdCell' || selectedEditType === 'rdConnCell') {
    if (manipulationMode === 'addNode') { return; }
    hideCancelEditButton();
    hideManipulationModeText();
  } else if (selectedEditType === 'rdRoute') {
    if (manipulationMode === 'addNode' || manipulationMode === 'addRoute') { return; }
    if (manipulationMode !== 'editNode') { return; }
    hideCancelEditButton();
    hideManipulationModeText();
  }

  // let ids = props.nodes;
  // let selNode = visNetwork_.body.data.nodes.get(ids);
  // let selNodeGroup = "";
  // if (selNode.length) {
  //   selNodeGroup = selNode[0].group;
  // }

  // if (selectedEditType === 'rdCell') {
  //   if (props.nodes.length == 0 || selNodeGroup === 'navnode') {
  //     hideDeleteButtons();
  //     clearNodePopUp();
  //     if (manipulationMode === 'addNode') return;
  //     hideManipulationModeText();
  //   }
  // } else if (selectedEditType === 'rdRoute') {
  //   if ((props.edges.length == 0 && props.nodes.length == 0) || selNodeGroup === 'wms') {
  //     hideDeleteButtons();
  //     if (manipulationMode === 'addNode' || manipulationMode === 'addEdge') return;
  //     hideManipulationModeText();
  //   }

  //   // As long as the selected node isn't a nav node
  //   if (selNodeGroup !== 'navnode') {
  //     clearNodePopUp();
  //   }
  // }
});

function editNodeHandler(params) {
  params.event.preventDefault();

  // --- 1. identify the selected node ---
  var nodeID = visNetwork_.getNodeAt(params.pointer.DOM);
  let selNode = visNetwork_.body.data.nodes.get(nodeID);

  // --- 2. identify the selected edit properties ---
  // let selectedEditType = $('.custom-radio input[type=radio]:checked').val();

  // --- 3. switch-on edit-mode & toolbar preparation ---
  // --- case 3-1 NavGraph Layer ---
  let manipulationMode = visNetwork_.manipulation.inMode;
  console.log("check manipulation mode when right click/double taps node on map: " + manipulationMode);

  if (selNode.group === 'navnode' && selectedEditType === 'rdRoute') {
    visNetwork_.selectNodes([nodeID]);
    visNetwork_.editNode();
    showManipulationModeText('Edit Vertex');

    hideDeleteButtons();
    // document.getElementById("edit-node").style.display = "inline";
    document.getElementById("delete-node").style.display = "inline";
    return;
  }

  // --- case 3-2 Cell Layer ---
  if (selNode.group === 'wms' && selectedEditType === 'rdCell') {
    visNetwork_.selectNodes([nodeID]);
    visNetwork_.editNode();
    showManipulationModeText('Edit Cell');

    hideDeleteButtons();
    // document.getElementById("edit-cell").style.display = "inline";
    document.getElementById("delete-cell").style.display = "inline";
    return;
  }
}

$(document).on('click', '#route-tool button', function () {
  if (this.id === 'add-node' || this.id === 'add-edge') {
    document.getElementById("delete-node").style.display = "none";
  }
});

$(document).on('click', '#cell-tool button', function () {
  if (this.id === 'add-cell') {
    document.getElementById("delete-cell").style.display = "none";
  }
});

// ========================
//   Vis-Network Data I/O     
// ========================
var currJsonGraph = {};
// ------ export network ------
function addConnections(elem, index) {
  elem.connections = visNetwork_.getConnectedNodes(elem.id);
}

function objectToArray(obj) {
  // console.log(obj);
  return Object.keys(obj).map(function (key) {
    obj[key].id = key;
    return obj[key];
  });
}

let jsonNavGraphCache_ = {};
let graphNodeNames_ = [];

function visDrawGraph(_network, _graph) {
  // console.log('--- vis draw graph ---');

  if (bNavGraph_) { return; }
  bNavGraph_ = true;

  // console.log(JSON.stringify(_graph));
  // var txtArea = document.getElementById('ta-graph');

  // --- nodes' coordinates transformation ---
  _graph.nodes.forEach((n) => {
    var tooltip = `<div class="farobot-map-tooltip" style="color: gray;">${n.label}@(${Number(n.x).toFixed(2)}, ${Number(n.y).toFixed(2)})</div>`;
    var pos = tfROS2Canvas(gMapMeta_, {
      x: n.x,
      y: n.y
    });
    n.x = pos.x;
    n.y = pos.y;
    n.group = 'navnode';
    n.title = tooltip;
  });

  // --- update nodes ---
  _network.body.data.nodes.update(_graph.nodes);

  // --- update edges ---
  _network.body.data.edges.update(_graph.edges);

  var nodes = objectToArray(_network.getPositions());
  nodes.forEach(addConnections);
};


// -- draw storage cells from json file --
let jsonCellCache_ = {};

function visDrawCellsTypeColor(_network, _cells) {
  let nodes = [];
  for (key in _cells) {
    var area = _cells[key];
    for (let i = 0; i < area.length; i++) {
      // var cellUUID = genUuid().replace(/-/g, '');
      // area[i].cell_uuid = cellUUID;
      var cell_id = area[i].cell_uuid;

      var cellStyle = _.flatten(Object.values(functionTypes_)).find(ft => ft.type === area[i].function_type);

      if (cellStyle === undefined) {
        cellStyle = {};
        cellStyle["recognition"] = [area[i].type];
      }
      cellStyle = cellStyle.recognition[0];

      var wmsNode = {};
      wmsNode.id = cell_id;
      wmsNode.color = {
        highlight: getColor(cellDetectionTypes_, cellStyle, 'focusColor'),
        background: getColor(cellDetectionTypes_, cellStyle, 'bgColor'),
        border: getColor(cellDetectionTypes_, cellStyle, 'borderColor')
      };
      nodes.push(wmsNode);

      // _network.body.data.nodes.update(wmsNode);
    }
  }
  _network.body.data.nodes.update(nodes);
}

function visDrawCells(_network, _cells) {
  // -- [protection] confirm the data is updated --
  if (bCells_) { return; }
  bCells_ = true;

  // var ang = 0;

  // --- read the data from WMS json file ---
  var nodes = [];
  for (key in _cells) {
    var area = _cells[key];

    for (let i = 0; i < area.length; i++) {
      var cellUUID = genUuid().replace(/-/g, '');
      area[i].cell_uuid = cellUUID;
      // console.log(area[i]);
      var cell_id = area[i].cell_id;
      // var status = area[i].if_occupied;
      var x = area[i].cell_coordinate[0];
      var y = area[i].cell_coordinate[1];
      var pos = tfROS2Canvas(gMapMeta_, {
        x: x,
        y: y
      });

      var r = cvtRad2Deg(Number(area[i].cell_coordinate[2])).toFixed(2);
      // ang = -cvtRad2Deg(r).toFixed(2);
      // var svgHtml =
      //   `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
      //     <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#044B94" fill-opacity="0.4"/>
      //    </svg>`;
      // var svgUrl = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtml);

      // console.log(area[i]);
      // console.log(area[i].function_type);
      // console.log(functionTypes_);
      var cellStyle = _.flatten(Object.values(functionTypes_)).find(ft => ft.type === area[i].function_type);
      // console.log(cellStyle);
      if (cellStyle === undefined) {
        cellStyle = {};
        cellStyle["recognition"] = [area[i].type];
      }
      // console.log(cellStyle);
      cellStyle = cellStyle.recognition[0];
      var tooltip = `<div class="farobot-map-tooltip"
                          style="color: ${getColor(cellDetectionTypes_, cellStyle, 'tooltipColor')}">
                      ${cell_id}@(${x.toFixed(2)}, ${y.toFixed(2)})
                     </div>`;
      var wmsNode = {
        title: tooltip,
        // id: cell_id,
        id: area[i].cell_uuid,
        x: pos.x,
        y: pos.y,
        r: r,
        w: area[i].width,
        l: area[i].length
      };
      wmsNode.cellUUID = area[i].cell_uuid;
      wmsNode.label = cell_id.slice(0, 5) + (cell_id.length > 5 ? ".." : "");
      // wmsNode.shape = 'image';
      // wmsNode.image = svgUrl;
      wmsNode.shape = 'circle';
      wmsNode.widthConstraint = 25;
      wmsNode.borderWidth = 2;
      // wmsNode.size = 20; // default: 25
      // wmsNode.color = 'rgba(4, 75, 148, 0.4)'
      wmsNode.color = {
        // highlight: getColor(cellDetectionTypes_, area[i].type, 'focusColor'),
        // background: getColor(cellDetectionTypes_, area[i].type, 'bgColor'),
        // border: getColor(cellDetectionTypes_, area[i].type, 'borderColor')
        highlight: getColor(cellDetectionTypes_, cellStyle, 'focusColor'),
        background: getColor(cellDetectionTypes_, cellStyle, 'bgColor'),
        border: getColor(cellDetectionTypes_, cellStyle, 'borderColor')
      };
      // wmsNode.cellType = area[i].type;
      // var typeTarget = cellTypeAdaptor.toUiCellType(cellTypes_, { 'detectionType': area[i].type, 'width': area[i].width, 'length': area[i].length });
      // wmsNode.cellType = (typeTarget === undefined) ? (area[i].type + area[i].width + area[i].length) : typeTarget.name;

      // var cellType = getCellType(area[i], cellTypes_); // trial
      var cellType = getCellType(area[i], functionTypes_);
      wmsNode.cellType = cellType;
      var functionType = getCellType(area[i], functionTypes_);
      wmsNode.functionType = functionType;
      wmsNode.cellTypeDetection = area[i].type;
      wmsNode.cellDirection = area[i].direction;
      // console.log(area[i].load);
      var cellLoad = cellLoadAdaptor.toUiCellLoad(area[i].load);
      wmsNode.cellLoad = cellLoad;
      wmsNode.font = {
        size: 7,
        color: getColor(cellDetectionTypes_, area[i].type, 'color'),
        bold: true
      };
      // wmsNode.interaction = {
      //   tooltipStyle: "background-color: black"
      // };
      wmsNode.group = 'wms';
      wmsNode.area = key;
      wmsNode.labelFullName = cell_id;
      wmsNode.functionType = area[i].function_type;
      wmsNode.markers = area[i].markers;
      wmsNode.markerOffset = area[i].marker_offset;
      wmsNode.cellSize = area[i].cell_size;
      // _network.body.data.nodes.update(wmsNode);
      nodes.push(wmsNode);
    }
  }
  _network.body.data.nodes.update(nodes);
};

// --- get the cell type. (default: typeName. if not, look-up the cellTypes) ---
function getCellType(_cellObj, _cellTypes) {
  if (_cellTypes === undefined) {
    return 'none';
  }

  if (_cellObj.typeName !== undefined) {
    return _cellObj.typeName;
  }

  // --- deal with the case, '_cellObj.typeName === undefined' ---
  // 1. look-up the cell type
  // var typeTarget = cellTypeAdaptor.toUiCellType(_cellTypes, { 'detectionType': _cellObj.type, 'width': _cellObj.width, 'length': _cellObj.length });
  console.log(_cellObj);
  console.log(_cellTypes);
  var typeTarget = cellTypeAdaptor.toUiCellType2(_cellTypes, { 'detectionType': _cellObj.type, 'width': _cellObj.width, 'length': _cellObj.length });
  // 2. if the cell type is NOT FOUND, return 'none', else return its name;
  var typeName = (typeTarget === undefined) ? 'none' : typeTarget.name;
  console.log(typeName);
  return typeName;
}

let gMapImg_;
let gMapData_;
let gMapMeta_;

function loadMapImage(_data) {
  var cvs = document.getElementById('map_canvas2');
  cvs.width = _data.w;
  cvs.height = _data.h;

  if (cvs === null) { return; }

  var ctx = cvs.getContext('2d');
  ctx.drawImage(gMapImg_, 0, 0);
}

async function loadBundledMapDataAsync(_mode = 'farobot') {
  // --- Model Adaption ---
  // --- generate map alias list ---
  $('#map-select').find('option').remove();
  let mapObj = {};
  if (_mode == 'farobot') {
    let data2 = await fetchGetAllMaps(rmtToken_);
    mapObj = await data2.json();
  }
  else {
    let data = await restGetAllMapData();
    let objArray = JSON.parse(data);
    console.log(data);

    objArray.forEach(obj => {
      mapObj[obj.name] = obj.alias_name;
    });
  }

  for (let key in mapObj) {
    $('#map-select').append(`<option value='${key}'>${mapObj[key]}</option>`);
  }

  // --- Visual View ---
  var e = document.getElementById('map-select');
  currSelectedMap_ = e.options[e.selectedIndex].value;
  // console.log(currSelectedMap_);

  // --- load map desscription and data ---
  gMapMeta_ = { w: 1, h: 1 };
  loadZoneList(currSelectedMap_);
  updateMapImageAsync(currSelectedMap_);
  updateMapMetaAsync2(currSelectedMap_);
}

async function reloadMapImage() {
  // --- reset unsaved map image objects ---
  if (mapImageChanged) {
    fCanvas.remove(...fCanvas.getObjects());
  }

  let selMap = $('#map-select').val();

  // --- get the map meta-data ---
  let res = await fetchGetMapMeta(rmtToken_, selMap);

  res = await res.json();
  gMapData_ = {
    w: res.image_width,
    h: res.image_height,
    data: res.image
  };

  await globalMapImageBackup();
}

async function updateMapMetaAsync2(_mapName) {
  let res = await fetchGetMapMeta(rmtToken_, _mapName);
  res = await res.json();
  // console.log(res);
  gMapMeta_ = {
    w: res.image_width,
    h: res.image_height,
    image: `${res.map_name}.png`,
    occupied_thresh: res.occupied_thresh,
    origin: {
      x: res.origin_x,
      y: res.origin_y,
      z: 0
    },
    resolution: res.resolution,
    nav_graph: `${res.map_name}.dot`,
    cell: `${res.map_name}.json`,
    triton: `${res.map_name}.amf`
  }
  console.log(gMapMeta_);

  visUpdateMapOrigin();

  // --- flush cell cache ---
  jsonCellCache_ = {};
  try {
    // --- load cell data from backend ---
    // let cellData = await restGetMapCells(gMapMeta_.cell);
    let res = await fetchGetMapCells(rmtToken_, gMapMeta_.cell);
    let cellData = await res.json();

    console.log(cellData);

    // --- global caching ---
    jsonCellCache_ = (typeof cellData === 'object') ? cellData : JSON.parse(cellData);
    console.log(jsonCellCache_);

    // --- [TEMP] global data adaption ---
    for (let arr of Object.values(jsonCellCache_)) {
      arr.map(a => a.function_type = (a.function_type === "None") ? "Position_cell" : a.function_type);
      // arr.map(a => a.cell_coordinate = (a.cell_coordinate.length < 3) ? a.cell_coordinate.push(NaN) : a.cell_coordinate);
      arr.forEach(a => {
        if (a.cell_coordinate.length < 3) {
          a.cell_coordinate.push(NaN);
        }
      });
    }
    console.log(jsonCellCache_);

    // --- rendering on UI ---
    bCells_ = false;
    visDrawCells(visNetwork_, jsonCellCache_);
  } catch (err) {
    console.error(err);
  }

  // --- flush graph cache ---
  jsonNavGraphCache_ = {};
  try {
    // --- load graph data from back-end ---
    // var graphData = await restGetMapGraph(gMapMeta_.nav_graph);
    let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
    let graphData = await res.text();
    // console.log(graphData);

    // --- global caching ---
    jsonNavGraphCache_ = cvtDot2Json(graphData);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);
    // console.log(graphNodeNames_);

    // --- rendering on UI ---
    var graph = cvtDot2Json(graphData);
    bNavGraph_ = false;
    visDrawGraph(visNetwork_, graph);
  } catch (err) {
    console.error(err);
  }

  drawFabricZones();
}

async function updateMapMetaAsync(_mapName) {
  var data = await restGetMapMetaInfo(_mapName);
  data = data.split(/\r?\n/);
  // --- update gMapMeta_ ---
  for (i in data) {
    var el = data[i].split(':');
    var key = el[0];
    // console.log(key);
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
  console.log(gMapMeta_);

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

  visUpdateMapOrigin();

  console.log(gMapMeta_);
  // --- flush cell cache ---
  jsonCellCache_ = {};
  try {
    // --- load cell data from backend ---
    // let cellData = await restGetMapCells(gMapMeta_.cell);
    let res = await fetchGetMapCells(rmtToken_, gMapMeta_.cell);
    let cellData = await res.json();

    console.log(cellData);

    // --- global caching ---
    jsonCellCache_ = (typeof cellData === 'object') ? cellData : JSON.parse(cellData);
    console.log(jsonCellCache_);

    // --- rendering on UI ---
    bCells_ = false;
    visDrawCells(visNetwork_, jsonCellCache_);
  } catch (err) {
    console.error(err);
  }

  // --- flush graph cache ---
  jsonNavGraphCache_ = {};
  try {
    // --- load graph data from back-end ---
    // var graphData = await restGetMapGraph(gMapMeta_.nav_graph);
    let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
    let graphData = await res.text();
    // console.log(graphData);

    // --- global caching ---
    jsonNavGraphCache_ = cvtDot2Json(graphData);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);
    // console.log(graphNodeNames_);

    // --- rendering on UI ---
    var graph = cvtDot2Json(graphData);
    bNavGraph_ = false;
    visDrawGraph(visNetwork_, graph);
  } catch (err) {
    console.error(err);
  }

  // drawFabricBoard();
  // console.log(`update map meta done`);
}

async function updateMapImageAsync(_mapName) {
  // gMapData_ = await restGetMapImg(_mapName);
  let res = await fetchGetMapMeta(rmtToken_, _mapName);
  res = await res.json();
  // console.log(res);
  gMapData_ = {
    w: res.image_width,
    h: res.image_height,
    data: res.image
  };

  await globalMapImageBackup();
  drawFabricMap();
  drawFabricZoneMap();

  gMapMeta_.w = gMapData_.w;
  gMapMeta_.h = gMapData_.h;

  visNetwork_.moveTo({
    position: {
      x: gMapData_.w / 2,
      y: gMapData_.h / 2
    },
    scale: (mapViewSize / gMapData_.h) * 3.5
  });
}


let currSelectedMap_;
function updateSelectedMap() {
  var e = document.getElementById('map-select');
  currSelectedMap_ = e.options[e.selectedIndex].value;
  // console.log(currSelectedMap_);

  // --- dataset reset ---
  fCanvas.clear();
  fZoneCanvas.clear();
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  // --- load map desscription and data ---
  gMapMeta_ = { w: 1, h: 1 };
  loadZoneList(currSelectedMap_);
  updateMapImageAsync(currSelectedMap_);
  updateMapMetaAsync2(currSelectedMap_);

  // --- reset the property radios ---
  $('.custom-control-input').prop('checked', false);
}

function globalMapImageBackup() {
  var mapDataURL = `data:image/png;base64,${gMapData_.data}`;
  gMapImg_ = new Image();
  gMapImg_.onload = function () {
    loadMapImage(gMapData_);
  };
  gMapImg_.src = mapDataURL;
}

function visUpdateMapOrigin(isHidden = false) {
  // update network node
  console.log('--- update map origin ---');
  var orgPos = tfROS2Canvas(gMapMeta_, {
    x: 0.0,
    y: 0.0
  });
  var orgNode = {
    id: 'Origin',
    x: (orgPos.x),
    y: (orgPos.y)
  };
  orgNode.shape = 'image';
  orgNode.image = `./${getImagePath()}/ucs.png`;
  orgNode.size = 50;
  orgNode.color = 'red';
  orgNode.label = 'Origin';
  orgNode.group = 'metadata';
  orgNode.hidden = isHidden;
  visNetwork_.body.data.nodes.update(orgNode);
}


// =====================
//     DOM Callbacks 
// =====================
async function btnSaveChanges() {
  executingFlow = await checkProtection();
  let is_map_using = executingFlow["using_map"].includes($('#map-select').val());

  if (!is_map_using) {
    switch (selectedEditType) {
      case 'rdMapImg':
        btnSaveMapImage();
        break;
      case 'rdRoute':
        btnSaveNavGraph();
        break;
      case 'rdCell':
        btnSaveCells();
        break;
      case 'rdConnCell':
        btnSaveConnCells();
        break;
      case 'rdZone':
        btnSaveZone();
        break;
    }
  } else {
    let msg = '';
    switch (selectedEditType) {
      case 'rdMapImg':
        msg = 'save image change'
        break;
      case 'rdRoute':
        msg = 'save route change'
        break;
      case 'rdCell':
      case 'rdConnCell':
        msg = 'save cell change'
        break;
    }
    notificationMsg(3, `Map has executing task, cannot ${msg}.`);
    return;
  }
}

async function btnSaveNavGraph() {
  var graph_nodes_id = [];
  var id_label_table = {};
  var id_connections = {};
  console.log(visNetwork_.body.nodes);
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group !== 'navnode') continue;

    graph_nodes_id.push(visNetwork_.body.nodes[i].options.id);
    id_label_table[visNetwork_.body.nodes[i].options.id] = visNetwork_.body.nodes[i].options.label;
    var conn = visNetwork_.body.nodes[i].edges.map(edge => edge.toId);
    conn = conn.filter(c => c !== visNetwork_.body.nodes[i].options.id);
    console.log(conn);
    id_connections[visNetwork_.body.nodes[i].options.id] = conn;
  }

  var nodes = objectToArray(visNetwork_.getPositions(graph_nodes_id));
  // console.log(nodes);

  // ------ [BEGIN] save the result to file [BEGIN] ------
  currJsonGraph.edges = [];
  console.log(jsonNavGraphCache_.nodes);
  nodes = _.uniqBy(nodes, function (n) {
    return n.id;
  });
  for (n in nodes) {
    var node = nodes[n];

    nodes[n]['label'] = id_label_table[nodes[n].id];
    nodes[n]['connections'] = _.uniq(id_connections[nodes[n].id]);

    // console.log(jsonNetwork[n].label);
    console.log(nodes[n].label);
    var nodeArr = jsonNavGraphCache_.nodes.find(ngc => ngc.id === nodes[n].id);
    nodes[n].x = nodeArr.x;
    nodes[n].y = nodeArr.y;

    for (it in node["connections"]) {
      var edge = {};
      edge["from"] = node["id"];
      edge["to"] = node["connections"][it];
      edge["weight"] = "1";
      edge["width"] = "0";
      const edgeExists = _.some(currJsonGraph.edges, { from: node["id"], to: node["connections"][it] });
      if (edgeExists) continue;
      currJsonGraph.edges.push(edge);
    }
  }
  // console.log(currJsonGraph);
  // console.log(JSON.stringify(nodes));
  var strGraph = cvtJson2Dot(nodes);
  console.log(strGraph);

  // --- POST graph data to server side ---
  // await restPostMapGraph(currSelectedMap_, strGraph);
  let res = await fetchPutMapGraph(rmtToken_, currSelectedMap_, strGraph);
  console.log(res);

  notificationMsg(0, 'NavGraph saved successfully!');
  // ------ [END] save the result to file [END] ------

  // --- refresh all the edges ---
  visNetwork_.body.data.edges.clear();

  // --- flush cache ---
  jsonNavGraphCache_ = {};
  // CRITICAL! Update graph contents
  try {
    // --- load graph data from back-end ---
    // var graphData = await restGetMapGraph(gMapMeta_.nav_graph);
    let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
    let graphData = await res.text();
    // console.log(graphData);

    // --- global cacheing ---
    jsonNavGraphCache_ = cvtDot2Json(graphData);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);

    // --- rendering on UI ---
    var graph = cvtDot2Json(graphData);
    bNavGraph_ = false;
    visDrawGraph(visNetwork_, graph);
  } catch (err) {
    console.error(err);
  }

  setVisInteractiveMode(2);

  // --- flush graph Difference Cache ---
  graphDiffCache_.flush();
  console.log(graphDiffCache_);
}

// --- WRAP-UP edited connections ---
async function btnSaveConnCells() {
  console.log('--- Ready to save Connected Cells ---');

  // 1. get all the connections
  // 2. convert data into object 
  const connCellDecks = document.querySelectorAll('#conn-cell-tool > .conn-card');
  console.log(connCellDecks);

  let connDataObj = {};
  connCellDecks.forEach((conn) => {
    console.log(conn);
    // --- connected cell tag ---
    let name = conn.querySelector('.conn-name').textContent;
    console.log(name)

    // --- connected cell values ---
    let selNode = conn.querySelector('select');
    let selVal = $(selNode).val();
    console.log(selVal);
    let selVal2 = $(selNode).find(':selected');
    console.log(selVal2);

    let connections = [];
    for (let opt of selVal2) {
      let c = $(opt).data();
      c = c.connCell.split(',');
      connections.push({ "map_name": c[0].trim(), "transition_cell": c[1].trim() });
    };
    console.log(connections);

    connDataObj[name] = connections;
  });

  console.log(connDataObj);

  // --- send to the server ---
  let res = await fetchPutCellRelation(rmtToken_, connDataObj);
  res = await res.json();
  // console.log(res);
  notificationMsg(0, 'Cell Connections ARE saved successfully!');
}

function getSelectValues(select) {
  var result = [];
  var options = select && select.options;
  console.log(options);
  var opt;

  for (var i = 0, iLen = options.length; i < iLen; i++) {
    opt = options[i];

    if (opt.selected) {
      result.push(opt.value || opt.text);
    }
  }
  return result;
}

async function btnSaveCells() {

  // --- update the cell filename in map-meta yaml ---
  console.log(currSelectedMap_);
  var cellFilename = currSelectedMap_;

  // ---[protection] update cell size. may be remove in near future ---
  // updateCellsType(jsonCellCache_, cellTypes_);

  // ---[protection] data-schema transformation ---
  console.log(jsonCellCache_);

  // --- [protection] prevent from duplcated cell name ---
  var dupCellIDArr = [];
  for (key in jsonCellCache_) {
    var cid = jsonCellCache_[key].map(c => c.cell_id);
    dupCellIDArr.push(...cid);
  }
  dupCellIDArr = _.uniq(_.filter(dupCellIDArr, (v, i, a) => a.indexOf(v) !== i));
  if (dupCellIDArr.length > 0) {
    alert(`Cell name cannot be repeated, please rename ${dupCellIDArr.join(" and ")}!`);
    return;
  }

  for (areaKey in jsonCellCache_) {
    for (cellKey in jsonCellCache_[areaKey]) {
      // --- remove json cache cell uuid before saving map json file ---
      if (jsonCellCache_[areaKey][cellKey].hasOwnProperty('cell_uuid')) {
        delete jsonCellCache_[areaKey][cellKey]['cell_uuid'];
        delete jsonCellCache_[areaKey][cellKey]['cellLabelFull'];
      }

      // --- data schema transformation protection ---
      if (!jsonCellCache_[areaKey][cellKey].hasOwnProperty('function_type')) {
        jsonCellCache_[areaKey][cellKey]['function_type'] = jsonCellCache_[areaKey][cellKey].type || 'Position_cell';
      }
      if (!jsonCellCache_[areaKey][cellKey].hasOwnProperty('markers')) {
        jsonCellCache_[areaKey][cellKey]['markers'] = 'None';
      }
      if (!jsonCellCache_[areaKey][cellKey].hasOwnProperty('marker_offset')) {
        jsonCellCache_[areaKey][cellKey]['marker_offset'] = [1, 1];
      }
      if (!jsonCellCache_[areaKey][cellKey].hasOwnProperty('cell_size')) {
        jsonCellCache_[areaKey][cellKey]['cell_size'] = [2, 2];
      }
    }
  }

  var content = JSON.stringify(jsonCellCache_);
  console.log(content);
  // TODO: change width and length format
  // if (!chkDataType) {
  //   remove width and length string double quote
  //   can not work on iPad
  //   content = content.replace(/(?<=\"width":)(\")(?=\d)|(?<=\"width":\"[0-9]*[.][0-9]*)(\")(?=\,)|(?<=\"length":)(\")(?=\d)|(?<=\"length":\"[0-9]*[.][0-9]*)(\")(?=\,)/g, '');
  // }
  // console.log(content);

  // var res = await restPostMapCells(cellFilename, content);
  var res = await fetchPutMapCells(rmtToken_, cellFilename, content);
  res = await res.json();
  console.log(res);
  notificationMsg(0, 'Storage cells saved successfully!');

  // --- refresh the contents ---
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  visUpdateMapOrigin(!isOriginVisible_);

  // --- flush cell cache ---
  jsonCellCache_ = {};
  try {
    // --- load cell data from back-end ---
    // var cellData = await restGetMapCells(gMapMeta_.cell);
    let res = await fetchGetMapCells(rmtToken_, gMapMeta_.cell);
    let cellData = await res.json();
    // console.log(cellData);

    // --- global caching ---
    jsonCellCache_ = (typeof cellData === 'object') ? cellData : JSON.parse(cellData);
    console.log(jsonCellCache_);

    // --- rendering on UI ---
    bCells_ = false;
    visDrawCells(visNetwork_, jsonCellCache_);
  } catch (err) {
    console.error(err);
  }

  // --- flush graph cache ---
  jsonNavGraphCache_ = {};
  try {
    // --- load graph data from back-end ---
    // var graphData = await restGetMapGraph(gMapMeta_.nav_graph);
    let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
    let graphData = await res.text();
    // console.log(graphData);

    // --- global caching ---
    jsonNavGraphCache_ = cvtDot2Json(graphData);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);

    // --- rendering on UI ---
    var graph = cvtDot2Json(graphData);
    bNavGraph_ = false;
    visDrawGraph(visNetwork_, graph);
  } catch (err) {
    console.error(err);
  }

  setVisInteractiveMode(3);

  // --- flush cell Difference Cache ---
  cellDiffCache_.flush();
}

function btnUndoRoute() {
  closeManipulationMode();

  // console.log(jsonNavGraphCache_);
  console.log(graphDiffCache_);
  var myGraphCache = graphDiffCache_.popHead();
  console.log(myGraphCache);
  if (myGraphCache === undefined) { return; }

  myGraphCache = myGraphCache.value;

  // case1: action: add. UNDO (add -> delete) ---
  if (myGraphCache.action === 'add' && myGraphCache.type === 'node') {
    // --- update visCache ---
    // console.log(network2_.body.data.nodes);
    visNetwork_.body.data.nodes.remove(myGraphCache.visID);
    // --- update rosCache ---
    jsonNavGraphCache_.nodes = jsonNavGraphCache_.nodes.filter(ca => ca.id !== myGraphCache.visID);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);
    // console.log(jsonNavGraphCache_);
    return;
  }

  // case2: action -> edit 
  if (myGraphCache.action === 'edit' && myGraphCache.type === 'node') {
    // --- update visCache ---
    console.log(myGraphCache);
    let myNode = { id: myGraphCache.visID, x: myGraphCache.visX, y: myGraphCache.visY };
    // console.log(myNode);
    visNetwork_.body.data.nodes.update(myNode);
    // --- update rosCache ---
    let nodeObj = jsonNavGraphCache_.nodes.find(ca => ca.id === myGraphCache.rosID);
    nodeObj.x = myGraphCache.rosX;
    nodeObj.y = myGraphCache.rosY;
    return;
  }

  // case3: action: delete. UNDO (delete -> add) ---
  if (myGraphCache.action === 'delete' && myGraphCache.type === 'node') {
    // --- update visCache ---
    visNetwork_.body.data.nodes.update(myGraphCache.visNavGraphNode);
    // --- update rosCache ---
    jsonNavGraphCache_.nodes.push(myGraphCache.rosNavGraphNode);
    graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);

    // --- update vis connections ---
    console.log(myGraphCache.connections);
    var connCache = myGraphCache.connections.map(function (c) {
      return {
        from: c.from,
        to: c.to,
        weight: c.weight,
        width: c.width,
      };
    });
    // console.log(connCache)
    // visNetwork_.body.data.edges.add(myGraphCache.connections);
    visNetwork_.body.data.edges.add(connCache);
    // --- update connections cache ---
    jsonNavGraphCache_.edges.concat(myGraphCache.connections);
    return;
  }

  // case4: [edge] action: add. UNDO (add -> delete) ---
  if (myGraphCache.action === 'add' && myGraphCache.type === 'edge') {
    // --- update visCache ---
    visNetwork_.body.data.edges.remove(myGraphCache.visID);
    // --- update rosCache ---
    jsonNavGraphCache_.edges = jsonNavGraphCache_.edges.filter(ca => ca.id !== myGraphCache.visID);
    return;
  }

  // case5: [edge] action: delete. UNDO (delete -> add) ---
  if (myGraphCache.action === 'delete' && myGraphCache.type === 'edge') {
    // --- update visCache ---
    visNetwork_.body.data.edges.update({ id: myGraphCache.visID, from: myGraphCache.fromId, to: myGraphCache.toId });
    // --- update rosCache ---
    jsonNavGraphCache_.edges.push({ from: myGraphCache.fromId, to: myGraphCache.toId, weight: "1", width: "0.0" });
    return;
  }
}

function btnUndoCell() {
  closeManipulationMode();

  // console.log(jsonNavGraphCache_);
  console.log(cellDiffCache_)
  var myCellCache = cellDiffCache_.popHead();
  console.log(cellDiffCache_)
  console.log(myCellCache);
  if (myCellCache === undefined) {
    return;
  }
  myCellCache = myCellCache.value;
  console.log(myCellCache);

  // case 1: action add. UNDO (add -> delete) 
  if (myCellCache.action === 'add') {
    // --- update visCache ---
    visNetwork_.body.data.nodes.remove(myCellCache.visID);
    // --- update rosCache ---
    jsonCellCache_[myCellCache.area] = jsonCellCache_[myCellCache.area].filter(ca => ca.cell_uuid !== myCellCache.rosID);
    return;
  }

  // case 2: action edit 
  if (myCellCache.action === 'edit') {
    console.log(myCellCache);

    // --- change area case ---
    let choose_area = myCellCache.area;

    if (myCellCache.area !== myCellCache.prevarea) {
      choose_area = myCellCache.prevarea;

      let cache_obj = JSON.parse(JSON.stringify(jsonCellCache_[myCellCache.area].filter(c => c.cell_uuid === myCellCache.rosID)));
      jsonCellCache_[myCellCache.area] = jsonCellCache_[myCellCache.area].filter(c => c.cell_uuid !== myCellCache.rosID);
      console.log(jsonCellCache_)
      console.log(cache_obj)

      if (jsonCellCache_[myCellCache.area].length === 0) {
        delete jsonCellCache_[myCellCache.area];
      }

      if (jsonCellCache_.hasOwnProperty(myCellCache.prevarea)) {
        jsonCellCache_[myCellCache.prevarea].push(cache_obj[0]);
      }
      else {
        jsonCellCache_[myCellCache.prevarea] = cache_obj;
      }
      console.log(JSON.stringify(jsonCellCache_))
    }

    // --- update visCache ---
    var tooltip = `<div class="farobot-map-tooltip" 
                        style="color: black">
                    ${myCellCache.labelFullName}@(${Number(myCellCache.rosX).toFixed(2)}, ${Number(myCellCache.rosY).toFixed(2)})
                  </div>`;

    let myNode = {
      id: myCellCache.visID,
      cellUUID: myCellCache.rosID,
      x: myCellCache.visX,
      y: myCellCache.visY,
      // cellLabelFull: myCellCache.cellLabelFull,
      cell_id: myCellCache.labelFullName,
      area: choose_area,
      label: myCellCache.labelFullName,         // cell appearing name tag
      labelFullName: myCellCache.labelFullName, // load to the column in pop-up box
      title: tooltip,
      cellLoad: myCellCache.cellLoad
    };
    // console.log(myNode);
    visNetwork_.body.data.nodes.update(myNode);
    // --- update rosCache ---
    let areaObj = jsonCellCache_[choose_area].find(ca => ca.cell_uuid === myCellCache.rosID);
    areaObj.cell_coordinate = [myCellCache.rosX, myCellCache.rosY];
    // areaObj.labelFullName = myCellCache.labelFullName;
    areaObj.cell_id = myCellCache.labelFullName;
    areaObj.load = myCellCache.cellLoad;
    areaObj.direction = myCellCache.cellDirection;
    areaObj.function_type = myCellCache.functionType;
    areaObj.markers = myCellCache.markers;
    areaObj.marker_offset = myCellCache.markerOffset;
    areaObj.cell_size = myCellCache.cellSize;
    console.log(areaObj);
    console.log(jsonCellCache_);
    return;
  }

  // case 3: action delete. UNDO (delete -> add) ---
  if (myCellCache.action === 'delete') {
    // --- update visCache ---
    // console.log(myCellCache.rosCellNode);
    visNetwork_.body.data.nodes.update(myCellCache.visCellNode);
    // --- update rosCache ---
    jsonCellCache_[choose_area].push(myCellCache.rosCellNode);
    return;
  }
}

// --- Callback passed as parameter is ignored ---
function clearNodePopUp() {
  document.getElementById("node-createButton").onclick = null;
  document.getElementById("node-cancelButton").onclick = null;
  document.getElementById("node-popUp").style.display = "none";

  document.getElementById("cell-createButton").onclick = null;
  document.getElementById("cell-cancelButton").onclick = null;
  document.getElementById("cell-popUp").style.display = "none";

  document.getElementById("zone-node-createButton").onclick = null;
  document.getElementById("zone-node-cancelButton").onclick = null;
  document.getElementById("zone-node-popUp").style.display = "none";

  hideNavAgentOptions();
}

function updatePopUpPosition(type_) {
  let windowTop = window.pageYOffset || document.documentElement.scrollTop;
<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
  let popUpId = `${type_}-popUp`;
  let popUpHeight = 0;
  let canvasRect = 0;

  switch (type_) {
    case 'node':
      popUpHeight = 230;
      canvasRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
      break;
    case 'cell':
      popUpHeight = 435;
      canvasRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
      break;
    case 'zone-node':
      popUpHeight = 170;
      canvasRect = document.getElementById('zoneCanvas').getBoundingClientRect();
=======
  let popUpId = "";
  let popUpHeight = 0;

  switch (type_) {
    case 'node':
      popUpId = 'node-popUp';
      popUpHeight = 230;
      break;
    case 'cell':
      popUpId = 'cell-popUp';
      popUpHeight = 435;
      break;
    case 'zonenode':
      popUpId = 'zone-node-popUp';
      popUpHeight = 170;
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js
      break;
    default:
      break;
  }
<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
=======

>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js
  let popupSel = document.getElementById(popUpId);

  if (window.innerWidth <= 767) {
    popupSel.style.position = "fixed";
    popupSel.style.top = '';
    popupSel.style.left = '0';
    popupSel.style.bottom = '0';
    popupSel.style.height = popUpHeight > window.innerHeight ? '100vh' : `${popUpHeight}px`;
  } else {
    popupSel.style.position = "absolute";
    popupSel.style.top = `${windowTop + canvasRect.y}px`;
    popupSel.style.left = `${canvasRect.x}px`;
    popupSel.style.bottom = '';
  }
}

function cancelNodeEdit(callback) {
  closeManipulationMode();
  callback(null);
}

function editEdgeWithoutDrag(data, callback) {
  // filling in the popup DOM elements
  console.log('-- edit edge without drag --');
  console.log(data);
  // document.getElementById("edge-label").value = data.label;
  document.getElementById("edge-label").value = data.id;
  document.getElementById("edge-weight").value = data.label;
  document.getElementById("edge-createButton").onclick = saveEdgeData.bind(
    this,
    data,
    callback
  );
  document.getElementById(
    "edge-cancelButton"
  ).onclick = cancelEdgeEdit.bind(this, callback);
  document.getElementById("edge-popUp").style.display = "block";
}

function clearEdgePopUp() {
  document.getElementById("edge-createButton").onclick = null;
  document.getElementById("edge-cancelButton").onclick = null;
  document.getElementById("edge-popUp").style.display = "none";
}

function cancelEdgeEdit(callback) {
  clearEdgePopUp();
  callback(null);
}

function saveEdgeData(data, callback) {
  if (typeof data.to === "object") data.to = data.to.id;
  if (typeof data.from === "object") data.from = data.from.id;
  data.label = document.getElementById("edge-label").value;
  console.log(data);
  clearEdgePopUp();
  callback(data);
}

// ---------------------------------
//     Toggle Data Visualization 
// ---------------------------------

function toggleMap(_status = true) {
  var cvs = document.getElementById('map_canvas2');
  cvs.width = gMapData_.w;
  cvs.height = gMapData_.h;

  if (cvs === null) {
    return;
  }
  var ctx = cvs.getContext('2d');

  if (_status) {
    visOptions_.edges.color = "black";
    visOptions_.edges.background.color = "white";
    ctx.drawImage(gMapImg_, 0, 0);
  } else {
    visOptions_.edges.color = "white";
    visOptions_.edges.background.color = "#393f52";
    ctx.clearRect(0, 0, gMapData_.w, gMapData_.h);
  }

  visNetwork_.setOptions(visOptions_);
  visNetwork_.redraw();
}

function toggleNavGraph(_status = true) {
  var nodesUpdate = [];
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group === 'navnode') {
      nodesUpdate.push({
        id: visNetwork_.body.nodes[i].id,
        hidden: !_status
      });
    }
  }

  visNodes_.update(nodesUpdate);
}

function toggleCells(_status = true) {
  var nodesUpdate = [];
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group === 'wms') {
      nodesUpdate.push({
        id: visNetwork_.body.nodes[i].id,
        hidden: !_status
      });
    }
  }

  visNodes_.update(nodesUpdate);
}

let isOriginVisible_ = true;
function toggleOrigin(_status = true) {
  isOriginVisible_ = _status;
  var nodesUpdate = [];
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group === 'metadata') {
      nodesUpdate.push({
        id: visNetwork_.body.nodes[i].id,
        hidden: !_status
      });
    }
  }

  visNodes_.update(nodesUpdate);
}

// ======================
//      Miscellaneous    
// ======================
async function getAvailableRobotFromFleetList() {
  var navAgents = [];

  var res = await fetchFleetStates(rmtToken_, '', 'agent_only');
  fleetData2_ = [];
  if (!res.ok) {
    notificationMsg(2, "No available robot in this map anchor function may not able to use")
    return;
  }

  var queryData = await res.json();

  if (queryData.hasOwnProperty("fleet_state")) {
    queryData["fleet_state"].forEach(function (fleet_item, fleet_index) {
      console.log(fleet_item);
      fleet_item["robots"].forEach(function (robot_item, robot_index) {
        navAgents.push(robot_item);
      });
    });
  } else {
    notificationMsg(2, "No available fleet state in this map anchor function may not able to use")
  }

  return navAgents;
}

async function getZoneMetaData() {
  let data = {};
  try {
    data = await fetchZoneMetadata(rmtToken_);
  } catch (err) {
    console.error(err);
  }
  return data;
}

async function fadeInEditToolbar(_target) {
  // --- toggle off all ---
  var bar;
  bar = document.getElementById('map-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('route-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('cell-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('conn-cell-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('zone-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('config-tool');
  bar.setAttribute("style", "display:none;");

  // --- toggle edit properties tools and swap tools position ---
  if (!isMobile()) {
    $('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');
  } else if (window.innerWidth <= 767 || window.innerHeight <= 767) {
    $('#edit-tools-card').detach().appendTo('#edit-tools-section');
    $('#edit-tools-list').hide();
    $('#edit-tools-section').css('display', _target === null || _target === undefined ? 'none' : 'block');
  } else {
    $('#edit-tools-card').detach().appendTo('#edit-tools-list');
    $('#edit-tools-section').hide();
    $('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');
  }
  // $('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');

  // -- toggle target toolbar ---
  if (_target === null || _target === undefined)
    return;

  bar = document.getElementById(_target);
  bar.setAttribute("style", "display:block;");

  // --- fulfill available agents in fleets --- 
  var navAgents = await getAvailableRobotFromFleetList();

  var mapName = $('#map-select').val();
  navAgents = _.filter(navAgents, na => na.map === mapName);
  navAgents = navAgents.map(na => na.robot_id);

  var toolbarId = 'nav-agent-select';

  var navSel = document.getElementById(toolbarId);
  $(navSel).empty();
  if (navAgents.length === 0) {
    // --- hide popups anchor button ---
    $('.anchorTD').hide();
    $('.posTD').attr("colspan", 2);
    // --- add default option ---
    var opt = document.createElement("option");
    opt.text = 'No Agents';
    opt.disabled = true;
    opt.selected = true;
    opt.hidden = true;
    navSel.options.add(opt);
    return;
  }
  // --- display popups anchor button ---
  $('.anchorTD').show();
  $('.posTD').removeAttr("colspan");
  navAgents.forEach(na => {
    var opt = document.createElement("option");
    opt.text = na;
    opt.value = na;
    navSel.options.add(opt);
  });
}

async function navRouteAnchor() {
  console.log('nav route');

  var selAgent = document.getElementById("nav-agent-select").value;
  var anchorLoc = undefined;

  var navAgents = await getAvailableRobotFromFleetList();
  // console.log(navAgents)

  navAgents.forEach(robot => {
    if (robot.robot_id === selAgent) {
      $("#node-x").val(robot.location.x);
      $("#node-y").val(robot.location.y);
    } else {
      notificationMsg(2, "No matched robot in fleet state anchor function may not able to use for vertex");
    }
  });
}

async function navCellAnchor() {
  console.log('nav cell');
  var selAgent = document.getElementById("nav-agent-select").value;
  var anchorLoc = undefined;

  var navAgents = await getAvailableRobotFromFleetList();
  // console.log(navAgents)

  navAgents.forEach(robot => {
    if (robot.robot_id === selAgent) {
      $("#cell-x").val(robot.location.x);
      $("#cell-y").val(robot.location.y);
    } else {
      notificationMsg(2, "No matched robot in fleet state anchor function may not able to use for cell");
    }
  });
}

// type 0: view only
// type 1: map metadata
// type 2: route graph 
// type 3: storage cell
// type 4: function zone
function setVisInteractiveMode(type) {
  if (type === 0) {
    visOptions_.nodes.fixed = {
      x: true,
      y: true
    };
    visOptions_.manipulation.enabled = false;
    visOptions_.interaction.dragNodes = false;
    visOptions_.interaction.selectable = false;
    visOptions_.interaction.selectConnectedEdges = false;
    visOptions_.interaction.navigationButtons = true;
    visNetwork_.setOptions(visOptions_);
    return;
  }

  if (type === 1) {
    // --- set network prperty ---
    visOptions_.nodes.fixed = {
      x: true,
      y: true
    };
    visOptions_.manipulation.enabled = false;
    visOptions_.interaction.dragNodes = false;
    visOptions_.interaction.selectable = false;
    visOptions_.interaction.selectConnectedEdges = false;
    visOptions_.interaction.navigationButtons = false;

    visNetwork_.setOptions(visOptions_);

    return;
  }

  if (type === 2) {
    // --- set network prperty ---
    // CRITICAL! HAVE TO PUT THIS BEFORE RETURN
    visOptions_.nodes.fixed = {
      x: false,
      y: false
    };
    visOptions_.manipulation.enabled = false;
    visOptions_.interaction.dragNodes = true;
    visOptions_.interaction.selectable = true;
    visOptions_.interaction.selectConnectedEdges = true;
    visOptions_.interaction.navigationButtons = false;

    visOptions_.manipulation.addNode = btnAddRouteNode;
    visOptions_.manipulation.editNode = btnEditRouteNode;
    visOptions_.manipulation.deleteNode = btnDeleteRouteNode;

    visOptions_.manipulation.addEdge = btnAddRouteEdge;
    // visOptions_.manipulation.editEdge = editNavEdgeData;
    visOptions_.manipulation.deleteEdge = btnDeleteRouteEdge;

    visOptions_.edges.background.color = "white";
    visOptions_.edges.color = "black";
    visNetwork_.setOptions(visOptions_);
    visNetwork_.off("dragStart");
    visNetwork_.on("dragStart", evtDragStartRouteNode);
    visNetwork_.off("dragEnd");
    visNetwork_.on("dragEnd", evtDragEndRouteNode);
    visNetwork_.off("controlNodeDragEnd");
    visNetwork_.on("controlNodeDragEnd", evtDragEndRouteEdge);

    for (i in visNetwork_.body.nodes) {
      if (visNetwork_.body.nodes[i].options.group === 'wms') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(128,128,128,0.8)';
        visNetwork_.body.nodes[i].options.color.border = 'rgba(220,220,220,1.0)';
        // console.log(visNetwork_.body.nodes[i].options);

      }
      if (visNetwork_.body.nodes[i].options.group === 'metadata') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(220,220,220,0.4)';
      }
    }
    return;
  }

  if (type === 3) {
    // --- set network prperty ---
    // CRITICAL! HAVE TO PUT THIS BEFORE RETURN
    visOptions_.nodes.fixed = {
      x: false,
      y: false
    };
    visOptions_.manipulation.enabled = false;
    visOptions_.interaction.dragNodes = true;
    visOptions_.interaction.selectable = true;
    visOptions_.interaction.selectConnectedEdges = true;
    visOptions_.interaction.navigationButtons = false;

    visOptions_.manipulation.addNode = btnAddCellNode;
    visOptions_.manipulation.editNode = btnEditCellNode;
    visOptions_.manipulation.deleteNode = btnDeleteCellNode;

    visOptions_.edges.background.color = "white";
    visOptions_.edges.color = "black";
    visNetwork_.setOptions(visOptions_);

    visNetwork_.off("dragStart");
    visNetwork_.on("dragStart", evtDragStartCellNode);
    visNetwork_.off("dragEnd");
    visNetwork_.on("dragEnd", evtDragEndCellNode);


    for (i in visNetwork_.body.nodes) {
      if (visNetwork_.body.nodes[i].options.group === 'navnode') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(220,220,220,1.0)';
        visNetwork_.body.nodes[i].options.color.border = 'rgba(220,220,220,1.0)';
      }
      if (visNetwork_.body.nodes[i].options.group === 'metadata') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(220,220,220,0.4)';
      }
    }
    visDrawCellsTypeColor(visNetwork_, jsonCellCache_);
    return;
  }

  if (type === 4) {
    // --- set network prperty ---
    // CRITICAL! HAVE TO PUT THIS BEFORE RETURN
    visOptions_.nodes.fixed = {
      x: false,
      y: false
    };
    visOptions_.manipulation.enabled = false;
    visOptions_.interaction.dragNodes = true;
    visOptions_.interaction.selectable = true;

    visOptions_.manipulation.addNode = btnAddZoneNode;

<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
    visOptions_.manipulation.addEdge = btnAddZoneBoundary;

=======
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js
    visNetwork_.setOptions(visOptions_);

    for (i in visNetwork_.body.nodes) {
      if (visNetwork_.body.nodes[i].options.group === 'navnode') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(220,220,220,1.0)';
        visNetwork_.body.nodes[i].options.color.border = 'rgba(220,220,220,1.0)';
      }
      if (visNetwork_.body.nodes[i].options.group === 'wms') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(128,128,128,0.8)';
        visNetwork_.body.nodes[i].options.color.border = 'rgba(220,220,220,1.0)';
      }
      if (visNetwork_.body.nodes[i].options.group === 'metadata') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(220,220,220,0.4)';
      }
    }
    return;
  }
}


// =============================
//     Routes Event Callbacks    
// =============================
let graphDiffCache_ = new FarCache();

// --- Route Graph Manipulation ---
function btnAddRouteNode(data, callback) {
  // --- update View ---
  document.getElementById("node-operation").innerText = "Add Vertex";
  document.getElementById("node-createButton").value = "Create";

  // var el = document.getElementById('node-label');
  // var elClone = el.cloneNode(true);
  // el.parentNode.replaceChild(elClone, el);
  // -- [protection] prevent from duplcated vertex name --
  // document.getElementById('node-label').addEventListener("keyup", function () {
  //   // console.log(this.value);
  //   console.log('AAAAAAAVVVVVV');
  //   if (graphNodeNames_.includes(this.value)) {
  //     alert('DUPLICATED VERTEX LABEL!');
  //   }
  // });

  data.id = 'fa' + data.id;

  document.getElementById("node-label").value = data.label;
  // console.log(data);

  // --- update node location from cache ---
  var rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });
  document.getElementById("node-x").value = rosPos.x;
  document.getElementById("node-y").value = rosPos.y;

  document.getElementById("node-createButton").onclick = btnSaveRouteNode.bind(this, data, callback, 'add');
  document.getElementById("node-cancelButton").onclick = clearNodePopUp.bind(this, callback);

  document.getElementById("node-popUp").style.display = "block";
  updatePopUpPosition('node');

  showNavAgentOptions();
}

function btnEditRouteNode(data, callback) {
  // filling in the popup DOM elements
  document.getElementById("node-operation").innerText = "Edit Vertex";
  document.getElementById("node-createButton").value = "OK"; // "Save"

  // console.log(data.label);
  // graphNodeNames_ = graphNodeNames_.filter(gnn => gnn !== data.label);
  // console.log(graphNodeNames_);
  // var el = document.getElementById('node-label');
  // var elClone = el.cloneNode(true);
  // el.parentNode.replaceChild(elClone, el);
  // -- [protection] prevent from duplcated vertex name --
  // document.getElementById('node-label').addEventListener("keyup", function () {
  //   // console.log(this.value);
  //   if (data.label === this.value) return; // skip original name
  //   if (graphNodeNames_.includes(this.value)) {
  //     alert('DUPLICATED VERTEX LABEL!');
  //   }
  // });

  document.getElementById("node-label").value = data.label;
  // --- edit selected nav-graph node case ---
  var navNode = jsonNavGraphCache_.nodes.find(ngc => ngc.id === data.id);
  console.log(navNode);

  document.getElementById("node-x").value = navNode.x;
  document.getElementById("node-y").value = navNode.y;

  document.getElementById("node-createButton").onclick = btnSaveRouteNode.bind(this, data, callback, 'edit');
  document.getElementById("node-cancelButton").onclick = cancelNodeEdit.bind(this, callback);

  document.getElementById("node-popUp").style.display = "block";
  updatePopUpPosition('node');

  showNavAgentOptions();
}

function btnDeleteRouteNode(data, callback) {
  var deletedNode = visNetwork_.body.data.nodes.get(data.nodes[0]);
  console.log(deletedNode);

  // --- cache the difference ---
  var rosNavGraphNode = jsonNavGraphCache_.nodes.find(aa => aa.id === deletedNode.id);
  console.log(rosNavGraphNode);
  // var navGraphDiff = { visNavGraphNode: deletedNode, rosNavGraphNode: rosNavGraphNode, action: 'delete', type: 'node', connections: [] };
  // graphDiffCache_.put(genUuid(), navGraphDiff);
  // console.log(graphDiffCache_);
  // --- cache the deleted node connection ---
  var deletedConnections = jsonNavGraphCache_.edges.filter(edg => edg.from === deletedNode.id || edg.to === deletedNode.id);
  console.log(deletedConnections);

  var navGraphDiff = { visNavGraphNode: deletedNode, rosNavGraphNode: rosNavGraphNode, action: 'delete', type: 'node', connections: deletedConnections };
  graphDiffCache_.put(genUuid(), navGraphDiff);
  console.log(graphDiffCache_);

  // var test = visNetwork_.body.data.edges.remove([targetEdge.id]);
  console.log(visNetwork_.body.edges);

  // --- [rosCache] remove the deleted nav-graph node ---
  // delete rosNavGraphNode;
  jsonNavGraphCache_.nodes = jsonNavGraphCache_.nodes.filter(aa => aa.id !== deletedNode.id);
  graphNodeNames_ = jsonNavGraphCache_.nodes.map(gn => gn.label);
  console.log(jsonNavGraphCache_);

  // --- [visCache] execute delete the cell node ---
  callback(data);
}

// function editNavEdgeData(data, callback) {
//   callback(data);
//   network2_.editEdgeMode();
// }

function btnSaveRouteNode(data, callback, _mode = 'edit') {
  console.log(data);
  // console.log(graphNodeNames_);
  // -- [protection] prevent from duplcated vertex name --
  var nodeLabel = document.getElementById("node-label").value;

  if (!inputCheck(nodeLabel)) {
    document.getElementById("node-label").style["border-color"] = "red";
    notificationMsg(2, `Input include invalid characters.`);
    return;
  } else {
    document.getElementById("node-label").style["border-color"] = "";
  }

  if (_mode === 'add' && graphNodeNames_.includes(nodeLabel)) {
    alert('DUPLICATED VERTEX LABEL!');
    return;
  }
  if (_mode === 'edit' && data.label !== nodeLabel && graphNodeNames_.includes(nodeLabel)) {
    alert('DUPLICATED VERTEX LABEL!');
    return;
  }
  console.log(jsonNavGraphCache_);
  // --- update the graph node ---
  if (!graphNodeNames_.includes(nodeLabel)) {
    graphNodeNames_.push(nodeLabel);
    // console.log(graphNodeNames_);
  }

  // the (x, y) is only to indcate the location on real scene
  var nodeX = document.getElementById("node-x").value;
  var nodeY = document.getElementById("node-y").value;
  console.log(`node pos: (${nodeX}, ${nodeY})`);
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(nodeX),
    y: Number(nodeY)
  });

  // --- cache the difference ---
  if (_mode === 'add') {
    data.id = data.id.replace(/-/g, '');
    var graphDiff = { visID: data.id, visX: Number(visPos.x), visY: Number(visPos.y), rosID: data.id, rosX: nodeX, rosY: nodeY, action: 'add', type: 'node' };
    graphDiffCache_.put(genUuid(), graphDiff);
  }
  if (_mode === 'edit') {
    console.log(jsonNavGraphCache_);
    let nodeObj = jsonNavGraphCache_.nodes.find(ng => ng.id === data.id);
    let rosX = nodeObj.x;
    let rosY = nodeObj.y;
    let visNode = visNetwork_.body.data.nodes.get(data.id);
    let visX = visNode.x;
    let visY = visNode.y;
    console.log(nodeObj);

    var graphDiff = { visID: data.id, visX: Number(visX), visY: Number(visY), rosID: data.id, rosX: rosX, rosY: rosY, action: 'edit', type: 'node' };
    graphDiffCache_.put(genUuid(), graphDiff);
  }
  console.log(graphDiffCache_);


  // --- add customized data here ---
  // -- [protection] user SHOULD NOT edit the id --
  // - * remove dash charactor from uuid --
  data.id = data.id.replace(/-/g, '');
  // - * add prefix to protect the uuid parser(dot) fail --
  // data.id = 'fa' + data.id;
  data.label = document.getElementById("node-label").value;
  data.shape = 'dot';
  data.size = 5;
  var tooltip = `<div class="farobot-map-tooltip" style="color: gray;">${data.label}@(${Number(nodeX).toFixed(2)}, ${Number(nodeY).toFixed(2)})</div>`;
  data.title = tooltip;
  data.color = 'gray';
  data.font = {
    size: 12,
    color: 'black'
  };
  data.group = 'navnode';
  data.x = visPos.x;
  data.y = visPos.y;

  clearNodePopUp();
  callback(data);

  if (_mode === 'add') {
    visNetwork_.addNodeMode();
  }

  var nodeObj = {};
  nodeObj['id'] = data.id;
  nodeObj['label'] = data.label;
  nodeObj['x'] = nodeX;
  nodeObj['y'] = nodeY;

  // --- check the node exist or not ---
  var nodeObjArr = jsonNavGraphCache_.nodes.filter(n => n.id === data.id);
  if (nodeObjArr.length === 0) {
    jsonNavGraphCache_.nodes.push(nodeObj);
    console.log(jsonNavGraphCache_);
    // var newMapGraphObj = {
    //   manipulationType: 'addNode',
    //   data: nodeObj
    // };
    // mapGraphQueue_ = getLimitSizeQueue(mapGraphQueue_, newMapGraphObj, 3);
    return;
  }

  // --- if it already exists, update the cell_coordinate [x, y] ---
  nodeObjArr[0].x = nodeX;
  nodeObjArr[0].y = nodeY;
  console.log(jsonNavGraphCache_);
}

function btnAddRouteEdge(data, callback) {
  callback(data);
  visNetwork_.addEdgeMode();
  // if (data.from == data.to) {
  //   var r = confirm("Do you want to connect the node to itself?");
  //   if (r != true) {
  //     callback(null);
  //     return;
  //   }
  // }
  // document.getElementById("edge-operation").innerText = "Add Edge";
  // editEdgeWithoutDrag(data, callback);
}

function btnDeleteRouteEdge(data, callback) {
  var delID = data.edges[0];
  var obj = visNetwork_.body.edges[delID];
  console.log(obj);
  var graphDiff = { visID: obj.id, fromId: obj.fromId, toId: obj.toId, action: 'delete', type: 'edge' };
  graphDiffCache_.put(genUuid(), graphDiff);

  // --- update jsonNavGraphCache_ ---
  jsonNavGraphCache_.edges = jsonNavGraphCache_.edges.filter(edge => !(edge.from === obj.fromId && edge.to === obj.toId));
  console.log(jsonNavGraphCache_);

  callback(data);
}

function evtDragStartRouteNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) { return; }

  // --- get node id ---
  var nodeId = params.nodes[0];
  console.log(nodeId);

  // --- get node group ---
  var dragNode = visNetwork_.body.nodes[nodeId];
  var group = dragNode.options.group;
  console.log(group);

  // --- update data cache ---
  if (group === 'navnode') {
    closeManipulationMode();

    // --- cache the difference ---
    // console.log(jsonNavGraphCache_.nodes);
    console.log(dragNode);
    let selNode = jsonNavGraphCache_.nodes.find(ca => ca.id === dragNode.id);
    console.log(selNode);
    let rosX = selNode.x;
    let rosY = selNode.y;
    let visX = dragNode.x;
    let visY = dragNode.y;

    var graphDiff = { visID: dragNode.id, visX: Number(visX), visY: Number(visY), rosID: selNode.id, rosX: rosX, rosY: rosY, action: 'edit', type: 'node' };
    graphDiffCache_.put(genUuid(), graphDiff);
  }
}

function evtDragEndRouteNode(params) {
  console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) { return; }

  // --- get node id ---
  var nodeId = params.nodes[0];
  console.log(nodeId);

  // --- get node group ---
  var group = visNetwork_.body.nodes[nodeId].options.group;
  console.log(group);

  // --- update data cache ---
  if (group === 'navnode') {
    // dragRouteNodeCb(nodeId);
    // --- update data ---
    var nodeX = visNetwork_.body.nodes[nodeId].x;
    var nodeY = visNetwork_.body.nodes[nodeId].y;
    var nodePos = tfCanvas2ROS(gMapMeta_, {
      x: nodeX,
      y: nodeY
    });
    // console.log(nodePos);

    var selNodeLabel = visNetwork_.body.nodes[nodeId].options.label;
    for (key in jsonNavGraphCache_.nodes) {
      if (jsonNavGraphCache_.nodes[key].label !== selNodeLabel) continue;

      jsonNavGraphCache_.nodes[key].x = nodePos.x;
      jsonNavGraphCache_.nodes[key].y = nodePos.y;
    }
  }
}

function evtDragEndRouteEdge(params) {
  // ---- [protection] pervent from duplicated callback ----
  if (params?.controlEdge.from === undefined || params?.controlEdge.to === undefined) { return; }

  // ---- [protection] ensure the both ends of an edge are nav-graph nodes ----
  let edges = visNetwork_.body.edges;
  let edgeArr = Object.values(edges);
  let edgeEnds = jsonNavGraphCache_.nodes.filter(ng => ng.id === params.controlEdge.from || ng.id === params.controlEdge.to);
  if (edgeEnds.length < 2) {
    let targetEdge = edgeArr.find(e => e.fromId === params.controlEdge.from && e.toId === params.controlEdge.to);
    visNetwork_.body.data.edges.remove([targetEdge.id]);
    visNetwork_.addEdgeMode();
    return;
  }

  // ---- [protection] prevent from adding duplicated edges ----
  let duplicatedEdges = edgeArr.filter(edg => edg.fromId === params.controlEdge.from && edg.toId === params.controlEdge.to);
  if (duplicatedEdges.length > 1) {
    let toDeleteEdges = duplicatedEdges.map(de => de.id);
    toDeleteEdges?.shift(); // skip the original one
    visNetwork_.body.data.edges.remove(toDeleteEdges);
    visNetwork_.addEdgeMode();
    return;
  }


  var target = Object.values(edges).filter(e => e.toId === params.controlEdge.to).map(e => e.id).at(-1);
  var edge = edges[target];

  var graphDiff = { visID: edge.id, fromId: edge.fromId, toId: edge.toId, action: 'add', type: 'edge' };
  graphDiffCache_.put(genUuid(), graphDiff);
}


// =============================
//     Cells Event Callbacks    
// =============================
let cellDiffCache_ = new FarCache();

// --- Storage Cell Manipulation ---
function btnAddCellNode(data, callback) {
  editCellMode = 'add';
  editCellData = data;
  // TODO: add validate manager
  // filling in the popup DOM elements
  document.getElementById("cell-operation").innerText = "Add Cell";
  document.getElementById("cell-createButton").innerText = "Create";

  // --- update cellsize type ---
  var cellSizeSel = document.getElementById("cell-type");
  $(cellSizeSel).empty();

  // ---previous way ---
  // cellTypes_.forEach(ct => {
  //   var opt = document.createElement("option");
  //   opt.text = ct.name;
  //   opt.value = ct.name;
  //   cellSizeSel.options.add(opt);
  // });
  // cellSizeSel.selectedIndex = 0; // default value

  // --- updated way ---
  var functionTypeOptions = _.flatten(Object.values(functionTypes_)).map(t => t.type);
  console.log(functionTypeOptions);
  // --- add `None` option ---
  functionTypeOptions.unshift('Position_cell');
  functionTypeOptions.forEach(ft => {
    var opt = document.createElement("option");
    opt.text = ft;
    opt.value = ft;
    cellSizeSel.options.add(opt);
  });
  cellSizeSel.selectedIndex = 0; // default value


  // --- default data to create a new cell ---
  data.area = 'default_area';
  data.cellUUID = genUuid().replace(/-/g, '');
  data.cellType = cellSizeSel.value;
  // console.log(data.cellType);
  data.cellDirection = 'forward';
  data.cellLoad = 'empty';
  data.labelFullName = 'new';
  // data.functionType = "None";
  data.functionType = cellSizeSel.value;
  data.markers = "None";
  data.markerOffset = "0.5, 0.5, 1.57"
  data.cellSize = "0.5, 0.5";

  // --- x, y should be assign from mouse event ---
  // --- read from mouse position --- 
  rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });

  document.getElementById("cell-x").value = rosPos.x;
  document.getElementById("cell-y").value = rosPos.y;
  document.getElementById("cell-r").value = "NaN"; // default value

  // data.r = 0.0; // default orientation: 0.0 in radian
  // var orient = cvtRad2Deg(data.r); // in degree on UI. 
  // orient = -orient; // transform orientation from ROS to UI
  // rosPos.r = orient;
  // document.getElementById("cell-r").value = rosPos.r.toFixed(2);

  document.getElementById("cell-label").value = data.labelFullName;
  document.getElementById("cell-area").value = data.area;

  // document.getElementById("cell-type").value = data.cellType;
  // document.getElementById("cell-direction").value = data.cellDirection;
  // document.getElementById("cell-load").value = data.cellLoad;
  // document.getElementById("cell-markers").value = data.markers || "None";
  // document.getElementById("marker-offset").value = data.markerOffset || "";
  // document.getElementById("cell-size").value = data.cellSize || "";
  document.getElementById("cell-type").value = data.functionType;
  document.getElementById("cell-direction").value = data.cellDirection;
  document.getElementById("cell-load").value = data.cellLoad;
  document.getElementById("cell-markers").value = data.markers;
  document.getElementById("marker-offset").value = data.markerOffset;
  document.getElementById("cell-size").value = data.cellSize;

  // --- save the edited properties ---
  document.getElementById("cell-createButton").onclick = btnSaveCellNodeData.bind(this, data, callback, 'add');
  document.getElementById("cell-cancelButton").onclick = clearNodePopUp.bind(this, callback);
  document.getElementById("cell-popUp").style.display = "block";
  updatePopUpPosition('cell');

  showNavAgentOptions();
}

function btnDeleteCellNode(data, callback) {
  editCellMode = 'delete';
  editCellData = data;
  var deletedNode = visNetwork_.body.data.nodes.get(data.nodes[0]);
  console.log(deletedNode);

  // --- cache the difference ---
  var rosCellNode = jsonCellCache_[deletedNode.area].find(aa => aa.cell_uuid === deletedNode.cellUUID);
  console.log(rosCellNode);
  var cellDiff = {
    visCellNode: deletedNode,
    rosCellNode: rosCellNode,
    action: 'delete',
    area: deletedNode.area
  };
  cellDiffCache_.put(genUuid(), cellDiff);
  console.log(cellDiffCache_);

  // --- [rosCache] remove the deleted  cell node ---
  for (key in jsonCellCache_) {
    // --- remove the cell node object ---
    jsonCellCache_[key] = jsonCellCache_[key].filter(aa => aa.cell_uuid !== deletedNode.cellUUID);

    // --- remove the area with no nodesR---
    if (jsonCellCache_[key].length === 0) {
      delete jsonCellCache_[key];
    }
  }
  console.log(jsonCellCache_);

  // --- [visCache] execute delete the cell node ---
  callback(data);
}

function btnEditCellNode(data, callback) {
  editCellMode = 'edit';
  editCellData = data;
  // TODO: add validate manager
  console.log(data);
  // --- filling in the popup DOM elements ---
  document.getElementById("cell-operation").innerText = "Edit Cell";
  document.getElementById("cell-createButton").innerText = "OK"; // "Save"

  // --- update cellsize type ---
  var cellSizeSel = document.getElementById("cell-type");
  $(cellSizeSel).empty();

  // ---previous way ---
  // cellTypes_.forEach(ct => {
  //   var opt = document.createElement("option");
  //   opt.text = ct.name;
  //   opt.value = ct.name;
  //   cellSizeSel.options.add(opt);
  // });
  // console.log(data.cellType);
  // cellSizeSel.value = data.cellType;

  // --- updated way ---
  var functionTypeOptions = _.flatten(Object.values(functionTypes_)).map(t => t.type);
  console.log(functionTypeOptions);
  functionTypeOptions.unshift('Position_cell');
  functionTypeOptions.forEach(ft => {
    var opt = document.createElement("option");
    opt.text = ft;
    opt.value = ft;
    cellSizeSel.options.add(opt);
  });
  // cellSizeSel.value = data.cellType || 'None'; // default value
  // v1.1
  // cellSizeSel.value = data.functionType || 'Position_cell'; // default value
  // v1.0
  cellSizeSel.value = functionTypeOptions.includes(data.functionType) ? data.functionType : 'None'; // default value

  console.log(jsonCellCache_);

  var areaObjArr = jsonCellCache_[data.area];
  var targetObj = areaObjArr.find(ca => ca.cell_uuid === data.cellUUID);
  var cellX = targetObj.cell_coordinate[0];
  var cellY = targetObj.cell_coordinate[1];
  var cellR = targetObj.cell_coordinate[2];
  rosPos = { x: cellX, y: cellY };

  document.getElementById("cell-x").value = rosPos.x;
  document.getElementById("cell-y").value = rosPos.y;
  let orient = cvtRad2Deg(cellR);
  document.getElementById("cell-r").value = orient || NaN;

  // data.r = -data.r;
  // var orient = cvtRad2Deg(data.r); // in degree on UI. 
  // orient = -orient; // transform orientation from ROS to UI
  // rosPos.r = orient;
  // document.getElementById("cell-r").value = rosPos.r.toFixed(2);

  document.getElementById("cell-label").value = data.labelFullName;
  document.getElementById("cell-area").value = data.area;

  // document.getElementById("cell-type").value = data.cellType;
  // document.getElementById("cell-type").value = data.functionType || "Position_cell";
  document.getElementById("cell-type").value = functionTypeOptions.includes(data.functionType) ? data.functionType : 'None';
  document.getElementById("cell-direction").value = data.cellDirection;
  document.getElementById("cell-load").value = cellLoadAdaptor.toUiCellLoad(data.cellLoad);
  document.getElementById("cell-markers").value = data.markers || "None";
  document.getElementById("marker-offset").value = data.markerOffset || "0.5, 0.5, 1.57";
  document.getElementById("cell-size").value = data.cellSize || "0.5, 0.5";

  // --- save the edited properties ---
  document.getElementById("cell-applyButton").onclick = btnApplyCellNodeData.bind(this, data, callback);
  document.getElementById("cell-createButton").onclick = btnSaveCellNodeData.bind(this, data, callback, 'edit');
  document.getElementById("cell-cancelButton").onclick = cancelNodeEdit.bind(this, callback);
  document.getElementById("cell-popUp").style.display = "block";
  updatePopUpPosition('cell');

  showNavAgentOptions();
}

async function btnApplyCellNodeData(data, callback) {
  let map = document.getElementById('map-select').value;
  // console.log(map);
  // console.log(data);  
  let targetVal = document.getElementById('cell-load').value;
  // --- FARobot 1 million idea, PATENTED! ---
  targetVal = (targetVal === 'occupied') ? 'rack' : targetVal;
  // console.log(targetVal);
  // --- trench RESTful API ---
  let cellLoad = {
    "cells_to_update": [{
      "map": map,
      "area_id": data.area,
      "cell_id": data.labelFullName,
      "status": "",
      "load": targetVal,
    }]
  };
  let res = await fetchPutCellLoad(rmtToken_, cellLoad);
  // res = await res.json();  

  // ------ response handler ------
  if (res.status === 200) {
    data.cellLoad = targetVal;
    callback(data);
    notificationMsg(1, 'The Cell Load is set successfully');
    clearNodePopUp();
  } else if (res.status === 400) {
    notificationMsg(3, 'The Target Cell in NOT Found!');
  } else if (res.status === 503) {
    notificationMsg(3, 'Failed to update Cell Config.');
  } else {
    notificationMsg(3, 'Fail to set Cell Load parameter');
  }
}

function btnSaveCellNodeData(data, callback, _mode = 'edit') {
  console.log(data);
  var cellUUID = data.cellUUID;
  var cellID = data.cellUUID;
  var cellX = document.getElementById("cell-x").value;
  var cellY = document.getElementById("cell-y").value;
  var visR = document.getElementById("cell-r").value;
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(cellX),
    y: Number(cellY)
  });

  var cellLabelFullName = document.getElementById("cell-label").value;
  var cellArea = document.getElementById("cell-area").value;
  var cellType = document.getElementById("cell-type").value;
  var functionType = document.getElementById("cell-type").value;
  var oriCellID = data.labelFullName;

  // var cellDetectionType = cellTypeAdaptor.toDetectionType(cellTypes_, cellType);
  // var cellDetectionType = cellTypeAdaptor.toDetectionType(functionTypes_, cellType);
  // var cellDetectionType = cellTypeAdaptor.toDetectionType2(functionTypes_, cellType);
  var cellDetectionType = cellTypeAdaptor.toDetectionType2(functionTypes_, functionType);
  // console.log(cellDetectionType);
  // --- [protection] prevent from detection type is null ---
  // if (cellDetectionType === null) {
  //   alert('type is NOT Set Yet!');
  //   return;
  // }
  cellDetectionType = (cellDetectionType === null) ? 'None' : cellDetectionType;

  // check cell label
  if (!inputCheck(cellLabelFullName)) {
    document.getElementById("cell-label").style["border-color"] = "red";
    notificationMsg(2, `Input include invalid characters.`);
    return;
  } else {
    document.getElementById("cell-label").style["border-color"] = "";
  }

  // check cell area
  // if (!inputCheck(cellArea)) {
  //   document.getElementById("cell-area").style["border-color"] = "red";
  //   notificationMsg(2, `Input include invalid characters.`);
  //   return;
  // }	else{
  //   document.getElementById("cell-area").style["border-color"] = "";
  // }

  // --- cache the difference ---
  if (_mode === 'add') {
    var cellDiff = {
      visID: data.id,
      visX: Number(visPos.x),
      visY: Number(visPos.y),
      visR: Number(visR),
      rosID: data.cellUUID,
      rosX: cellX,
      rosY: cellY,
      rosR: cvtDeg2Rad(Number(visR)),
      action: 'add',
      area: cellArea,
      labelFullName: cellLabelFullName,
      cellLoad: cellLoad
    };
    cellDiffCache_.put(genUuid(), cellDiff);
  }
  if (_mode === 'edit') {
    // TODO: enhance the mechaniam to identify the cell object.
    let cellObj;
    let cellPrevArea;
    for (let area in jsonCellCache_) {
      // cellObj = jsonCellCache_[area].find(ca => ca.cell_uuid === data.cellUUID);
      cellObj = jsonCellCache_[area].find(function (value, index) {
        if (value.cell_uuid === data.cellUUID) {
          cellPrevArea = area;
          console.log(value)
          return value;
        }
      });
      if (cellObj !== undefined) break;
    }
    console.log(JSON.stringify(cellObj));
    let rosX = cellObj.cell_coordinate[0];
    let rosY = cellObj.cell_coordinate[1];
    let visNode = visNetwork_.body.data.nodes.get(data.id);
    let visX = visNode.x;
    let visY = visNode.y;

    console.log(data); // UI data
    var cellDiff = {
      visID: data.id,
      visX: Number(visX),
      visY: Number(visY),
      visR: Number(visR),
      rosID: data.cellUUID,
      rosX: rosX,
      rosY: rosY,
      rosR: visR,
      action: 'edit',
      area: cellArea,
      prevarea: cellPrevArea,
      labelFullName: data.labelFullName,
      cellLoad: cellLoadAdaptor.toFileCellLoad(data.cellLoad),
      cellDirection: cellObj.direction,
      functionType: cellObj.function_type,
      markers: cellObj.markers,
      markerOffset: cellObj.marker_offset,
      cellSize: cellObj.cell_size
    };
    console.log(cellDiff);
    cellDiffCache_.put(genUuid(), cellDiff);
  }
  console.log(cellDiffCache_);

  console.log(data.cellType);
  // var target = cellTypes_.find(ct => ct.name === cellType);
  // var target = _.flatten(Object.values(functionTypes_)).find(ct => ct.type === cellType);
  var target = _.flatten(Object.values(functionTypes_)).find(ct => ct.type === functionType);
  console.log(target);
  var cellWidth = 1.0;
  var cellLength = 1.0;
  if (target !== undefined) {
    cellWidth = Number(target.width);
    cellLength = Number(target.length);
  }

  console.log(`width: ${cellWidth}, length: ${cellLength}`)

  // TODO: unify the cell creation info 
  var cellDirection = document.getElementById("cell-direction").value;
  var cellLoad = document.getElementById("cell-load").value;
  // console.log(`cell type: ${cellType}, cell detection type: ${cellDetectionTypes_}, cell direction: ${cellDirection}, cell load: ${cellLoad}`);
  var markers = document.getElementById("cell-markers").value;
  var markerOffset = document.getElementById("marker-offset").value;
  var cellSize = document.getElementById("cell-size").value;

  // --- update modified data here ---
  data.group = 'wms';
  data.label = cellLabelFullName;
  data.labelFullName = cellLabelFullName;
  data.area = cellArea;
  data.shape = 'circle';
  data.widthConstraint = 25;
  data.borderWidth = 2;
  // data.size = 20; // default: 25
  // data.color = 'rgba(4, 75, 148, 0.4)'
  var tooltip = `<div class="farobot-map-tooltip" 
                      style="color: ${getColor(cellDetectionTypes_, cellDetectionType, 'tooltipColor')}">
                  ${cellLabelFullName}@(${Number(cellX).toFixed(2)}, ${Number(cellY).toFixed(2)})
                 </div>`;
  data.title = tooltip;
  data.color = {
    background: getColor(cellDetectionTypes_, cellDetectionType, 'bgColor'),
    highlight: getColor(cellDetectionTypes_, cellDetectionType, 'focusColor'),
    border: getColor(cellDetectionTypes_, cellDetectionType, 'borderColor')
  };
  data.font = {
    size: 7,
    color: getColor(cellDetectionTypes_, cellDetectionType, 'color'),
    bold: true
  };
  data.x = visPos.x;
  data.y = visPos.y;
  data.w = cellWidth;
  data.l = cellLength;
  data.cellType = cellType || data.cellType;
  data.functionType = functionType || data.functionType;
  data.cellDetectionType = cellDetectionType;
  data.cellDirection = cellDirection;
  data.cellLoad = cellLoad;
  // data.markers = markers;
  // data.markerOffset = markerOffset;
  // data.cellSize = cellSize;
  data.labelFullName = document.getElementById("cell-label").value;
  // --- [trial] updated wms data ---
  var markers = document.getElementById("cell-markers").value;
  var markerOffset = document.getElementById("marker-offset").value;
  markerOffset = markerOffset.split(",").map(i => parseFloat(i));
  var cellSize = document.getElementById("cell-size").value;
  cellSize = cellSize.split(",").map(i => parseFloat(i));
  data.markers = markers;
  data.markerOffset = markerOffset;
  // data.cellSize = cellSize;
  // console.log(data.cellType);

  console.log(data);

  // --- [protection] prevent area from empty input ---
  var emptyCheck = cellArea.replace(/\s/g, "");
  if (cellArea === "" || emptyCheck === "") {
    alert('area SHOULD NOT be empty');
    return;
  }

  // --- [protection] prevent from duplcated cell name ---
  var cellIdArr = []
  for (key in jsonCellCache_) {
    var cid = jsonCellCache_[key].map(c => c.cell_id);
    cellIdArr.push(...cid);
  }
  // console.log(cellIdArr);
  // console.log(data);
  // console.log(Object.keys(data));
  if (_mode === 'add' && cellIdArr.includes(cellLabelFullName)) {
    alert('DUPLICATED CELL LABEL!');
    return;
  }
  if (_mode === 'edit' && oriCellID !== cellLabelFullName && cellIdArr.includes(cellLabelFullName)) {
    alert('DUPLICATED CELL LABEL!');
    return;
  }

  // --- render to canvas ---
  // console.log(data);
  clearNodePopUp();
  callback(data); // write-in to the cache
  // if (data.editType === 'add') {
  if (_mode === 'add') {
    visNetwork_.addNodeMode();
  }

  // --- update loaded cache ---
  var cellObj = {
    cell_uuid: cellUUID,
    cell_coordinate: [Number(cellX), Number(cellY), Number(cvtDeg2Rad(Number(visR)))],
    cell_id: cellLabelFullName,
    direction: cellDirection,
    load: cellLoadAdaptor.toFileCellLoad(cellLoad),
    status: "empty",
    function_type: cellDetectionType,
    width: cellWidth,
    length: cellLength,
    markers: markers,            // trial
    marker_offset: markerOffset, // trial
    cell_size: cellSize,         // trial
  };

  // --- check the cell label full name exist or not. ---
  console.log(data);
  // var areaObj = jsonCellCache_[cellArea].find(ca => ca.cell_uuid === data.cellUUID);
  console.log(data.cellUUID);
  // console.log(data.id);
  var areaObj;
  var cacheArea;
  for (let area in jsonCellCache_) {
    // console.log(area);
    areaObj = jsonCellCache_[area].find(ca => ca.cell_uuid === data.cellUUID);
    // console.log(jsonCellCache_[area]);
    // console.log(areaObj);
    if (areaObj !== undefined) {
      cacheArea = area;
      break;
    }
    console.log(jsonCellCache_[area]);
    // areaObj = jsonCellCache_[area].find(ca => ca.cell_uuid === data.id);
  }
  console.log(areaObj);

  // --- new cell case ---
  if (areaObj === undefined) {
    // console.log(cellArea);
    if (jsonCellCache_.hasOwnProperty(cellArea)) {
      jsonCellCache_[cellArea].push(cellObj);
    }
    else {
      jsonCellCache_[cellArea] = [cellObj];
    }
    console.log(jsonCellCache_);
    return;
  }

  // --- change area case ---
  if (cacheArea !== cellArea) {
    jsonCellCache_[cacheArea] = jsonCellCache_[cacheArea].filter(c => c.cell_uuid !== data.cellUUID)
    if (jsonCellCache_[cacheArea].length === 0) {
      delete jsonCellCache_[cacheArea];
    }

    if (jsonCellCache_.hasOwnProperty(cellArea)) {
      jsonCellCache_[cellArea].push(cellObj);
    }
    else {
      jsonCellCache_[cellArea] = [cellObj];
    }
    console.log(jsonCellCache_);
    return;
  }

  // --- update the cell_coordinate [x, y] ---
  areaObj.cell_uuid = cellObj['cell_uuid'];
  areaObj.cell_coordinate = cellObj['cell_coordinate'];
  areaObj.cell_id = cellObj['cell_id'];
  areaObj.direction = cellObj['direction'];
  areaObj.load = cellObj['load'];
  areaObj.status = cellObj['status'];
  areaObj.type = cellObj['type'];
  areaObj.width = cellObj['width'];
  areaObj.length = cellObj['length'];
  areaObj.function_type = cellObj['function_type'];
  areaObj.markers = cellObj['markers'];
  areaObj.marker_offset = cellObj['marker_offset'];
  areaObj.cell_size = cellObj['cell_size'];

  console.log(areaObj);
  delete areaObj;

  console.log(jsonCellCache_);
}

function evtDragStartCellNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) { return; }

  // --- get node id ---
  var nodeId = params.nodes[0];
  console.log(nodeId);

  var dragNode = visNetwork_.body.nodes[nodeId];
  // console.log(dragNode)
  // --- get node group ---
  var group = dragNode.options.group;
  console.log(group);

  // --- update data cache ---
  if (group === 'wms') {
    closeManipulationMode();
    // console.log(network2_.body.nodes[nodeId]);
    // --- get area data ---
    // var dragNode = visNetwork_.body.data.nodes.get(nodeId);
    // var dragNode = visNetwork_.body.nodes[nodeId];
    var cellArea = dragNode.options.area;
    console.log(cellArea);

    // --- cache the difference ---
    // let areaObj = jsonCellCache_[dragNode.area].find(ca => ca.cell_uuid === dragNode.cellUUID);
    console.log(jsonCellCache_[cellArea]);
    let areaObj = jsonCellCache_[cellArea].find(ca => ca.cell_uuid === dragNode.options.cellUUID);
    console.log(areaObj);
    let rosX = areaObj.cell_coordinate[0];
    let rosY = areaObj.cell_coordinate[1];
    let visX = dragNode.x;
    let visY = dragNode.y;

    var cellDiff = {
      visID: dragNode.id,
      visX: Number(visX),
      visY: Number(visY),
      rosID: dragNode.options.cellUUID,
      rosX: rosX,
      rosY: rosY,
      action: 'edit',
      area: cellArea,
      labelFullName: dragNode.options.labelFullName
    };
    cellDiffCache_.put(genUuid(), cellDiff);
    console.log(cellDiffCache_);
  }
}

function evtDragEndCellNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) { return; }

  // --- get node id ---
  var nodeId = params.nodes[0];
  console.log(nodeId);

  // --- get node group ---
  var dragNode = visNetwork_.body.nodes[nodeId];
  console.log(dragNode);
  var group = dragNode.options.group;

  // --- update data cache ---
  if (group === 'wms') {
    // console.log(network2_.body.nodes[nodeId]);
    // --- get area data ---
    // var dragNode = visNetwork_.body.data.nodes.get(nodeId);
    // var dragNode = visNetwork_.body.nodes[nodeId];
    var cellArea = dragNode.options.area;

    // --- [rosCache] update data in jsonCellCache_ ---
    // var cellX = visNetwork_.body.nodes[nodeId].x;
    // var cellY = visNetwork_.body.nodes[nodeId].y;
    var cellX = dragNode.x;
    var cellY = dragNode.y;
    var cellR = dragNode.r;
    var cellPos = tfCanvas2ROS(gMapMeta_, {
      x: cellX,
      y: cellY,
      theta: cellR
    });

    for (key in jsonCellCache_[cellArea]) {
      console.log(jsonCellCache_[cellArea][key]);
      if (jsonCellCache_[cellArea][key].cell_uuid !== dragNode.options.cellUUID) continue;

      jsonCellCache_[cellArea][key].cell_coordinate = [Number(cellPos.x), Number(cellPos.y), Number(cellPos.theta)];
    }
  }
}

// =============================
//     Zone Event Callbacks    
// =============================
function btnAddZoneNode(data, callback) {
  // --- update View ---
<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
  document.getElementById("zone-node-operation").innerText = "Add Node";
  document.getElementById("zone-node-createButton").value = "Create";
=======
  document.getElementById("zone-node-operation").innerText = "Add Vertex";
  document.getElementById("zone-node-createButton").value = "Create";

  // --- update node location from cache ---
  var rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });
  document.getElementById("zone-node-x").value = rosPos.x;
  document.getElementById("zone-node-y").value = rosPos.y;

  document.getElementById("zone-node-createButton").onclick = btnSaveZoneNode.bind(this, data, callback);
  document.getElementById("zone-node-cancelButton").onclick = clearNodePopUp.bind(this, callback);

  document.getElementById("zone-node-popUp").style.display = "block";
  updatePopUpPosition('zonenode');
}


let last_node_id = '';
function btnSaveZoneNode(data, callback) {
  console.log(data);

  // the (x, y) is only to indcate the location on real scene
  var nodeX = document.getElementById("zone-node-x").value;
  var nodeY = document.getElementById("zone-node-y").value;
  console.log(`node pos: (${nodeX}, ${nodeY})`);
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(nodeX),
    y: Number(nodeY)
  });
  console.log(`vis pos: (${Number(visPos.x)}, ${Number(visPos.y)})`);

  let hasStartNode = false;
  for (i in visNetwork_.body.nodes) {
    if (visNetwork_.body.nodes[i].options.group === 'zonenode') {
      if (visNetwork_.body.nodes[i].options.id.includes('start')) {
        console.log(visNetwork_.body.nodes[i].options.x);
        console.log(visNetwork_.body.nodes[i].options.y);
        hasStartNode = true;
        break;
      }
    }
  }

  data.shape = 'dot';
  data.size = 5;
  data.color = 'blue';
  data.group = 'zonenode';
  data.x = visPos.x;
  data.y = visPos.y;

  if (!hasStartNode) {
    data.id = 'start-' + data.id.replace(/-/g, '');
    // data.label = 'start_zone_node';
    clearNodePopUp();
    callback(data);
  } else {
    data.id = 'con-' + data.id.replace(/-/g, '');
    // data.label = 'zone_related_node';

    clearNodePopUp();
    callback(data);

    // --- add a connection edge ---
    var dir_edges = [];
    var dir_edge = {
      from: last_node_id,
      to: data.id,
      arrows: {
        to: {
          enabled: false
        }
      }
    }
    dir_edges.push(dir_edge);
    visNetwork_.body.data.edges.update(dir_edges);
  }
  visNetwork_.addNodeMode();

  last_node_id = data.id;
}

let selectedEditType = '';
$('.custom-radio input').on('change', async function () {
  $('#btn-create-cell-conn').hide();
  $('#edit-tools-list').css('width', ''); // reset tools list width
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js

  // --- update node location from cache ---
  var rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });
  document.getElementById("zone-node-x").value = rosPos.x;
  document.getElementById("zone-node-y").value = rosPos.y;

  document.getElementById("zone-node-createButton").onclick = btnSaveZoneNode.bind(this, data, callback);
  document.getElementById("zone-node-cancelButton").onclick = clearNodePopUp.bind(this, callback);

  document.getElementById("zone-node-popUp").style.display = "block";
  updatePopUpPosition('zone-node');
}

function btnSaveZoneNode(data, callback) {
  console.log(data);

  // the (x, y) is only to indcate the location on real scene
  var nodeX = document.getElementById("zone-node-x").value;
  var nodeY = document.getElementById("zone-node-y").value;
  console.log(`node pos: (${nodeX}, ${nodeY})`);
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(nodeX),
    y: Number(nodeY)
  });
  console.log(`vis pos: (${Number(visPos.x)}, ${Number(visPos.y)})`);

  let prioritySelect = document.getElementById("zone-priority-select");
  let zoneNameInput = document.getElementById("zone-name");

  data.shape = 'dot';
  data.size = 5;
  data.color = prioritySelect.value === '0' ? 'rgba(241,160,76,1)' : 'rgba(89,121,215,1)';
  data.group = 'zonenode';
  data.x = visPos.x;
  data.y = visPos.y;
  data.id = data.id.replace(/-/g, '');
  data.zone = zoneNameInput.value;

  clearNodePopUp();
  callback(data);

  visNetwork_.addNodeMode();
}

function btnAddZoneBoundary(data, callback) {
  let prioritySelect = document.getElementById("zone-priority-select");
  data.color = prioritySelect.value === '0' ? 'rgba(241,160,76,1)' : 'rgba(89,121,215,1)';
  data.arrows = {
    to: {
      enabled: false
    }
  };

  if (data.from !== data.to) {
    callback(data);

    // let zone_name = document.getElementById("zone-name").value;
    // if (!jsonZoneCache_.hasOwnProperty(zone_name)) {
    //   jsonZoneCache_[zone_name] = {};
    // }
    // if (jsonZoneCache_[zone_name].hasOwnProperty('edges')) {
    //   jsonZoneCache_[zone_name].edges.push(data.id);
    // } else {
    //   jsonZoneCache_[zone_name].edges = [data.id];
    // }
  }

  visNetwork_.addEdgeMode();
  // console.log(jsonZoneCache_);
}

let selectedEditType = '';
$('.custom-radio input').on('change', async function () {
  $('#btn-create-cell-conn').hide();
  $('#edit-tools-list').css('width', ''); // reset tools list width

  selectedEditType = this.value;
  closeManipulationMode();
  closeZoneEditor();

  if (this.value === 'rdMapImg') {
    fadeInEditToolbar('map-tool');
    hideInfoSection();

    var divFab = document.getElementById('sub_tab_2');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_1');
    divNetwork.classList.remove('active');
    var divZoneFab = document.getElementById('sub_tab_3');
    divZoneFab.classList.remove('active');

    setVisInteractiveMode(1);

    return;
  }

  if (this.value === 'rdRoute') {
    reloadMapImage();
    fadeInEditToolbar('route-tool');
    showInfoSection();
    hideAdditionalTools();

    var divFab = document.getElementById('sub_tab_1');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_2');
    divNetwork.classList.remove('active');
    var divZoneFab = document.getElementById('sub_tab_3');
    divZoneFab.classList.remove('active');

    setVisInteractiveMode(2);

    return;
  }

  if (this.value === 'rdCell') {
    reloadMapImage();
    fadeInEditToolbar('cell-tool');
    showInfoSection();
    hideAdditionalTools();

    var divFab = document.getElementById('sub_tab_1');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_2');
    divNetwork.classList.remove('active');
    var divZoneFab = document.getElementById('sub_tab_3');
    divZoneFab.classList.remove('active');

    setVisInteractiveMode(3);

    return;
  }

  if (this.value === 'rdConnCell') {
    reloadMapImage();
    fadeInEditToolbar('conn-cell-tool');
    showInfoSection();
    hideAdditionalTools();

    var divFab = document.getElementById('sub_tab_1');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_2');
    divNetwork.classList.remove('active');
    var divZoneFab = document.getElementById('sub_tab_3');
    divZoneFab.classList.remove('active');

    setVisInteractiveMode(3);

    // --- `Edit Properties Tools` box transformation ---
    $('#btn-create-cell-conn').show();
    $('#edit-tools-list').css('width', '36%');

    // --- load the connected cells ---
    await loadConnCells();

    return;
  }

  if (this.value === 'rdZone') {
<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
    fadeInEditToolbar('zone-tool');

    var divZoneFab = document.getElementById('sub_tab_3');
    divZoneFab.classList.add('active');
    var divFab = document.getElementById('sub_tab_1');
    divFab.classList.remove('active');
    var divNetwork = document.getElementById('sub_tab_2');
    divNetwork.classList.remove('active');

    setVisInteractiveMode(1);
=======
    reloadMapImage();
    fadeInEditToolbar();

    var divFab = document.getElementById('sub_tab_1');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_2');
    divNetwork.classList.remove('active');

    setVisInteractiveMode(4);
    visNetwork_.addNodeMode();
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js

    return;
  }

  // if (this.value === 'rdConfig') {
  //   fadeInEditToolbar('config-tool');
  //   hideInfoSection();
  //   console.log('recv visual config');
  //   setVisInteractiveMode(0);

  //   drawFabricMap();

  //   var divFab = document.getElementById('sub_tab_2');
  //   divFab.classList.add('active');
  //   var divNetwork = document.getElementById('sub_tab_1');
  //   divNetwork.classList.remove('active');

  //   return;
  // }
});

let gridSpan_ = 50;

function btnSetMapGridScale() {
  var meterPerGrid = Number($('#grid-span').val());

  gridSpan_ = meterPerGrid / gMapMeta_.resolution;

  visNetwork_.redraw();

  // --- update fabric canvas ---
  // drawFabricBoard();
}

function toggleEdit(_status = true) {
  var st = !_status;

  document.getElementById('customRadio1').disabled = st;
  document.getElementById('customRadio2').disabled = st;
  document.getElementById('customRadio3').disabled = st;
  document.getElementById('customRadio4').disabled = st;
  document.getElementById('customRadio5').disabled = st;

  document.getElementById('customCheckbox1').disabled = _status;
  document.getElementById('customCheckbox2').disabled = _status;
  document.getElementById('customCheckbox3').disabled = _status;
  document.getElementById('customCheckbox4').disabled = _status;

  if (_status === false) {
    document.getElementById('customRadio1').checked = _status;
    document.getElementById('customRadio2').checked = _status;
    document.getElementById('customRadio3').checked = _status;
    document.getElementById('customRadio4').checked = _status;
    document.getElementById('customRadio5').checked = _status;

    fadeInEditToolbar();

    // for (j in network2_.body.edges) {
    // console.log(network2_.body.edges[j]);
    // network2_.body.edges[j].options.color.color = 'rgba(0,0,0,1.0)';
    // }
  }

  $('#map-select').prop('disabled', _status);

  var color = (_status) ? 'white' : 'lightgray';
  $('#property-group').css('background-color', color);

  var color2 = (_status) ? 'lightgray' : 'white';
  $('#property-view-group').css('background-color', color2);

  // swtich to the default view
  var divFab = document.getElementById('sub_tab_1');
  divFab.classList.add('active');
  var divNetwork = document.getElementById('sub_tab_2');
  divNetwork.classList.remove('active');

  setVisInteractiveMode(0);
}

// --- Nav graph ---
function cancelEdit(_btn_id) {
  closeManipulationMode();
  setVisInteractiveMode(_btn_id === 'cancel-route-edit' ? 2 : 3);
}

function btnAddVertex() {
  visNetwork_.addNodeMode();
  showManipulationModeText("Add Vertex");
  clearNodePopUp();
}

function btnAddEdge() {
  visNetwork_.addEdgeMode();
  showManipulationModeText("Add Edge");
  clearNodePopUp();
}

function btnEditVertex() {
  visNetwork_.editNode();
}

function btnEditEdge() {
  visNetwork_.editEdgeMode();
}

// --- Cell ---
function btnAddCell() {
  visNetwork_.addNodeMode();
  showManipulationModeText("Add Cell");
  clearNodePopUp();
}

function btnEditCell() {
  visNetwork_.editNode();
}

// --- Zone ---
function btnAddNode() {
  visNetwork_.addNodeMode();
  clearNodePopUp();
}

function btnAddBoundary() {
  visNetwork_.addEdgeMode();
  clearNodePopUp();
}

let geometry = new Geometry();
let isDrawMode = false;
let isAddVertexMode = false;
let pointArray = [];
let lineArray = [];
let activeShape;
let activeLine;
function btnAddZonePolygon() {
  isDrawMode = true;
  fZoneCanvas.selection = false;

  // --- rebind mouse events ---
  removeZoneCanvasEvents();
  fZoneCanvas.on('mouse:down', function (opt) {
    if (!isDrawMode) return;

    // --- click on toolbar 'Draw Zone' ---
    // --- [protection] repeat drawing / invalid zone type ---
    let zone_name = document.getElementById("zone-name").value;
    let zone_type = document.getElementById('zone-type-select').selectedIndex;
    if (zone_type === 0) {
      alert('Please select zone type first!');
      return;
    }
    let dupZone = _.find(jsonZoneArray, { zone_name: zone_name });
    if (dupZone !== undefined) {
      if (dupZone.hasOwnProperty('vertices') && dupZone.vertices.length > 0) {
        alert('DUPLICATED ZONE NAME!');
        return;
      }
    }
    if (jsonZoneCache_.hasOwnProperty('vertices')) {
      if (jsonZoneCache_.vertices.length > 0) {
        alert('ZONE is drawn!');
        return;
      }
    }

    if (pointArray.length > 0 && opt.target && opt.target.id === pointArray[0].id) {
      // --- finish drawing ---
      if (pointArray.length < 3) {
        alert('Can NOT add a zone under 3 points!');
        this.selection = false;
      } else {
        // --- check for overlapping zones(boundary level) ---
        let p1 = pointArray[pointArray.length - 1];
        let p2 = pointArray[0];
        let zoneIds = getLineIntersectZones(p1, p2);
        let isOverlapping = isIntersectSameZone(zoneIds);
        if (isOverlapping) {
          alert('Same zone type and priority can NOT overlap!');
          this.selection = false;
          return;
        }
        generatePolygonZone();
        toggleToolbarEditing();
      }
    } else {
      // --- check for overlapping zones(vertex level) ---
      let zoom = fZoneCanvas.getZoom();
      let x = opt.e.layerX / zoom;
      let y = opt.e.layerY / zoom;
      let p = new Point2D(x, y);
      let zoneIds = getVertexIntersectZones(p);
      let isOverlapping = isIntersectSameZone(zoneIds);
      if (isOverlapping) {
        alert('Same zone type and priority can NOT overlap!');
        this.selection = false;
        return;
      }
      addPoint(opt);
    }
  });

  fZoneCanvas.on('mouse:up', function (opt) {
    this.selection = true;
  });

  fZoneCanvas.on('mouse:move', function (opt) {
    if (!isDrawMode) return;
    if (activeLine && activeLine.class === 'line') {
      const pointer = fZoneCanvas.getPointer(opt.e);
      // --- change destination point ---
      activeLine.set({
        x2: pointer.x,
        y2: pointer.y
      });
      const points = activeShape.get('points');
      points[pointArray.length] = {
        x: pointer.x,
        y: pointer.y,
      };
      activeShape.set({
        points
      });
    }
    fZoneCanvas.renderAll();
  });
}

function btnEditZonePolygon() {
  drawFabricEditingZonePoints();
  clearNodePopUp();
  resetAddZoneVertexParams();
  toggleZoneEditButtonState('edit');

  // --- rebind mouse events ---
  removeZoneCanvasEvents();
  fZoneCanvas.on('mouse:down', function (opt) {
    if (opt.button !== 1 || opt.target === null || !opt.target.id.includes(ZONE_POINT_PREFIX)) return;
    // --- highlight point color ---
    let x = opt.target.left;
    let y = opt.target.top;
    removeFabricZoneObjects(ZONE_POINT_PREFIX);
    drawFabricSelectedZonePoints(selShape, 'edit', { x, y });

    var rosPos = tfCanvas2ROS(gMapMeta_, { x: x, y: y });
    document.getElementById("zone-node-x").value = rosPos.x;
    document.getElementById("zone-node-y").value = rosPos.y;
    document.getElementById("zone-node-createButton").onclick = btnEditPolygonPoint.bind(opt.target);
    document.getElementById("zone-node-cancelButton").onclick = clearNodePopUp;
    document.getElementById("zone-node-popUp").style.display = "block";
    updatePopUpPosition('zone-node');
    fZoneCanvas.setActiveObject(opt.target);
  });
}

function btnDeleteZonePolygon() {
  clearNodePopUp();
  resetAddZoneVertexParams();

  let id = jsonZoneCache_.uuid;
  removeFabricZoneObjects(id);
  delete jsonZoneCache_.vertices;
  toggleZoneEditTabButtons();
  let configZone = _.find(jsonZoneArray, { uuid: id });
  if (configZone === undefined) return;
  delete configZone.vertices;
}

let vertexIdx;
let oldPoints = [];
let newPoints = [];
function btnAddZoneVertex() {
  isAddVertexMode = true;
  drawFabricEditingZonePoints();
  clearNodePopUp();
  toggleZoneEditButtonState('add');

  // --- rebind mouse events ---
  removeZoneCanvasEvents();
  fZoneCanvas.on('mouse:down', function (opt) {
    if (opt.button !== 1) return;
    if (opt.target !== null && opt.target.id.includes(ZONE_POINT_PREFIX)) {
      // --- start add zone vertex mode ---
      if (!isAddVertexMode || newPoints.length > 0) return;
      document.getElementById('add-zone-vertex').disabled = true;

      oldPoints = JSON.parse(JSON.stringify(selShape.points));
      let x = opt.target.left;
      let y = opt.target.top;
      vertexIdx = _.findIndex(selShape.points, { x, y });
      console.log(`Start index: ${vertexIdx} (${x}, ${y})`);

      // --- highlight point color ---
      removeFabricZoneObjects(ZONE_POINT_PREFIX);
      drawFabricSelectedZonePoints(selShape, 'add', { x, y });
    } else {
      if (!isAddVertexMode || vertexIdx === undefined) return;
      document.getElementById('add-zone-vertex').style.display = "none";
      document.getElementById('end-zone-vertex').style.display = "block";
      document.getElementById('cancel-zone-edit').disabled = true;

      let zoom = fZoneCanvas.getZoom();
      let x = opt.e.layerX / zoom;
      let y = opt.e.layerY / zoom;
      let newPoint = { x, y, theta: 0 };
      newPoints.push(newPoint);

      const points = selShape.get('points');
      if (newPoints.length === 1) {
        points.splice(vertexIdx + 1, 0, newPoint);
      } else {
        let prevPoint = newPoints[newPoints.length - 2];
        vertexIdx = _.findIndex(selShape.points, { x: prevPoint.x, y: prevPoint.y });
        points.splice(vertexIdx + 1, 0, newPoint);
      }
      // console.log(points);

      drawFabricEditingZone(newPoint);
    }
  });
}

function btnEndZoneVertex() {
  document.getElementById('add-zone-vertex').style.display = "block";
  document.getElementById('add-zone-vertex').disabled = false;
  document.getElementById('end-zone-vertex').style.display = "none";
  document.getElementById('cancel-zone-edit').disabled = false;

  if (vertexIdx === undefined) return;
  // --- re-render zone on canvas ---
  let id = selShape.id.replace(ZONE_POLYGON_PREFIX, '');
  let index = _.findIndex(jsonZoneArray, { uuid: id });
  if (index !== -1) {
    let type = jsonZoneArray[index].zone_type;
    let color = zoneTypes_[type].fillColor;
    let priority = jsonZoneArray[index].priority;
    selShape.set({
      fill: color,
      opacity: priority === 0 ? 0.3 : 0.8
    });
  }
  removeFabricZoneObjects(selShape.id);
  drawFabricZone(id);
  removeFabricZoneObjects(ZONE_POINT_PREFIX);
  drawFabricSelectedZonePoints(selShape);

  // --- update zone cache data ---
  updateZoneVertices(id, selShape.points);
  console.log(selShape.points);

  vertexIdx = undefined;
  oldPoints = [];
  newPoints = [];
}

function btnDeleteZoneVertex() {
  drawFabricEditingZonePoints();
  clearNodePopUp();
  resetAddZoneVertexParams();
  toggleZoneEditButtonState('delete');

  // --- rebind mouse events ---
  removeZoneCanvasEvents();
  fZoneCanvas.on('mouse:down', function (opt) {
    if (opt.button !== 1 || opt.target === null || !opt.target.id.includes(ZONE_POINT_PREFIX)) return;

    fZoneCanvas.setActiveObject(opt.target);

    let id = opt.target.id.replace(ZONE_POINT_PREFIX, '');
    let zone_obj = getFabricZoneObject(id);
    if (zone_obj.points.length < 4) {
      alert('Zone can NOT under 3 points!');
      fZoneCanvas.selection = false;
      return;
    }

    // --- update zone cache data ---
    let x = opt.target.left;
    let y = opt.target.top;
    _.remove(zone_obj.points, p => p.x === x && p.y === y);
    updateZoneVertices(id, zone_obj.points);

    // --- re-render zone on canvas ---
    removeFabricZoneObjects(opt.target.id);
    drawFabricSelectedZonePoints(selShape);
  });
}

function btnCancelZoneEdit() {
  clearNodePopUp();
  removeZoneCanvasEvents();
  resetSelectedPolygonPoints();
  resetAddZoneVertexParams();
  toggleZoneEditButtonState();
}

// --- delete ---
function btnDeleteSelected() {
  // var delMapGraphObj = {
  //   manipulationType: 'delete',
  //   data: network2_.getSelection()
  // };
  // delMapGraphObj = JSON.parse(JSON.stringify(delMapGraphObj));
  // mapGraphQueue_ = getLimitSizeQueue(mapGraphQueue_, delMapGraphObj, 3);

  visNetwork_.deleteSelected(visNetwork_);
  closeManipulationMode();
}

async function btnEnableTriton() {
  // console.log('triton pop-up');
  // --- check swarm-core whether the triton map exist or not ---
  var tritonData = restGetTritonMap();
  // TODO: change the state by tritonData
  // --- fetch triton data success case ---
  $('#include-triton-map').prop('disabled', false);
  $('#map-exsitence-msg').text('Lasted modified data: 2020/05/17');
  // --- fetch triton data fail case ---
  // $('#include-triton-map').prop('disabled', true);
  // $('#map-exsitence-msg').text('No triton map is available for this graph');

  // --- check the triton agent is available or not ---
  // if (agents_.length <= 0) {
  //   // --- create triton map button is off ---
  //   // $('#create-triton-map').prop('disabled', true);
  //   return;
  // }

  // --- create triton map button is on ---
  $('#create-triton-map').prop('disabled', false);
  $("#agent-select").find('option').remove();
  // agents_ = agents_.sort();
  // agents_.forEach(function (filename) {
  //   console.log(filename);
  //   $("#agent-select").append(`<option value='${filename}'>${filename}</option>`);
  // });
}

$('#include-triton-map').click(function () {
  var bIncluded = this.checked;

  $.get(`/maps/${currSelectedMap_}/info`, function (data, status) {
    if (status === "success") {
      console.log("------ type of map data in yaml ------");
      // console.log(typeof data);
      // console.log(`${data}`);

      data = data.split(/\r?\n/);

      var bTritonParam = false;
      for (var i in data) {
        var el = data[i].split(':');

        var key = el[0];
        if (key === 'triton') {
          data[i] = `triton: ${bIncluded}`;
          bTritonParam = true;
          break;
        }
      }

      if (!bTritonParam) {
        data.push(`triton: ${bIncluded}`);
      }

      console.log(data);
      // serialized back to a string.
      var updatedData = '';
      for (var j in data) {
        if (data[j] !== "") {
          updatedData += data[j] + '\n';
        }
      }

      // --- save updated data back to server ---
      $.post(`/maps/${currSelectedMap_}/info`, {
        content: updatedData
      }, function () {
        console.log('post request return');
      });
    }
  });
});

// --- send task function ---
$('#create-triton-map').click(function () {
  console.log('send the triton_mapping role');

  // var task_params = document.getElementById('task_params').value;
  // var el = document.getElementById('agent-select');
  // var robot_id = el.options[el.selectedIndex].text;

  // --- fill new_task ---
  // new_task.robot_id = robot_id;
  // new_task.priority = 1;
  // new_task.task_type = "ground_texture_mapping";
  // new_task.task_name = "TestTriton";
  // new_task.task_params = "";

  // --- fill send_msgs ---
  // send_msgs.tasks = [new_task];

  // console.log(robot_id);
  // task_request_pub.publish(send_msgs);
  // wsPubTaskReq(send_msgs);
});

async function deleteMap() {
  // --- get the selected map filename ---
  var sel = document.getElementById('map-select');
  var mapName = sel.options[sel.selectedIndex].value;
  var mapAlias = sel.options[sel.selectedIndex].text;
  // console.log(`selected fleet: ${mapName}`);

  // --- check if map is included by fleet ---
  fleetFileArray = [];
  let fleetFiles = await fetchGetFleets(rmtToken_);
  fleetFiles = Object.keys(fleetFiles);

  for (const fleetFile of fleetFiles) {
    var fleetName = fleetFile;
    let fltSettings = {};
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetName);
    fltSettings[fleetName] = fltConfigs;

    var fltKey = Object.keys(fltSettings)[0]; // suppose only one key-value pair
    var index = fltSettings[fltKey].maps.indexOf(mapName);
    if (index >= 0) {
      fleetFileArray.push(fltKey);
    }
  }
  console.log(fleetFileArray);
  if (fleetFileArray.length > 0) {
    if (confirm("There are fleets still using this map. Are you sure you want to delete?")) {
      await deleteTargetMap(mapName, mapAlias);
    }
  } else {
    await deleteTargetMap(mapName, mapAlias);
  }
}

async function deleteTargetMap(_mapName, _mapAlias) {
  // --- send the delete request ---
  // await restDeleteMapData(mapName);
  // await restDeleteMapAlias(mapName);
  let res = await fetchDeleteMap(rmtToken_, _mapName);
  res = await res.json();
  console.log(res);

  // --- remove maps from fleet ---
  for (let fleetFile of fleetFileArray) {
    let fltSettings = {};
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetFile);
    fltSettings[fleetFile] = fltConfigs;
    console.log(fltSettings);

    var index = fltSettings[fleetFile].maps.indexOf(_mapName);
    fltSettings[fleetFile].maps.splice(index, 1);
    fltSettings = JSON.stringify(fltSettings);
    console.log(fltSettings);
    await restPostFleetSettings(fleetFile, fltSettings);
    fleetFileArray = fleetFileArray.filter(e => e !== fleetFile);
  }

  // --- refresh the load procedure ---
  loadBundledMapDataAsync();

  // --- pop-up to hint the user ---
  notificationMsg(0, `${_mapAlias} is deleted!`);
}

$(document).on('click', 'button', function (e) {
  var btn_id = $(this).attr('id');
  if (btn_id === 'save-changes') {
    // applyConfigService.callService(srvRequest, function (_message) {
    //   console.log(`apply config. done with messages: ${_message}`);
    // })

    // --- send the service to apply configuration ---
    // wsApplyConfig();
  }
});

$('#hide-sidebar').on('click', function () {
  $('.control-sidebar').ControlSidebar('toggle');
});

// --- default value update on FAR-2172 ---
const FUNCTION_TYPES = {
  "charger": [
    {
      "model": "far",
      "offset": [
        0.185,
        -0.271,
        0
      ],
      "recognition": [
        "Charger"
      ],
      "type": "charger_template"
    }
  ],
  "rack": [
    {
      "offset": [
        0,
        0,
        0
      ],
      "payload": 10,
      "recognition": [
        "Rack"
      ],
      "size": [
        0.76,
        1.06,
        0.5
      ],
      "type": "rack_template"
    }
  ]
};


function popSidebarCellTypes() {
  // console.log('pop up sidebar cell type');
  const cellTypeDeck = $('#sb-cell-size-deck');
  cellTypeDeck.empty();
  // console.log(cellTypes_);
  // console.log(cellDetectionTypes_);

  // cellTypes_.forEach(ct => {
  // ====== previous WMS version ======
  // // console.log(cellSizeAssets_[key]);
  // const template = document.querySelector('#sb-cell-type-row');
  // const node = document.importNode(template.content, true);
  // var nameNode = node.querySelector('.cell-type-name');
  // nameNode.textContent = ct.name;

  // var detectionTypesSel = node.querySelector('.cell-detection-type');
  // $(detectionTypesSel).empty();
  // cellDetectionTypes_.forEach(cdt => {
  //   var opt = document.createElement("option");
  //   opt.text = cdt.label;
  //   opt.value = cdt.name;
  //   detectionTypesSel.options.add(opt);
  // });
  // detectionTypesSel.value = ct.detectionType;
  // // console.log(ct.detectionType);

  // var widthNode = node.querySelector('.cell-width');
  // widthNode.setAttribute("id", `${key}-width`);
  // widthNode.value = Number(ct.width).toFixed(2);
  // var lengthNode = node.querySelector('.cell-length');
  // lengthNode.setAttribute("id", `${key}-length`);
  // lengthNode.value = Number(ct.length).toFixed(2);

  // var editNode = node.querySelector('.edit-cell-size');
  // var cardNode = node.querySelector('.card-body');
  // editNode.addEventListener('click', editCellTypeCb.bind(cardNode));

  // });

  for (var [ftKey, ftVal] of Object.entries(functionTypes_)) {
    ftVal.forEach(ft => {
      console.log(ft);
      // ====== updated WMS version ======
      const template = document.querySelector('#sb-cell-type-row2');
      const node = document.importNode(template.content, true);
      var paramNode = node.querySelector('.cell-type-name');
      paramNode.textContent = ft.type;

      paramNode = node.querySelector('.cell-detection-type');
      paramNode.addEventListener('change', updateFunctionTypeParams.bind(paramNode, functionTypes_));
      paramNode.value = ftKey;

      paramNode = node.querySelector('.device-recognition');
      // paramNode.value = ft.recognition[0] || "";

      // --- switch to corresponding recognition option ---
      var recogParentNode = node.querySelector('.device-recognition').parentNode;
      var recogNode = node.querySelector('.device-recognition');
      recogNode.remove();
      var newRecogNode = document.createElement("select");
      newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
      var opt = document.createElement("option");
      console.log(ftKey);
      opt.text = (ftKey === 'rack') ? 'Rack' : 'Charger';
      opt.value = (ftKey === 'rack') ? 'Rack' : 'Charger'; // name equals cell type value
      newRecogNode.options.add(opt);
      newRecogNode.disabled = true;
      recogParentNode.append(newRecogNode);


      // paramNode = node.querySelector('.device-offset');
      // // paramNode.textContent = ft.offset;
      // paramNode.value = ft.offset;


      const extraParamTemplate = document.querySelector(`#sb-cell-type-${ftKey}`)
      const extraParamNodes = document.importNode(extraParamTemplate.content, true);
      paramNode = node.querySelector('.function-type-params');
      paramNode.append(extraParamNodes);

      if (ftKey === 'rack') {
        paramNode = node.querySelector('.device-size');
        // --- [FAR-2277] swap function type rack width, length ---
        [ft.size[1], ft.size[0]] = [ft.size[0], ft.size[1]];
        paramNode.value = ft.size;
        paramNode.disabled = true;

        paramNode = node.querySelector('.device-payload');
        paramNode.value = ft.payload;
        paramNode.disabled = true;
      }
      if (ftKey === 'charger') {
        paramNode = node.querySelector('.device-offset');
        paramNode.value = ft.offset;
        paramNode.disabled = true;

        paramNode = node.querySelector('.device-model');
        $(paramNode).select2();
        // console.log(ft.model);
        console.log(fleetModels_);

        if (fleetModels_.length > 0) {
          fleetModels_.forEach(fm => {
            console.log(fm);
            let opt = document.createElement("option");
            opt.text = opt.value = fm; // name equals cell type value
            paramNode.options.add(opt);
          });
        }

        // paramNode.value = ft.model;
        console.log(ft.model);
        console.log($(paramNode));
        $(paramNode).val(ft.model);
        $(paramNode).trigger('change');

        paramNode.disabled = true;
      }

      let cardNode = node.querySelector('.card-body');

      var editNode = node.querySelector('.edit-cell-size');
      editNode.setAttribute('id', `ft-edit-${ft.type}`);
      editNode.addEventListener('click', editCellTypeCb2.bind(cardNode));

      var deleteNode = node.querySelector('.delete-cell-size');
      deleteNode.setAttribute('id', `ft-delete-${ft.type}`);
      deleteNode.addEventListener('click', deleteCellTypeCb2.bind(cardNode));

      cellTypeDeck.append(node);
    });

  }


  // --- load cell dectection types on adding a new cell type row ---
  var sbDetectionTypesSel = document.getElementById('sbft-cell-detection-type');
  $(sbDetectionTypesSel).empty();

  // --- previous way ---
  // cellDetectionTypes_.forEach(cdt => {
  //   var opt = document.createElement("option");
  //   opt.text = cdt.label;
  //   opt.value = cdt.name; // name equals cell type value
  //   sbDetectionTypesSel.options.add(opt);
  // });

  functionDetectionTypes_ = [
    {
      label: 'Rack',
      name: 'rack'
    }, {
      label: 'Charger',
      name: 'charger'
    }
  ];
  // --- updated way ---
  functionDetectionTypes_.forEach(fdt => {
    var opt = document.createElement("option");
    opt.text = fdt.label;
    opt.value = fdt.name; // name equals cell type value
    sbDetectionTypesSel.options.add(opt);
  });
  sbDetectionTypesSel.addEventListener('change', updateFunctionTypeParams.bind(sbDetectionTypesSel, functionTypes_));

  // --- update save button binding ---
  var placeholder = $('#save-placeholder');
  placeholder.empty();
  var saveBtn = document.createElement('button');
  saveBtn.setAttribute('class', 'btn btn-primary btn-lg');
  saveBtn.innerHTML = "Save";
  saveBtn.addEventListener('click', saveCellSizesCb.bind(this));

  placeholder.append(saveBtn);

  applyFontSize(getSavedFontSize(), '#cell-size-sb');
}

function updateFunctionTypeParams(_funcTypes) {
  var paramNode = this.parentNode.parentNode.querySelector('.function-type-params');
  var typeName = this.parentNode.parentNode.querySelector('.cell-type-name').textContent;

  var targetTemplate = document.querySelector(`#sb-cell-type-${this.value}`);
  const targetNodes = document.importNode(targetTemplate.content, true);
  var node = {};

  if (this.value === 'rack') {
    // --- switch to corresponding recognition option ---
    var recogParentNode = paramNode.querySelector('.device-recognition').parentNode;
    var recogNode = paramNode.querySelector('.device-recognition');
    recogNode.remove();
    var newRecogNode = document.createElement("select");
    newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
    var opt = document.createElement("option");
    opt.text = 'Rack';
    opt.value = 'Rack'; // name equals cell type value
    newRecogNode.options.add(opt);
    recogParentNode.append(newRecogNode);

    // --- flush previous params ---
    var dNode = paramNode.querySelector('.device-model');
    if (dNode !== undefined) {
      dNode.parentNode.remove();
    }
    var dNode = paramNode.querySelector('.device-offset');
    if (dNode !== undefined) {
      dNode.parentNode.remove();
    }

    // --- append new params --- 
    var target = _funcTypes[this.value].find(t => t.type === typeName);
    var size = (target) ? target.size : "";
    var payload = (target) ? target.payload : "";
    // console.log(size);
    // console.log(payload);

    node = targetNodes.querySelector('.device-size');
    node.value = size;
    node = targetNodes.querySelector('.device-payload');
    node.value = payload;
  }

  if (this.value === 'charger') {
    // --- switch to corresponding recognition option ---
    var recogParentNode = paramNode.querySelector('.device-recognition').parentNode;
    var recogNode = paramNode.querySelector('.device-recognition');
    recogNode.remove();
    var newRecogNode = document.createElement("select");
    newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
    var opt = document.createElement("option");
    opt.text = 'Charger';
    opt.value = 'Charger'; // name equals cell type value
    newRecogNode.options.add(opt);
    recogParentNode.append(newRecogNode);

    // --- flush previous params ---
    var dNode = paramNode.querySelector('.device-size');
    if (dNode !== undefined) {
      dNode.parentNode.remove();
    }
    dNode = paramNode.querySelector('.device-payload');
    if (dNode !== undefined) {
      dNode.parentNode.remove();
    }

    // --- append new params --- 
    var target = _funcTypes[this.value].find(t => t.type === typeName);

    node = targetNodes.querySelector('.device-model');
    $(node).select2();
    if (fleetModels_.length > 0) {
      fleetModels_.forEach(fm => {
        var opt = document.createElement("option");
        opt.text = opt.value = fm; // name equals cell type value
        node.options.add(opt);
      });
    }

  }

  paramNode.append(targetNodes);
}

function editCellTypeCb() {
  var detectionTypeNode = this.querySelector('.cell-detection-type');
  var widthNode = this.querySelector('.cell-width');
  var lengthNode = this.querySelector('.cell-length');

  detectionTypeNode.disabled = !detectionTypeNode.disabled;
  widthNode.readOnly = !widthNode.readOnly;
  lengthNode.readOnly = !lengthNode.readOnly;

  var btnIconNode = this.querySelector('.fas');
  btnIconNode.classList.toggle("fa-pen");
  btnIconNode.classList.toggle("fa-eye");
}

function deleteCellTypeCb2() {
  this.parentNode.remove();
}

function editCellTypeCb2() {
  var node = this.querySelector('.cell-detection-type');
  node.disabled = !node.disabled;
  var ftClass = node.value;

  node = this.querySelector('.device-recognition');
  // node.disabled = !node.disabled;
  node.disabled = true;
  // node = this.querySelector('.device-offset');
  // node.disabled = !node.disabled;

  if (ftClass === 'rack') {
    node = this.querySelector('.device-size');
    node.disabled = !node.disabled;

    node = this.querySelector('.device-payload');
    node.disabled = !node.disabled;
  }
  if (ftClass === 'charger') {
    node = this.querySelector('.device-offset');
    node.disabled = !node.disabled;

    node = this.querySelector('.device-model');
    node.disabled = !node.disabled;
  }

  var btnIconNode = this.querySelector('.fas');
  btnIconNode.classList.toggle("fa-pen");
  btnIconNode.classList.toggle("fa-eye");
}

async function saveCellSizesCb() {
  // console.log('save cell sizes');
  const cellSizeDeck = document.querySelectorAll('#sb-cell-size-deck > .card');
  console.log(cellSizeDeck);

  // --- STALE VERSION ---
  // var functionTypes = {
  //   rack: [],
  //   charger: [],
  //   position: []
  // }

  // cellSizeDeck.forEach((cell) => {
  //   console.log(cell);

  //   // ------ previous version ------
  //   // var name = cell.querySelector('.cell-type-name').textContent;
  //   // var value = cell.querySelector('.cell-detection-type').value;
  //   // var width = cell.querySelector('.cell-width').value;
  //   // var length = cell.querySelector('.cell-length').value;
  //   // console.log(`name: ${name}, value: ${value}, w: ${width}, h: ${length}`);

  //   // console.log(cellTypes_);
  //   // var target = cellTypes_.find(ct => ct.name === name);
  //   // console.log(target);
  //   // let typeUUID = genUuid().replace(/-/g, '');
  //   // if (target === undefined) {
  //   //   target = {
  //   //     "name": name,
  //   //     "id": `${name}-${typeUUID}`,
  //   //     "detectionType": value,
  //   //     "width": width,
  //   //     "length": length,
  //   //   };
  //   //   cellTypes_.push(target);
  //   // } else {
  //   //   target.detectionType = value;
  //   //   target.width = width;
  //   //   target.length = length;
  //   // }

  //   // ------ updated version ------
  //   var name = cell.querySelector('.cell-type-name').textContent;
  //   var value = cell.querySelector('.cell-detection-type').value;
  //   var recog = cell.querySelector('.device-recognition').value;
  //   console.log(`name: ${name}, value: ${value}`);

  //   var target = {};
  //   target['type'] = name;
  //   target['recognition'] = [recog];

  //   if (value === 'rack') {
  //     var size = cell.querySelector('.device-size').value;
  //     var payload = cell.querySelector('.device-payload').value;
  //     console.log(`size: ${size}, payload: ${payload}`);
  //     target['size'] = size.split(',');
  //     // --- [FAR-2277] swap function type rack width, length ---
  //     [target.size[1], target.size[0]] = [target.size[0], target.size[1]];
  //     target['payload'] = payload;
  //   }
  //   if (value === 'charger') {
  //     var offset = cell.querySelector('.device-offset').value;
  //     console.log(`recog: ${recog}, offset: ${offset}`);
  //     target['offset'] = offset.split(',');

  //     // var model = cell.querySelector('.device-model').value;
  //     // target['model'] = model;
  //     var models = cell.querySelector('.device-model');
  //     var selectedModels = $(models).select2("data");
  //     selectedModels = selectedModels.map(s => s.text);
  //     console.log(selectedModels);
  //     target['model'] = selectedModels;
  //   }
  //   functionTypes[value].push(target);

  // });

  // --- data type rectification: convert string to float ---
  // var tmp = functionTypes['rack'];
  // for (let i in tmp) {
  //   // tmp[i].offset = tmp[i].offset.map(a => parseFloat(a));
  //   // console.log(functionTypes_.rack.find(r=>r.type===tmp[i].type));
  //   // --- load the original offset ---
  //   let funcType = functionTypes_.rack.find(r => r.type === tmp[i].type);
  //   tmp[i].offset = funcType?.offset || [0, 0, 0];
  //   tmp[i].size = tmp[i].size.map(a => parseFloat(a));
  //   tmp[i].payload = parseFloat(tmp[i].payload);
  // }
  // tmp = functionTypes['charger'];
  // for (let i in tmp) {
  //   tmp[i].offset = tmp[i].offset.map(a => parseFloat(a));
  // }
  // console.log(functionTypes);



  // --- UPDATED VERSION ---
  let functionTypes = [];

  cellSizeDeck.forEach((cell) => {
    console.log(cell);

    // ------ updated version ------
    var name = cell.querySelector('.cell-type-name').textContent;
    var value = cell.querySelector('.cell-detection-type').value;
    var recog = cell.querySelector('.device-recognition').value;
    console.log(`name: ${name}, value: ${value}`);

    var target = {};
    target['type'] = name;
    target['category'] = value;
    target['recognition'] = [recog];

    if (value === 'rack') {
      var size = cell.querySelector('.device-size').value;
      var payload = cell.querySelector('.device-payload').value;
      console.log(`size: ${size}, payload: ${payload}`);
      // target['size'] = size.split(',');
      // --- [FAR-2277] swap function type rack width, length ---
      target['offset'] = {
        x: 0,
        y: 0,
        theta: 0
      };
      let targetSize = size.split(',');
      target['size'] = {
        width: Number(targetSize[0]) || 0,
        length: Number(targetSize[1]) || 0,
        height: Number(targetSize[2]) || 0
      };
      target['payload'] = Number(payload);
    }
    if (value === 'charger') {
      var offset = cell.querySelector('.device-offset').value;
      console.log(`recog: ${recog}, offset: ${offset}`);
      // target['offset'] = offset.split(',');
      let targetOffset = offset.split(',');
      target['offset'] = {
        x: Number(targetOffset[0]) || 0,
        y: Number(targetOffset[1]) || 0,
        theta: Number(targetOffset[2]) || 0
      }

      // var model = cell.querySelector('.device-model').value;
      // target['model'] = model;
      var models = cell.querySelector('.device-model');
      var selectedModels = $(models).select2("data");
      selectedModels = selectedModels.map(s => s.text);
      console.log(selectedModels);
      target['model'] = selectedModels;
    }

    // functionTypes[value].push(target);
    functionTypes.push(target);
  });


  // console.log(cellTypes_);
  // var settingsObj = { "cell.types": cellTypes_ };
  // await restPutSettings(settingsObj);

  // --- [native API] ---
  // // await restPostCellTypes(cellTypes_);
  // let res = await restPostFunctionTypes(functionTypes);
  // console.log(res);
  // if (res === 'OK') {
  //   notificationMsg(0, 'Function types saved Successfully!');
  // }
  // else {
  //   notificationMsg(1, 'Fail to save function types!');
  // }
  // --- [swarm API] ---
  console.log(functionTypes_);
  console.log(functionTypes);

  let ftRes = await fetchPutFunctionTypes(rmtToken_, functionTypes);
  console.log(ftRes);
  if (ftRes?.statusText === 'OK') {
    notificationMsg(1, 'Function types saved Successfully!');
  } else {
    notificationMsg(3, 'Fail to save function types!');
  }
  let myRes = await ftRes.json();
  console.log(myRes);

  // --- reload function-type menu data ---
  // --- [native API] ---
  // functionTypes_ = await restGetFunctionTypes();
  // functionTypes_ = JSON.parse(functionTypes_);

  // --- [swarm API] ---
  ftRes = await fetchGetFunctionTypes(rmtToken_);
  functionTypes_ = await ftRes.json();
}

function createNewCellSize0() {
  // --- fetch the expected value from input widgets ---
  // [protection] if name is empty, return 
  if ($('#sbft-cell-name').val() === "") {
    alert('The cell type name should NOT be empty!');
    return;
  }

  var cellTypeName = $('#sbft-cell-name').val();
  var cellDetectionType = $('#sbft-cell-detection-type').val();
  var cellTypeWidth = $('#sbft-cell-width').val() || '1.0';
  var cellTypeLength = $('#sbft-cell-length').val() || '1.0';

  // --- append the value to the cell-size-deck ---
  const cellTypeDeck = $('#sb-cell-size-deck');

  const template = document.querySelector('#sb-cell-type-row');
  const node = document.importNode(template.content, true);
  var nameNode = node.querySelector('.cell-type-name');
  nameNode.textContent = cellTypeName;

  // --- update cellsize type ---
  var addTypeSel = document.getElementById('sbft-cell-detection-type');
  console.log(addTypeSel);
  var selectedType = addTypeSel.value;
  console.log(selectedType);

  var detectTypesSel = node.querySelector('.cell-detection-type');
  $(detectTypesSel).empty();
  cellDetectionTypes_.forEach(cdt => {
    var opt = document.createElement("option");
    opt.text = cdt.label;
    console.log(cdt);
    opt.value = cdt.name;
    detectTypesSel.options.add(opt);
  });
  detectTypesSel.value = selectedType;

  var detectionNode = node.querySelector('.cell-detection-type');
  console.log(cellDetectionType);
  detectionNode.value = cellDetectionType;
  var widthNode = node.querySelector('.cell-width');
  widthNode.setAttribute("id", `${cellTypeName}-width`);
  widthNode.value = Number(cellTypeWidth).toFixed(2);
  var lengthNode = node.querySelector('.cell-length');
  lengthNode.setAttribute("id", `${cellTypeName}-length`);
  lengthNode.value = Number(cellTypeLength).toFixed(2);

  var editNode = node.querySelector('.edit-cell-size');
  var cardNode = node.querySelector('.card-body');
  editNode.addEventListener('click', editCellTypeCb.bind(cardNode));

  cellTypeDeck.append(node);

  // --- empty the input ---
  $('#sbft-cell-name').val('');
  $('#sbft-cell-width').val('');
  $('#sbft-cell-length').val('');
}

function createNewCellSize() {
  // --- fetch the expected value from input widgets ---
  // [protection] if name is empty, return 
  if ($('#sbft-cell-name').val() === "") {
    alert('The cell type name should NOT be empty!');
    return;
  }

  var cellTypeName = $('#sbft-cell-name').val();
  var cellDetectionType = $('#sbft-cell-detection-type').val();
  console.log(cellDetectionType);


  // --- append the value to the cell-size-deck ---
  const cellTypeDeck = $('#sb-cell-size-deck');

  const template = document.querySelector('#sb-cell-type-row2');
  const node = document.importNode(template.content, true);
  var nameNode = node.querySelector('.cell-type-name');
  nameNode.textContent = cellTypeName;

  // --- update cell detection type ---
  var addTypeSel = document.getElementById('sbft-cell-detection-type');
  // console.log(addTypeSel);
  var selectedType = addTypeSel.value;
  console.log(selectedType);

  var detectTypesSel = node.querySelector('.cell-detection-type');
  // $(detectTypesSel).empty();

  detectTypesSel.value = selectedType;

  const extraParamTemplate = document.querySelector(`#sb-cell-type-${cellDetectionType}`)
  const extraParamNodes = document.importNode(extraParamTemplate.content, true);
  paramNode = node.querySelector('.function-type-params');
  paramNode.append(extraParamNodes);

  let userDefinedParams = document.querySelector('#user-defined-params');
  let pNode = {};

  paramNode = node.querySelector('.device-recognition');
  pNode = userDefinedParams.querySelector('.device-recognition');
  paramNode.value = pNode.value;
  $(paramNode).prop('disabled', true);
  pNode.selectedIndex = 0;
  // pNode.value = "";

  // paramNode = node.querySelector('.device-offset');
  // pNode = userDefinedParams.querySelector('.device-offset');
  // paramNode.value = pNode.value;
  // $(paramNode).prop('disabled', true);
  // pNode.value = "";

  if (cellDetectionType === 'rack') {
    paramNode = node.querySelector('.device-size');
    pNode = userDefinedParams.querySelector('.device-size');
    paramNode.value = pNode.value;
    $(paramNode).prop('disabled', true);
    pNode.value = "";

    paramNode = node.querySelector('.device-payload');
    pNode = userDefinedParams.querySelector('.device-payload');
    paramNode.value = pNode.value;
    $(paramNode).prop('disabled', true);
    pNode.value = "";
  }
  if (cellDetectionType === 'charger') {
    paramNode = node.querySelector('.device-offset');
    pNode = userDefinedParams.querySelector('.device-offset');
    paramNode.value = pNode.value;
    $(paramNode).prop('disabled', true);
    pNode.value = "";

    paramNode = node.querySelector('.device-model');
    $(paramNode).select2();
    pNode = userDefinedParams.querySelector('.device-model');
    if (fleetModels_.length > 0) {
      fleetModels_.forEach(fm => {
        var opt = document.createElement("option");
        opt.text = opt.value = fm; // name equals cell type value
        paramNode.options.add(opt);
      });
    }
    // paramNode.value = pNode.value;
    // $(paramNode).select2('val', pNode.value);
    var selectedModels = $(pNode).select2("data");
    // console.log(selectedModels);
    selectedModels = selectedModels.map(s => s.text);
    console.log(selectedModels);
    $(paramNode).val(selectedModels);
    $(paramNode).trigger('change');
    $(paramNode).prop('disabled', true);

    $(pNode).val("");
  }

  var cardNode = node.querySelector('.card-body');

  var editNode = node.querySelector('.edit-cell-size');
  editNode.setAttribute('id', `ft-edit-${cellTypeName}`)
  editNode.addEventListener('click', editCellTypeCb2.bind(cardNode));

  var deleteNode = node.querySelector('.delete-cell-size');
  deleteNode.setAttribute('id', `ft-delete-${cellTypeName}`);
  deleteNode.addEventListener('click', deleteCellTypeCb2.bind(cardNode));

  cellTypeDeck.append(node);

  // --- flush the general inputs ---
  $('#sbft-cell-name').val('');
  $('#sbft-cell-detection-type').val('rack');
}

function createZoneCardView(_zone) {
  const template = document.querySelector('#zone-card');
  const node = document.importNode(template.content, true);

  let cardNode = node.querySelector('.card');
  cardNode.setAttribute('id', `zone-container-${_zone.uuid}`);
  cardNode.addEventListener('click', editZoneEditor.bind(cardNode, _zone));
  cardNode.style.pointerEvents = _zone.activate ? 'none' : 'auto';

  let switchNode = node.querySelector('.activation-switch');
  switchNode.id = _zone.uuid;
  switchNode.checked = _zone.activate;

  let isDarkMode = getSavedTheme() === 'dark';
  let type = zoneTypes_[_zone.zone_type] || 'none';
  let suffix = isDarkMode ? '-light' : '';
  let imgNode = node.querySelector('.zone-type-img');
  imgNode.src = `${getImagePath()}/zones/${type.imgName}${suffix}.png`;

  let titleNode = node.querySelector('.zone-option-name');
  titleNode.textContent = _zone.zone_name;

  return node;
}

function createZoneItemView(_configName, _defs) {
  const template = document.querySelector('#zone-config-item');
  const node = document.importNode(template.content, true);

  var topNode = node.querySelector('.item');
  topNode.addEventListener('click', enrollConfigToZone.bind(this, _configName, _defs));

  let scaleFontSize = getFontScaleSize(16);
  var titleNode = node.querySelector('.product-title');
  titleNode.style.fontSize = `${scaleFontSize}px`;
  titleNode.textContent = _configName;

  return node;
}

function createZoneConfigRow(_configName, _configVal, _defs) {
  const template = document.querySelector('#edit-config-row');
  const node = document.importNode(template.content, true);

  var rowNode = node.querySelector('.row');
  var labelNode = node.querySelector('.config-label > span');
  labelNode.textContent = _configName;

  let desc = _defs.description || '';
  let range = _defs.valid_value.data_range || '';
  let rangeTranStr = range.replace(/[[\]]/g, '').replace(':', '~');
  let unit = _defs.unit || '';
  let type = _defs.valid_value.data_type;

  if (!isEmptyString(desc) || !isEmptyString(rangeTranStr) || !isEmptyString(unit)) {
    let descStr = !isEmptyString(desc) ? `${langTemplateObj_.editor.ttp_Desc}: ${desc}\n` : '';
    let rangeStr = !isEmptyString(rangeTranStr) ? `\n${langTemplateObj_.editor.ttp_Range}: ${rangeTranStr}\n` : '';
    let unitStr = !isEmptyString(unit) ? `\n${langTemplateObj_.editor.ttp_Unit}: ${unit}` : '';
    var infoNode = node.querySelector('.custom-tooltip');
    infoNode.title = descStr + rangeStr + unitStr;
    infoNode.style.display = 'block';
  }

  var inputNode = node.querySelector('.form-control');
  inputNode.id = _configName;

  var btnEditNode = node.querySelector('.config-edit');
  if (type === 'bool') {
    var inputVal, boolString;
    if (typeof _configVal === "boolean") {
      boolString = (!_configVal).toString();
      inputVal = _configVal;
    } else {
      if (_configVal === "false") {
        boolString = "true";
        inputVal = _configVal;
      } else {
        boolString = "false";
        inputVal = "true";
      }
    }
    boolString = boolString.charAt(0).toUpperCase() + boolString.slice(1);
    inputNode.value = inputVal;

    btnEditNode.remove();
    var inputGroupAppend = node.querySelector('.input-group-append');
    var btn = document.createElement('button');
    btn.innerHTML = `Set ${boolString}`;
    btn.classList.add('btn', 'btn-sm', 'btn-secondary');
    inputGroupAppend.appendChild(btn);
    btn.addEventListener('click', boolButtonSwitch.bind(inputNode.parentElement));
  } else {
    inputNode.value = _configVal;
    btnEditNode.addEventListener('click', editButtonSwitch.bind(inputNode.parentElement));
  }

  var btnRemoveNode = node.querySelector('.remove-config');
  btnRemoveNode.addEventListener('click', removeConfigfromZone.bind(rowNode, _configName));
  return node;
}

function createDeleteZoneButton(_zone) {
  removeDeleteZoneButton();
  var deleteBtn = document.createElement('button');
  deleteBtn.setAttribute('class', 'btn btn-default');
  deleteBtn.innerHTML = langTemplateObj_.editor.btn_Delete;
  deleteBtn.addEventListener('click', removeZone.bind(this, _zone));
  document.getElementById('delete-btn-placeholder').append(deleteBtn);
}

function removeDeleteZoneButton() {
  var placeholder = document.getElementById('delete-btn-placeholder');
  removeAllChildNodes(placeholder);
}

function showCancelEditButton() {
  document.getElementById("cancel-route-edit").style.display = "inline";
  document.getElementById("cancel-cell-edit").style.display = "inline";
}

function hideCancelEditButton() {
  document.getElementById("cancel-route-edit").style.display = "none";
  document.getElementById("cancel-cell-edit").style.display = "none";
}

function hideDeleteButtons() {
  document.getElementById("delete-node").style.display = "none";
  document.getElementById("delete-cell").style.display = "none";
}

function showNavAgentOptions() {
  $('#nav-agent').show();
}

function hideNavAgentOptions() {
  $('#nav-agent').hide();
}

function showManipulationModeText(modeName) {
  $('#edit-status').show();
  $('#edit-status-title').text('Status.');
  $('#edit-status-badge').text(`${modeName} Mode`);
  showCancelEditButton();
}

function hideManipulationModeText() {
  $('#edit-status').hide();
}

function showAdditionalTools() {
  $('#additional-tools').show();
}

function hideAdditionalTools() {
  $('#additional-tools').hide();
}

function showInfoSection() {
  $('#edit-tools-info').show();
}

function hideInfoSection() {
  $('#edit-tools-info').hide();
}

function openZoneEditor() {
  toggleZoneEditTabButtons();
  document.getElementById('zone-editor').style.display = "block";
}

function closeZoneEditor() {
  clearNodePopUp();
  restoreUnsavedFabricZone();
  removeUnsavedFabricZones();
  removeDeleteZoneButton();
  resetSelectedPolygonPoints();
  resetSelectedZoneCard();
  resetZoneCacheData();
  resetAddZoneVertexParams();

  document.getElementById('zone-editor').style.display = "none";

  // --- rebind mouse events ---
  removeZoneCanvasEvents();
  fZoneCanvas.on('mouse:down', function (opt) {
    if (opt.button !== 1 || opt.target === null || !opt.target.id.includes(ZONE_POLYGON_PREFIX)) return;
    // --- click on polygon ---

    // --- open zone editor ---
    let zone_uuid = opt.target.id.replace(ZONE_POLYGON_PREFIX, '');
    let switchNode = document.querySelector(`#zone-container-${zone_uuid} .activation-switch`);
    if (switchNode.checked) return;
    document.getElementById(`zone-container-${zone_uuid}`).click();
    fZoneCanvas.selection = false;
    activeSelectedZone(opt.target);
    selShape = opt.target;
  });
}

function resetAddZoneVertexParams() {
  isAddVertexMode = false;
  vertexIdx = undefined;
  oldPoints = [];
  newPoints = [];
}

function closeManipulationMode() {
  hideCancelEditButton();
  hideDeleteButtons();
  hideManipulationModeText();
  clearNodePopUp();
}

$(window).bind('beforeunload', function () {
  var unsaved = false;
  switch (selectedEditType) {
    case 'rdMapImg':
      unsaved = mapImageChanged;
      // unsaved = fCanvas._objects.length > 0;
      console.log(unsaved);
      break;
    case 'rdRoute':
      unsaved = Number(graphDiffCache_.size) > 0;
      break;
    case 'rdCell':
    case 'rdConnCell':
      unsaved = Number(cellDiffCache_.size) > 0;
      break;
    default:
      break;
  }

  if (unsaved) {
    return "You have unsaved changes on this page. Are you sure you want to leave?";
  }
});

// --- add input values validator ---
function validateVertexInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const vertex_validatorManager = new ValidatorMananger();

  const nodeLabelRule = new Rule('node-label', nodeLabelValidation);
  const nodeXRule = new Rule('node-x', numberValueValidation);
  const nodeYRule = new Rule('node-y', numberValueValidation);

  vertex_validatorManager.addValidator(nodeLabelRule);
  vertex_validatorManager.addValidator(nodeXRule);
  vertex_validatorManager.addValidator(nodeYRule);
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
    if (res.bValid) {
      $(this).css('box-shadow', '');
      $(this).css('border-color', "");
      $(this).css('outline-color', "");
    } else {
      $(this).css('box-shadow', '0 0 10px #CC0000');
      $(this).css('border-color', "red");
      $(this).css('outline-color', "red");
    }

    console.log(res.strMsg);
    $(this).data('powertip', res.strMsg).powerTip();
    $.powerTip.show($(this));

    // --- [UX] Interaction Logics ---
    $('#node-createButton').prop('disabled', !res.bValid);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('node-input');
  for (let el of targets) {
    console.log(el)
    el.addEventListener("keyup", validationFlow.bind(el, vertex_validatorManager));
  }
}

// --- add input values validator ---
function validateCellInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();

  const cellLabelRule = new Rule('cell-label', textNameValidation);
  const cellAreaRule = new Rule('cell-area', textNameValidation);
  const cellXRule = new Rule('cell-x', numberValueValidation);
  const cellYRule = new Rule('cell-y', numberValueValidation);
  const markerOffsetRule = new Rule('marker-offset', markerOffsetValidation);
  const cellSizeRule = new Rule('cell-size', cellSizeValidation);

  validatorManager.addValidator(cellLabelRule);
  validatorManager.addValidator(cellAreaRule);
  validatorManager.addValidator(cellXRule);
  validatorManager.addValidator(cellYRule);
  validatorManager.addValidator(markerOffsetRule);
  validatorManager.addValidator(cellSizeRule);
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
    if (res.bValid) {
      $(this).css('box-shadow', '');
      $(this).css('border-color', "");
      $(this).css('outline-color', "");
    } else {
      $(this).css('box-shadow', '0 0 10px #CC0000');
      $(this).css('border-color', "red");
      $(this).css('outline-color', "red");
    }

    console.log(res.strMsg);
    $(this).data('powertip', res.strMsg).powerTip();
    $.powerTip.show($(this));

    // --- [UX] Interaction Logics ---
    $('#cell-createButton').prop('disabled', !res.bValid);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('cell-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

// --- add input values validator ---
function validateZoneInputEvent(inputNode) {
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
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const zone_validatorManager = new ValidatorMananger();
  const zoneNameRule = new Rule('zone-name', textNameValidation, $('#save-changes'));
  zone_validatorManager.addValidator(zoneNameRule);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {

    let validator;
    if (this.id === 'zone-name') {
      validator = vm.getValidator(this.id);
    } else {
      validator = new Rule(this.id, zoneConfigValidation, $('#save-changes'));
    }

    if (validator == undefined) { return; }

    validator.run(this.value);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('zone-input');
  for (let el of targets) {
    el.addEventListener("input", validationFlow.bind(el, zone_validatorManager));
  }

  if (inputNode.id === 'zone-name') return;
  if (!inputNode.classList.contains("input-handler")) {
    inputNode.classList.add("input-handler");
    inputNode.addEventListener("input", debounce(validationFlow.bind(inputNode)));
  }
}

// ====== customized page-related validation rules ======
function markerOffsetValidation(inputVal) {
  const isThreeParams = inputVal.split(',').length === 3;
  const bRes = isThreeParams;
  let strMsg = "";
  strMsg += (isThreeParams) ? strMsg : '- the number of delimiter(,) SHOULD BE 2';

  return { bValid: bRes, strMsg: strMsg }
}

function cellSizeValidation(inputVal) {
  const isTwoParams = inputVal.split(',').length === 2;
  const bRes = isTwoParams;
  let strMsg = "";
  strMsg += (isTwoParams) ? strMsg : '- the number of delimiter(,) SHOULD BE 1';

  return { bValid: bRes, strMsg: strMsg }
}

function nodeLabelValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$]+$/;                   // forbidden characters \ / : * ? " < > |
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names
  const reg4 = /\s/;                                      // space
  const reg5 = /^\d[0-9a-zA-Z]*$/;                        // start with number

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && !reg5.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (!reg5.test(inputVal)) ? '' : '- cannot start with number';
  strMsg.push(msg5);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

function zoneConfigValidation(inputVal) {
  // console.log(inputVal);
  let bRes = true;
  let strMsg = "";
  let type = allZoneConfs[this._name].valid_value.data_type;
  let range = allZoneConfs[this._name].valid_value.data_range.replace(/[[\]]/g, '');

  switch (type) {
    case 'double': {
      var minVal = parseFloat(range.split(":")[0]);
      var maxVal = parseFloat(range.split(":")[1]);
      var doubleVal = parseFloat(inputVal);
      if (!$.isNumeric(doubleVal)) {
        strMsg = "value type should be " + type;
      }
      if (doubleVal > maxVal || doubleVal < minVal) {
        strMsg = "value must be in the range of " + range.replace(':', '~');
      }
      bRes = $.isNumeric(doubleVal) && doubleVal <= maxVal && doubleVal >= minVal;
      break;
    }
    case 'int': {
      var minVal = parseInt(range.split(":")[0]);
      var maxVal = parseInt(range.split(":")[1]);
      var intVal = parseInt(inputVal);
      if (!Number.isInteger(Number(inputVal))) {
        strMsg = "value type should be " + type;
      }
      if (intVal > maxVal || intVal < minVal) {
        strMsg = "value must be in the range of " + range.replace(':', '~');
      }
      bRes = Number.isInteger(Number(inputVal)) && intVal <= maxVal && intVal >= minVal;
      break;
    }
  }
  return { bValid: bRes, strMsg: strMsg }
}

// --- add input values validator ---
function validateRenameInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#rename_confirm');

  const mapnameRule = new Rule('new-mapname', reNameMapValidation, interaction_element);

  validatorManager.addValidator(mapnameRule);
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
  const targets = document.getElementsByClassName('rename-map-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

function reNameMapValidation(inputVal) {
  var mapName = $("#map-select option:selected").text();

  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  const mapNameNoChange = (mapName === inputVal ? true : false);
  let mapAliasExists = false
  if (inputVal == "" || mapNameNoChange) {
    // pass
  } else {
    let data = restMapAliasExistence(inputVal);
    if (data == null) {
      mapAliasExists = false;
    } else {
      if (mapAliasExists === undefined) {
        mapAliasExists = false;
      } else {
        mapAliasExists = data.responseJSON;
      }
    }
  }

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && !mapNameNoChange && !mapAliasExists;
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
  if (mapNameNoChange) {
    const msg6 = (!mapNameNoChange) ? '' : '- map name no change';
    strMsg.push(msg6);
  } else {
    const msg6 = (!mapAliasExists) ? '' : '- map name exist';
    strMsg.push(msg6);
  }

  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// ------ generic validation rules ------
function cellNameValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  let cell_name_duplicate = false;

  var cellIdArr = []
  for (key in jsonCellCache_) {
    var cid = jsonCellCache_[key].map(c => c.cell_id);
    cellIdArr.push(...cid);
  }

  if (editCellMode === 'add' && cellIdArr.includes(inputVal)) {
    cell_name_duplicate = true;
  }
  if (editCellMode === 'edit' && editCellData.labelFullName !== inputVal && cellIdArr.includes(inputVal)) {
    cell_name_duplicate = true;
  }

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && !cell_name_duplicate;
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
  const msg6 = (!cell_name_duplicate) ? '' : '- duplicate cell name';
  strMsg.push(msg6);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// --- add input values validator ---
function validationFlowFunctionType(vm) {
  const validator = vm.getValidator(this.id);
  if (validator == undefined) { return; }

  const res = validator.run(this.value);

  // --- [STYLING] reflect validation result ---

  // --- [UX] Interaction Logics ---
}

function validateFunctionTypeInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#sbft-add-btn');

  const functiontypenameRule = new Rule('sbft-cell-name', textNameValidation, interaction_element);
  const sizeRule = new Rule('sbft-size', sizeValueValidation, interaction_element);
  const payloadRule = new Rule('sbft-payload', payloadValueValidation, interaction_element);

  validatorManager.addValidator(functiontypenameRule);
  validatorManager.addValidator(sizeRule);
  validatorManager.addValidator(payloadRule);
  // console.log(validatorManager);

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('function-type-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlowFunctionType.bind(el, validatorManager));
  }
  return validatorManager;
}

function sizeValueValidation(inputVal) {
  const reg1 = /^(-?(0|[1-9]\d*)(\.\d+)?)\,(-?(0|[1-9]\d*)(\.\d+)?)\,(-?(0|[1-9]\d*)(\.\d+)?)$/;  // (+|-)0-9
  const bRes = reg1.test(inputVal);

  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid number or includes invalid charaters or invalid format';
  strMsg.push(msg1);

  return { bValid: bRes, strMsg: strMsg }
}

function payloadValueValidation(inputVal) {
  const reg1 = /^[0-9.]+$/;                // (+|-)0-9
  const bRes = reg1.test(inputVal);
  let strMsg = "";
  strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters';

  return { bValid: bRes, strMsg: strMsg }
}

// ======== Connected Cells ========
// --- LOAD existing connections ---
async function loadConnCells() {
  // --- swarm_core-wise ---
  let data2 = await fetchGetAllMaps(rmtToken_);
  mapObj = await data2.json();
  console.log(Object.keys(mapObj));
  let allMaps = Object.keys(mapObj)

  // --- fetch all the options ---
  const asyncGetFleetCells = async (mapArray) => {
    let result = [];
    try {
      for (const m of mapArray) {
        let res = await fetchGetMapCells(rmtToken_, m);
        res = await res.json();
        res = (typeof res === 'string') ? JSON.parse(res) : res;
        console.log(res);
        for (let a of Object.keys(res)) {
          res[a].forEach(c => {
            if (c.cell_coordinate.length < 3 || c?.cell_coordinate[2] === null) { return; }
            result.push({ map: m, area: a, cell: c.cell_id });
          });
        }
        await sleep(1000); // sleep 1s
      }
    } catch (e) {
      console.log(e);
    }
    return result;
  };

  let fullConnData = await asyncGetFleetCells(allMaps);
  console.log(fullConnData);

  setLocalStorageByKey('connCells', JSON.stringify(fullConnData));

  // --- fetch all the connections ---
  // ------ get the connected cells history from API ------
  const apiCellConn = await fetchGetCellRelation(rmtToken_);
  // console.log(apiCellConn);

  // const apiCellConn = {
  //   "conn1":
  //     [
  //       {
  //         "map_name": 'AUO',
  //         "transition_cell": "a_temp_cell"
  //       },
  //     ],
  // };

  // --- hide overlay ---
  $('#preload-overlay').hide();

  // --- render the history with option ---
  $('#conn-cell-tool').empty();

  if (typeof apiCellConn !== 'object') { return; }
  for (let connName of Object.keys(apiCellConn)) {
    // console.log(apiCellConn);
    let historyConn = apiCellConn[connName];
    let historyConnCells = historyConn.map(c => c.transition_cell);

    // --- start to fill the options ---
    genCellConnectionDom(connName, fullConnData, historyConnCells);
  }
}

// --- CREATE a new connection ---
function btnCreateCellConn() {
  $('#name-cell-conn').modal('show');
}

$('#confirm-conn-name').on('click', function () {
  let newConnName = $('#new-conn-name').val();
  // console.log(newConnName);

  // --- get all the cells data from local storage ---
  let fullConnData = getSavedCellRelation();
  fullConnData = JSON.parse(fullConnData);
  console.log(fullConnData);

  // --- start to fill the options ---
  let isDup = testDupConnName(newConnName);

  if (isDup) {
    $('#new-conn-name').val('');
    alert('The Connection name is duplcated!');
    return;
  }

  genCellConnectionDom(newConnName, fullConnData, '');
  // --- reset procedure ---
  $('#name-cell-conn').modal('hide');
});

function cbEditCellConn() {
  this.toggleAttribute('disabled');
  let iconNode = $(this).parent().parent().find('.edit-cell-conn > i');
  iconNode.toggleClass('fas fa-pen');
  iconNode.toggleClass('fas fa-eye');
}

function cbDeleteCellConn() {
  this.remove();
}

function genCellConnectionDom(_connName, _fullConnData, _selOptions) {
  let template = document.querySelector('#tmpl-cell-conn');

  const node = document.importNode(template.content, true);
  let mainCard = node.querySelector('.conn-card');
  let nameNode = node.querySelector('.conn-name');

  nameNode.textContent = _connName;

  let selNode = node.querySelector('.conn-combination');
  $(selNode).attr('id', _connName);

  // --- add data into drop-down menu with format ---
  let dataCells = [];
  _fullConnData.forEach(fcd => {
    let mapObj = dataCells.find(cd => cd.text === fcd.map);
    //  --- case: exist ---
    if (mapObj !== undefined) {
      mapObj.children.push({ id: fcd.cell, text: fcd.cell });
      return;
    }
    //  --- case: not-exist ---
    dataCells.push({ id: fcd.map, text: fcd.map, children: [{ id: fcd.cell, text: fcd.cell }] });
  });
  console.log(dataCells);

  function formatState(data, container) {
    opt = $(data.element);
    console.log(opt);
    og = opt.closest('optgroup').attr('label');
    $(data.element).attr('data-conn-cell', og + ', ' + data.text)
    return og + ', ' + data.text;
  };

  $(selNode).select2({
    placeholder: 'Select Connected Cells',
    width: "100%",
    multiple: true,
    templateSelection: formatState,
    data: dataCells,
  }).val(_selOptions).trigger('change');

  // TODO: refactor the mechanism
  $(selNode).on('change', function (e) {
    // let selVal2 = $(this).find(':selected');
    let selId = e.target.id;
    let selVal2 = $(`#${selId}`).find(':selected');
    console.log(selVal2);
    let connections = [];
    for (let opt of selVal2) {
      let c = $(opt).data();
      console.log(c);
      c = c.connCell.split(',');
      connections.push({ "map_name": c[0].trim(), "transition_cell": c[1].trim() });
    };
    console.log(connections);
  });

  $(selNode).on('select2:select', function (e) {
    let selId = e.target.id;
    let selData = $(`#${selId}`).val();
    let validResult = testDupConnCell(selId, selData);

    if (validResult.isValid) {
      alert(`The CELL ${e.params.data.text} is used by other connections!!`);
      $(selNode).val(validResult.data).trigger('change');
    }
  })

  // --- event bindings ---
  let editCellConn = node.querySelector('.edit-cell-conn');
  editCellConn.addEventListener('click', cbEditCellConn.bind(selNode));

  let delCellConn = node.querySelector('.delete-cell-conn');
  delCellConn.addEventListener('click', cbDeleteCellConn.bind(mainCard));

  $('#conn-cell-tool').append(node);
}

function testDupConnCell(_selId, _arrCells) {
  if (!_arrCells.length) { return; }
  // 1. get all cells
  const connCellDecks = document.querySelectorAll('#conn-cell-tool > .conn-card');
  console.log(connCellDecks);

  let connDataObj = [];
  connCellDecks.forEach((conn) => {
    console.log(conn);
    // --- connected cell tag ---
    let name = conn.querySelector('.conn-name').textContent;
    console.log(name)
    if (name === _selId) { return; }

    // --- connected cell values ---
    let selNode = conn.querySelector('select');
    let selVal2 = $(selNode).val();
    console.log(selVal2);
    connDataObj.push(...selVal2);
  });

  console.log(connDataObj);
  let res = connDataObj.filter(cdo => _arrCells.includes(cdo));
  let data = _arrCells.filter(ac => !connDataObj.includes(ac));
  return (res.length) ? { isValid: true, data: data } : { isValid: false, data: data };
}

function testDupConnName(_newConnName) {
  const connCellDecks = document.querySelectorAll('#conn-cell-tool > .conn-card');

  let arrConnName = [];
  connCellDecks.forEach((conn) => {
    console.log(conn);
    // --- connected cell tag ---
    let name = conn.querySelector('.conn-name').textContent;
    // console.log(name);
    arrConnName.push(name);
  });
  // console.log(arrConnName);

  return arrConnName.includes(_newConnName) ? true : false;
}

let pinchZoom_ = false;
function bindTSPinch2Zoom() {
  if (!isTouchDevice()) return;
  fCanvas.on('touch:gesture', function (opt) {
    var e = opt.e;
    if (e.touches && e.touches.length == 2) {
      pinchZoom_ = true;
      // --- avoid origin scaling and rotation ---
      if (originSprite !== undefined) {
        fCanvas.remove(originSprite);
      }

      // var point = new fabric.Point(opt.self.x, opt.self.y);
      if (opt.self.state == "start") {
        // zoomStartScale = fCanvas.getZoom();
        zoomStartScale = fCanvas.getZoom() * canvasScale_;
      }
      // var delta = zoomStartScale * opt.self.scale;
      var delta = zoomStartScale * (opt.self.scale * canvasScale_);
      if (delta > 30) delta = 30;
      if (delta < 1) delta = 1;
      // fCanvas.zoomToPoint(point, delta);
      fCanvas.setZoom(delta * (1 / canvasScale_));

      // --- grid canvas ---
      // if (opt.self.state == "start") {
      //   zoomStartScale = gridCanvas.getZoom();
      // }
      // var delta = zoomStartScale * opt.self.scale;
      // if (delta > 10) delta = 10;
      // if (delta < 1) delta = 1;
      // gridCanvas.zoomToPoint(point, delta);
    }
  });
}

function initMobileResizeEvts() {
  if (!isMobile()) return;
  window.addEventListener('resize', function () {
    refreshPopUps();
    updateToolsPosition();
  });

  function refreshPopUps() {
    updatePopUpPosition('node');
    updatePopUpPosition('cell');
<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
    updatePopUpPosition('zone-node');
=======
    updatePopUpPosition('zonenode');
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js
  }

  function updateToolsPosition() {
    // --- toggle edit properties tools and swap tools position ---
    if (window.innerWidth <= 767 || window.innerHeight <= 767) {
      if (!$('#edit-tools-list').is(":hidden")) {
        $('#edit-tools-card').detach().appendTo('#edit-tools-section');
        $('#edit-tools-list').hide();
        $('#edit-tools-section').show();
      }
    } else {
      if (!$('#edit-tools-section').is(":hidden")) {
        $('#edit-tools-card').detach().appendTo('#edit-tools-list');
        $('#edit-tools-section').hide();
        $('#edit-tools-list').show();
      }
    }
  }
}

<<<<<<< HEAD:server/public/dist/js/pages/far_map_legacy_0316.js
// ======== Function Zones ========
let zoneMetaData = {};
async function initZoneEditProperties() {

  zoneMetaData = await getZoneMetaData();

  // --- create zone type options ---
  let zoneTypeSelect = document.getElementById('zone-type-select');
  for (const [key, value] of Object.entries(zoneMetaData)) {
    let optVal = value.zone_type.default_value || "0";
    optVal = optVal.concat("@", key);
    var typeOption = document.createElement('option');
    typeOption.value = optVal;
    typeOption.text = key.replace(/_/g, ' ');
    zoneTypeSelect.append(typeOption);
  }
  zoneTypeSelect.addEventListener('change', updateZoneType);

  // --- bind events ---
  let createBtn = document.getElementById('create-zone-btn');
  createBtn.addEventListener('click', createZone);
  let zoneNameInput = document.getElementById('zone-name');
  zoneNameInput.addEventListener('input', updateZoneName);
  let zonePrioSelect = document.getElementById('zone-priority-select');
  zonePrioSelect.addEventListener('change', updateZonePriority);

  let switchEditBtns = document.getElementsByClassName('switch-edit-mode');
  for (const btn of switchEditBtns) {
    btn.addEventListener('click', editButtonSwitch.bind(btn.parentElement));
  }
  document.getElementById('open-zone-configs').addEventListener('click', popupZoneConfigModal);
}

async function loadZoneList(_mapName) {
  var zoneDeck = document.getElementById('zones-deck');
  removeAllChildNodes(zoneDeck);
  // resetZoneEditor();
  closeZoneEditor();

  // --- reset zone cache data ---
  jsonZoneArray = [];

  let data = await fetchGetZoneConfig(rmtToken_, _mapName);
  data = await data.json();
  if (typeof data !== 'object') {
    removeAllFabricZones();
    return;
  }

  jsonZoneArray = data;
  console.log(jsonZoneArray)

  if (jsonZoneArray.length === 0) {
    removeAllFabricZones();
  }
  // --- redraw map zones ---
  // removeAllFabricZones();
  // drawFabricZones();

  data.forEach((_zone) => {
    var node = createZoneCardView(_zone);
    zoneDeck.append(node);
  });

  // --- init zone list cards background color ---
  resetSelectedZoneCard();
  // --- init activation switches for each zone ---
  $('.activation-switch').bootstrapSwitch({
    'size': 'mini',
    'onColor': 'success',
    'offColor': 'danger',
    'onSwitchChange': async function (e, state) {
      let uuid = e.target.id;
      let res = await fetchPutZoneActivation(rmtToken_, _mapName, uuid, state);
      if (res.ok) {
        res = await fetchGetZoneConfig(rmtToken_, _mapName);
        res = await res.json();

        let stateText = state ? $(e.target).attr('data-on-text') : $(e.target).attr('data-off-text');
        stateText = stateText.replace(/\s/g, '').toUpperCase();
        let index = _.findIndex(res, { uuid: uuid });
        if (index === -1) {
          $(e.target).bootstrapSwitch('state', !state);
          alert(`${stateText} FAILED!`);
          return;
        }

        if (res[index].activate !== state) {
          $(e.target).bootstrapSwitch('state', res[index].activate);
          alert(`${stateText} FAILED!`);
          return;
        }

        closeZoneEditor();
        document.getElementById(`zone-container-${uuid}`).style.pointerEvents = state ? "none" : "auto";
      }
    }
  });
  const activationSwitches = document.querySelectorAll('.bootstrap-switch');
  activationSwitches.forEach((activationSwitch) => {
    activationSwitch.style.pointerEvents = "auto";
    activationSwitch.style.borderColor = "gray";
  });
}

function updateZoneVertices(_id, _vertices) {
  let rosPoints = tfPolygonPoints2ROS(_vertices);
  updateZoneCacheData('vertices', rosPoints);
  let index = _.findIndex(jsonZoneArray, { uuid: _id });
  if (index !== -1) {
    jsonZoneArray[index].vertices = rosPoints;
  }
}

function updateZoneName() {
  updateZoneCacheData('zone_name', this.value);
}

function updateZonePriority() {
  updateZoneCacheData('priority', this.selectedIndex);
}

function updateZoneType() {
  let zone_type = parseInt(this.value.split('@')[0]);
  let zone_type_name = this.value.split('@')[1];
  updateZoneCacheData('properties', {});
  updateZoneCacheData('zone_type', zone_type);
  updateZoneSettings(zone_type_name);
}

function updateNewZoneCacheData() {
  updateZoneCacheData('uuid', genUuid().replace(/-/g, ''));
  updateZoneCacheData('zone_name', document.getElementById('zone-name').value);
  updateZoneCacheData('priority', document.getElementById('zone-priority-select').selectedIndex);
  updateZoneCacheData('zone_type', document.getElementById('zone-type-select').selectedIndex);
  updateZoneCacheData('activate', false);
  updateZoneCacheData('properties', {});
}

function updateZoneCacheData(key, value) {
  if (value === undefined) return;
  jsonZoneCache_[key] = value;
}

function resetZoneCacheData() {
  jsonZoneCache_ = {};
}

function restoreZonePropertiesCache(diffConfigs) {
  jsonZoneCache_.properties = _.pickBy(jsonZoneCache_.properties, function (v, k) {
    return !diffConfigs.includes(k);
  });
}

let allZoneConfs = {};
function updateZoneSettings(_zoneType = null) {
  allZoneConfs = zoneMetaData[_zoneType] || {};
  allZoneConfs = _.omit(allZoneConfs, ['zone_type']) || {};

  // --- reset zone settings ---
  let configDeck = document.getElementById('edit-config-deck');
  removeAllChildNodes(configDeck);
  document.getElementById('open-zone-configs').disabled = isEmpty(allZoneConfs);
  if (isEmpty(allZoneConfs)) {
    $('#zone-params-card').CardWidget('collapse');
  } else {
    $('#zone-params-card').CardWidget('expand');
  }
}

function createZone() {
  // --- [protection] no zone created ---
  let emptyZoneId;
  jsonZoneArray.forEach(zone => {
    let obj = getFabricZoneObject(zone.uuid);
    if (obj) return;
    alert(`${zone.zone_name} not created on map!`);
    emptyZoneId = zone.uuid;
  });
  if (emptyZoneId) {
    let switchNode = document.querySelector(`#zone-container-${emptyZoneId} .activation-switch`);
    if (switchNode.checked) return;
    document.getElementById(`zone-container-${emptyZoneId}`).click();
    return;
  }

  clearNodePopUp();
  restoreUnsavedFabricZone();
  removeUnsavedFabricZones();
  removeDeleteZoneButton();
  resetSelectedPolygonPoints();
  resetSelectedZoneCard();
  resetZoneEditor();

  // --- update zone cache data ---
  resetZoneCacheData();
  updateNewZoneCacheData();

  openZoneEditor();
}

function resetZoneEditor() {
  document.getElementById('zone-type-select').selectedIndex = 0;
  document.getElementById('zone-name').value = "Zone";
  document.getElementById('zone-priority-select').selectedIndex = 0;

  updateZoneSettings();
}

function editZoneEditor(_zone) {
  // --- [protection] no zone created ---
  let emptyZoneId;
  jsonZoneArray.forEach(zone => {
    if (zone.zone_name === _zone.zone_name) return;
    let obj = getFabricZoneObject(zone.uuid);
    if (obj) return;
    alert(`${zone.zone_name} not created on map!`);
    emptyZoneId = zone.uuid;
  });
  if (emptyZoneId) {
    let switchNode = document.querySelector(`#zone-container-${emptyZoneId} .activation-switch`);
    if (switchNode.checked) return;
    document.getElementById(`zone-container-${emptyZoneId}`).click();
    return;
  }

  clearNodePopUp();
  restoreUnsavedFabricZone();
  removeUnsavedFabricZones();
  resetSelectedPolygonPoints();

  // --- zone list ---
  resetSelectedZoneCard();
  let isDarkMode = getSavedTheme() === 'dark';
  this.style.backgroundColor = isDarkMode ? '#4b545c' : 'rgba(52, 58, 64, 0.5)';

  // --- zone editor ---
  document.getElementById('zone-type-select').selectedIndex = _zone.zone_type || 0;
  document.getElementById('zone-name').value = _zone.zone_name || '';
  document.getElementById('zone-priority-select').selectedIndex = _zone.priority || 0;

  // --- zone settings ---
  let zone_type_name = document.getElementById('zone-type-select').value.split('@')[1];
  updateZoneSettings(zone_type_name);

  let configDeck = document.getElementById('edit-config-deck');
  let configZone = _.find(jsonZoneArray, { uuid: _zone.uuid });
  let zoneProps = {};
  if (configZone !== undefined) {
    zoneProps = _.pickBy(configZone.properties, (value, key) => value !== null);
    for (const [key, value] of Object.entries(zoneProps)) {
      let defs = allZoneConfs[key];
      let node = createZoneConfigRow(key, value, defs);
      configDeck.append(node);
    }
  }

  // --- add delete button to footer ---
  createDeleteZoneButton(_zone);

  // --- update zone cache data ---
  resetZoneCacheData();
  updateZoneCacheData('uuid', _zone.uuid);
  updateZoneCacheData('zone_name', _zone.zone_name);
  updateZoneCacheData('priority', _zone.priority);
  updateZoneCacheData('zone_type', _zone.zone_type);
  updateZoneCacheData('activate', false);
  updateZoneCacheData('properties', zoneProps);
  let zone_obj = getFabricZoneObject(_zone.uuid);
  if (zone_obj) {
    let rosPoints = tfPolygonPoints2ROS(zone_obj.points);
    updateZoneCacheData('vertices', rosPoints);
  }

  openZoneEditor();
}

function resetSelectedZoneCard() {
  let isDarkMode = getSavedTheme() === 'dark';
  const zoneContainers = document.querySelectorAll(".zone-container");
  zoneContainers.forEach((container) => {
    container.style.backgroundColor = isDarkMode ? '#343a40' : 'lightgray';
  });
}

function removeZone(_zone) {
  let uuid = _zone.uuid;
  let name = _zone.zone_name;
  if (confirm(`Are you sure to delete the zone: ${name} ?`)) {
    //  --- remove zone card from the zone list ---
    document.getElementById(`zone-container-${uuid}`).remove();
    // --- remove cache data ---
    let removeZone = _.remove(jsonZoneArray, { zone_name: name });
    if (jsonZoneCache_.hasOwnProperty('zone_name')) {
      if (jsonZoneCache_.zone_name === name) {
        closeZoneEditor();
      }
    }
    // --- remove zone from map ---
    let removeId = removeZone[0].uuid; // suppose only one key-value pair
    removeFabricZoneObjects(removeId);
  }
}

function editButtonSwitch() {
  var inputNode = this.querySelector('.form-control');
  var btnIconNode = this.querySelector('.fas');

  inputNode.readOnly = !inputNode.readOnly;

  // --- mount input validation mechanism ---
  if (!inputNode.readOnly) {
    validateZoneInputEvent(inputNode);
  }

  btnIconNode.classList.toggle("fa-pen");
  btnIconNode.classList.toggle("fa-eye");
}

function boolButtonSwitch() {
  var inputNode = this.querySelector('.form-control');
  var btnNode = this.querySelector('.btn');

  var inputBoolVal = inputNode.value.toLowerCase() == "true";
  inputNode.value = !inputBoolVal;

  var boolString = inputBoolVal.toString();
  boolString = boolString.charAt(0).toUpperCase() + boolString.slice(1);
  btnNode.innerHTML = `Set ${boolString}`;
}

async function popupZoneConfigModal() {
  let data = allZoneConfs;

  if (jsonZoneCache_.hasOwnProperty('properties')) {
    let selConifgs = Object.keys(jsonZoneCache_.properties);
    let configKeys = _.difference(Object.keys(data), selConifgs);
    data = _.pickBy(data, function (v, k) {
      return configKeys.includes(k);
    });
  }
  updateZoneConfigOptions(data);

  $('#zone-config-modal').modal('show');
}

function updateZoneConfigOptions(_configs) {
  var ul = document.getElementById('available-items-list');
  removeAllChildNodes(ul);

  for (const [key, value] of Object.entries(_configs)) {
    var node = createZoneItemView(key, value);
    ul.appendChild(node);
  }

  document.getElementById("available-items-title").textContent = langTemplateObj_.modal.ttl_AddConfig;
}

function enrollConfigToZone(_configName, _defs) {
  let configDeck = document.getElementById('edit-config-deck');
  let configVal = _defs.default_value === 'null' ? null : _defs.default_value || '';
  let node = createZoneConfigRow(_configName, configVal, _defs);
  configDeck.append(node);

  // --- update zone cache data ---
  jsonZoneCache_.properties[_configName] = configVal;

  let selConifgs = Object.keys(jsonZoneCache_.properties);
  let configKeys = _.difference(Object.keys(allZoneConfs), selConifgs);
  let configs = _.pickBy(allZoneConfs, function (v, k) {
    return configKeys.includes(k);
  });
  updateZoneConfigOptions(configs);
}

function removeConfigfromZone(_configName) {
  this.remove();
  delete jsonZoneCache_.properties[_configName];
}

async function btnSaveZone() {
  let diffConfigs = [];
  if (!isEmpty(jsonZoneCache_)) {
    if (!jsonZoneCache_.hasOwnProperty('vertices')) {
      notificationMsg(3, `${jsonZoneCache_.zone_name} not created on map!`);
      return;
    }

    if (jsonZoneCache_.zone_type === 0) {
      alert('Please select zone type!')
      return;
    }

    if (jsonZoneCache_.hasOwnProperty('properties')) {
      let selConifgs = Object.keys(jsonZoneCache_.properties);
      selConifgs.map(function (conf) {
        var confVal = document.getElementById(conf).value || '';
        confVal = isEmptyString(confVal) ? null : confVal;
        let type = allZoneConfs[conf].valid_value.data_type;
        if (confVal !== null && (type === 'double' || type === 'int')) {
          confVal = Number(confVal);
        } else if (confVal !== null && type === 'bool') {
          confVal = (confVal.toLowerCase() === 'true');
        }
        jsonZoneCache_.properties[conf] = confVal;
      });
      diffConfigs = _.difference(Object.keys(allZoneConfs), selConifgs);
      diffConfigs.map(function (conf) {
        jsonZoneCache_.properties[conf] = null;
      });
    }

    // --- replace same id data ---
    let index = _.findIndex(jsonZoneArray, { uuid: jsonZoneCache_.uuid });
    if (index !== -1) {
      jsonZoneArray.splice(index, 1, jsonZoneCache_);
    } else {
      jsonZoneArray.push(jsonZoneCache_);
    }
  }

  // --- check for overlapping zones(zone level) ---
  let isOverlapping = false;
  jsonZoneArray.forEach(zone => {
    let id = zone.uuid;
    let pol = zone.vertices;
    let zoneIds = getPolygonIntersectZones(id, pol);
    let names = getSameZones(zoneIds, zone.zone_type, zone.priority);
    if (names.length > 0) {
      alert(`${zone.zone_name} can NOT overlap with ${names.join()}!`);
      isOverlapping = true;
    }
  });
  if (isOverlapping) {
    notificationMsg(3, 'Fail to save zones!');
    restoreZonePropertiesCache(diffConfigs);
    return;
  }

  // console.log(jsonZoneArray);
  // console.log(JSON.stringify(jsonZoneArray));
  try {
    let res = await fetchPutZoneConfig(rmtToken_, currSelectedMap_, jsonZoneArray);
    if (res.ok) {
      notificationMsg(0, 'Map zones saved successfully!');
    } else {
      notificationMsg(3, 'Fail to save zones!');
    }
  } catch (err) {
    console.error(err);
    notificationMsg(3, 'Fail to save zones!');
  }

  restoreZonePropertiesCache(diffConfigs);
  // --- reload zone list ---
  loadZoneList(currSelectedMap_);
=======
function coordIsInsidePolygon(point, vs) {
  var x = point[0], y = point[1];

  var inside = false;
  for (var i = 0, j = vs.length - 1; i < vs.length; j = i++) {
    var xi = vs[i][0], yi = vs[i][1];
    var xj = vs[j][0], yj = vs[j][1];

    var intersect = ((yi > y) != (yj > y))
      && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }

  return inside;
>>>>>>> feature/multi-func-zone:server/public/dist/js/pages/far_map.js
}