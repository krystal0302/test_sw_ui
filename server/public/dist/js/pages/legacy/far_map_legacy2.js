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

    // --- if the list is empty ---
    if (this.size === 0) {
      this.head = newNode;
      this.tail = newNode;
      this.size++;
      this.cache[_key] = newNode;
      return this;
    }

    if (this.size === this.maxSize) {
      // --- remove from cache ---
      delete this.cache[this.tail.key]

      // --- set new tail ---
      this.tail = this.tail.next;
      this.tail.prev = null;
      this.size--;
    }

    console.log(this.head);
    console.log(newNode);
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
      return undefined
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
      return undefined
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
// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  init();
});

let cellTypes_;
let cellDetectionTypes_;

async function init() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'map', 'map.html');

  initRenameMapNameForm();

  // ------ load cell types ------
  var settingsData = await restGetSettings();
  console.log(settingsData);

  cellDetectionTypes_ = settingsData['cell.detectionTypes'];
  console.log(cellDetectionTypes_);
  // cellTypes_ = settingsData['cell.types'];
  // console.log(cellTypes_);

  var cellTypesData = await restGetCellTypes();
  // cellTypesData = JSON.parse(cellTypesData);
  cellTypes_ = cellTypesData;
  console.log(cellTypes_);

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

  // -- Properties View --
  toggleNavGraph(true);
  toggleCells(true);

  // ------ calculate canvas DOM width & height ------
  var viewHeight = document.getElementsByClassName('vis-network')[0].getBoundingClientRect().height;
  var viewWidth = document.getElementsByClassName('vis-network')[0].getBoundingClientRect().width;
  mapViewSize = Math.min(viewWidth, viewHeight);
  // console.log(mapViewSize);
  $('#cont').css('height', `${mapViewSize}px`);
  $('#grid').css({
    'width': `${mapViewSize}px`,
    'height': `${mapViewSize}px`
  });
  $('#c').css({
    'width': `${mapViewSize}px`,
    'height': `${mapViewSize}px`
  });

  // $('#cont > .canvas-container > canvas').prop('width', `${mapViewSize}px`);
  // .css({
  //   'width': `${mapViewSize}px !important`,
  //   'height': `${mapViewSize}px !important`,
  //   'left': `calc(50% ${mapViewSize/2}px) !important`
  // });
}

function initRenameMapNameForm() {
  $.validator.addMethod("chkDupMapName", function (value, element) {
    let data = restMapAliasExistence(value);
    let mapAliasExists = data.responseJSON;
    if (mapAliasExists === undefined) {
      return false;
    }
    return !mapAliasExists;
  });

  $.validator.addMethod("chkUnchangedMapName", function (value, element) {
    var mapName = $('#map-select').val();
    let new_mapname = value;
    if (new_mapname !== mapName) {
      return true;
    }
    return false;
  });

  $('#rename-map-form').validate({
    rules: {
      newMapName: {
        required: true,
        chkUnchangedMapName: true,
        chkDupMapName: true
      }
    },
    messages: {
      newMapName: {
        required: 'Please enter map name',
        chkUnchangedMapName: 'Map name not changed',
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

async function renameMapAlias(e) {
  e.preventDefault();
  if (!$('#rename-map-form').valid()) { return; }
  $('#rename-map-modal').modal('hide');
  var mapFileName = $('#map-select').val();
  var newMapName = $('#new-mapname').val();
  // console.log(mapFileName)
  // console.log(newMapName)
  var data = await restPostMapAlias(mapFileName, newMapName);
  if (data.status_code === 200) {
    notificationMsg(1, data.message);
    $(`#map-select option[value="${mapFileName}"]`).text(`${newMapName}`);
  } else {
    notificationMsg(3, data.message);
  }
}

// =======================
//     Animation Ticks 
// =======================
let liveOn;
function toggleCellStatus() {
  console.log('live cell status');
  if (!liveOn) {
    liveOn = setInterval(updateVisCellStatus(wms_obj_.msg), 400);
    $('#live-btn-text').text('Live Status Off');
  } else {
    clearInterval(liveOn)
    liveOn = null;
    updateVisCellStatus();
    $('#live-btn-text').text('Live Status On');
  }
}

// --- update WMS status ---
function updateVisCellStatus(_msgs) {
  console.log(_msgs);
  var cellNodes = [];

  // --- LIVE-OFF cell status ---
  if (_msgs === undefined) {
    _msgs = visNetwork_.body.nodes;
    // console.log(_msgs);
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
    return;
  }

  // --- LIVE-ON cell status ---
  for (i in _msgs) {
    var wmsNode = {};
    wmsNode.id = _msgs[i].text; // cell-id
    // console.log(_msgs[i].text);

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

let gridCanvas = new fabric.Canvas('grid', {
  selection: false
});

// ======================
//       Events
// ======================
let dragScale_ = 1.0;
let mapViewSize = 720;
function drawFabricMap() {
  var mapDataURL = `data:image/png;base64,${gMapData_.data}`;

  var img = new Image();
  img.onload = function () {
    var finalDim = Math.max(img.width, img.height);
    dragScale_ = Math.floor(finalDim / mapViewSize);
    console.log(dragScale_);

    var leftest = (finalDim - img.width) / 2;
    console.log(finalDim);
    console.log(img.width);
    console.log(leftest);
    // var f_img = new fabric.Image(img, { left: leftest, top: 0 });
    var f_img = new fabric.Image(img);
    currentImage = f_img;

    // fCanvas.setWidth(f_img.width);
    // fCanvas.setHeight(f_img.height);

    fCanvas.setWidth(finalDim);
    fCanvas.setHeight(finalDim);

    fCanvas.setBackgroundImage(f_img).renderAll();
  };
  img.src = mapDataURL;
}

function drawFabricBoard() {
  // ------ build grid ------
  var canvasWidth = mapViewSize; // [CFG] canvas DOM width
  var canvasHeight = mapViewSize; // [CFG] canvas DOM height 
  gridCanvas.clear();

  // --- transform formula ---
  var meterPerGrid = Number($('#grid-span').val());
  var grid = meterPerGrid * (canvasHeight / gMapData_.h) / gMapMeta_.resolution;

  for (var i = 0; i < (canvasWidth / grid); i++) {
    gridCanvas.add(new fabric.Line([i * grid, 0, i * grid, canvasHeight], {
      type: 'line',
      stroke: '#ccc',
      selectable: false
    }));
    gridCanvas.add(new fabric.Line([0, i * grid, canvasWidth, i * grid], {
      type: 'line',
      stroke: '#ccc',
      selectable: false
    }))
  };
}

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

  fabric.Image.fromURL('dist/img/ucs.png', function (myImg) {

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
}

let mapOrigin_ = {
  left: 0,
  top: 0,
  width: 128,
  height: 128
};

async function btnSetOrigin() {
  isOriginSet_ = true;
  fCanvas.getObjects().forEach(function (o) {
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

      visUpdateMapOrigin()
      // bNavGraph_ = false;
      // visDrawGraph();

      // bCells_ = false;
      // visDrawCells();
      // console.log('--- update drawings ---');
    }
  });

  fCanvas.remove(originSprite);

  // --- DOM render logic ---
  document.getElementById("get-origin").style.display = "inline";
  document.getElementById("set-origin").style.display = "none";
  document.getElementById("cancel-origin").style.display = "none";

  // --- fetch data from server ---
  var data = await restGetMapMetaInfo(currSelectedMap_);
  // console.log(data);

  // --- data process (origin update) ---
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

  // --- save updated data back to server ---
  var mapName = currSelectedMap_;
  await restPostMapMetaInfo2(mapName, updatedData);
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
}

async function btnSaveMapImage() {
  console.log('---- btn save map image ----');
  if (!isOriginSet_) {
    console.log();
    alert('Orgin is NOT Set Yet!');
    return;
  }

  fCanvasfitToViewport()
  // var map_data = fCanvas.toDataURL('png');
  var map_data = fCanvas.toDataURL({
    format: 'png',
    top: 0,
    left: 0,
    width: gMapData_.w,
    height: gMapData_.h
  });

  await restPostMapImage(currSelectedMap_, map_data);
  mapImageChanged = false;
  notificationMsg(0, 'Map saved successfully!');
  // toast('Map saved successfully!')
}

function btnCropMap() {
  console.log(`--- crop map ---`);
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

fCanvas.on('touch:gesture', function (opt) {
  var e = opt.e;
  if (e.touches && e.touches.length == 2) {
    var point = new fabric.Point(opt.self.x, opt.self.y);
    if (opt.self.state == "start") {
      zoomStartScale = fCanvas.getZoom();
    }
    var delta = zoomStartScale * opt.self.scale;
    if (delta > 30) delta = 30;
    if (delta < 1) delta = 1;
    fCanvas.zoomToPoint(point, delta);

    // --- grid canvas ---
    if (opt.self.state == "start") {
      zoomStartScale = gridCanvas.getZoom();
    }
    var delta = zoomStartScale * opt.self.scale;
    if (delta > 10) delta = 10;
    if (delta < 1) delta = 1;
    gridCanvas.zoomToPoint(point, delta);
  }
});

fCanvas.on('mouse:wheel', function (opt) {
  var delta = opt.e.deltaY;
  var zoom = fCanvas.getZoom();
  zoom *= 0.999 ** delta;
  if (zoom > 30) zoom = 30;
  // if (zoom < 0.01) zoom = 0.01;
  if (zoom < 1) zoom = 1;
  fCanvas.zoomToPoint({
    x: opt.e.offsetX,
    y: opt.e.offsetY
  }, zoom);
  fCanvas.setZoom(zoom);
  opt.e.preventDefault();
  opt.e.stopPropagation();

  // --- grid canvas ---
  zoom = gridCanvas.getZoom();
  zoom *= 0.999 ** delta;
  if (zoom > 10) zoom = 10;
  if (zoom < 1) zoom = 1;
  gridCanvas.zoomToPoint({
    x: opt.e.offsetX,
    y: opt.e.offsetY
  }, zoom);
  gridCanvas.setZoom(zoom);
  opt.e.preventDefault();
  opt.e.stopPropagation();
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
      vpt[4] += (clientX - this.lastPosX) * dragScale_;
      vpt[5] += (clientY - this.lastPosY) * dragScale_;
      this.requestRenderAll();
      this.lastPosX = clientX;
      this.lastPosY = clientY;
    } else {
      vpt[4] += (e.clientX - this.lastPosX) * dragScale_;
      vpt[5] += (e.clientY - this.lastPosY) * dragScale_;
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

function fitToViewport() {
  if (selectedEditType === 'rdMapImg') {
    fCanvasfitToViewport();
  } else {
    visFitToViewport();
  }
}

function fCanvasfitToViewport() {
  // --- canvas.setViewportTransform([zoom, 0, 0, zoom, panX, panY]) ---
  fCanvas.setViewportTransform([1, 0, 0, 1, 0, 0]);
  gridCanvas.setViewportTransform([1, 0, 0, 1, 0, 0]);
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
        vpt[4] += (clientX - this.lastPosX) * dragScale_;
        vpt[5] += (clientY - this.lastPosY) * dragScale_;
        this.requestRenderAll();
        this.lastPosX = clientX;
        this.lastPosY = clientY;
      } else {
        vpt[4] += (e.clientX - this.lastPosX) * dragScale_;
        vpt[5] += (e.clientY - this.lastPosY) * dragScale_;
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
    if (!inst.isEnable()) {
      return;
    }

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
  if (canvas === null) {
    console.log('canvas is null');
    return;
  }
  var width = canvas.width;
  var height = canvas.height;

  // --- clean before drawing ---
  ctx.clearRect(0, 0, width, height);

  if (canvas === null) {
    return;
  }

  ctx.drawImage(canvas, 0, 0, width, height);
  // ctx.globalAlpha = 1.0;
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

visNetwork_.on("oncontext", function (params) {
  params.event.preventDefault();

  // --- 1. identify the selected node ---
  var nodeID = visNetwork_.getNodeAt(params.pointer.DOM);
  let selNode = visNetwork_.body.data.nodes.get(nodeID);

  // --- 2. identify the selected edit properties ---
  // let selectedEditType = $('.custom-radio input[type=radio]:checked').val();


  // --- 3. switch-on edit-mode & toolbar preparation ---
  // --- case 3-1 NavGraph Layer ---
  let manipulationMode = visNetwork_.manipulation.inMode;
  console.log("check manipulation mode when right click node on map: " + manipulationMode);

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
});

// --- dummy click without selecting anything ---
visNetwork_.on("click", function (props) {
  let manipulationMode = visNetwork_.manipulation.inMode;
  console.log("check manipulation mode when click on map: " + manipulationMode);

  clearNodePopUp();

  if (manipulationMode === 'editEdge') return;

  hideDeleteButtons();

  if (selectedEditType === 'rdCell') {
    if (manipulationMode === 'addNode') return;
    hideCancelEditButton();
    hideManipulationModeText();
  } else if (selectedEditType === 'rdRoute') {
    if (manipulationMode === 'addNode' || manipulationMode === 'addRoute') return;
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
  //     if (manipulationMode === 'addNode' || manipulationMode === 'addRoute') return;
  //     hideManipulationModeText();
  //   }

  //   // As long as the selected node isn't a nav node
  //   if (selNodeGroup !== 'navnode') {
  //     clearNodePopUp();
  //   }
  // }
});

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

  if (bNavGraph_) return;
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

function visDrawCells(_network, _cells) {
  // -- [protection] confirm the data is updated --
  if (bCells_) return;
  bCells_ = true;

  // var ang = 0;

  // --- read the data from WMS json file ---
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

      var r = area[i].cell_coordinate[2];
      ang = -cvtRad2Deg(r).toFixed(2);
      // var svgHtml =
      //   `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 40">
      //     <polygon points="6,4 26,4 40,20 26,36 6,36 15,20" transform="rotate(${ang} 20 20)" fill="#044B94" fill-opacity="0.4"/>
      //    </svg>`;
      // var svgUrl = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svgHtml);

      // console.log(area[i]);
      var tooltip = `<div class="farobot-map-tooltip"
                          style="color: ${getColor(cellDetectionTypes_, area[i].type, 'tooltipColor')}">
                      ${cell_id}@(${x.toFixed(2)}, ${y.toFixed(2)})
                     </div>`;
      var wmsNode = {
        title: tooltip,
        id: cell_id,
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
        highlight: getColor(cellDetectionTypes_, area[i].type, 'focusColor'),
        background: getColor(cellDetectionTypes_, area[i].type, 'bgColor'),
        border: getColor(cellDetectionTypes_, area[i].type, 'borderColor')
      };
      // wmsNode.cellType = area[i].type;
      // var typeTarget = cellTypeAdaptor.toUiCellType(cellTypes_, { 'detectionType': area[i].type, 'width': area[i].width, 'length': area[i].length });
      // wmsNode.cellType = (typeTarget === undefined) ? (area[i].type + area[i].width + area[i].length) : typeTarget.name;
      var cellType = getCellType(area[i], cellTypes_);
      wmsNode.cellType = cellType;
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
      _network.body.data.nodes.update(wmsNode);
    }
  }
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
  var typeTarget = cellTypeAdaptor.toUiCellType(_cellTypes, { 'detectionType': _cellObj.type, 'width': _cellObj.width, 'length': _cellObj.length });
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

  if (cvs === null) {
    return;
  }
  var ctx = cvs.getContext('2d');
  ctx.drawImage(gMapImg_, 0, 0);
}

async function loadBundledMapDataAsync() {
  // --- generate map alias list ---
  $('#map-select').find('option').remove();
  var data = await restGetAllMapData();
  var objArray = JSON.parse(data);
  for (let obj of objArray) {
    $('#map-select').append(`<option value='${obj.name}'>${obj.alias_name}</option>`);
  }

  var e = document.getElementById('map-select');
  currSelectedMap_ = e.options[e.selectedIndex].value;
  // console.log(currSelectedMap_);

  // --- load map desscription and data ---
  gMapMeta_ = { w: 1, h: 1 };
  updateMapImageAsync(currSelectedMap_);
  updateMapMetaAsync(currSelectedMap_);
}

async function reloadMapImage() {
  var e = document.getElementById('map-select');
  currSelectedMap_ = e.options[e.selectedIndex].value;

  // --- get the map meta-data ---
  var e = document.getElementById('map-select');
  var selMap = e.options[e.selectedIndex].value;
  gMapData_ = await restGetMapImg(selMap);
  await globalMapImageBackup();
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
    var cellData = await restGetMapCells(gMapMeta_.cell);
    // console.log(cellData);

    // --- global caching ---
    jsonCellCache_ = JSON.parse(cellData);

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
    var graphData = await restGetMapGraph(gMapMeta_.nav_graph);

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
  gMapData_ = await restGetMapImg(_mapName);
  await globalMapImageBackup();
  drawFabricMap();

  gMapMeta_.w = gMapData_.w;
  gMapMeta_.h = gMapData_.h;

  visNetwork_.moveTo({
    position: {
      x: gMapData_.w / 2,
      y: gMapData_.h / 2
    },
    scale: (mapViewSize / gMapData_.h) * 3.5
  });

  console.log(`update map flow done`);
}


let currSelectedMap_;
function updateSelectedMap() {
  var e = document.getElementById('map-select');
  currSelectedMap_ = e.options[e.selectedIndex].value;
  // console.log(currSelectedMap_);

  // --- dataset reset ---
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  // --- load map desscription and data ---
  gMapMeta_ = { w: 1, h: 1 };
  updateMapImageAsync(currSelectedMap_);
  updateMapMetaAsync(currSelectedMap_);
}

function globalMapImageBackup() {
  var mapDataURL = `data:image/png;base64,${gMapData_.data}`;
  gMapImg_ = new Image();
  gMapImg_.onload = function () {
    loadMapImage(gMapData_);
  };
  gMapImg_.src = mapDataURL;
}

function visUpdateMapOrigin() {
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
  orgNode.image = './dist/img/ucs.png';
  orgNode.size = 50;
  orgNode.color = 'red';
  orgNode.label = 'Origin';
  orgNode.group = 'metadata';
  visNetwork_.body.data.nodes.update(orgNode);
}


// =====================
//     DOM Callbacks 
// =====================
function btnSaveChanges() {
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
  for (n in nodes) {
    var node = nodes[n];

    nodes[n]['label'] = id_label_table[nodes[n].id];
    nodes[n]['connections'] = id_connections[nodes[n].id];

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
      currJsonGraph.edges.push(edge);
    }
  }

  // console.log(JSON.stringify(nodes));
  var strGraph = cvtJson2Dot(nodes);
  // console.log(strGraph);

  // --- POST graph data to server side ---
  await restPostMapGraph(currSelectedMap_, strGraph);

  notificationMsg(0, 'NavGraph saved successfully!');
  // toast('NavGraph saved successfully!')
  // ------ [END] save the result to file [END] ------

  // --- refresh all the edges ---
  visNetwork_.body.data.edges.clear();

  // --- flush cache ---
  jsonNavGraphCache_ = {};
  // CRITICAL! Update graph contents
  try {
    // --- load graph data from back-end ---
    var graphData = await restGetMapGraph(gMapMeta_.nav_graph);

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
}

// var chkDataType = false;


async function btnSaveCells() {

  // --- update the cell filename in map-meta yaml ---
  console.log(currSelectedMap_);
  var cellFilename = currSelectedMap_;

  // ---[protection] update cell size. may be remove in near future ---
  updateCellsType(jsonCellCache_, cellTypes_);

  for (areaKey in jsonCellCache_) {
    for (cellKey in jsonCellCache_[areaKey]) {
      // remove json cache cell uuid before saving map json file
      if (jsonCellCache_[areaKey][cellKey].hasOwnProperty('cell_uuid')) {
        delete jsonCellCache_[areaKey][cellKey]['cell_uuid'];
      }
      // // replace old cell load before saving map json file
      // if (jsonCellCache_[areaKey][cellKey].hasOwnProperty('load')) {
      //   jsonCellCache_[areaKey][cellKey]['load'] = jsonCellCache_[areaKey][cellKey]['load'].replace('rack', 'occupied');
      // }
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
  console.log(content);

  await restPostMapCells(cellFilename, content);
  notificationMsg(0, 'Storage cells saved successfully!');
  // toast('Storage cells saved successfully!')

  // --- refresh the contents ---
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  // --- flush graph cache ---
  jsonNavGraphCache_ = {};
  try {
    // --- load graph data from back-end ---
    var graphData = await restGetMapGraph(gMapMeta_.nav_graph);

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

  // --- flush cell cache ---
  jsonCellCache_ = {};
  try {
    // --- load cell data from back-end ---
    var cellData = await restGetMapCells(gMapMeta_.cell);
    // console.log(cellData);

    // --- global caching ---
    jsonCellCache_ = JSON.parse(cellData);
    // console.log(jsonCellCache_);

    // --- rendering on UI ---
    bCells_ = false;
    visDrawCells(visNetwork_, jsonCellCache_);
  } catch (err) {
    console.error(err);
  }

  setVisInteractiveMode(3);

  // --- flush cell Difference Cache ---
  cellDiffCache_.flush();
}

function btnUndoRoute() {
  // console.log(jsonNavGraphCache_);
  console.log(graphDiffCache_);
  var myGraphCache = graphDiffCache_.popHead();
  console.log(myGraphCache);
  if (myGraphCache === undefined) {
    return;
  }
  myGraphCache = myGraphCache.value;

  // case1: action: add. UNDO (add -> delete) ---
  if (myGraphCache.action === 'add') {
    // --- update visCache ---
    // console.log(network2_.body.data.nodes);
    visNetwork_.body.data.nodes.remove(myGraphCache.visID);
    // --- update rosCache ---
    jsonNavGraphCache_.nodes = jsonNavGraphCache_.nodes.filter(ca => ca.id !== myGraphCache.visID);
    // console.log(jsonNavGraphCache_);
    return
  }

  // case2: action -> edit 
  if (myGraphCache.action === 'edit') {
    // --- update visCache ---
    console.log(myGraphCache);
    let myNode = { id: myGraphCache.visID, x: myGraphCache.visX, y: myGraphCache.visY };
    // console.log(myNode);
    visNetwork_.body.data.nodes.update(myNode);
    // --- update rosCache ---
    let nodeObj = jsonNavGraphCache_.nodes.find(ca => ca.id === myGraphCache.rosID);
    nodeObj.x = myGraphCache.rosX;
    nodeObj.y = myGraphCache.rosY;
    return
  }

  // case3: action: delete. UNDO (delete -> add) ---
  if (myGraphCache.action === 'delete') {
    // --- update visCache ---
    visNetwork_.body.data.nodes.update(myGraphCache.visNavGraphNode);
    // --- update rosCache ---
    jsonNavGraphCache_.nodes.push(myGraphCache.rosNavGraphNode);
    return
  }

}

function btnUndoCell() {
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
    return
  }

  // case 2: action edit 
  if (myCellCache.action === 'edit') {
    // --- update visCache ---
    let myNode = { id: myCellCache.visID, cellUUID: myCellCache.rosID, x: myCellCache.visX, y: myCellCache.visY };
    // console.log(myNode);
    visNetwork_.body.data.nodes.update(myNode);
    // --- update rosCache ---
    console.log(jsonCellCache_[myCellCache.area]);
    let areaObj = jsonCellCache_[myCellCache.area].find(ca => ca.cell_uuid === myCellCache.rosID);
    console.log(areaObj);
    areaObj.cell_coordinate = [myCellCache.rosX, myCellCache.rosY];
    return
  }

  // case 3: action delete. UNDO (delete -> add) ---
  if (myCellCache.action === 'delete') {
    // --- update visCache ---
    // console.log(myCellCache.rosCellNode);
    visNetwork_.body.data.nodes.update(myCellCache.visCellNode);
    // --- update rosCache ---
    jsonCellCache_[myCellCache.area].push(myCellCache.rosCellNode);
    return
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

  hideNavAgentOptions();
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

function toggleOrigin(_status = true) {
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
function fadeInEditToolbar(_target) {
  // --- toggle off all ---
  var bar;
  bar = document.getElementById('map-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('route-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('cell-tool');
  bar.setAttribute("style", "display:none;");
  bar = document.getElementById('config-tool');
  bar.setAttribute("style", "display:none;");

  // --- toggle edit properties tools list
  $('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');

  // -- toggle target toolbar ---
  if (_target === null || _target === undefined)
    return;

  bar = document.getElementById(_target);
  bar.setAttribute("style", "display:block;");

  // --- fulfill available agents in fleets --- 
  var navAgents = [];
  console.log(fleet_fleet_obj_);
  for (key in fleet_fleet_obj_) {
    navAgents.push(...fleet_fleet_obj_[key].robots);
  }
  navAgents = navAgents.map(na => na.robot_id);
  console.log(navAgents);

  var toolbarId = 'nav-agent-select';

  var navSel = document.getElementById(toolbarId);
  $(navSel).empty();
  navAgents.forEach(na => {
    var opt = document.createElement("option");
    opt.text = na;
    opt.value = na;
    navSel.options.add(opt);
  });
}

function navRouteAnchor() {
  console.log('nav route');
  console.log(fleet_fleet_obj_)
  var selAgent = document.getElementById("nav-agent-select").value;
  var anchorLoc = undefined;
  for (key in fleet_fleet_obj_) {
    var robots = fleet_fleet_obj_[key].robots;
    var agentObj = robots.find(r => r.robot_id === selAgent);
    // console.log(agentObj);
    if (agentObj === undefined) continue;
    // console.log(agentObj.location);
    anchorLoc = agentObj.location;
  }

  if (anchorLoc !== undefined) {
    $("#node-x").val(anchorLoc.x);
    $("#node-y").val(anchorLoc.y);
  }
}

function navCellAnchor() {
  // console.log(fleet_fleet_obj_)
  var selAgent = document.getElementById("nav-agent-select").value;
  var anchorLoc = undefined;
  for (key in fleet_fleet_obj_) {
    var robots = fleet_fleet_obj_[key].robots;
    var agentObj = robots.find(r => r.robot_id === selAgent);
    // console.log(agentObj);
    if (agentObj === undefined) continue;
    // console.log(agentObj.location);
    anchorLoc = agentObj.location;
  }

  if (anchorLoc !== undefined) {
    $("#cell-x").val(anchorLoc.x);
    $("#cell-y").val(anchorLoc.y);
  }
}

// type 0: view only
// type 1: map metadata
// type 2: route graph 
// type 3: storage cell
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
    // options2_.manipulation.editEdge = editNavEdgeData;

    visOptions_.edges.background.color = "white";
    visOptions_.edges.color = "black";
    visNetwork_.setOptions(visOptions_);
    visNetwork_.on("dragStart", evtDragStartRouteNode);
    visNetwork_.on("dragEnd", evtDragEndRouteNode);

    for (i in visNetwork_.body.nodes) {
      if (visNetwork_.body.nodes[i].options.group === 'wms') {
        visNetwork_.body.nodes[i].options.fixed.x = true;
        visNetwork_.body.nodes[i].options.fixed.y = true;
        visNetwork_.body.nodes[i].options.color.background = 'rgba(128,128,128,0.8)';
        visNetwork_.body.nodes[i].options.color.border = 'rgba(220,220,220,1.0)';
        // console.log('wms node opt');
        console.log(visNetwork_.body.nodes[i].options);

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
    visNetwork_.on("dragStart", evtDragStartCellNode);
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
  document.getElementById("node-createButton").value = "Create"; // 

  var el = document.getElementById('node-label');
  var elClone = el.cloneNode(true);
  el.parentNode.replaceChild(elClone, el);
  document.getElementById('node-label').addEventListener("keyup", function () {
    // console.log(this.value);
    if (graphNodeNames_.includes(this.value)) {
      alert('DUPLICATED LABEL!');
    }
  });

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
  showNavAgentOptions();
}

function btnEditRouteNode(data, callback) {
  // filling in the popup DOM elements
  document.getElementById("node-operation").innerText = "Edit Vertex";
  document.getElementById("node-createButton").value = "OK"; // "Save"

  // console.log(data.label);
  graphNodeNames_ = graphNodeNames_.filter(gnn => gnn !== data.label);
  // console.log(graphNodeNames_);
  var el = document.getElementById('node-label');
  var elClone = el.cloneNode(true);
  el.parentNode.replaceChild(elClone, el);
  document.getElementById('node-label').addEventListener("keyup", function () {
    // console.log(this.value);
    // --- update the node label ---
    if (graphNodeNames_.includes(this.value)) {
      alert('DUPLICATED LABEL!');
    }
  });

  document.getElementById("node-label").value = data.label;
  // --- edit selected nav-graph node case ---
  var navNode = jsonNavGraphCache_.nodes.find(ngc => ngc.label === data.label);
  console.log(navNode);

  document.getElementById("node-x").value = navNode.x;
  document.getElementById("node-y").value = navNode.y;

  document.getElementById("node-createButton").onclick = btnSaveRouteNode.bind(this, data, callback, 'edit');
  document.getElementById("node-cancelButton").onclick = cancelNodeEdit.bind(this, callback);

  document.getElementById("node-popUp").style.display = "block";
  showNavAgentOptions();
}

function btnDeleteRouteNode(data, callback) {
  var deletedNode = visNetwork_.body.data.nodes.get(data.nodes[0]);
  console.log(deletedNode);

  // --- cache the difference ---
  var rosNavGraphNode = jsonNavGraphCache_.nodes.find(aa => aa.id === deletedNode.id);
  console.log(rosNavGraphNode);
  var navGraphDiff = { visNavGraphNode: deletedNode, rosNavGraphNode: rosNavGraphNode, action: 'delete' };
  graphDiffCache_.put(genUuid(), navGraphDiff);
  console.log(graphDiffCache_);

  // --- [rosCache] remove the deleted nav-graph node ---
  // delete rosNavGraphNode;
  jsonNavGraphCache_.nodes = jsonNavGraphCache_.nodes.filter(aa => aa.id !== deletedNode.id);
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
  // -- [protection] No duplicated labels --
  var nodeLabel = document.getElementById("node-label").value;
  if (graphNodeNames_.includes(nodeLabel)) {
    alert('DUPLICATED LABEL!');
    return;
  }
  console.log(jsonNavGraphCache_);
  // --- update the graph node ---
  graphNodeNames_.push(data.label);

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
    var graphDiff = { visID: data.id, visX: Number(visPos.x), visY: Number(visPos.y), rosID: data.id, rosX: nodeX, rosY: nodeY, action: 'add' };
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

    var graphDiff = { visID: data.id, visX: Number(visX), visY: Number(visY), rosID: data.id, rosX: rosX, rosY: rosY, action: 'edit' };
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

function evtDragStartRouteNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) return;

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

    var cellDiff = { visID: dragNode.id, visX: Number(visX), visY: Number(visY), rosID: selNode.id, rosX: rosX, rosY: rosY, action: 'edit' };
    graphDiffCache_.put(genUuid(), cellDiff);
    // console.log(cellDiffCache_);
  }
}

function evtDragEndRouteNode(params) {
  console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) return;

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


// =============================
//     Cells Event Callbacks    
// =============================
let cellDiffCache_ = new FarCache();

// --- Storage Cell Manipulation ---
function btnAddCellNode(data, callback) {
  // filling in the popup DOM elements
  document.getElementById("cell-operation").innerText = "Add Cell";
  document.getElementById("cell-createButton").value = "Create";

  // --- update cellsize type ---
  var cellSizeSel = document.getElementById("cell-type");
  $(cellSizeSel).empty();

  cellTypes_.forEach(ct => {
    var opt = document.createElement("option");
    opt.text = ct.name;
    opt.value = ct.name;
    cellSizeSel.options.add(opt);
  });
  cellSizeSel.selectedIndex = 0; // default value

  // --- default data to create a new cell ---
  data.area = 'default area';
  data.cellUUID = genUuid().replace(/-/g, '');
  data.cellType = cellSizeSel.value;
  // console.log(data.cellType);
  data.cellDirection = 'forward';
  data.cellLoad = 'empty';
  data.labelFullName = 'new';

  // --- x, y should be assign from mouse event ---
  // --- read from mouse position --- 
  rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });

  document.getElementById("cell-x").value = rosPos.x;
  document.getElementById("cell-y").value = rosPos.y;

  // data.r = 0.0; // default orientation: 0.0 in radian
  // var orient = cvtRad2Deg(data.r); // in degree on UI. 
  // orient = -orient; // transform orientation from ROS to UI
  // rosPos.r = orient;
  // document.getElementById("cell-r").value = rosPos.r.toFixed(2);

  document.getElementById("cell-label").value = data.labelFullName;
  document.getElementById("cell-area").value = data.area;

  document.getElementById("cell-type").value = data.cellType;
  document.getElementById("cell-direction").value = data.cellDirection;
  document.getElementById("cell-load").value = data.cellLoad;

  // --- save the edited properties ---
  document.getElementById("cell-createButton").onclick = btnSaveCellNodeData.bind(this, data, callback, 'add');
  document.getElementById("cell-cancelButton").onclick = clearNodePopUp.bind(this, callback);
  document.getElementById("cell-popUp").style.display = "block";
  showNavAgentOptions();
}

function btnDeleteCellNode(data, callback) {
  var deletedNode = visNetwork_.body.data.nodes.get(data.nodes[0]);
  console.log(deletedNode);

  // --- cache the difference ---
  var rosCellNode = jsonCellCache_[deletedNode.area].find(aa => aa.cell_uuid === deletedNode.cellUUID);
  console.log(rosCellNode);
  var cellDiff = { visCellNode: deletedNode, rosCellNode: rosCellNode, action: 'delete', area: deletedNode.area };
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
  console.log(data);
  // --- filling in the popup DOM elements ---
  document.getElementById("cell-operation").innerText = "Edit Cell";
  document.getElementById("cell-createButton").value = "OK"; // "Save"

  // --- update cellsize type ---
  var cellSizeSel = document.getElementById("cell-type");
  $(cellSizeSel).empty();

  cellTypes_.forEach(ct => {
    var opt = document.createElement("option");
    opt.text = ct.name;
    opt.value = ct.name;
    cellSizeSel.options.add(opt);
  });

  console.log(data.cellType);
  cellSizeSel.value = data.cellType;

  var areaObjArr = jsonCellCache_[data.area];
  var targetObj = areaObjArr.find(ca => ca.cell_uuid === data.cellUUID);
  var cellX = targetObj.cell_coordinate[0];
  var cellY = targetObj.cell_coordinate[1];
  rosPos = { x: cellX, y: cellY };

  document.getElementById("cell-x").value = rosPos.x;
  document.getElementById("cell-y").value = rosPos.y;

  // data.r = -data.r;
  // var orient = cvtRad2Deg(data.r); // in degree on UI. 
  // orient = -orient; // transform orientation from ROS to UI
  // rosPos.r = orient;
  // document.getElementById("cell-r").value = rosPos.r.toFixed(2);

  document.getElementById("cell-label").value = data.labelFullName;
  document.getElementById("cell-area").value = data.area;

  document.getElementById("cell-type").value = data.cellType;
  document.getElementById("cell-direction").value = data.cellDirection;
  document.getElementById("cell-load").value = data.cellLoad;

  // --- save the edited properties ---
  document.getElementById("cell-createButton").onclick = btnSaveCellNodeData.bind(this, data, callback, 'edit');
  document.getElementById("cell-cancelButton").onclick = cancelNodeEdit.bind(this, callback);
  document.getElementById("cell-popUp").style.display = "block";
  showNavAgentOptions();
}

function btnSaveCellNodeData(data, callback, _mode = 'edit') {
  console.log(data);
  var cellUUID = data.cellUUID;
  var cellX = document.getElementById("cell-x").value;
  var cellY = document.getElementById("cell-y").value;
  var visPos = tfROS2Canvas(gMapMeta_, {
    x: Number(cellX),
    y: Number(cellY)
  });

  var cellId = document.getElementById("cell-label").value;
  var cellArea = document.getElementById("cell-area").value;
  var cellType = document.getElementById("cell-type").value;
  var cellDetectionType = cellTypeAdaptor.toDetectionType(cellTypes_, cellType);
  // console.log(cellDetectionType);
  // --- [protection] prevent from detection type is null ---
  if (cellDetectionType == null) {
    alert('type is NOT Set Yet!');
    return;
  }

  // --- cache the difference ---
  if (_mode === 'add') {
    var cellDiff = { visID: data.id, visX: Number(visPos.x), visY: Number(visPos.y), rosID: data.cellUUID, rosX: cellX, rosY: cellY, action: 'add', area: cellArea };
    cellDiffCache_.put(genUuid(), cellDiff);
  }
  if (_mode === 'edit') {
    // console.log(jsonCellCache_);
    // TODO: enhance the mechaniam to identify the cell object.
    let cellObj;
    for (let area in jsonCellCache_) {
      cellObj = jsonCellCache_[area].find(ca => ca.cell_uuid === data.cellUUID);
      if (cellObj !== undefined) break;
    }
    let rosX = cellObj.cell_coordinate[0];
    let rosY = cellObj.cell_coordinate[1];
    let visNode = visNetwork_.body.data.nodes.get(data.id);
    let visX = visNode.x;
    let visY = visNode.y;
    // console.log(clickedNode);

    var cellDiff = { visID: data.id, visX: Number(visX), visY: Number(visY), rosID: data.cellUUID, rosX: rosX, rosY: rosY, action: 'edit', area: cellArea };
    cellDiffCache_.put(genUuid(), cellDiff);
  }
  console.log(cellDiffCache_);


  console.log(data.cellType);
  var target = cellTypes_.find(ct => ct.name === cellType);
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

  // --- update modified data here ---
  data.group = 'wms';
  data.label = cellId;
  data.area = cellArea;
  data.shape = 'circle';
  data.widthConstraint = 25;
  data.borderWidth = 2;
  // data.size = 20; // default: 25
  // data.color = 'rgba(4, 75, 148, 0.4)'
  var tooltip = `<div class="farobot-map-tooltip" 
                      style="color: ${getColor(cellDetectionTypes_, cellDetectionType, 'tooltipColor')}">
                  ${cellId}@(${Number(cellX).toFixed(2)}, ${Number(cellY).toFixed(2)})
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
  data.cellDetectionType = cellDetectionType;
  data.cellDirection = cellDirection;
  data.cellLoad = cellLoad;
  data.labelFullName = document.getElementById("cell-label").value;
  console.log(data.cellType);

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
  // if (Object.keys(data).includes("id") && cellIdArr.includes(cellId)) {
  //   alert('DUPLICATED cell name! Please rename.');
  //   return;
  // }

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
    cell_coordinate: [Number(cellX), Number(cellY)],
    cell_id: cellId,
    direction: cellDirection,
    load: cellLoadAdaptor.toFileCellLoad(cellLoad),
    status: "empty",
    type: cellDetectionType,
    width: cellWidth,
    length: cellLength
  };

  // --- check the cellId exist or not. ---
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
    // console.log(jsonCellCache_);
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
    // console.log(jsonCellCache_);
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

  console.log(areaObj);
  delete areaObj;

  console.log(jsonCellCache_);
}

function evtDragStartCellNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) return;

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

    var cellDiff = { visID: dragNode.id, visX: Number(visX), visY: Number(visY), rosID: dragNode.options.cellUUID, rosX: rosX, rosY: rosY, action: 'edit', area: cellArea };
    cellDiffCache_.put(genUuid(), cellDiff);
    console.log(cellDiffCache_);
  }
}

function evtDragEndCellNode(params) {
  // console.log(params);
  // --- protection ---
  if (params.nodes.length === 0) return;

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
    var cellPos = tfCanvas2ROS(gMapMeta_, {
      x: cellX,
      y: cellY
    });

    for (key in jsonCellCache_[cellArea]) {
      console.log(jsonCellCache_[cellArea][key]);
      if (jsonCellCache_[cellArea][key].cell_uuid !== dragNode.options.cellUUID) continue;

      jsonCellCache_[cellArea][key].cell_coordinate = [Number(cellPos.x), Number(cellPos.y)];
    }
  }
}

let selectedEditType = '';
$('.custom-radio input').on('change', function () {

  selectedEditType = this.value;
  closeManipulationMode();

  if (this.value === 'rdMapImg') {
    fadeInEditToolbar('map-tool');
    hideInfoSection();

    var divFab = document.getElementById('sub_tab_2');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_1');
    divNetwork.classList.remove('active');

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

    setVisInteractiveMode(3);

    return;
  }

  if (this.value === 'rdConfig') {
    fadeInEditToolbar('config-tool');
    hideInfoSection();
    console.log('recv visual config');
    setVisInteractiveMode(0);

    drawFabricMap();

    var divFab = document.getElementById('sub_tab_2');
    divFab.classList.add('active');
    var divNetwork = document.getElementById('sub_tab_1');
    divNetwork.classList.remove('active');

    return;
  }
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
  // document.getElementById('customRadio4').disabled = st;

  document.getElementById('customCheckbox1').disabled = _status;
  document.getElementById('customCheckbox2').disabled = _status;
  document.getElementById('customCheckbox3').disabled = _status;
  document.getElementById('customCheckbox4').disabled = _status;

  if (_status === false) {
    document.getElementById('customRadio1').checked = _status;
    document.getElementById('customRadio2').checked = _status;
    document.getElementById('customRadio3').checked = _status;
    // document.getElementById('customRadio4').checked = _status;

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
function cancelEdit() {
  closeManipulationMode();
  visOptions_.manipulation.enabled = false;
  visNetwork_.setOptions(visOptions_);
}

function btnAddNode() {
  visNetwork_.addNodeMode();
  showManipulationModeText("Add Vertex");
  clearNodePopUp();
}

function btnAddEdge() {
  visNetwork_.addEdgeMode();
  showManipulationModeText("Add Edge");
  clearNodePopUp();
}

function btnEditNode() {
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


// -------- get robot state -------- 
// --- [TBC] ---
// let agents_ = [];
// let robotStateTopic = new ROSLIB.Topic({
//   ros: ros,
//   name: '/robot_state',
//   messageType: 'far_fleet_msgs/RobotState'
// });

// robotStateTopic.subscribe(function (message) {
//   if (agents_.indexOf(message.robot_id) === -1) {
//     // console.log('robot id: ' + message.robot_id);
//     agents_.push(message.robot_id);
//   }
// });


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
  //   return
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
// --- [TBC] ---
// var new_task = new ROSLIB.Message({
//   robot_id: '',
//   priority: 0,
//   task_id: '',
//   task_type: '',
//   task_name: '',
//   task_params: '',
// });

// var send_msgs = new ROSLIB.Message({
//   tasks: null
// });

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

  // --- send the delete request ---
  await restDeleteMapData(mapName);
  await restDeleteMapAlias(mapName);

  // --- refresh the load procedure ---
  loadBundledMapDataAsync();

  // --- pop-up to hint the user ---
  notificationMsg(0, `${mapAlias} is deleted!`);
  // toast(`${mapName} is deleted!`);
}

// let applyConfigService = new ROSLIB.Service({
//   ros: ros,
//   name: '/apply_configurations',
//   serviceType: 'std_srvs/srv/Empty',
// });

// let srvRequest = new ROSLIB.ServiceRequest();


$(document).on('click', 'button', function (e) {
  var btn_id = $(this).attr('id');
  if (btn_id === 'save-changes') {
    // applyConfigService.callService(srvRequest, function (_message) {
    //   console.log(`apply config. done with messages: ${_message}`);
    // })

    // --- send the service to apply configuration ---
    wsApplyConfig();
  }
});

$('#hide-sidebar').on('click', function () {
  $('.control-sidebar').ControlSidebar('toggle');
});

function popSidebarCellTypes() {
  // console.log('pop up sidebar cell type');
  const cellTypeDeck = $('#sb-cell-size-deck');
  cellTypeDeck.empty();
  // console.log(cellTypes_);
  // console.log(cellDetectionTypes_);

  cellTypes_.forEach(ct => {
    // console.log(cellSizeAssets_[key]);
    const template = document.querySelector('#sb-cell-type-row');
    const node = document.importNode(template.content, true);
    var nameNode = node.querySelector('.cell-type-name');
    nameNode.textContent = ct.name;

    var detectionTypesSel = node.querySelector('.cell-detection-type');
    $(detectionTypesSel).empty();
    cellDetectionTypes_.forEach(cdt => {
      var opt = document.createElement("option");
      opt.text = cdt.label;
      opt.value = cdt.name;
      detectionTypesSel.options.add(opt);
    });
    detectionTypesSel.value = ct.detectionType;
    // console.log(ct.detectionType);

    var widthNode = node.querySelector('.cell-width');
    widthNode.setAttribute("id", `${key}-width`);
    widthNode.value = Number(ct.width).toFixed(2);
    var lengthNode = node.querySelector('.cell-length');
    lengthNode.setAttribute("id", `${key}-length`);
    lengthNode.value = Number(ct.length).toFixed(2);

    var editNode = node.querySelector('.edit-cell-size');
    var cardNode = node.querySelector('.card-body');
    editNode.addEventListener('click', editCellTypeCb.bind(cardNode));

    cellTypeDeck.append(node);
  });

  // --- load cell dectection types on adding a new cell type row ---
  var sbDetectionTypesSel = document.getElementById('sbft-cell-detection-type');
  $(sbDetectionTypesSel).empty();
  cellDetectionTypes_.forEach(cdt => {
    var opt = document.createElement("option");
    opt.text = cdt.label;
    opt.value = cdt.name; // name equals cell type value
    sbDetectionTypesSel.options.add(opt);
  });

  // --- update save button binding ---
  var placeholder = $('#save-placeholder');
  placeholder.empty();
  var saveBtn = document.createElement('button');
  saveBtn.setAttribute('class', 'btn btn-primary btn-lg');
  saveBtn.innerHTML = "Save";
  saveBtn.addEventListener('click', saveCellSizesCb.bind(this));

  placeholder.append(saveBtn);
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

async function saveCellSizesCb() {
  // console.log('save cell sizes');
  const cellSizeDeck = document.querySelectorAll('#sb-cell-size-deck > .card');
  console.log(cellSizeDeck);

  cellSizeDeck.forEach((cell) => {
    console.log(cell);

    var name = cell.querySelector('.cell-type-name').textContent;
    var value = cell.querySelector('.cell-detection-type').value;
    var width = cell.querySelector('.cell-width').value;
    var length = cell.querySelector('.cell-length').value;
    console.log(`name: ${name}, value: ${value}, w: ${width}, h: ${length}`);

    console.log(cellTypes_);
    var target = cellTypes_.find(ct => ct.name === name);
    console.log(target);
    let typeUUID = genUuid().replace(/-/g, '');
    if (target === undefined) {
      target = {
        "name": name,
        "id": `${name}-${typeUUID}`,
        "detectionType": value,
        "width": width,
        "length": length,
      };
      cellTypes_.push(target);
    } else {
      target.detectionType = value;
      target.width = width;
      target.length = length;
    }

  });

  console.log(cellTypes_);
  // var settingsObj = { "cell.types": cellTypes_ };

  // await restPutSettings(settingsObj);
  await restPostCellTypes(cellTypes_);
  notificationMsg(0, 'cell types size are saved!');
  // toast('cell types size are saved!');
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
      unsaved = Number(cellDiffCache_.size) > 0;
      break;
    default:
      break;
  }

  if (unsaved) {
    return "You have unsaved changes on this page. Are you sure you want to leave?";
  }
});