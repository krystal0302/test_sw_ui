const toolbar = new ToolbarManager();
const canvasView = new CanvasViewManager();
var hiddenCanvas;

let cellDetectionTypes_;

let functionTypes2_ = [];
let functionDetectionTypes_;

let fleetModels_ = {};

var fleetFileArray = [];

let visCanvas = new VisCanvas(visNetwork_, VisOptions);

// ============================
//   Window UI Event-bindings    
// ============================
// --- Load Ready (page load initialization) ---
document.addEventListener("DOMContentLoaded", function (event) {
  'use strict'
  init();
});

// --- Bind Layer-View-Toggle Events ---
const mapViewCheckbox = document.getElementById('map-view-cb');
mapViewCheckbox.addEventListener('click', toggleLayerView.bind(mapViewCheckbox, 'map'));

const originViewCheckbox = document.getElementById('origin-view-cb');
originViewCheckbox.addEventListener('click', toggleLayerView.bind(originViewCheckbox, 'origin'));

const routeViewCheckbox = document.getElementById('route-view-cb');
routeViewCheckbox.addEventListener('click', toggleLayerView.bind(routeViewCheckbox, 'route'));

const cellViewCheckbox = document.getElementById('cell-view-cb');
cellViewCheckbox.addEventListener('click', toggleLayerView.bind(cellViewCheckbox, 'cell'));

// --- Bind Layer-Switch Events ---
const customRadios = document.querySelectorAll('#property-group > .custom-radio input');
customRadios.forEach((radio) => radio.addEventListener('change', async function (event) {
  document.getElementById('btn-create-cell-conn').style.display = 'none';
  document.getElementById('edit-tools-list').style.width = ''; // reset tools list width

  closeManipulationMode(); // hide the previous layer widgets
  const selLayer = event.target.value;

  // --- switch TOOLBAR mode by layer ---
  toolbar.setMode(selLayer);

  // --- update CANVAS view state ---
  const scale = visNetwork_.getScale();
  const offset = visNetwork_.getViewPosition();
  canvasView.setMode(selLayer, option = { scale: scale, offset: offset });

  // --- get working map ---
  const selMap = document.getElementById('map-select').value;

  // --- draw map image ---
  const cvsContent = document.getElementById('map-viewer-content'); // card-body width 
  canvasView.drawMapImage(gMapData_.data, cvsContent.clientWidth * 0.96);

  if (selLayer === 'rdMapImg') {
    console.log('map layer now');
  }

  if (selLayer === 'rdRoute') {
    console.log('route layer now');
  }

  if (selLayer === 'rdCell') {
    console.log('cell layer now');
    validateFunctionTypeInputEvent('rack');
  }

  if (selLayer === 'rdConnCell') {
    // --- load connected cells data ---
    await loadConnCells();
    document.getElementById('btn-create-cell-conn').style.display = 'inline';
  }

  if (selLayer === 'rdFuncZone') {
    zoneDM.clearAll();

    let metaData = await fetchZoneMetadata(rmtToken_);
    zoneDM.loadMetadata(metaData);

    let res = await fetchGetZoneConfig(rmtToken_, selMap);
    let sysData = await res.json();
    zoneDM.loadFromSystem(sysData);

    const uiData = zoneDM.getAllData();
    console.log(uiData);
    canvasView.renderObjects(uiData);
    toolbar.load(uiData);
  }

  if (selLayer === 'rdReflector') {
    // --- fetch reflector data from system ---
    const res = await fetchGetReflectorConfig(rmtToken_, selMap);
    const sysData = await res.json();

    // --- fetching data verification ---
    // 1. if mixed reflector type exist, show error
    const fetchReflectors = sysData.reflectors;
    const cylinderType = fetchReflectors.filter((fr) => fr.type === 'cylinder');
    const CylinderNum = cylinderType.length;
    const ReflectorNum = fetchReflectors.length;
    if (CylinderNum > 0 && CylinderNum < ReflectorNum) {
      document.getElementById('add-cylinder-reflector').disabled = true;
      document.getElementById('add-board-reflector').disabled = true;
      console.error('Error: SHOULD ONLY ONE REFLECTOR TYPE EXIST!')
    }

    // 2. enable/disable the reflector button creator
    const bCylinderType = (CylinderNum === ReflectorNum);
    document.getElementById('add-cylinder-reflector').disabled = (bCylinderType) ? false : true;
    document.getElementById('add-board-reflector').disabled = (bCylinderType) ? true : false;

    // --- load system reflector data to data manager ---
    reflectorDM.loadFromSystem(sysData);

    // --- get converted reflector and load onto canvas ---
    const uiData = reflectorDM.getAllData();
    // console.log(uiData);
    canvasView.renderObjects(uiData);
  }
}));


// NOTE: false to async
const renameMapTrig = document.getElementById('rename_map');
renameMapTrig.addEventListener('click', () => mapOpTriggerCheck('rename-map-modal'), false);

const deleteMapTrig = document.getElementById('delete_map');
deleteMapTrig.addEventListener('click', () => mapOpTriggerCheck('delete-map-modal'), false);

const deleteMapConfirm = document.getElementById('confirm-delete-map');
deleteMapConfirm.addEventListener('click', () => mapDeleteConfirmCb(), false);

// ------ bind the widgets verification events ------
initRenameMapNameForm();
validateVertexInputEvent();
validateCellInputEvent();


// --- confirmation query before the page reload-- -
$(window).bind('beforeunload', function () {
  let unsaved = false;
  const currLayer = getSelectedLayer();

  switch (currLayer) {
    case 'rdMapImg':
      unsaved = mapImageDM.isExistUnsavedChanges();
      break;
    case 'rdRoute':
      unsaved = routeDM.isExistUnsavedChanges();
      break;
    case 'rdCell':
    case 'rdConnCell':
      const cellDM = CellDataManager.getInstance(visNetwork_);
      unsaved = cellDM.isExistUnsavedChanges();
      break;
    case 'rdReflector':
      unsaved = reflectorDM.isExistUnsavedChanges();
    case 'rdFuncZone':
      unsaved = zoneDM.isExistUnsavedChanges();
    default:
      break;
  }

  if (!unsaved) return;
  const reminder = 'You have unsaved changes on this page. Are you sure you want to leave?';
  return reminder;
});

const collapseSidebar = document.getElementById('hide-sidebar');
collapseSidebar.addEventListener('click', (e) => {
  $('.control-sidebar').ControlSidebar('toggle');
});

const fitViewportDom = document.getElementById('fit-viewport');
fitViewportDom.addEventListener('click', (e) => {
  canvasView.fitViewport();
});

const mapSelect = document.getElementById('map-select');
mapSelect.addEventListener('click', async (e) => {
  // --- reset dataset on canvas ---
  fCanvas.clear();
  visNetwork_.body.data.nodes.clear();
  visNetwork_.body.data.edges.clear();

  // --- load map description and data ---
  gMapMeta_ = { w: 1, h: 1 };

  // --- data fetching and rendering ---
  const selMap = document.getElementById('map-select').value;
  await updateMapAsync_refetch(selMap)
  updateMapAsync_render(selMap);
});

const editPropCheckbox = document.getElementById('edit-properties-cb');
editPropCheckbox.addEventListener('click', toggleEditProperties.bind(this));

// ============================
//     Supporting Functions    
// ============================
async function init() {
  // ====== 1. routine initialization (idling time ticking) ======
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  let statusData = await restLoginStatus();
  getLoginStatus(statusData, 'map', 'map.html');

  // ------ language switch ------
  await initLanguageSupport();
  let lng = getSetLang() || 'en';
  langTemplateObj_ = await restGetTemplateLang(lng, 'edit_map');

  // ====== 2. fetch the data ======
  const token = await rmtTokenCheck();
  // ------ load cell types ------
  const settingsData = await restGetSettings();
  cellDetectionTypes_ = settingsData['cell.detectionTypes'];

  // ------ load function types ------
  const res = await fetchGetFunctionTypes(token);
  functionTypes2_ = await res.json();
  console.log(functionTypes2_);

  // ------ load models in the fleet ------
  const selFleet = document.getElementById('fleet-select').value;
  try {
    let res = await fetchRobotsTemplates2(token);
    if (res.ok) {
      fleetModels_ = await res.json();
      fleetModels_ = Object.keys(fleetModels_);
    }
  } catch (err) {
    console.error(err);
  }

  // -------- Data Sourcing by Polling --------
  // TODO: un-comment this in production version
  pollWmsStates(apiQueryData_, rmtToken_);


  // ====== 3. render the layer view ======
  // ------ vis-network initialization ------
  visCanvas.init();

  // ------ load map ------
  loadBundledMapDataAsync();

  // --- Edit Properties ---
  toggleEditProperties();

  // --- Properties View ---
  toggleLayerView('route');
  toggleLayerView('cell');


  // --- calculate visNetwork canvas DOM width & height ---
  // --- get the viewport dimensions ---
  const vpDim = visCanvas.getMapViewSize();
  console.log(vpDim);
  document.getElementById('fabric-canvas-container').style.height = `${vpDim}px`;
}

const cvsViewerDim_ = { width: 0, height: 0 };
function toggleEditProperties() {
  const EditLayers = ['map-edit-rd', 'route-edit-rd', 'cell-edit-rd', 'conn-cell-edit-rd', 'zone-edit-rd', 'reflector-edit-rd'];
  const ViewLayers = ['map-view-cb', 'origin-view-cb', 'route-view-cb', 'cell-view-cb'];

  // --- preserve canvas view dimensions ---
  const state = document.getElementById('edit-properties-cb').checked;
  if (state) {
    const visNetworkRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
    cvsViewerDim_.width = visNetworkRect.width
    cvsViewerDim_.height = visNetworkRect.height
  }

  EditLayers.forEach((rdID) => {
    document.getElementById(rdID).disabled = (!state);
    const prev = document.getElementById(rdID).checked;
    document.getElementById(rdID).checked = (!state) ? false : prev;
  });

  ViewLayers.forEach((cbID) => {
    document.getElementById(cbID).disabled = state;
  });

  document.getElementById('edit-tools-list').style.display = 'none';

  // --- set map select state ---
  $('#map-select').prop('disabled', state);
  $('#property-group').css('background-color', state ? 'white' : 'lightgray');
  $('#property-view-group').css('background-color', state ? 'lightgray' : 'white');

  // --- set canvas state, switch to the default view ---
  canvasView.setMode('rdRoute', { offset: { x: 0, y: 0 }, scale: 1 });
  visCanvas.setMode('rdMapImg')
}

function initRenameMapNameForm() {
  $.validator.addMethod("chkDupMapName", function (value, element) {
    const data = restMapAliasExistence(value);
    const mapAliasExists = data.responseJSON;
    if (mapAliasExists === undefined) {
      return false;
    }
    return !mapAliasExists;
  });

  $('#rename-map-form').validate({
    rules: {
      newMapName: {
        required: true,
        chkInputVal: true,
        chkDupMapName: true,
      },
    },
    messages: {
      newMapName: {
        required: 'Please enter map name',
        chkInputVal: 'Map Name include invalid characters',
        chkDupMapName: 'Map name already exists',
      },
    },
    errorElement: 'div',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.after(error);
    },
  });

  const renameConfirmBtn = document.getElementById('rename-map-form');
  renameConfirmBtn.addEventListener('submit', renameMapAlias);
}

async function renameMapAlias(e) {
  e.preventDefault();

  if (!$('#rename-map-form').valid()) { return; }
  $('#rename-map-modal').modal('hide');
  const currMapName = document.getElementById('map-select').value;
  const newMapName = document.getElementById('new-mapname').value;

  // --- request sending ---
  const res = await fetchPutMapNameAlias(rmtToken_, currMapName, newMapName);
  // --- response handling ---
  const statusText = await res.json();
  if (res.ok) {
    notificationMsg(1, statusText);
  } else {
    notificationMsg(3, statusText);
  }

  $(`#map-select option[value="${currMapName}"]`).text(`${newMapName}`);
}

async function mapOpTriggerCheck(dataTarget) {
  const executingFlow = await checkProtection();
  const selMap = document.getElementById('map-select').value;
  const bMapInUse = executingFlow["using_map"].includes(selMap);

  if (bMapInUse) {
    notificationMsg(3, 'Map has executing task, operation CANNOT be conducted.');
    return;
  }

  $(`#${dataTarget}`).modal("show");
}

async function mapDeleteConfirmCb() {
  // --- get the selected map filename ---
  const mapName = document.getElementById('map-select').value;
  const mapAlias = document.getElementById('map-select').text;

  // --- check if map is included by fleet ---
  fleetFileArray = [];
  let fleetFiles = await fetchGetFleets(rmtToken_);
  fleetFiles = Object.keys(fleetFiles);

  for (const fleetFile of fleetFiles) {
    const fleetName = fleetFile;
    let fltSettings = {};
    let fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetName);
    fltSettings[fleetName] = fltConfigs;

    const fltKey = Object.keys(fltSettings)[0]; // suppose only one key-value pair
    const index = fltSettings[fltKey].maps.indexOf(mapName);
    if (index >= 0) {
      fleetFileArray.push(fltKey);
    }
  }

  // --- re-confirm the delete operation ---
  const reminder = 'There are fleets still using this map. Are you sure you want to delete?';
  if (fleetFileArray.length && !confirm(reminder)) {
    return;
  }

  // --- send the request to conduct the delete operation ---
  const res = await deleteTargetMap(mapName, mapAlias);
  if (res.ok) {
    notificationMsg(1, `${mapAlias} is deleted on Success!`);
  } else {
    notificationMsg(3, `${mapAlias} is deleted on Failure!`);
  }

  // --- reload the load procedure ---
  loadBundledMapDataAsync();
}

// ======================
//       Events
// ======================
async function updateMapMetaToServer(_mapName) {
  // --- fetch data from server ---
  let res = await fetchGetMapMeta(rmtToken_, _mapName);
  let mapMeta = await res.json();
  mapMeta['origin_x'] = gMapMeta_.origin.x;
  mapMeta['origin_y'] = gMapMeta_.origin.y;

  // --- save updated data back to server ---
  res = await fetchPutMapMeta(rmtToken_, _mapName, mapMeta);
  res = await res.json();
  return;
}

let gMapData_;
var gMapMeta_;

async function loadBundledMapDataAsync() {
  // ====== load map select ======
  // --- fetch map list data ---
  let data = await fetchGetAllMaps(rmtToken_);
  let mapObj = await data.json();

  // --- remove all options in map select ---
  let mapSelect = document.getElementById('map-select');
  while (mapSelect.firstChild) {
    mapSelect.removeChild(select.firstChild);
  }

  // --- append the options ---
  const mapArr = Object.entries(mapObj);
  mapArr.forEach((map) => {
    const option = document.createElement('option');
    option.value = map[0];
    option.text = map[1];
    mapSelect.add(option);
  });

  // ====== load canvas by layers ======
  // --- Visual View ---
  const selMap = document.getElementById('map-select').value;

  // --- load map desscription and data ---
  gMapMeta_ = { w: 1, h: 1 };

  // --- data fetching and rendering ---
  await updateMapAsync_refetch(selMap);
  updateMapAsync_render(selMap);
}

async function updateMapAsync_refetch(_mapName) {
  try {
    // === fetch map image data ===
    let res = await fetchGetMapMeta(rmtToken_, _mapName);
    res = await res.json();
    gMapData_ = {
      w: res.image_width,
      h: res.image_height,
      data: res.image
    };

    await loadImageData2HiddenCanvas(gMapData_, true);

    // === fetch map meta data ===
    res = await fetchGetMapMeta(rmtToken_, _mapName);
    res = await res.json();
    gMapMeta_ = {
      w: res.image_width,
      h: res.image_height,
      image: `${res.map_name}.png`,
      occupied_thresh: res.occupied_thresh,
      origin: {
        x: res.origin_x,
        y: res.origin_y,
        z: 0,
      },
      resolution: res.resolution,
      nav_graph: `${res.map_name}.dot`,
      cell: `${res.map_name}.json`,
      triton: `${res.map_name}.amf`,
    }

    // ====== fetch Cell data ======
    // --- load cell data from system ---
    res = await fetchGetMapCells(rmtToken_, gMapMeta_.cell);
    // --- protection for absent cell data ---
    const cellData = (res.ok) ? await res.json() : [];

    // --- caching ---
    console.log(cellData);
    const cellDM = CellDataManager.getInstance(visNetwork_);
    cellDM.loadFromSystem(cellData);
    console.log(cellDM.getAllData());

    // --- cells status indicator ---
    // ====== fetch navGraph data ====== 

    // --- load graph data from back-end ---
    res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
    // --- protection for absent graph data ---
    const graphData = (res.ok) ? await res.text() : '';

    // --- caching ---
    routeDM.loadFromSystem(graphData);

    // --- TODO: rendering on UI ---
  } catch (err) {
    console.error(err);
  }
}

async function updateMapAsync_render() {
  // --- draw vis-network canvas ---
  const viewportDim = visCanvas.getMapViewSize();

  visNetwork_.moveTo({
    position: {
      x: gMapData_.w / 2,
      y: gMapData_.h / 2
    },
    scale: (viewportDim / gMapData_.h) * 3.5
  });

  // === render map meta data ===
  visUpdateMapOrigin();
  const cellDM = CellDataManager.getInstance(visNetwork_);
  const cellAllData = cellDM.getAllData();
  visDrawCells(visNetwork_, cellAllData);
  const routeAllData = routeDM.getAllData();
  visDrawRoutes(visNetwork_, routeAllData);
}

async function loadImageData2HiddenCanvas(mapEntity, bShow) {
  const mapDataURL = `data:image/png;base64,${mapEntity.data}`;
  let img = new Image();
  img.onload = await function () {
    hiddenCanvas = document.createElement('canvas');
    hiddenCanvas.width = mapEntity.w;
    hiddenCanvas.height = mapEntity.h;

    // if (hiddenCanvas === null) { return; }

    const ctx = hiddenCanvas.getContext('2d');
    if (!bShow) {
      ctx.clearRect(0, 0, mapEntity.w, mapEntity.h);
      return;
    }

    ctx.drawImage(img, 0, 0);
  };
  img.src = mapDataURL;
}


// =====================
//     DOM Callbacks 
// =====================
async function btnSetOrigin() {
  isOriginSet_ = true;

  // --- update origin object ---
  const objs = fCanvas.getObjects()
  const obj = objs?.find((o) => o.id === 'mapOrigin');
  if (obj) {
    mapOrigin_.left = obj.aCoords.tl.x;
    mapOrigin_.top = obj.aCoords.tl.y;
    mapOrigin_.width = obj.aCoords.br.x - obj.aCoords.tl.x;
    mapOrigin_.height = obj.aCoords.br.y - obj.aCoords.tl.y;

    const x = mapOrigin_.left + mapOrigin_.width / 2;
    const y = mapOrigin_.top + mapOrigin_.height / 2;
    const center = new Point2D(x, y);

    const rosPos = tfCanvas2ROS(gMapMeta_, center);
    gMapMeta_.origin.x -= Number(rosPos.x);
    gMapMeta_.origin.y -= Number(rosPos.y);

    // --- redraw routes and cells ---
    visNetwork_.body.data.nodes.clear();
    visNetwork_.body.data.edges.clear();

    visUpdateMapOrigin();

    const cellDM = CellDataManager.getInstance(visNetwork_);
    const cellAllData = cellDM.getAllData();
    visDrawCells(visNetwork_, cellAllData);

    try {
      // --- load graph data from back-end ---
      const res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
      const graphData = await res.text();

      // --- rendering on UI ---
      const routes = cvtDot2Json(graphData);
      visDrawRoutes(visNetwork_, routes);
    } catch (err) {
      console.error(err);
    }
  }

  fCanvas.remove(originSprite);

  // --- UI display ---
  document.getElementById('get-origin').style.display = 'inline';
  document.getElementById('set-origin').style.display = 'none';
  document.getElementById('cancel-origin').style.display = 'none';
  document.getElementById('undo-draw').style.display = 'inline';

  // --- put to the system ---
  const selMap = document.getElementById('map-select').value;
  await updateMapMetaToServer(selMap);
}

async function btnSaveChanges() {
  const executingFlow = await checkProtection();
  const selMap = document.getElementById('map-select').value;
  const bMapInUse = executingFlow["using_map"].includes(selMap);

  // --- map is in-use case ---
  if (bMapInUse) {
    // --- [FAR-3356] show saving fail reason ---
    notificationMsg(3, `Map ${selMap} is still in use. Please check task executing states.`);
    return;
  }

  // --- map is idle case ---
  const RunFunctionForMapSave = {
    'rdMapImg': btnSaveMapImage,
    'rdRoute': btnSaveRoutes,
    'rdCell': btnSaveCells,
    'rdConnCell': btnSaveConnCells,
    'rdFuncZone': btnSaveFunctionZones,
    'rdReflector': btnSaveReflectors,
  }

  const currLayer = getSelectedLayer();
  const bRegisterType = RunFunctionForMapSave.hasOwnProperty(currLayer);
  if (bRegisterType) {
    RunFunctionForMapSave[currLayer]();
  }
}

async function btnSaveFunctionZones() {
  const errMsg = zoneDM.getInvalidMsg();
  if (!isEmptyString(errMsg)) {
    alert(errMsg);
    return;
  }

  // 1. dump the data from data manager
  const data = zoneDM.getDataInSysFormat();
  console.log(data);

  // 2. put the data to system 
  const selMap = document.getElementById('map-select').value;
  let res = await fetchPutZoneConfig(rmtToken_, selMap, data);

  // 3. response handling & notification 
  if (res.ok) {
    notificationMsg(1, 'Function Zones are saved on Success!');
  } else {
    notificationMsg(3, 'Function Zones are saved on Failure!');
  }

  // --- clear cached zone list ---
  zoneDM.clearDataset();

  // --- reload canvas and toolbar ---
  res = await fetchGetZoneConfig(rmtToken_, selMap);
  const sysData = await res.json();
  zoneDM.loadFromSystem(sysData);

  const uiData = zoneDM.getAllData();
  canvasView.renderObjects(uiData);
  toolbar.load(uiData);

  // --- keep toolbar selected options ---
  let zone = zoneDM.getZone();
  if (isEmpty(zone)) return;
  updateZoneTypeSelect(zone.type);
  updateZoneModelFilter(zone.model);
}

async function btnSaveReflectors() {
  // 1. dump the data from data manager
  const data = reflectorDM.getDataInSysCoord();

  // 2. put the data by api
  const selMap = document.getElementById('map-select').value;
  const res = await fetchPutReflectorConfig(rmtToken_, selMap, data);

  // 3. response handling & notification 
  if (res.ok) {
    notificationMsg(1, 'Reflectors are saved on Success!');
  } else {
    notificationMsg(3, 'Reflectors are saved on Failure!');
  }
}


// ---------------------------------
//     Toggle Data Visualization 
// ---------------------------------
var bMapShow = true;

async function toggleLayerView(layer) {
  const tagTable = {
    map: 'map-view-cb',
    origin: 'origin-view-cb',
    route: 'route-view-cb',
    cell: 'cell-view-cb',
  };
  const domID = tagTable[layer] || null;
  if (!domID) { return; }

  const bChecked = document.getElementById(domID).checked;
  // console.log(bChecked);

  // --- map layer ---
  if (layer === 'map') {
    bMapShow = bChecked;
    const options = {
      edges: {
        color: (bChecked) ? "black" : "white",
        background: {
          color: (bChecked) ? "white" : "#393f52"
        }
      },
    };

    visNetwork_.setOptions(options);
    visNetwork_.redraw();
    return;
  }

  // --- origin, route, cell layers ---
  const layerTokenMap = {
    'origin': 'metadata',
    'route': 'navnode',
    'cell': 'wms',
  };
  const token = layerTokenMap[layer];
  const visBodyNodes = Object.values(visNetwork_.body.nodes);
  const targetNodes = visBodyNodes.filter((n) => n.options.group === token);
  const nodesUpdate2 = targetNodes.map((tn) => {
    return {
      id: tn.id,
      hidden: !bChecked,
    };
  });
  // console.log(nodesUpdate2);
  visNetwork_.body.data.nodes.update(nodesUpdate2);
}


// =================================
//     Common Utilities 
// =================================
// --- get current user selected layer ---
function getSelectedLayer() {
  const customRadios = document.getElementsByClassName('layer-radio');
  const radio = Array.from(customRadios).find((cr) => cr.checked);
  // console.log(selLayer?.value);
  const selLayer = (radio) ? radio?.value : null;
  return selLayer;
}

function clearNodePopUp() {
  document.getElementById('node-createButton').onclick = null;
  document.getElementById('node-cancelButton').onclick = null;
  document.getElementById('node-popUp').style.display = "none";

  document.getElementById('cell-createButton').onclick = null;
  document.getElementById('cell-cancelButton').onclick = null;
  document.getElementById('cell-popUp').style.display = "none";

  document.getElementById('zone-node-createButton').onclick = null;
  document.getElementById('zone-node-cancelButton').onclick = null;
  document.getElementById('zone-node-popUp').style.display = "none";

  document.getElementById('nav-agent').style.display = "none";
}


// =================================
//     Data Fetching Auxiliaries 
// =================================
async function rmtTokenCheck() {
  let token;
  try {
    token = await fetchToken();
  } catch (err) {
    // console.error(err);
    await sleep(5000);
    token = rmtTokenCheck();
  }
  return token;
}


async function deleteTargetMap(_mapName, _mapAlias) {
  // --- send the delete request ---
  let res = await fetchDeleteMap(rmtToken_, _mapName);
  res = await res.json();

  // --- remove maps from fleet ---
  for (let fleetFile of fleetFileArray) {
    let fltSettings = {};
    const fltConfigs = await fetchGetFleetConfigs(rmtToken_, fleetFile);
    fltSettings[fleetFile] = fltConfigs;

    const index = fltSettings[fleetFile].maps.indexOf(_mapName);
    fltSettings[fleetFile].maps.splice(index, 1);
    fltSettings = JSON.stringify(fltSettings);
    await restPostFleetSettings(fleetFile, fltSettings);
    fleetFileArray = fleetFileArray.filter((e) => e !== fleetFile);
  }
}

// // --- polling WMS status data ----
// function pollWmsStates(_dataObj, _token, _inteval = 1000) {
//   setInterval(async function () {
//     try {
//       const selMap = document.getElementById('map-select').value;
//       const res = await fetchWmsStates(_token, selMap);
//       if (!res.ok) { return; }
//       _dataObj['wms'] = await res.json();
//     } catch (e) {
//       _dataObj['wms'] = undefined;
//     }
//   }, _inteval);
// };

function showManipulationModeText(modeName) {
  $('#edit-status').show();
  $('#edit-tools-info').show();

  $('#edit-status-title').text('Status.');
  $('#edit-status-badge').text(`${modeName} Mode`);
  showCancelEditButton();
}

function hideManipulationModeText() {
  $('#edit-status').hide();
  $('#edit-tools-info').hide();
}
