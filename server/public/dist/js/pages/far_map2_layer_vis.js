// ===============================
//       VIS-NETWORK DRAWING 
// ===============================
// --- vis-network configuration ---
const data = {
  nodes: new vis.DataSet([]),
  edges: new vis.DataSet([]),
};

const visContainer = document.getElementById('far-network2');

// --- create a network instance ---
let visNetwork_ = new vis.Network(visContainer, data, VisOptions);

// --- vis-network event-bindings ---
visNetwork_.on("beforeDrawing", function (ctx) {
  // console.log(hiddenCanvas);
  if (!hiddenCanvas) { return; }
  const width = hiddenCanvas.width;
  const height = hiddenCanvas.height;

  // --- clean before drawing ---
  ctx.clearRect(0, 0, width, height);
  if (!bMapShow) { return; } // toggle-off case
  ctx.drawImage(hiddenCanvas, 0, 0, width, height);
});

// --- click with selecting edge on `Route` layer ---
visNetwork_.on("selectEdge", function (props) {

  const selLayer = getSelectedLayer();
  if (props.nodes.length == 0 && selLayer === 'rdRoute') {
    visNetwork_.editEdgeMode();
    document.getElementById('edit-status').style.display = 'block';
    $('#edit-status-title').text('Status.');
    $('#edit-status-badge').text('Edit Edge Mode');
    showCancelEditButton(true);
  }

  showDeleteButtons(false);
  document.getElementById('delete-node').style.display = 'inline';
});

visNetwork_.on('oncontext', function (params) {
  editNodeHandler(params);
});

// --- dummy click without selecting anything ---
visNetwork_.on('click', function (props) {
  console.log('here is on click');
  console.log(props);
  const manipulationMode = visNetwork_.manipulation.inMode;

  clearNodePopUp();

  if (manipulationMode === 'editEdge') { return; }
  console.log(manipulationMode);

  showDeleteButtons(false);

  const selLayer = getSelectedLayer();
  console.log(selLayer);
  if (selLayer === 'rdCell' || selLayer === 'rdConnCell') {
    if (manipulationMode === 'addNode') { return; }
    showCancelEditButton(false);
    document.getElementById('edit-status').style.display = 'none';
  } else if (selLayer === 'rdRoute') {
    if (manipulationMode === 'addNode' || manipulationMode === 'addRoute') { return; }
    if (manipulationMode !== 'editNode') { return; }
    showCancelEditButton(false);
    document.getElementById('edit-status').style.display = 'none';
  }
});

// --- vis-network supporting functions ---
function visFitToViewport() {
  const vpDim = visCanvas.getMapViewSize();
  console.log(vpDim);
  visNetwork_.moveTo({
    position: {
      x: gMapMeta_.w / 2,
      y: gMapMeta_.h / 2,
    },
    scale: (vpDim / gMapMeta_.h),
  });
}

function editNodeHandler(params) {
  params.event.preventDefault();

  // --- 1. identify the selected node ---
  const nodeID = visNetwork_.getNodeAt(params.pointer.DOM);
  const selNode = visNetwork_.body.data.nodes.get(nodeID);

  // --- 2. switch-on edit-mode & toolbar preparation ---
  // --- case 2-1 NavGraph Layer ---
  let manipulationMode = visNetwork_.manipulation.inMode;
  const selLayer = getSelectedLayer();

  if (selNode.group === 'navnode' && selLayer === 'rdRoute') {
    visNetwork_.selectNodes([nodeID]);
    visNetwork_.editNode();
    document.getElementById('edit-status').style.display = 'block';
    const modeName = 'Edit Vertex';
    $('#edit-status-title').text('Status.');
    $('#edit-status-badge').text(`${modeName} Mode`);
    showCancelEditButton(true);

    showDeleteButtons(false);
    document.getElementById('delete-node').style.display = 'inline';
    return;
  }

  // --- case 2-2 Cell Layer ---
  // --- 2. identify the selected edit properties ---
  if (selNode.group === 'wms' && selLayer === 'rdCell') {
    visNetwork_.selectNodes([nodeID]);
    visNetwork_.editNode();
    document.getElementById('edit-status').style.display = 'block';
    const modeName = 'Edit Cell';
    $('#edit-status-title').text('Status.');
    $('#edit-status-badge').text(`${modeName} Mode`);
    showCancelEditButton(true);

    showDeleteButtons(false);
    document.getElementById('delete-cell').style.display = 'inline';
    return;
  }
}


// ========================
//   Vis-Network Data I/O     
// ========================
function visUpdateMapOrigin(bShow = true) {
  // update network node
  const orgPos = tfROS2Canvas(gMapMeta_, new Point2D(0.0, 0.0));
  const orgNode = {
    id: 'Origin',
    x: (orgPos.x),
    y: (orgPos.y),
    shape: 'image',
    image: `./${getImagePath()}/ucs.png`,
    size: 50,
    color: 'red',
    label: 'Origin',
    group: 'metadata',
    hidden: !bShow,
  };
  visNetwork_.body.data.nodes.update(orgNode);
}

// --- Callback passed as parameter is ignored ---
function clearNodePopUp() {
  document.getElementById('node-createButton').onclick = null;
  document.getElementById('node-cancelButton').onclick = null;
  document.getElementById('node-popUp').style.display = 'none';

  document.getElementById('cell-createButton').onclick = null;
  document.getElementById('cell-cancelButton').onclick = null;
  document.getElementById('cell-popUp').style.display = 'none';

  document.getElementById('nav-agent').style.display = 'none';
}

function updatePopUpPosition(type_) {
  const windowTop = window.pageYOffset || document.documentElement.scrollTop;
  const popUpId = `${type_}-popUp`;
  let popUpHeight = 0;
  let canvasRect = 0;
  let popupSel = document.getElementById(popUpId);

  switch (type_) {
    case 'node':
      popUpHeight = 230;
      canvasRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
      break;
    case 'cell':
      popUpHeight = 560;
      canvasRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
      break;
    case 'zone-node':
      popUpHeight = 170;
      canvasRect = document.getElementById('fabric_canvas').getBoundingClientRect();
      break;
    default:
      break
  }

  if (window.innerWidth <= IPAD_PORTRAIT_WIDTH) {
    popupSel.style.position = 'fixed';
    popupSel.style.top = '';
    popupSel.style.left = '0';
    popupSel.style.bottom = '0';
    popupSel.style.height = popUpHeight > window.innerHeight ? '100vh' : `${popUpHeight}px`;
  } else {
    popupSel.style.position = 'absolute';
    popupSel.style.top = `${windowTop + canvasRect.y}px`;
    popupSel.style.left = `${canvasRect.x}px`;
    popupSel.style.bottom = '';
  }
}

function showCancelEditButton(show = true) {
  const showStyle = (show) ? 'inline' : 'none';
  document.getElementById('cancel-route-edit').style.display = showStyle;
  document.getElementById('cancel-cell-edit').style.display = showStyle;
}

function showDeleteButtons(show = true) {
  const showStyle = (show) ? 'inline' : 'none';
  document.getElementById('delete-node').style.display = showStyle;
  document.getElementById('delete-cell').style.display = showStyle;
}

function closeManipulationMode() {
  showCancelEditButton(false);
  showDeleteButtons(false);
  document.getElementById('edit-status').style.display = 'none'
  clearNodePopUp();
}


class VisMapMode {
  constructor(network, options) {
    this.network = network;
    this.options = options || {};
    console.log(this.options);
  }

  init() {
    console.log(this.options);
    this.options.nodes.fixed = { x: true, y: true };
    this.options.manipulation.enabled = false;
    this.options.interaction.dragNodes = false;
    this.options.interaction.selectable = false;
    this.options.interaction.selectConnectedEdges = false;
    this.options.interaction.navigationButtons = false;
    this.network.setOptions(this.options);
  }
}

class VisRouteMode {
  constructor(network, options) {
    this.network = network;
    this.options = options || {};
  }

  init() {
    // --- set network prperty ---
    const options = {
      interaction: {
        dragNodes: true,
        selectable: true,
        selectConnectedEdges: true,
        navigationButtons: false,
      },
      edges: {
        background: { color: 'white' },
        color: 'black',
      }
    }
    this.network.setOptions(options);

    console.log(' --- config the route fixed coordinates ---');
    // console.log(this.network);

    // --- freeze the all the nodes ---
    const networkNodes = Object.values(this.network.body.nodes);
    networkNodes.filter((node) => node.options.group === 'navnode').forEach(node => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'gray';
      node.options.color.border = 'black';
    });

    networkNodes.filter((node) => node.options.group === 'wms').forEach(node => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'rgba(220,220,220,1.0)';
      node.options.color.border = 'rgba(220,220,220,1.0)';
    });

    networkNodes.filter((node) => node.options.group === 'metadata').forEach(node => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'rgba(220,220,220,0.4)';
    });

    this.network.redraw();
  }
}

class VisCellMode {
  constructor(network, options) {
    this.network = network;
    console.log(options);
    this.options = options || {};
  }

  init() {
    // --- set network prperty ---
    const options = {
      manipulation: {
        enabled: false,
      },
      interaction: {
        dragNodes: true,
        selectable: true,
        selectConnectedEdges: true,
        navigationButtons: false,
      },
      edges: {
        background: { color: 'white' },
        color: 'black',
      }
    }
    this.network.setOptions(options);
    console.log('running the vis cell mode');

    // --- freeze the all the nodes ---
    const networkNodes = Object.values(this.network.body.nodes);
    networkNodes.filter((node) => node.options.group === 'wms').forEach((node) => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'gray';
      node.options.color.border = 'black';
    });

    networkNodes.filter((node) => node.options.group === 'navnode').forEach((node) => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'rgba(220,220,220,1.0)';
      node.options.color.border = 'rgba(220,220,220,1.0)';
    });

    networkNodes.filter((node) => node.options.group === 'metadata').forEach((node) => {
      node.options.fixed.x = node.options.fixed.y = true;
      node.options.color.background = 'rgba(220,220,220,0.4)';
    });

    this.network.redraw();
  }
}

class VisConnCellMode {
  constructor(network, options) {
    this.network = network;
    this.options = options || {};
  }

  init() {
  }
}

class VisCanvas {
  constructor(network, options) {
    this.network = network;
    this.options = options || {};

    this.modeMap = new Map();
    this.mapMode = new VisMapMode(network, options);
    this.routeMode = new VisRouteMode(network, options);
    this.cellMode = new VisCellMode(network, options);
    this.connCellMode = new VisConnCellMode(network, options);

    this.modeMap.set('rdMapImg', this.mapMode);
    this.modeMap.set('rdRoute', this.routeMode);
    this.modeMap.set('rdCell', this.cellMode);
    this.modeMap.set('rdConnCell', this.connCellMode);
    this.modeMap.set('rdFuncZone', this.mapMode);
    this.modeMap.set('rdReflector', this.mapMode);
    this.currentMode = this.mapMode;
  }

  init() {
    // ------ vis-network options ------
    this.options.nodes.fixed = { x: true, y: true };
    this.options.manipulation.enabled = false;
    this.options.interaction.dragNodes = false;
    this.options.interaction.selectable = false;
    this.options.interaction.selectConnectedEdges = false;
    this.options.interaction.navigationButtons = true;

    // --- vis dataset reset ---
    this.network.setOptions(this.options);
    this.network.fit();

    // --- flush dataset ---
    this.network.body.data.nodes.clear();
    this.network.body.data.edges.clear();
  }

  setMode(newMode) {
    console.log(newMode);
    this.currentMode = this.modeMap.get(newMode);
    console.log(this.currentMode);
    this.currentMode.init();
  }

  getMapViewSize() {
    const visNetworkRect = document.getElementsByClassName('vis-network')[0].getBoundingClientRect();
    const viewHeight = visNetworkRect.height;
    const viewWidth = visNetworkRect.width;
    const mapViewSize = Math.min(viewWidth, viewHeight);

    return mapViewSize;
  }
}
