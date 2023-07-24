class MapToolset {
	constructor(options) {
		this.options = options || {};
	}
}

class RouteToolset {
	constructor(options) {
		this.options = options || {};
	}
}

class CellToolset {
	constructor(options) {
		this.options = options || {};
	}
}

class ConnCellToolset {
	constructor(options) {
		this.options = options || {};
	}
}

class FunctionZoneToolset {
	constructor(options) {
		this.options = options || {};
	}

	load(objects) {
		// --- create zone list cards ---
		removeAllChildNodes(zonesDeck);
		objects.forEach((obj) => {
			const node = createZoneCardView(obj);
			zonesDeck.append(node);
		});

		initZoneCards();

		// --- create zone type options ---
		const types = zoneDM.getAllTypes();
		createZoneTypeOptions(types);

		// --- create model options ---
		if (fleetModels_.length) {
			createZoneModelOptions(fleetModels_);
		}

		restoreInitState();
	}
}

class ReflectorToolset {
	constructor(options) {
		this.options = options || {};
	}
}

class ToolbarManager {
	constructor() {
		this.modes = new Map();
		this.mapMode = new MapToolset();
		this.routeMode = new RouteToolset();
		this.cellMode = new CellToolset();
		this.connCellMode = new ConnCellToolset();
		this.funcZoneMode = new FunctionZoneToolset();
		this.reflectorMode = new ReflectorToolset();
		this.modes.set('rdMapImg', this.mapMode);
		this.modes.set('rdRoute', this.routeMode);
		this.modes.set('rdCell', this.cellMode);
		this.modes.set('rdConnCell', this.connCellMode);
		this.modes.set('rdFuncZone', this.funcZoneMode);
		this.modes.set('rdReflector', this.reflectorMode);
		this.currentMode = this.routeMode;
		this.ToolMode = {
			'rdMapImg': 'map-tool',
			'rdRoute': 'route-tool',
			'rdCell': 'cell-tool',
			'rdConnCell': 'conn-cell-tool',
			'rdFuncZone': 'zone-tool',
			'rdReflector': 'reflector-tool',
		}
	}

	setMode(newMode) {
		this.currentMode = this.modes.get(newMode);
		this.#showUiLayout(newMode);
	}

	load(objects) {
		this.currentMode.load(objects);
	}

	#showUiLayout(mode) {
		this.#toggleToolbar(this.ToolMode[mode]);
		constructAvailableAgents();
		if (mode === 'route-tool' || mode === 'cell-tool') {
			document.getElementById('edit-tools-info').style.display = 'block'
			document.getElementById('additional-tools').style.display = 'none'
		} else if (mode === 'conn-cell-tool') {
			document.getElementById('edit-tools-info').style.display = 'block'
			document.getElementById('additional-tools').style.display = 'none'

			// --- `Edit Properties Tools` box transformation ---
			document.getElementById('btn-create-cell-conn').style.display = 'block'
			$('#edit-tools-list').css('width', '36%');
		} else {
			document.getElementById('edit-tools-info').style.display = 'none'
		}
	}

	#toggleToolbar(_target) {
		// -- toggle target toolbar, if input is `null` or `undefined` ---
		if (!_target) { return; }

		// --- reset zone toolbar ---
		restoreInitState();
		closeZoneEditor();

		const toolbars = ['map-tool', 'route-tool', 'cell-tool', 'conn-cell-tool', 'zone-tool', 'reflector-tool'];
		toolbars.forEach((tbid) => {
			document.getElementById(tbid).style.display = (tbid === _target) ? "block" : "none";
		});

		// --- [UX] toggle edit properties tools and swap tools position ---
		if (!isMobile()) {
			$('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');
		} else if (window.innerWidth <= IPAD_PORTRAIT_WIDTH || window.innerHeight <= IPAD_PORTRAIT_WIDTH) {
			$('#edit-tools-card').detach().appendTo('#edit-tools-section');
			$('#edit-tools-list').hide();
			$('#edit-tools-section').css('display', _target === null || _target === undefined ? 'none' : 'block');
		} else {
			$('#edit-tools-card').detach().appendTo('#edit-tools-list');
			$('#edit-tools-section').hide();
			$('#edit-tools-list').css('display', _target === null || _target === undefined ? 'none' : 'block');
		}
	}
}

async function constructAvailableAgents() {
	// --- fetch available agents in fleets --- 
	let navAgents = await getAvailableRobotFromFleetList();

	const mapName = document.getElementById('map-select').value;
	navAgents = _.filter(navAgents, (na) => na.map === mapName);
	navAgents = navAgents.map((na) => na.robot_id);

	// --- reset selection options ---
	var toolbarId = 'nav-agent-select';
	var navSel = document.getElementById(toolbarId);
	$(navSel).empty();

	// --- update selection options ---
	// --- hide popups anchor button ---
	if (navAgents.length === 0) {
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
	navAgents.forEach((na) => {
		var opt = document.createElement("option");
		opt.text = na;
		opt.value = na;
		navSel.options.add(opt);
	});
}

class CanvasViewManager {
	constructor() {
		this.currView = new RouteModeCanvas();
	}

	setMode(newMode, options = {}) {
		document.getElementById('fabric_tab_div').style.display = 'none';
		document.getElementById('vis_tab_div').style.display = 'none';

		if (newMode === 'rdMapImg') {
			this.currView = new MapModeCanvas(fCanvas, options);
			document.getElementById('fabric_tab_div').style.display = 'block';
		} else if (newMode === 'rdRoute') {
			this.currView = new RouteModeCanvas(visNetwork_);
			document.getElementById('vis_tab_div').style.display = 'block';
		} else if (newMode === 'rdCell') {
			this.currView = new CellModeCanvas(visNetwork_);
			document.getElementById('vis_tab_div').style.display = 'block';
		} else if (newMode === 'rdConnCell') {
			this.currView = new ConnCellModeCanvas(visNetwork_);
			document.getElementById('vis_tab_div').style.display = 'block';
		} else if (newMode === 'rdFuncZone') {
			this.currView = new FuncZoneModeCanvas(fCanvas, options);
			document.getElementById('fabric_tab_div').style.display = 'block';
		} else if (newMode === 'rdReflector') {
			this.currView = new ReflectorModeCanvas(fCanvas, options);
			document.getElementById('fabric_tab_div').style.display = 'block';
		}
		this.currView.init();
	}

	renderObjects(objects) {
		this.currView.renderObjects(objects);
	}

	drawMapImage(base64Data, dim) {
		this.currView.drawMapImage(base64Data, dim);
	}

	fitViewport() {
		this.currView.fitViewport();
	}
}

class MapModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		console.log(this.canvas);
		this.options = options || { offset: { x: 0, y: 0 }, scale: 1 };
	}

	init() {
		visCanvas.setMode('rdMapImg');

		// --- run the procedure ---
		this.canvas._objects.forEach((o) => {
			o.opacity = (o.layer === 'rdMapImg') ? 1.0 : 0.3;
			o.selectable = (o.layer === 'rdMapImg') ? true : false;
			// console.log(o);
			return o;
		});

		// --- enable zoom and pan ---
		this.fitViewport();
		const zoom = this.canvas.width / gMapData_.w;
		const tool = new PanZoom(this.canvas);
		tool.setMinZoomRatio(zoom);

		this.canvas.renderAll();
	}

	renderObjects() {
		// console.log('loading ...');
		this.canvas.renderAll();
	}

	/**
	 * @param {string} base64Data 
	 * @param {Number} dim 
	 */
	drawMapImage(base64Data, dim) {
		const inst = this;

		// --- clear other layer objects initially ---
		inst.canvas.getObjects().forEach(async function (o) {
			if (o.layer === 'rdMapImg' || o.path !== undefined) { return; }
			inst.canvas.remove(o);
		});

		let img = new Image();
		const mapDataURL = `data:image/png;base64,${base64Data}`;
		img.src = mapDataURL;

		img.onload = function () {
			const maxDim = Math.max(img.width, img.height);
			const zoom = (maxDim >= dim) ? (dim / maxDim) : 1.0;
			const viewWidth = img.width * zoom;
			const viewHeight = img.height * zoom;

			const fImg = new fabric.Image(img);

			inst.canvas.setWidth(cvsViewerDim_.width);
			inst.canvas.setHeight(cvsViewerDim_.height);
			inst.canvas.setZoom(zoom);

			inst.canvas.setBackgroundImage(fImg).renderAll();
		};
	}

	fitViewport() {
		// console.log('map fit to viewport');
		const cvsContent = document.getElementById("fabric-canvas-container"); // card-body width 
		const maxScale = 1.0;
		const calcScale = cvsContent.clientWidth / gMapMeta_.w;
		const scale = (calcScale > maxScale) ? maxScale : calcScale;
		this.canvas.setViewportTransform([scale, 0, 0, scale, 0, 0]);
	}

}

class RouteModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		this.options = options || {};
	}

	init() {
		this.canvas.redraw();
		visCanvas.setMode('rdRoute');
	}

	renderObjects(objects) {
		console.log('loading...');
	}

	drawMapImage(base64Data, dim) {
		console.log("draw map image...");
	}

	fitViewport() {
		console.log('route zone fit to viewport');
		visFitToViewport()
	}

}

class CellModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		this.options = options || {};
	}

	init() {
		this.canvas.redraw();
		visCanvas.setMode('rdCell');
	}

	renderObjects(objects) {
		console.log('loading...');
	}

	drawMapImage(base64Data, dim) {
		console.log("draw map image...");
	}

	fitViewport() {
		console.log('cell zone fit to viewport');
		visFitToViewport()
	}

}

class ConnCellModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		this.options = options || {};
	}

	init() {
		visCanvas.setMode('rdRoute');
		visCanvas.setMode('rdCell');
	}

	renderObjects(objects) {
		console.log('loading...');
	}

	drawMapImage(base64Data, dim) {
		console.log("draw map image...");
	}

	fitViewport() {
		console.log('connected cell zone fit to viewport');
		visFitToViewport()
	}

}

class FuncZoneModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		this.options = options || {};
		this.factory = new ZonePolygonFactory();
		this.routeFactory = new RouteFactory();
		this.cellFactory = new CellFactory();
	}

	init() {
		visCanvas.setMode('rdFuncZone');

		// --- run the procedure ---
		this.canvas._objects.forEach((o) => {
			o.opacity = (o.layer === 'rdFuncZone') ? 1.0 : 0.3;
			o.selectable = (o.layer === 'rdFuncZone') ? true : false;
			return o;
		});

		// --- enable zoom and pan ---
		this.fitViewport();
		const zoom = this.canvas.width / gMapData_.w;
		const tool = new PanZoom(this.canvas);
		tool.setMinZoomRatio(zoom);

		this.canvas.renderAll();
	}

	renderObjects(objects) {
		// --- clear the objects initially ---
		// console.log(this.canvas.getObjects());
		this.canvas.remove(...this.canvas.getObjects());

		// --- draw routes ---
		const routeAllData = routeDM.getAllData();
		console.log(routeAllData);
		routeAllData.edges.forEach((edge) => {
			const fromNode = _.find(routeAllData.nodes, { id: edge.from });
			const toNode = _.find(routeAllData.nodes, { id: edge.to });
			if (fromNode && toNode) {
				const edgeObj = { fromX: fromNode.x, fromY: fromNode.y, toX: toNode.x, toY: toNode.y };
				const routeEdge = this.routeFactory.createEdge(edgeObj);
				this.canvas.add(routeEdge);
			}
			this.canvas.renderAll();
		});
		routeAllData.nodes.forEach((node) => {
			const routeNode = this.routeFactory.createNode(node);
			this.canvas.add(routeNode);
			this.canvas.renderAll();
		});
		// --- draw cells ---
    const cellDM = CellDataManager.getInstance(visNetwork_);
		const cellAllData = cellDM.getAllData();
		cellAllData.forEach((cellData) => {
			const cell = this.cellFactory.create(cellData);
			this.canvas.add(cell);
			this.canvas.renderAll();
		});

		if (objects.length === 0) { return; }
		objects.forEach((obj) => {
			if (!obj.hasOwnProperty('points') || obj.points.length === 0) { return; }
			const zone = this.factory.create(obj);
			this.canvas.add(zone);
			this.canvas.renderAll();
		});
	}

	drawMapImage(base64Data, dim) {
		let img = new Image();
		const mapDataURL = `data:image/png;base64,${base64Data}`;
		img.src = mapDataURL;
		const inst = this;

		img.onload = function () {
			const maxDim = Math.max(img.width, img.height);
			const zoom = (maxDim >= dim) ? (dim / maxDim) : 1.0;
			const viewWidth = img.width * zoom;
			const viewHeight = img.height * zoom;

			const fImg = new fabric.Image(img);

			inst.canvas.setWidth(cvsViewerDim_.width);
			inst.canvas.setHeight(cvsViewerDim_.height);
			inst.canvas.setZoom(zoom);

			inst.canvas.setBackgroundImage(fImg).renderAll();
		};
	}

	fitViewport() {
		console.log('function zone fit to viewport');
		const cvsContent = document.getElementById("fabric-canvas-container"); // card-body width
		const maxScale = 1.0;
		const calcScale = cvsContent.clientWidth / gMapMeta_.w;
		const scale = (calcScale > maxScale) ? maxScale : calcScale;
		this.canvas.setViewportTransform([scale, 0, 0, scale, 0, 0]);
	}
}

class ReflectorModeCanvas {
	constructor(canvas, options) {
		this.canvas = canvas;
		this.options = options || {};
		this.factory = new ReflectorMarkerFactory();
	}

	init() {
		visCanvas.setMode('rdReflector');

		// --- run the procedure ---
		this.canvas._objects.forEach((o) => {
			o.opacity = (o.layer === 'rdReflector') ? 1.0 : 0.3;
			o.selectable = (o.layer === 'rdReflector') ? true : false;
			return o;
		});

		// --- enable zoom and pan ---
		this.fitViewport();
		const zoom = this.canvas.width / gMapData_.w;
		const tool = new PanZoom(this.canvas);
		tool.setMinZoomRatio(zoom);

		this.canvas.renderAll();
	}

	renderObjects(objects) {
		const inst = this;
		const reflectors = objects;
		console.log(reflectors);
		// --- clear the objects initially ---
		this.canvas.remove(...inst.canvas.getObjects());

		if (!reflectors?.length) { return; }
		reflectors.forEach((rf) => {
			// console.log(rf);
			const marker = inst.factory.create(rf.type, { id: rf.id, center: rf.cvsCenter });
			// console.log(marker);
			inst.canvas.add(marker);
		});
		inst.canvas.renderAll();
	}

	drawMapImage(base64Data, dim) {
		let img = new Image();
		const mapDataURL = `data:image/png;base64,${base64Data}`;
		img.src = mapDataURL;
		const inst = this;

		img.onload = function () {
			const maxDim = Math.max(img.width, img.height);
			const zoom = (maxDim >= dim) ? (dim / maxDim) : 1.0;
			const viewWidth = img.width * zoom;
			const viewHeight = img.height * zoom;

			let fImg = new fabric.Image(img);

			inst.canvas.setWidth(cvsViewerDim_.width);
			inst.canvas.setHeight(cvsViewerDim_.height);
			inst.canvas.setZoom(zoom);

			inst.canvas.setBackgroundImage(fImg).renderAll();
		};
	}

	fitViewport() {
		const cvsContent = document.getElementById("fabric-canvas-container"); // card-body width 
		const maxScale = 1.0;
		const calcScale = cvsContent.clientWidth / gMapMeta_.w;
		const scale = (calcScale > maxScale) ? maxScale : calcScale;
		this.canvas.setViewportTransform([scale, 0, 0, scale, 0, 0]);
	}
}

/**
 * make fabric canvas respond to window dimensions changes
 */
$(window).resize(() => {
	// --- responsive window ---
	const selLayer = getSelectedLayer();
	if (['rdRoute', 'rdCell', 'rdConnCell'].includes(selLayer)) {
		const canvas = visNetwork_.canvas.frame.canvas;
		const containerWidth = canvas.clientWidth;
		const containerHeight = canvas.clientHeight;

		cvsViewerDim_.width = containerWidth;
		cvsViewerDim_.height = containerHeight;
		return;
	}

	const outerCanvasContainer = $('#fabric-canvas-container')[0];
	const containerWidth = outerCanvasContainer.clientWidth;
	const containerHeight = outerCanvasContainer.clientHeight;

	const ratio = fCanvas.getWidth() / fCanvas.getHeight();
	const scale = containerWidth / fCanvas.getWidth();
	const zoom = fCanvas.getZoom() * scale;

	fCanvas.setDimensions({ width: containerWidth, height: containerWidth / ratio });
	fCanvas.setViewportTransform([zoom, 0, 0, zoom, 0, 0]);

	cvsViewerDim_.width = containerWidth;
	cvsViewerDim_.height = containerHeight;
});
