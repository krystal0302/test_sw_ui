const ZONE_POINT_PREFIX = 'p-';
const ZONE_LINE_PREFIX = 'l-';
const ZONE_POLYGON_PREFIX = 'z-';

let selShape;
let pointArray = [];
let oldPoints = [];
let newPoints = [];
let vertexIdx;
let editVertexIdx;
let beforeEditPoints = [];
let isDrawZoneMode = false;
let isAddVertexMode = false;

// =========================================
//   Function Zone Layer UI Event-bindings    
// =========================================
const zonesDeck = document.getElementById('zones-deck');
const createZoneBtn = document.getElementById('create-zone-btn');
const deleteZoneBtn = document.getElementById('delete-zone-btn');
const saveZoneBtn = document.getElementById('save-changes');
const panZoomZoneBtn = document.getElementById('pan-zoom-zone');
const addZonePolygon = document.getElementById('add-zone-polygon');
const editZoneVertex = document.getElementById('edit-zone-vertex');
const addZoneVertex = document.getElementById('add-zone-vertex');
const endAddZoneVertex = document.getElementById('end-zone-vertex');
const delZoneVertex = document.getElementById('delete-zone-vertex');
const delZonePolygon = document.getElementById('delete-zone-polygon');
const cancelZoneEdit = document.getElementById('cancel-zone-edit');
const undoZone = document.getElementById('undo-zone');
const openZoneConfigs = document.getElementById('open-zone-configs');

// --- configurations ---
const zoneNameInput = document.getElementById('zone-name');
const zoneTypeSelect = document.getElementById('zone-type-select');
const zonePriorSelect = document.getElementById('zone-priority-select');
const modelFilterSelect = document.getElementById('model-filter-select');

// --- event bindings ---
createZoneBtn.addEventListener('click', () => {
	// --- [protection] no zone created ---
	// let emptyZoneId;
	// const data = zoneDM.getAllData();
	// data.forEach(zone => {
	// 	const obj = getFabricZoneObject(zone.id);
	// 	if (obj) { return; }
	// 	alert(`${zone.name} not created on map!`);
	// 	emptyZoneId = zone.id;
	// });
	// if (emptyZoneId) {
	// 	const switchNode = document.querySelector(`#zone-container-${emptyZoneId} .activation-switch`);
	// 	if (!switchNode || switchNode.checked) return;
	// 	document.getElementById(`zone-container-${emptyZoneId}`).click();
	// 	return;
	// }

	restoreToolButtonState();
	// --- toggle create and cancel create zone ---
	if (createZoneBtn.textContent === 'Create new zone') {
		createZoneBtn.textContent = 'Cancel create new zone';
	} else {
		restoreCreateZoneButton();
		const id = zoneDM.getDataId();
		removeZoneCache(id);
		closeZoneEditor();
		isDrawZoneMode = false;
		return;
	}

	clearNodePopUp();
	restoreUnsavedFabricZone();
	removeUnsavedFabricZones();
	removeDeleteZoneButton();
	resetSelectedPolygonPoints();
	resetSelectedZoneCard();

	resetZoneEditor();
	toggleZoneEditTabButtons();
	toggleZoneEditorState(false);
	openZoneEditor();

	// --- update zone cache data ---
	zoneDM.add();
	// --- clear the dataCache used by undo ---
	zoneDM.clearCache();

	// TODO: bind the evnets for create zone polygon 
});

panZoomZoneBtn.addEventListener('click', () => {
	const zoom = fCanvas.width / gMapData_.w;
	const tool = new PanZoom(fCanvas);
	tool.setMinZoomRatio(zoom);
});

addZonePolygon.addEventListener('click', () => {
	const tool = new AddZonePolygonTool(fCanvas, zoneDM);
	console.log(' activate zone polygon mode ');

	isDrawZoneMode = true;
});

editZoneVertex.addEventListener('click', () => {
	const tool = new EditZoneVertexTool(fCanvas);
	console.log(' --- edit zone vertex --- ');

	drawFabricEditingZonePoints();
	clearNodePopUp();
	resetAddZoneVertexParams();
	toggleZoneEditButtonState('edit');
});

addZoneVertex.addEventListener('click', () => {
	const tool = new AddZoneVertexTool(fCanvas);
	console.log(' --- add zone vertex --- ');

	isAddVertexMode = true;
	drawFabricEditingZonePoints();
	clearNodePopUp();
	toggleZoneEditButtonState('add');
});

endAddZoneVertex.addEventListener('click', () => {
	addZoneVertex.style.display = "block";
	addZoneVertex.disabled = false;
	endAddZoneVertex.style.display = "none";
	cancelZoneEdit.disabled = false;
	saveZoneBtn.disabled = false;

	if (vertexIdx === undefined) return;
	// --- re-render zone on canvas ---
	let type;
	let priority;
	const id = zoneDM.getDataId();
	const zone = zoneDM.getTarget(id);
	if (!isEmpty(zone)) {
		type = zone.type;
		priority = zone.priority;
	} else {
		type = zoneTypeSelect.selectedIndex;
		priority = zonePriorSelect.selectedIndex;
	}
	selShape.set({
		fill: ZONE_TYPES[type].fillColor,
		opacity: priority === 0 ? 0.3 : 0.8
	});

	removeFabricZoneObjects(selShape.id);
	drawFabricZone(id);
	removeFabricZoneObjects(ZONE_POINT_PREFIX);
	drawFabricSelectedZonePoints(selShape);

	vertexIdx = undefined;
	oldPoints = [];
	newPoints = [];
});

delZoneVertex.addEventListener('click', () => {
	const tool = new DelZoneVertexTool(fCanvas);
	console.log(' --- delete zone vertex --- ');

	drawFabricEditingZonePoints();
	clearNodePopUp();
	resetAddZoneVertexParams();
	toggleZoneEditButtonState('delete');
});

delZonePolygon.addEventListener('click', () => {
	const tool = new DelZonePolygonTool(fCanvas);
	console.log(' --- delete zone polygon --- ');

	clearNodePopUp();
	resetAddZoneVertexParams();

	// --- remove the objects from canvas ---
	const id = zoneDM.getDataId();
	removeFabricZoneObjects(id);

	// --- flush the cache ---
	const zone = zoneDM.getTarget(id);
	if (!isEmpty(zone)) {
		updateZoneVertices('delete', zone.points);
	}

	toggleZoneEditTabButtons(id);
});

cancelZoneEdit.addEventListener('click', () => {
	const tool = new CancelZoneEditTool(fCanvas);
	console.log(' --- cancel zone edit --- ');

	clearNodePopUp();
	resetSelectedPolygonPoints();
	resetAddZoneVertexParams();
	toggleZoneEditButtonState();
});

undoZone.addEventListener('click', () => {
	zoneDM.undo();
	const uiData = zoneDM.getAllData();
	canvasView.renderObjects(uiData);

	toggleZoneEditTabButtons(zoneDM.getDataId());
});

openZoneConfigs.addEventListener('click', () => {
	const zone = zoneDM.getZone();
	const selConifgs = Object.keys(zone.properties);
	let zone_confs = zoneDM.getConfigs();
	let configKeys = _.difference(Object.keys(zone_confs), selConifgs);
	zone_confs = _.pickBy(zone_confs, function (v, k) {
		return configKeys.includes(k);
	});
	updateZoneConfigOptions(zone_confs);

	$('#zone-config-modal').modal('show');
});

zoneNameInput.addEventListener('input', (e) => {
	const name = e.target.value;
	zoneDM.update({ name: name });
});

zoneTypeSelect.addEventListener('change', (e) => {
	const type = parseInt(e.target.value);
	zoneDM.update({ type: type, properties: {} });

	// --- update zone model filter default options ---
	const model = zoneDM.getModelByType(type);
	$(modelFilterSelect).val(model).change();

	updateZoneSettings(type);
});

zonePriorSelect.addEventListener('change', (e) => {
	const priority = e.target.selectedIndex;
	zoneDM.update({ priority: priority });
});

// ========================
//       Data Classes    
// ========================
// --- backend data scheme ---
/*
{
	map_name    string,
	uuid        string,
	zone_name   string,
	zone_type   number,
	priority    number,
	model       string,
	activate    boolean,
	vertices    object[], object: {x, y, theta}
	properties  object
}
*/

// --- frontend data scheme ---
/*
{
	id          string,
	name        string,
	type        number,
	priority    number,
	model       string,
	activate    boolean,
	points      Point2D[],
	properties  object
}
*/

class ZoneDataManager {
	constructor() {
		this.types = [];
		this.dataId = "";
		this.dataset = [];
		this.dataCache = new FarCache();
	}

	loadFromSystem(data) {
		if (typeof data !== 'object') { return; }
		data.forEach(zone => {
			const props = _.pickBy(zone.properties, (value, key) => value !== null);

			const entity = {
				id: zone.uuid,
				name: zone.zone_name,
				type: zone.zone_type,
				priority: zone.priority,
				model: zone.model,
				activate: zone.activate,
				points: [],
				properties: props
			};

			zone.vertices.map((vtx) => {
				const visPos = tfROS2Canvas(gMapMeta_, { x: vtx.x, y: vtx.y });
				entity.points.push(new Point2D(Number(visPos.x), Number(visPos.y)));
			});
			this.dataset.push(entity);
		});
		console.log(this.dataset);
	}

	loadMetadata(data) {
		if (typeof data !== 'object') { return; }
		for (const [key, value] of Object.entries(data)) {
			const entity = {
				id: key,
				name: key.replace(/_/g, ' '),
				typeString: value.zone_type.default_value || '0',
				modelString: value.model.default_value || 'ALL',
				configs: value
			}
			this.types.push(entity);
		}
		console.log(this.types);
	}

	clearAll() {
		// 1. flush the data
		this.types = [];
		this.dataId = "";
		this.dataset = [];
		// 2. flush the date cache 
		this.dataCache.flush();
	}

	clearDataset() {
		this.dataset = [];
	}

	clearCache() {
		this.dataCache.flush();
	}

	add() {
		const newId = genUuid().replace(/-/g, '');
		const entity = {
			id: newId,
			name: zoneNameInput.value,
			type: zoneTypeSelect.selectedIndex,
			priority: zonePriorSelect.selectedIndex,
			activate: false,
			properties: {}
		};

		this.dataset.push(entity);
		this.setDataId(newId);
	}

	delete(id) {
		this.dataset = this.dataset.filter((d) => d.id !== id);
	}

	update(obj) {
		let target = this.getZone();
		for (const [key, value] of Object.entries(obj)) {
			target[key] = value;
		}
		console.log(target);
	}

	addPolygon(points) {
		// --- add points to cache---
		this.update({ points });

		// --- cache the change ---
		this.dataCache.put(genUuid(), { action: 'add' });
	}

	deletePolygon(points) {
		// --- cache the change ---
		this.dataCache.put(genUuid(), { action: 'delete', points });

		// --- delete points from cache---
		let target = this.getZone();
		delete target.points;
	}

	updatePolygon(points) {
		this.dataCache.put(genUuid(), { action: 'update', points });
	}

	undo() {
		const zoneCache = this.dataCache.popHead();
		const change = zoneCache?.value;
		if (!change) { return; }

		const target = this.getZone();
		switch (change.action) {
			case 'add':
				delete target.points;
				break;
			case 'update':
			case 'delete':
				target.points = (change?.points) ? change.points : target.points;
				break;
		}

		console.log(change);
	}

	isExistUnsavedChanges() {
		return (this.dataCache.getSize() > 0) ? true : false;
	}

	getTarget(id) {
		const target = _.find(this.dataset, { id: id });
		if (target === undefined) {
			return {};
		}
		return target;
	}

	getZone() {
		const id = this.getDataId();
		const target = this.getTarget(id);
		return target;
	}

	getModelByType(type_val) {
		const typeString = type_val === null ? "" : type_val.toString();
		const target = _.find(this.getAllTypes(), { typeString: typeString });
		if (target === undefined) {
			return {};
		}
		return target.modelString;
	}

	getConfigsByType(type_val) {
		const typeString = type_val === null ? "" : type_val.toString();
		let target = _.find(this.getAllTypes(), { typeString: typeString });
		if (target === undefined) {
			return {};
		}
		target.configs = _.omit(target.configs, ['zone_type', 'model']);
		return target.configs;
	}

	getConfigs() {
		const target = this.getZone();
		return this.getConfigsByType(target.type);
	}

	setDataId(id) {
		this.dataId = id;
	}

	getDataId() {
		return this.dataId;
	}

	getAllData() {
		return this.dataset;
	}

	getAllTypes() {
		return this.types;
	}

	getAllNames() {
		return _.map(this.dataset, 'name');
	}

	getDataInSysFormat() {
		const inst = this;
		let cpDataset = JSON.parse(JSON.stringify(this.dataset));
		let sysData = cpDataset.map((zone) => ({
			map_name: document.getElementById('map-select').value,
			uuid: zone.id,
			zone_name: zone.name,
			zone_type: zone.type,
			priority: zone.priority,
			model: zone.model,
			activate: zone.activate,
			vertices: zone.points,
			properties: zone.properties
		}));

		sysData.forEach((data) => {
			const allConfigs = inst.getConfigsByType(data.zone_type);
			let selConfigs = Object.keys(data.properties);
			selConfigs.map(function (conf) {
				let confVal = null;
				if (inst.getDataId() === data.uuid) {
					confVal = document.getElementById(conf).value || '';
					confVal = isEmptyString(confVal) ? null : confVal;
				}
				if (confVal === null) { return; }
				const type = allConfigs[conf].valid_value.data_type;
				if (type === 'double' || type === 'int') {
					confVal = Number(confVal);
				} else if (type === 'bool') {
					confVal = (confVal.toLowerCase() === 'true');
				}
				data.properties[conf] = confVal;
			});
			let diffConfigs = _.difference(Object.keys(allConfigs), selConfigs);
			diffConfigs.map(function (conf) {
				data.properties[conf] = null;
			});

			if (data.vertices === undefined) { return; } // suppose vertices can not be undefined
			data.vertices.map((obj) => {
				let rosPos = tfCanvas2ROS(gMapMeta_, { x: obj.x, y: obj.y });
				obj.x = Number(rosPos.x);
				obj.y = Number(rosPos.y);
				obj.theta = 0;
				return obj;
			});
		});

		return sysData;
	}

	getInvalidMsg() {
		const dupNames = _.uniq(_.filter(this.getAllNames(), (v, i, a) => a.indexOf(v) !== i))
		if (dupNames.length > 0) {
			return `${dupNames.join()} already exists!`;
		}

		for (let i = 0; i < this.dataset.length; i++) {
			const zone = this.dataset[i];
			if (!zone.hasOwnProperty('points')) {
				return `${zone.name} not created on map!`;
			}
			if (zone.type === 0) {
				return `Please select ${zone.name}'s type!`;
			}
			const id = zone.id;
			const pol = zone.points;
			const zoneIds = getPolygonIntersectZones(id, pol);
			const names = getSameZones(zoneIds, zone.type, zone.priority);
			if (names.length > 0) {
				return `${zone.name} can NOT overlap with ${names.join()}!`;
			}
		}
		return "";
	}

	isExistUnsavedChanges() {
		return (this.dataCache.getSize > 0) ? true : false;
	}
}

let zoneDM = new ZoneDataManager();

// ============================
//     Supporting Functions    
// ============================
// --- 1st-tier supporting functions ---
function activeSelectedZone(_selZone) {
	disableFabricObjResizing(_selZone);
	disableFabricObjDragging(_selZone);
	fCanvas.setActiveObject(_selZone);
	fCanvas.bringToFront(_selZone);
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

	const points = _selObj.points;
	points.forEach(point => {
		let fillColor = _selObj.fill;
		if (!isEmpty(_selCoord)) {
			fillColor = _selCoord.x === point.x && _selCoord.y === point.y ? modeColor : _selObj.fill;
		}

		const pointOption = {
			id: _selObj.id.replace(ZONE_POLYGON_PREFIX, ZONE_POINT_PREFIX),
			radius: 7 / fCanvas.getZoom(),
			fill: fillColor,
			stroke: _selObj.stroke,
			strokeWidth: 1,
			left: point.x,
			top: point.y,
			selectable: true,
			hasBorders: false,
			hasControls: false,
			originX: 'center',
			originY: 'center',
			objectCaching: false
		};
		const circle = new fabric.Circle(pointOption);
		fCanvas.add(circle);
	});
}

function drawFabricEditingZonePoints() {
	fCanvas.discardActiveObject().renderAll();
	const id = zoneDM.getDataId();
	if (isEmptyString(id)) return;
	const obj = getFabricZoneObject(id);
	fCanvas.bringToFront(obj);
	drawFabricSelectedZonePoints(obj);
	selShape = obj;
}

function drawFabricZone(_id) {
	const options = {};
	if (selShape.strokeDashArray !== undefined) {
		options.strokeDashArray = selShape.strokeDashArray;
	}
	const polygon = new fabric.Polygon(selShape.points, {
		id: ZONE_POLYGON_PREFIX + _id,
		stroke: selShape.stroke,
		fill: selShape.fill,
		opacity: selShape.opacity,
		objectCaching: false,
		moveable: false,
		selectable: false,
		...options
	});
	fCanvas.add(polygon);
}

function getFabricZoneObject(_id) {
	let obj;
	_id = ZONE_POLYGON_PREFIX + _id;
	fCanvas.forEachObject(function (o) {
		if (o.id && o.id === _id) {
			obj = o;
		}
	});
	return obj;
}

function getPolygonIntersectZones(id, pol) {
	let overlapZoneIds = [];
	fCanvas.forEachObject(function (o) {
		if (!o.hasOwnProperty('id')) return;
		if (!o.id.includes(ZONE_POLYGON_PREFIX)) return;
		const zoneId = o.id.replace(ZONE_POLYGON_PREFIX, '');
		if (id === zoneId) return;
		const intPols = intersectPolygons(pol, o.points);
		if (intPols.length > 0) {
			overlapZoneIds.push(zoneId);
		}
	});
	return overlapZoneIds;
}

function getSameZones(_overlapIds, _type, _priority) {
	let overlapZoneNames = [];
	_overlapIds.forEach((id) => {
		const overlapZone = zoneDM.getTarget(id);
		const isDupType = (overlapZone.type === _type);
		const isDupPriority = (overlapZone.priority === _priority);
		if (isDupType && isDupPriority) {
			overlapZoneNames.push(overlapZone.name);
		}
	});
	return overlapZoneNames;
}

function resetSelectedPolygonPoints() {
	if (selShape === undefined) return;
	fCanvas.discardActiveObject().renderAll();
	removeFabricZoneObjects(ZONE_POINT_PREFIX + selShape.id.replace(ZONE_POLYGON_PREFIX, ''));
	selShape = undefined;
}

function restoreUnsavedFabricZone() {
	if (selShape === undefined || oldPoints.length === 0) return;
	const id = selShape.id.replace(ZONE_POLYGON_PREFIX, '');
	const zone = zoneDM.getTarget(id);
	if (!isEmpty(zone)) {
		const type = zone.type;
		const color = ZONE_TYPES[type].fillColor;
		const priority = zone.priority;
		selShape.set({
			fill: color,
			opacity: priority === 0 ? 0.3 : 0.8,
			points: oldPoints
		});
	}
	removeFabricZoneObjects(selShape.id);
	drawFabricZone(id);
	zoneDM.update({ points: oldPoints });
}

function removeZoneCache(_removeId) {
	//  --- remove zone card from the zone list ---
	if (document.getElementById(`zone-container-${_removeId}`)) {
		document.getElementById(`zone-container-${_removeId}`).remove();
	}
	// --- remove cache data ---
	zoneDM.delete(_removeId);
	// --- remove zone from map ---
	removeFabricZoneObjects(_removeId);
}

function removeFabricZoneObjects(_removeId) {
	let objects = fCanvas.getObjects();
	// --- filter-out object without id ---
	objects = objects.filter((o) => o.hasOwnProperty('id'));
	// console.log(objects);
	objects.forEach(async function (o) {
		if (o.id.includes(_removeId)) {
			// console.log(o.id);
			fCanvas.remove(o);
		}
	});
}

function removeUnsavedFabricZones() {
	fCanvas.getObjects().forEach(async function (o) {
		if (!o.hasOwnProperty('id')) return;
		const data = zoneDM.getAllData();
		const filteredZone = _.filter(data, (zone) => o.id.includes(zone.id));
		if (filteredZone.length > 0) return;
		fCanvas.remove(o);
	});
}

function createDeleteZoneButton(_zone) {
	removeDeleteZoneButton();
	let deleteBtn = document.createElement('button');
	deleteBtn.id = 'delete-zone-btn';
	deleteBtn.setAttribute('class', 'btn btn-default');
	deleteBtn.innerHTML = langTemplateObj_.editor.btn_Delete;
	deleteBtn.disabled = _zone.activate;
	deleteBtn.addEventListener('click', removeZone.bind(this, _zone));
	document.getElementById('delete-btn-placeholder').append(deleteBtn);
}

function removeDeleteZoneButton() {
	const placeholder = document.getElementById('delete-btn-placeholder');
	removeAllChildNodes(placeholder);
}

function restoreCreateZoneButton() {
	createZoneBtn.textContent = 'Create new zone';
}

function restoreToolButtonState() {
	createZoneBtn.disabled = false;
	saveZoneBtn.disabled = false;
}

function initZoneCards() {
	// --- init zone list cards background color ---
	resetSelectedZoneCard();
	// --- init activation switches for each zone ---
	$('.activation-switch').bootstrapSwitch({
		'size': 'mini',
		'onColor': 'success',
		'offColor': 'danger',
		'onSwitchChange': async function (e, state) {
			const selMap = document.getElementById('map-select').value;
			const uuid = e.target.id;
			let res = await fetchPutZoneActivation(rmtToken_, selMap, uuid, state);
			if (res.ok) {
				// restoreInitState();
				// closeZoneEditor();
				// document.getElementById(`zone-container-${uuid}`).style.pointerEvents = state ? "none" : "auto";

				res = await fetchGetZoneConfig(rmtToken_, selMap);
				res = await res.json();

				let stateText = state ? $(e.target).attr('data-on-text') : $(e.target).attr('data-off-text');
				stateText = stateText.replace(/\s/g, '').toUpperCase();

				const index = _.findIndex(res, { uuid: uuid });
				if (index === -1) { return; }
				if (res[index].activate !== state) {
					$(e.target).bootstrapSwitch('state', res[index].activate);
					alert(`${stateText} FAILED!`);
				} else {
					zoneDM.setDataId(uuid);
					zoneDM.update({ activate: state });

					// --- reopen editor if it was expanded ---
					if (document.getElementById('zone-editor').style.display === 'block') {
						closeZoneEditor();
						document.getElementById(`zone-container-${uuid}`).click();
					}

					if (!document.getElementById('delete-zone-btn')) { return; }
					document.getElementById('delete-zone-btn').disabled = state;
				}
			}
		}
	});
	const activationSwitches = document.querySelectorAll('.bootstrap-switch');
	activationSwitches.forEach((activationSwitch) => {
		// activationSwitch.style.pointerEvents = "auto";
		activationSwitch.style.borderColor = "gray";
	});
}

function resetSelectedZoneCard() {
	const isDarkMode = getSavedTheme() === 'dark';
	const zoneContainers = document.querySelectorAll(".zone-container");
	zoneContainers.forEach((container) => {
		container.style.backgroundColor = isDarkMode ? '#343a40' : 'lightgray';
	});
}

function resetZoneEditor() {
	zoneTypeSelect.selectedIndex = 0;
	zoneNameInput.value = "Zone";
	zonePriorSelect.selectedIndex = 0;
	$(modelFilterSelect).val('').change();

	updateZoneSettings();
}

function openZoneEditor() {
	document.getElementById('zone-editor').style.display = "block";
}

function closeZoneEditor() {
	document.getElementById('zone-editor').style.display = "none";
}

function restoreInitState() {
	clearNodePopUp();
	restoreToolButtonState();
	restoreUnsavedFabricZone();
	removeUnsavedFabricZones();
	restoreCreateZoneButton();
	removeDeleteZoneButton();
	resetSelectedPolygonPoints();
	resetSelectedZoneCard();
	resetAddZoneVertexParams();
	resetCreateZoneParams();
}

function editZoneEditor(_zone) {
	// --- [protection] no zone created ---
	let emptyZoneId;
	const data = zoneDM.getAllData();
	data.forEach((zone) => {
		if (zone.name === _zone.name) return;
		const obj = getFabricZoneObject(zone.id);
		if (obj) { return; }
		alert(`${zone.name} not created on map!`);
		emptyZoneId = zone.id;
	});
	if (emptyZoneId) {
		const switchNode = document.querySelector(`#zone-container-${emptyZoneId} .activation-switch`);
		if (!switchNode || switchNode.checked) return;
		document.getElementById(`zone-container-${emptyZoneId}`).click();
		return;
	}

	clearNodePopUp();
	restoreToolButtonState();
	restoreUnsavedFabricZone();
	removeUnsavedFabricZones();
	resetSelectedPolygonPoints();
	restoreCreateZoneButton();
	zoneDM.setDataId(_zone.id);
	// --- clear the dataCache used by undo ---
	zoneDM.clearCache();

	// --- zone list ---
	resetSelectedZoneCard();
	const isDarkMode = getSavedTheme() === 'dark';
	this.style.backgroundColor = isDarkMode ? '#4b545c' : 'rgba(52, 58, 64, 0.5)';

	// --- zone editor ---
	zoneTypeSelect.selectedIndex = _zone.type || 0;
	zoneNameInput.value = _zone.name;
	zonePriorSelect.selectedIndex = _zone.priority || 0;
	const models = _zone.model.split(';');
	$(modelFilterSelect).val(models).change();

	// --- zone settings ---
	const type = zoneTypeSelect.value;
	updateZoneSettings(type);

	let configDeck = document.getElementById('edit-config-deck');
	for (const [key, value] of Object.entries(_zone.properties)) {
		const defs = zoneDM.getConfigs()[key];
		const node = createZoneConfigRow(key, value, defs);
		configDeck.append(node);
	}

	// --- add delete button to footer ---
	createDeleteZoneButton(_zone);

	toggleZoneEditTabButtons(_zone.id);
	toggleZoneEditorState(_zone.activate);
	openZoneEditor();
}

function resetCreateZoneParams() {
	lineArray = [];
	pointArray = [];
	activeLine = null;
	activeShape = null;
	isDrawZoneMode = false;
}

function resetAddZoneVertexParams() {
	isAddVertexMode = false;
	vertexIdx = undefined;
	oldPoints = [];
	newPoints = [];
}

// --- 2nd-tier supporting functions ---
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

function updateZoneTypeSelect(_zoneType) {
	zoneTypeSelect.selectedIndex = _zoneType || 0;
}

function updateZoneModelFilter(_zoneModels) {
	const models = _zoneModels.split(';');
	$(modelFilterSelect).val(models).change();
}

function updateZoneSettings(_zoneType = null) {
	// --- reset zone settings ---
	let configDeck = document.getElementById('edit-config-deck');
	removeAllChildNodes(configDeck);

	const zone_confs = zoneDM.getConfigsByType(_zoneType);
	const bEmptyZone = isEmpty(zone_confs);
	openZoneConfigs.disabled = bEmptyZone;
	const toggleCollapse = (bEmptyZone) ? 'collapse' : 'expand';
	$('#zone-params-card').CardWidget(toggleCollapse);
}

function updateZoneConfigOptions(_configs) {
	let ul = document.getElementById('available-items-list');
	removeAllChildNodes(ul);

	for (const [key, value] of Object.entries(_configs)) {
		const node = createZoneItemView(key, value);
		ul.appendChild(node);
	}

	document.getElementById("available-items-title").textContent = langTemplateObj_.modal.ttl_AddConfig;
}

function updateZoneVertices(_type, _vertices = []) {
	switch (_type) {
		case 'add':
			zoneDM.addPolygon(_vertices);
			break;
		case 'update':
			zoneDM.updatePolygon(_vertices);
			break;
		case 'delete':
			zoneDM.deletePolygon(_vertices);
			break;
	}
}

function removeZone(_zone) {
	const name = _zone.name;
	if (confirm(`Are you sure to delete the zone: ${name} ?`)) {
		const id = _zone.id;
		removeZoneCache(id);
		restoreInitState();
		closeZoneEditor();
	}
}

function toggleZoneEditorState(isActivate) {
	zoneTypeSelect.disabled = isActivate;
	zonePriorSelect.disabled = isActivate;
	modelFilterSelect.disabled = isActivate;
	openZoneConfigs.disabled = isActivate;

	document.querySelectorAll('#draw-zone > ul > li > button:not([style*="display: none"])').forEach(btn => {
		if (btn.id === 'pan-zoom-zone') { return; }
		btn.disabled = isActivate
	});
	document.querySelectorAll('#edit-config-deck .input-group-append, #edit-config-deck .remove-config').forEach(el => {
		if (!isActivate) { return; }
		el.parentNode.removeChild(el);
	});

	if (!isActivate) {
		if (document.getElementById('edit-name-switch')) { return };
		const inputAppend = document.createElement('div');
		inputAppend.classList.add('input-group-append');
		inputAppend.id = 'edit-name-switch';
		const editBtn = document.createElement('div');
		editBtn.classList.add('input-group-text');
		editBtn.innerHTML = '<i class="fas fa-pen"></i>';
		inputAppend.append(editBtn);
		document.getElementById('zone-name-group').append(inputAppend);

		const switchEditBtn = document.getElementById('edit-name-switch');
		switchEditBtn.addEventListener('click', editButtonSwitch.bind(switchEditBtn.parentElement));
	} else {
		if (!document.getElementById('edit-name-switch')) { return };
		document.getElementById('edit-name-switch').remove();
	}

}

function toggleZoneEditTabButtons(id) {
	let bHasVertexCache = false;
	const zone = zoneDM.getTarget(id);
	if (!isEmpty(zone)) {
		bHasVertexCache = zone.hasOwnProperty('points');
	}
	addZonePolygon.style.display = bHasVertexCache ? "none" : "block";
	editZoneVertex.style.display = bHasVertexCache ? "block" : "none";
	addZoneVertex.style.display = bHasVertexCache ? "block" : "none";
	endAddZoneVertex.style.display = "none";
	delZoneVertex.style.display = bHasVertexCache ? "block" : "none";

	resetAddZoneVertexParams();
	toggleZoneEditButtonState();
}

function toggleZoneEditButtonState(_mode = 'default') {
	cancelZoneEdit.style.display = "block";
	cancelZoneEdit.disabled = false;
	panZoomZoneBtn.disabled = true;
	switch (_mode) {
		case 'delete':
			editZoneVertex.disabled = true;
			delZoneVertex.disabled = false;
			addZoneVertex.disabled = true;
			break;
		case 'edit':
			editZoneVertex.disabled = false;
			delZoneVertex.disabled = true;
			addZoneVertex.disabled = true;
			break;
		case 'add':
			editZoneVertex.disabled = true;
			delZoneVertex.disabled = true;
			addZoneVertex.disabled = false;
			break;
		case 'default':
			editZoneVertex.disabled = false;
			delZoneVertex.disabled = false;
			addZoneVertex.disabled = false;
			panZoomZoneBtn.disabled = false;
			cancelZoneEdit.style.display = "none";
			break;
	}
}

function toggleToolbarEditing(_status = true) {
	zoneTypeSelect.disabled = !_status;
	zonePriorSelect.disabled = !_status;
	panZoomZoneBtn.disabled = !_status;
	delZonePolygon.disabled = !_status;
	undoZone.disabled = !_status;
	saveZoneBtn.disabled = !_status;
	createZoneBtn.disabled = !_status;
}

function editButtonSwitch() {
	let inputNode = this.querySelector('.form-control');
	let btnIconNode = this.querySelector('.fas');

	inputNode.readOnly = !inputNode.readOnly;

	// --- mount input validation mechanism ---
	if (!inputNode.readOnly) {
		validateZoneInputEvent(inputNode);
	}

	btnIconNode.classList.toggle("fa-pen");
	btnIconNode.classList.toggle("fa-eye");
}

function boolButtonSwitch() {
	let inputNode = this.querySelector('.form-control');
	let btnNode = this.querySelector('.btn');

	const inputBoolVal = inputNode.value.toLowerCase() == "true";
	inputNode.value = !inputBoolVal;

	let boolString = inputBoolVal.toString();
	boolString = boolString.charAt(0).toUpperCase() + boolString.slice(1);
	btnNode.innerHTML = `Set ${boolString}`;
}

// --- template functions ---
function createZoneTypeOptions(_types) {
	const defaultOpt = $("#zone-type-select option[value='0").detach();
	$("#zone-type-select").empty().append(defaultOpt);

	_types.forEach(type => {
		let option = document.createElement('option');
		option.value = type.typeString;
		option.text = type.name;
		zoneTypeSelect.append(option);
	});
}

function createZoneModelOptions(_models) {
	$(modelFilterSelect).empty();
	$(modelFilterSelect).select2({
		width: '100%'
	});

	let option = document.createElement("option");
	option.text = option.value = 'ALL';
	modelFilterSelect.options.add(option);

	_models.forEach((model) => {
		let option = document.createElement("option");
		option.text = option.value = model;
		modelFilterSelect.options.add(option);
	});

	$(modelFilterSelect).on("change.select2", function (e) {
		const selModels = $(this).val();
		// console.log(selModels);
		zoneDM.update({ model: selModels.join(';') });
	});

	$(modelFilterSelect).on('select2:select', function (e) {
		const selOpt = e.params.data.id;
		if (selOpt === 'ALL') {
			// --- only select all ---
			$(this).val('ALL').change();
		} else {
			let values = $(this).val();
			const i = values.indexOf('ALL');
			if (values.length > 1 && i !== -1) {
				values.splice(i, 1);
				// --- deselect all ---
				$(this).val(values).change();
			}

			if (values.length === _models.length) {
				// --- only select all ---
				$(this).val('ALL').change();
			}
		}
	})
}

function createZoneCardView(_zone) {
	const template = document.querySelector('#zone-card');
	const node = document.importNode(template.content, true);

	let cardNode = node.querySelector('.card');
	cardNode.setAttribute('id', `zone-container-${_zone.id}`);
	cardNode.addEventListener('click', editZoneEditor.bind(cardNode, _zone));
	// cardNode.style.pointerEvents = _zone.activate ? 'none' : 'auto';

	let switchNode = node.querySelector('.activation-switch');
	switchNode.id = _zone.id;
	switchNode.checked = _zone.activate;

	const isDarkMode = getSavedTheme() === 'dark';
	const type = ZONE_TYPES[_zone.type] || 'none';
	const suffix = isDarkMode ? '-light' : '';
	let imgNode = node.querySelector('.zone-type-img');
	imgNode.src = `${getImagePath()}/zones/${type.imgName}${suffix}.png`;

	let titleNode = node.querySelector('.zone-option-name');
	titleNode.textContent = _zone.name;

	return node;
}

function createZoneItemView(_configName, _defs) {
	const template = document.querySelector('#zone-config-item');
	const node = document.importNode(template.content, true);

	let topNode = node.querySelector('.item');
	topNode.addEventListener('click', enrollConfigToZone.bind(this, _configName, _defs));

	const scaleFontSize = getFontScaleSize(16);
	let titleNode = node.querySelector('.product-title');
	titleNode.style.fontSize = `${scaleFontSize}px`;
	titleNode.textContent = _configName;

	return node;
}

function createZoneConfigRow(_configName, _configVal, _defs) {
	const template = document.querySelector('#edit-config-row');
	const node = document.importNode(template.content, true);

	const rowNode = node.querySelector('.row');
	let labelNode = node.querySelector('.config-label > span');
	labelNode.textContent = _configName;

	const desc = _defs.description || '';
	const range = _defs.valid_value.data_range || '';
	const rangeTranStr = range.replace(/[[\]]/g, '').replace(':', '~');
	const unit = _defs.unit || '';
	const type = _defs.valid_value.data_type;

	if (!isEmptyString(desc) || !isEmptyString(rangeTranStr) || !isEmptyString(unit)) {
		const descStr = !isEmptyString(desc) ? `${langTemplateObj_.editor.ttp_Desc}: ${desc}\n` : '';
		const rangeStr = !isEmptyString(rangeTranStr) ? `\n${langTemplateObj_.editor.ttp_Range}: ${rangeTranStr}\n` : '';
		const unitStr = !isEmptyString(unit) ? `\n${langTemplateObj_.editor.ttp_Unit}: ${unit}` : '';
		let infoNode = node.querySelector('.custom-tooltip');
		infoNode.title = descStr + rangeStr + unitStr;
		infoNode.style.display = 'block';
	}

	let inputNode = node.querySelector('.form-control');
	inputNode.id = _configName;

	let btnEditNode = node.querySelector('.config-edit');
	if (type === 'bool') {
		let inputVal, boolString;
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
		let inputGroupAppend = node.querySelector('.input-group-append');
		let btn = document.createElement('button');
		btn.innerHTML = `Set ${boolString}`;
		btn.classList.add('btn', 'btn-sm', 'btn-secondary');
		inputGroupAppend.appendChild(btn);
		btn.addEventListener('click', boolButtonSwitch.bind(inputNode.parentElement));
	} else {
		inputNode.value = _configVal;
		btnEditNode.addEventListener('click', editButtonSwitch.bind(inputNode.parentElement));
	}

	let btnRemoveNode = node.querySelector('.remove-config');
	btnRemoveNode.addEventListener('click', removeConfigfromZone.bind(rowNode, _configName));
	return node;
}

function enrollConfigToZone(_configName, _defs) {
	let configDeck = document.getElementById('edit-config-deck');
	const configVal = _defs.default_value === 'null' ? null : _defs.default_value || '';
	let node = createZoneConfigRow(_configName, configVal, _defs);
	configDeck.append(node);

	// --- update zone cache data ---
	let zone = zoneDM.getZone();
	zone.properties[_configName] = configVal;

	const all_confs = zoneDM.getConfigs();
	const selConifgs = Object.keys(zone.properties);
	let configKeys = _.difference(Object.keys(all_confs), selConifgs);
	const configs = _.pickBy(all_confs, function (v, k) {
		return configKeys.includes(k);
	});
	updateZoneConfigOptions(configs);
}

function removeConfigfromZone(_configName) {
	this.remove();
	let zone = zoneDM.getZone();
	delete zone.properties[_configName];
}

// ========================
//       Tool Classes    
// ========================
// ------ Zone Polygon ------ 
let lineArray = [];
let activeShape;
let activeLine;

class AddZonePolygonTool extends Tool {
	constructor(canvas, dm) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.g = new Geometry();
		this.dataManager = dm;
		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = false;

		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});
		inst.canvas.on('mouse:move', function (opt) {
			inst.#onMouseMove(opt);
		});
		// inst.canvas.on('mouse:up', function (opt) {
		// 	inst.#onMouseUp(opt);
		// });
	}

	#onMouseDown(opt) {
		const inst = this;
		const evt = opt.e;

		// --- tool specific ---
		clearNodePopUp();
		if (!isDrawZoneMode) return;
		console.log('now is the canvas drawing mode');
		// --- click on toolbar 'Draw Zone' ---
		// --- [protection] repeat drawing / invalid zone type ---
		const zone_type = zoneTypeSelect.selectedIndex;
		if (zone_type === 0) {
			alert('Please select zone type first!');
			return;
		}

		// --- add zone polygon ---
		if (pointArray.length > 0 && opt.target && opt.target.id === pointArray[0].id) {
			// --- finish drawing ---
			if (pointArray.length < 3) {
				alert('Can NOT add a zone under 3 points!');
				inst.canvas.selection = false;
			} else {
				// --- check for overlapping zones(boundary level) ---
				const p1 = pointArray[pointArray.length - 1];
				const p2 = pointArray[0];
				const zoneIds = this.#getLineIntersectZones(p1, p2);
				const isOverlapping = this.#isIntersectSameZone(zoneIds);
				if (isOverlapping) {
					alert('Same zone type and priority can NOT overlap!');
					return;
				}
				this.#generatePolygonZone();
				toggleToolbarEditing();
			}
		} else {
			// --- check for overlapping zones(vertex level) ---
			const clickPoint = opt.absolutePointer;
			const x = clickPoint.x;
			const y = clickPoint.y;
			const p = new Point2D(x, y);
			const zoneIds = this.#getVertexIntersectZones(p);
			const isOverlapping = this.#isIntersectSameZone(zoneIds);
			if (isOverlapping) {
				alert('Same zone type and priority can NOT overlap!');
				return;
			}
			this.#addPoint(opt);
		}

	}

	#onMouseMove(opt) {
		const inst = this;
		const evt = opt.e;

		if (!isDrawZoneMode) { return; }

		if (activeLine && activeLine.class === 'line') {
			const pointer = inst.canvas.getPointer(evt);
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
		inst.canvas.renderAll();
	}

	// #onMouseUp(opt) {
	// 	let inst = this;
	// 	inst.canvas.selection = true;
	// }

	// --- in-use ---
	#generatePolygonZone() {
		const inst = this;
		let points = [];
		for (const point of pointArray) {
			points.push({
				x: point.left,
				y: point.top,
			});
			inst.canvas.remove(point);
		}

		// --- remove lines from canvas ---
		for (const line of lineArray) {
			inst.canvas.remove(line);
		}

		// --- remove selected shape ---
		inst.canvas.remove(activeShape);

		// --- create polygon from collected points ---
		const id = this.dataManager.getDataId();
		const selType = zoneTypeSelect.selectedIndex;
		const selPrio = zonePriorSelect.selectedIndex;
		const type = ZONE_TYPES[selType] || 'none';
		const color = type.fillColor || 'gray';
		const polygon = new fabric.Polygon(points, {
			id: ZONE_POLYGON_PREFIX + id,
			stroke: color,
			strokeWidth: 3,
			fill: color,
			opacity: selPrio === 0 ? 0.3 : 0.8,
			objectCaching: false,
			moveable: false,
			selectable: false
		});
		inst.canvas.add(polygon);

		// --- save vertices to zone cache ---
		updateZoneVertices('add', points);

		//  --- add zone card to the zone list ---
		if (!document.getElementById(`zone-container-${id}`)) {
			const zone = this.dataManager.getZone();
			const node = createZoneCardView(zone);
			zonesDeck.append(node);
			initZoneCards();
		}

		toggleZoneEditTabButtons(id);

		// --- close draw mode ---
		resetCreateZoneParams();
		inst.canvas.selection = true;
	}

	// --- in-use ---
	#addPoint(opt) {
		const inst = this;
		const evt = opt.e;

		toggleToolbarEditing(false);
		const id = this.dataManager.getDataId();
		const selType = zoneTypeSelect.selectedIndex;
		const type = ZONE_TYPES[selType] || 'none';
		const color = type.pointColor || 'black';
		const zoom = inst.canvas.getZoom();
		const clickPoint = opt.absolutePointer;
		const pointOption = {
			id: ZONE_POINT_PREFIX + id,
			radius: 5 / zoom,
			fill: color,
			stroke: color,
			strokeWidth: 1,
			left: clickPoint.x,
			top: clickPoint.y,
			selectable: false,
			hasBorders: false,
			hasControls: false,
			originX: 'center',
			originY: 'center',
			objectCaching: false
		};
		let point = new fabric.Circle(pointOption);

		// --- fill first point with highlight color ---
		if (pointArray.length === 0) {
			point.set({
				radius: 7 / zoom,
				fill: 'red'
			});
		} else {
			// --- check for overlapping zones(boundary level) ---
			const p1 = pointArray[pointArray.length - 1];
			const p2 = point;
			const zoneIds = this.#getLineIntersectZones(p1, p2);
			const isOverlapping = this.#isIntersectSameZone(zoneIds);
			if (isOverlapping) {
				alert('Same zone type and priority can NOT overlap!');
				inst.canvas.selection = false;
				return;
			}
		}
		inst.canvas.add(point);
		pointArray.push(point);

		// --- add a new line with the same start and end point ---
		const p = { x: clickPoint.x, y: clickPoint.y };
		const linePoints = [p.x, p.y, p.x, p.y];

		const lineOption = {
			id: ZONE_LINE_PREFIX + id,
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
		let line = new fabric.Line(linePoints, lineOption);
		line.class = 'line';
		inst.canvas.add(line);
		activeLine = line;
		lineArray.push(line);

		const polygonOption = {
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
		if (activeShape) {
			let points = activeShape.get('points');
			points.push({
				x: clickPoint.x,
				y: clickPoint.y
			});
			const polygon = new fabric.Polygon(points, polygonOption);
			inst.canvas.remove(activeShape);
			inst.canvas.add(polygon);
			inst.canvas.renderAll();
			activeShape = polygon;
		} else {
			const polyPoint = [{
				x: clickPoint.x,
				y: clickPoint.y,
			},];
			const polygon = new fabric.Polygon(polyPoint, polygonOption);
			inst.canvas.add(polygon);
			activeShape = polygon;
		}
	}

	// --- in-use ---
	#getVertexIntersectZones(p) {
		const inst = this;
		let overlapZoneIds = [];
		inst.canvas.forEachObject(function (o) {
			if (!o.hasOwnProperty('id')) return;
			if (!o.id.includes(ZONE_POLYGON_PREFIX) || !inst.g.pointInPolygon(o.points, o.points.length, p)) return;
			overlapZoneIds.push(o.id.replace(ZONE_POLYGON_PREFIX, ''));
		});
		return overlapZoneIds;
	}

	#getLineIntersectZones(p1, p2) {
		const inst = this;
		let overlapZoneIds = [];
		let isIntersect = false;
		inst.canvas.forEachObject(function (o) {
			if (!o.hasOwnProperty('id')) return;
			if (!o.id.includes(ZONE_POLYGON_PREFIX)) return;
			isIntersect = false;
			const edges = inst.g.polygonLines(o.points);
			const drawFromPoint = new Point2D(p1.left, p1.top);
			const drawToPoint = new Point2D(p2.left, p2.top);
			const drawLine = new Line2D(drawFromPoint, drawToPoint);
			for (let i = 0; i < edges.length; i++) {
				isIntersect = inst.g.lineIntersectLine(drawLine, edges[i]);
				if (isIntersect) break;
			}
			if (isIntersect) {
				overlapZoneIds.push(o.id.replace(ZONE_POLYGON_PREFIX, ''));
			}
		});
		return overlapZoneIds;
	}

	#isIntersectSameZone(_overlapIds) {
		const selType = zoneTypeSelect.selectedIndex;
		const selPrio = zonePriorSelect.selectedIndex;
		let isIntersect = false;
		if (_overlapIds.length > 0) {
			_overlapIds.forEach((id) => {
				const overlapZone = zoneDM.getTarget(id);
				const isDupType = (overlapZone.type === selType);
				const isDupPriority = (overlapZone.priority === selPrio);
				if (isDupType && isDupPriority) {
					isIntersect = true;
					return;
				}
			});
			return isIntersect;
		}
		return false;
	}

	#findEdgeIntersection(edge1, edge2) {
		let x1 = edge1[0].x;
		let x2 = edge1[1].x;
		let x3 = edge2[0].x;
		let x4 = edge2[1].x;
		let y1 = edge1[0].y;
		let y2 = edge1[1].y;
		let y3 = edge2[0].y;
		let y4 = edge2[1].y;
		let nom1 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
		let nom2 = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
		let denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
		let t1 = nom1 / denom;
		let t2 = nom2 / denom;
		let interPoints = [];

		// 1. lines are parallel or edges don't intersect 
		if (((denom === 0) && (nom1 !== 0)) || (t1 <= 0) || (t1 >= 1) || (t2 < 0) || (t2 > 1)) {
			return interPoints;
		}

		// 2. lines are collinear 
		if ((nom1 === 0) && (denom === 0)) {
			//check if endpoints of edge2 lies on edge1
			for (var i = 0; i < 2; i++) {
				var classify = classifyPoint(edge2[i], edge1);
				//find position of this endpoints relatively to edge1
				if (classify.loc == "ORIGIN" || classify.loc == "DESTINATION") {
					interPoints.push({ x: edge2[i].x, y: edge2[i].y, t: classify.t });
				}
				else if (classify.loc == "BETWEEN") {
					x = +((x1 + classify.t * (x2 - x1)).toFixed(9));
					y = +((y1 + classify.t * (y2 - y1)).toFixed(9));
					interPoints.push({ x: x, y: y, t: classify.t });
				}
			}
			return interPoints;
		}

		// 3. edges intersect
		for (var i = 0; i < 2; i++) {
			var classify = classifyPoint(edge2[i], edge1);
			if (classify.loc == "ORIGIN" || classify.loc == "DESTINATION") {
				interPoints.push({ x: edge2[i].x, y: edge2[i].y, t: classify.t });
			}
		}
		if (interPoints.length > 0) {
			return interPoints;
		}
		let x = +((x1 + t1 * (x2 - x1)).toFixed(9));
		let y = +((y1 + t1 * (y2 - y1)).toFixed(9));
		interPoints.push({ x: x, y: y, t: t1 });
		return interPoints;
	}
}

class DelZonePolygonTool extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = true;
	}
}

class EditZoneVertexTool extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.g = new Geometry();
		this.#init();
	}

	setOpMode(mode) {
		this.cursorMode = mode;
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = false;

		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});

		inst.canvas.on('mouse:move', function (opt) {
			inst.#onMouseMove(opt);
		});

		inst.canvas.on('mouse:up', function (opt) {
			inst.#onMouseUp(opt);
		});
	}

	#onMouseDown(opt) {
		const cursor = opt.pointer;
		if (cursor.x < 0 || cursor.y < 0) { return; }

		// --- tool specific ---
		clearNodePopUp();

		if (!opt.target || !opt.target.id.includes(ZONE_POINT_PREFIX)) return;
		const inst = this;
		const x = opt.target.left;
		const y = opt.target.top;
		if (opt.button === 1) {
			// --- dragging case begin ---
			editVertexIdx = _.findIndex(selShape.points, { x, y });
			beforeEditPoints = JSON.parse(JSON.stringify(selShape.points));
			inst.canvas.isDragging = true;
		} else if (opt.button === 3) {
			// --- highlight point color ---
			removeFabricZoneObjects(ZONE_POINT_PREFIX);
			drawFabricSelectedZonePoints(selShape, 'edit', { x, y });

			const rosPos = tfCanvas2ROS(gMapMeta_, { x: x, y: y });
			document.getElementById("zone-node-x").value = rosPos.x;
			document.getElementById("zone-node-y").value = rosPos.y;
			document.getElementById("zone-node-createButton").onclick = this.#btnEditPolygonPoint.bind(opt.target);
			document.getElementById("zone-node-cancelButton").onclick = clearNodePopUp;
			document.getElementById("zone-node-popUp").style.display = "block";
			updatePopUpPosition('zone-node');
			inst.canvas.setActiveObject(opt.target);
		}
	}

	#onMouseMove(opt) {
		const inst = this;
		if (!inst.canvas.isDragging) { return; }

		const evt = opt.e;
		const pointer = inst.canvas.getPointer(evt);
		const x = pointer.x;
		const y = pointer.y;
		selShape.points.splice(editVertexIdx, 1, new Point2D(x, y));
	}

	#onMouseUp(opt) {
		const cursor = opt.pointer;
		if (cursor.x < 0 || cursor.y < 0) { return; }

		const inst = this;
		// --- dragging case end ---
		if (opt.target && opt.button === 1 && opt.target.id.includes(ZONE_POINT_PREFIX) && inst.canvas.isDragging) {
			// --- backup original zone vertices ---
			updateZoneVertices('update', beforeEditPoints);

			const x = opt.target.left;
			const y = opt.target.top;
			selShape.points.splice(editVertexIdx, 1, new Point2D(x, y));

			// --- re-render zone on canvas ---
			const id = opt.target.id.replace(ZONE_POINT_PREFIX, '');
			removeFabricZoneObjects(selShape.id);
			drawFabricZone(id);
			removeFabricZoneObjects(ZONE_POINT_PREFIX);
			drawFabricSelectedZonePoints(selShape, 'edit');

			editVertexIdx = undefined;
			beforeEditPoints = [];
			inst.canvas.isDragging = false;
		}
	}

	#btnEditPolygonPoint() {
		// --- backup original zone vertices ---
		const oriPoints = JSON.parse(JSON.stringify(selShape.points));
		updateZoneVertices('update', oriPoints);

		const id = this.id.replace(ZONE_POINT_PREFIX, '');
		const nodeX = document.getElementById("zone-node-x").value;
		const nodeY = document.getElementById("zone-node-y").value;
		const visPos = tfROS2Canvas(gMapMeta_, {
			x: Number(nodeX),
			y: Number(nodeY)
		});

		const index = _.findIndex(selShape.points, { x: this.left, y: this.top });
		selShape.points.splice(index, 1, new Point2D(Number(visPos.x), Number(visPos.y)));

		// --- re-render zone on canvas ---
		removeFabricZoneObjects(selShape.id);
		drawFabricZone(id);
		removeFabricZoneObjects(ZONE_POINT_PREFIX);
		drawFabricSelectedZonePoints(selShape);

		// --- close the modal ---
		$('#zone-node-popUp').hide();
	}

}

class AddZoneVertexTool extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.g = new Geometry();
		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		let inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = false;
		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});
	}

	#onMouseDown(opt) {
		const inst = this;
		const evt = opt.e;

		if (opt.button !== 1) return;
		if (opt.target !== null && opt.target.id.includes(ZONE_POINT_PREFIX)) {
			// --- start add zone vertex mode ---
			if (!isAddVertexMode || newPoints.length > 0) return;
			addZoneVertex.disabled = true;

			// --- save global parameters ---
			oldPoints = JSON.parse(JSON.stringify(selShape.points));
			const x = opt.target.left;
			const y = opt.target.top;
			vertexIdx = _.findIndex(selShape.points, { x, y });
			console.log(`Start index: ${vertexIdx} (${x}, ${y})`);

			// --- highlight point color ---
			removeFabricZoneObjects(ZONE_POINT_PREFIX);
			drawFabricSelectedZonePoints(selShape, 'add', { x, y });

			// --- backup original zone vertices ---
			updateZoneVertices('update', oldPoints);
		} else {
			if (!isAddVertexMode || vertexIdx === undefined) return;
			addZoneVertex.style.display = "none";
			endAddZoneVertex.style.display = "block";
			cancelZoneEdit.disabled = true;
			saveZoneBtn.disabled = true;

			const clickPoint = opt.absolutePointer;
			const x = clickPoint.x;
			const y = clickPoint.y;
			const newPoint = new Point2D(x, y);
			newPoints.push(newPoint);

			const points = selShape.get('points');
			if (newPoints.length === 1) {
				points.splice(vertexIdx + 1, 0, newPoint);
			} else {
				const prevPoint = newPoints[newPoints.length - 2];
				vertexIdx = _.findIndex(selShape.points, { x: prevPoint.x, y: prevPoint.y });
				points.splice(vertexIdx + 1, 0, newPoint);
			}

			this.#drawFabricEditingZone(newPoint);
		}
	}

	#drawFabricEditingZone(_point) {
		const inst = this;
		const id = selShape.id.replace(ZONE_POLYGON_PREFIX, '');
		const pointOption = {
			id: ZONE_POINT_PREFIX + id,
			radius: 7 / inst.canvas.getZoom(),
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
		const point = new fabric.Circle(pointOption);
		inst.canvas.add(point);

		selShape.set({
			fill: '#adadad',
			opacity: 0.3
		});
		removeFabricZoneObjects(selShape.id);
		drawFabricZone(id);
	}
}

class DelZoneVertexTool extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.g = new Geometry();
		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		let inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = false;
		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});
	}

	#onMouseDown(opt) {
		const inst = this;
		const evt = opt.e;

		if (opt.button !== 1 || opt.target === null || !opt.target.id.includes(ZONE_POINT_PREFIX)) return;

		inst.canvas.setActiveObject(opt.target);

		const id = opt.target.id.replace(ZONE_POINT_PREFIX, '');
		let zone_obj = getFabricZoneObject(id);
		if (zone_obj.points.length < 4) {
			alert('Zone can NOT under 3 points!');
			inst.canvas.selection = false;
			return;
		}

		// --- backup original zone vertices ---
		const oriPoints = JSON.parse(JSON.stringify(zone_obj.points));
		updateZoneVertices('update', oriPoints);

		// --- update zone cache data ---
		const x = opt.target.left;
		const y = opt.target.top;
		_.remove(zone_obj.points, p => p.x === x && p.y === y);

		// --- re-render zone on canvas ---
		removeFabricZoneObjects(opt.target.id);
		drawFabricSelectedZonePoints(selShape);
	}
}

class CancelZoneEditTool extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		let inst = this;
		// --- [TEMP] for creating zone polygon --- 
		inst.canvas.isDrawingMode = false;
		inst.canvas.selection = true;
	}
}

class ZonePolygonFactory {
	create(obj) {
		const type = ZONE_TYPES[obj.type] || 'none';
		const color = type.fillColor || 'gray';
		const opacity = obj.priority === 0 ? 0.3 : 0.8;
		const options = {};
		if (!obj.activate) {
			options['strokeDashArray'] = [5, 3];
		}
		return new fabric.Polygon(obj.points, {
			id: ZONE_POLYGON_PREFIX + obj.id,
			stroke: color,
			strokeWidth: 3,
			fill: color,
			opacity: opacity,
			objectCaching: false,
			moveable: false,
			selectable: false,
			...options
		});
	}
}