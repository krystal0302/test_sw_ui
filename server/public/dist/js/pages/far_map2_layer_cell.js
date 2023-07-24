// ========================
//   Data-Manager Classes    
// ========================
/* ver. 202307013
|       Backend      |      Frontend      |
| ------------------ | ------------------ |
| --------- CELL STATIC ATTRIBUTES ------ |
|       (NULL)       | group ['wms']      |
| uuid               | id                 |
|       (NULL)       | x (visPos.x)       |
|       (NULL)       | y (visPos.y)       |
|       (NULL)       | visPos             |
| cell_coordinate[0] | sysPos.x           |
| cell_coordinate[1] | sysPos.y           |
| cell_coordinate[2] | r                  |
| width              | w                  |
| height             | h                  |
| length             | l                  |
| area               | area               |
| cell_id            | cellId             |
| display_name       | labelFullName      |
|       (NULL)       | cellType           |
| type               | cellTypeDetection  |
| cell_size          | cellSize           |
| direction          | cellDirection      |
| function_type      | functionType       |
| makers             | makers             |
| maker_offset       | makerOffset        |
|                    |                    |
| --------- STATEFUL ATTRIBUTES --------- |
| load               | state.load         |
| --------- STYLING ATTRIBUTES    ------- |
|       (NULL)       | title              |
|       (NULL)       | label              |
|       (NULL)       | widthConstraint    |
|       (NULL)       | borderWidth        |
|       (NULL)       | color              |
|       (NULL)       | font               |
|       (NULL)       | shape              |
*/

class CellDataManager {
	static instance = null;

	constructor(canvas) {
		this.canvas = canvas;
		this.dataset = [];
		this.dataCache = new FarCache();
	}

	static getInstance(canvas) {
		if (CellDataManager.instance == null) {
			CellDataManager.instance = new CellDataManager(canvas);
		}

		return CellDataManager.instance;
	}

	loadFromSystem(data) {
		console.log(data);

		const inst = this;
		// --- data scheme transformation ---
		let cellData = (typeof data === 'object') ? data : JSON.parse(data);

		for (let arr of Object.values(cellData)) {
			arr.map((a) => a.function_type = (a.function_type === "None") ? "Position_cell" : a.function_type);
		}

		// --- clear the dataset first ---
		inst.dataset = [];
		// --- data transformation & load the dataset ---
		for (const [area, cellArr] of Object.entries(cellData)) {
			const cells = cellArr.map((cell) => ({ ...cell, area: area }));
			inst.dataset = [...inst.dataset, ...cells];
		}
		// console.log(inst.dataset);
		// TODO: convert Backend to Frontend data scheme 
		inst.dataset = inst.dataset.map((co) => {
			// console.log(co);
			const x = co.cell_coordinate[0];
			const y = co.cell_coordinate[1];
			const SYS = new Point2D(x, y);
			const VIS = tfROS2Canvas(gMapMeta_, SYS);
			const r = (co.cell_coordinate.length > 2) ? co.cell_coordinate[2] : null;
			const cellId = co.cell_id || genUuid();
			return {
				id: cellId,
				x: VIS.x,
				y: VIS.y,
				r: r,
				w: co.width,
				h: co.height,
				l: co.length,
				area: co.area,
				cellId: cellId,
				labelFullName: co.display_name,
				cellTypeDetection: co.type,
				cellSize: co.cell_size,
				cellDirection: co.direction,
				functionType: co.function_type,
				markers: co.markers,
				markerOffset: co.marker_offset,
				party: co?.party,
			};
		});

		// --- clear the dataCache ---
		this.dataCache.flush();
	}

	add(data) {
		this.dataCache.put(genUuid(), { action: 'add', ...data });
		this.dataset.push(data);
		// console.log(this.dataset);
	}

	delete(id) {
		// --- cache the to-be-deleted cell ---
		const cellObj = this.canvas.body.data.nodes.get(id);
		console.log(cellObj);
		this.dataCache.put(genUuid(), { action: 'delete', ...cellObj });

		// --- update the dataset ---
		this.dataset = this.dataset.filter((co) => co.id !== id);
	}

	update(change, coord = 'sys') {
		// --- cache the change ---
		const target = this.dataset.find((co) => co.id === change.id);
		this.dataCache.put(genUuid(), { action: 'edit', ...target });

		// --- data update ---
		if (coord === 'sys') {
			const SYS = change.sysPos;
			console.log(SYS);
			const pos = tfROS2Canvas(gMapMeta_, SYS);
			const VIS = new Point2D(pos.x, pos.y);
			change.x = VIS.x;
			change.y = VIS.y;
		}
		if (coord === 'vis') {
			const VIS = change.visPos;
			const pos = tfCanvas2ROS(gMapMeta_, VIS);
			const SYS = new Point2D(pos.x, pos.y);
			change.sysPos = SYS;
		}
		this.dataset = this.dataset.filter((co) => co.id !== change.id);
		this.dataset.push(change);
	}

	undo() {
		const changeNode = this.dataCache.popHead();
		const change = changeNode?.value;
		if (!change) { return; }
		// console.log(change);

		const inst = this;
		// case 1: action add. UNDO (add -> delete) 
		if (change.action === 'add') {
			console.log(change)
			// --- update visCache ---
			inst.canvas.body.data.nodes.remove(change.id);
			// === update rosCache ===
			inst.dataset = inst.dataset.filter((co) => co.id !== change.id);
			return;
		}

		// case 2: action edit 
		if (change.action === 'edit') {
			// console.log(change);
			const inst = this;

			// --- change area case ---
			// === update the dataset in the data manager ===
			inst.dataset = inst.dataset.filter((co) => co.id !== change.id);
			inst.dataset.push(change);

			// --- update visCache ---
			const tooltip = `<div class="farobot-map-tooltip" 
                        style="color: black">
                    ${change.labelFullName}@(${Number(change.rosX).toFixed(2)}, ${Number(change.rosY).toFixed(2)})
                  </div>`;

			const myNode = {
				id: change.id,
				x: change.x,
				y: change.y,
				cellId: change.cellId,
				area: change.area,
				label: change.labelFullName,         // cell appearing name tag
				labelFullName: change.labelFullName, // load to the column in pop-up box
				title: tooltip,
				//cellLoad: change.cellLoad
			};

			inst.canvas.body.data.nodes.update(myNode);

			return;
		}

		// case 3: action delete. UNDO (delete -> add) 
		if (change.action === 'delete') {
			const inst = this;
			delete change.action;
			console.log(change);
			// --- update visCache ---
			inst.canvas.body.data.nodes.update(change);
			// === update rosCache ===
			inst.dataset.push(change);
			console.log(inst.dataset);
			return;
		}
	}

	getTargetByID(id) {
		// console.log(this.dataset);
		return this.dataset.find((d) => d.id === id);
	}

	getTargetByLabelFullName(idName) {
		// console.log(this.dataset);
		return this.dataset.find((d) => d.labelFullName === idName);
	}

	getTargetByCellId(id) {
		// console.log(this.dataset);
		return this.dataset.find((d) => d.cellId === id);
	}

	clear() {
		this.dataset = [];
		this.dataCache.flush();
	}

	getAllData() {
		return this.dataset;
	}

	getAllData4System() {
		let allData = this.dataset.map((co) => {
			console.log(co);
			const VIS = new Point2D(co.x, co.y);
			const pos = tfCanvas2ROS(gMapMeta_, VIS);
			const SYS = new Point2D(pos.x, pos.y);

			return {
				cell_coordinate: (co.r === null) ? [SYS.x, SYS.y] : [SYS.x, SYS.y, co.r],
				width: co.w,
				height: co.h,
				length: co.l,
				area: co.area,
				cell_id: co.cellId,
				display_name: co.labelFullName,
				type: co.cellTypeDetection,
				cell_size: co.cellSize,
				direction: co.cellDirection,
				function_type: co.functionType,
				markers: co.markers,
				marker_offset: co.markerOffset,
				party: co?.party,
			}
		});

		const areas = this.dataset.map((c) => c.area);
		const areaSet = [...new Set(areas)];

		const finalResult = {};
		areaSet.forEach((a) => {
			const areaCells = allData.filter((c) => c.area === a)
			areaCells.forEach((ac) => {
				delete ac['area'];
			});
			finalResult[a] = areaCells;
		});

		// console.log(finalResult);
		return finalResult;
	}

	isExistUnsavedChanges() {
		return (this.dataCache.getSize() > 0) ? true : false;
	}
}


// ========================
//       Tool Classes    
// ========================
class AddCellTool extends VisTool {
	constructor(canvas, dm, options) {
		super(canvas);

		// --- vis-config ---
		this.#initCanvasNodes(this.canvas, 'wms');

		// --- initialization ---
		this.dataManager = dm;
		this.options = options || {};

		this.#init();
	}

	#init() {
		this.#bindEvents();

		// --- legacy ---
		this.canvas.addNodeMode();
		$('#edit-tools-info').show();
		$('#edit-status').show();
		$('#edit-status-title').text('Status.');
		$('#edit-status-badge').text('Add Cell Mode');
		showCancelEditButton(true);
		clearNodePopUp();
	}

	#bindEvents() {
		const inst = this;

		// --- bind mouse events ---

		// --- bind vis-canvas manipulation events ---
		const cfgAddNode = (data, callback) => { return inst.#onAddCellNode(data, callback, inst); };
		const cfgEditNode = (data, callback) => { return inst.#onEditCellNode(data, callback, inst); };
		const cfgDeleteNode = (data, callback) => { return inst.#onDeleteCellNode(data, callback, inst); };
		const options = {
			manipulation: {
				enabled: false,
				addNode: cfgAddNode,
				editNode: cfgEditNode,
				deleteNode: cfgDeleteNode
			}
		};
		inst.canvas.setOptions(options);

		const cfgDragEndNode = (params) => { return inst.#onDragEndCellNode(params, inst); };
		inst.canvas.off("dragEnd");
		inst.canvas.on("dragEnd", cfgDragEndNode);
	}

	// --- Storage Cell Manipulation ---
	#onAddCellNode(data, callback, self) {
		// filling in the popup DOM elements
		document.getElementById("cell-operation").innerText = "Add Cell";
		document.getElementById("cell-createButton").innerText = "Create";

		// --- update cellsize type ---
		let cellSizeSel = document.getElementById("cell-type");
		$(cellSizeSel).empty();

		// --- updated way ---
		let functionTypeOptions = functionTypes2_.map((t) => t.type);
		// --- add `None` option ---
		functionTypeOptions.unshift('Position_cell');
		functionTypeOptions.forEach((ft) => {
			const opt = document.createElement("option");
			opt.text = ft;
			opt.value = ft;
			cellSizeSel.options.add(opt);
		});
		cellSizeSel.selectedIndex = 0; // default value

		// ====== Vis Data Cache ======
		// --- default data to create a new cell ---
		data.id = genUuid();
		data.area = 'default_area';
		// data.cellUUID = genUuid().replace(/-/g, '');
		data.cellType = cellSizeSel.value;
		data.cellDirection = 'forward';
		//data.cellLoad = 'empty';
		data.labelFullName = 'new'; // default cell name
		data.functionType = cellSizeSel.value;
		data.markers = 'None';
		data.markerOffset = '0.5, 0.5, 1.57'
		data.cellSize = '0.5, 0.5';
		data.party = '';

		// ====== User Input ======
		// --- x, y should be assign from mouse event ---
		// --- read from mouse position --- 
		const cvsPos = new Point2D(data.x, data.y)
		const sysPos = tfCanvas2ROS(gMapMeta_, cvsPos);

		document.getElementById("cell-x").value = sysPos.x;
		document.getElementById("cell-y").value = sysPos.y;
		document.getElementById("cell-r").value = "";    // default rotation value
		document.getElementById("cell-label").value = data.labelFullName;
		document.getElementById("cell-area").value = data.area;

		document.getElementById("cell-type").value = data.functionType;
		document.getElementById("cell-direction").value = data.cellDirection;
		// document.getElementById("cell-load").value = data.cellLoad;
		document.getElementById("cell-markers").value = data.markers;
		document.getElementById("marker-offset").value = data.markerOffset;
		document.getElementById("cell-size").value = data.cellSize;
		document.getElementById("cell-party").value = data.party;

		// --- save the edited properties ---
		const createBtn = document.getElementById("cell-createButton");
		const cancelBtn = document.getElementById("cell-cancelButton");
		document.getElementById("cell-createButton").onclick = self.#onSaveCellNodeData.bind(this, data, callback, self, 'add');
		document.getElementById("cell-cancelButton").onclick = clearNodePopUp.bind(this, callback);
		document.getElementById("cell-popUp").style.display = "block";

		// ---  [UX] position and styling ---
		updatePopUpPosition('cell');

		document.getElementById('nav-agent').style.display = 'block';
	}

	#onDeleteCellNode(data, callback, self) {
		console.log(data);
		const deletedNode = visNetwork_.body.data.nodes.get(data.nodes[0]);

		// --- cache the difference ---
		console.log(deletedNode);

		// === [rosCache] remove the deleted  cell node ===
		self.dataManager.delete(deletedNode.id);

		// --- [visCache] execute delete the cell node ---
		callback(data);
	}

	#onEditCellNode(data, callback, self) {
		// --- filling in the popup DOM elements ---
		document.getElementById("cell-operation").innerText = "Edit Cell";
		document.getElementById("cell-createButton").innertText = "OK";

		// --- update cellsize type ---
		const cellSizeSel = document.getElementById("cell-type");
		$(cellSizeSel).empty();

		// --- updated way ---
		let functionTypeOptions = functionTypes2_.map((t) => t.type);
		functionTypeOptions.unshift('Position_cell');
		functionTypeOptions.forEach((ft) => {
			const opt = document.createElement("option");
			opt.text = ft;
			opt.value = ft;
			cellSizeSel.options.add(opt);
		});
		cellSizeSel.value = data.functionType || 'Position_cell'; // default value

		const targetObj = self.dataManager.getTargetByID(data.id);
		// console.log(targetObj);
		const x = targetObj.x;
		const y = targetObj.y;
		const visPos = new Point2D(x, y);
		const pos = tfCanvas2ROS(gMapMeta_, visPos);
		const sysPos = new Point2D(pos.x, pos.y);
		document.getElementById("cell-x").value = sysPos.x;
		document.getElementById("cell-y").value = sysPos.y;

		const cellR = targetObj.r;
		const orient = (targetObj.r === null) ? "" : Math.round(cvtRad2Deg(cellR).toFixed(2));
		// const orient = (targetObj.r === null) ? NaN : Math.floor(cellR);
		console.log(orient);
		document.getElementById("cell-r").value = orient;

		document.getElementById("cell-label").value = data.labelFullName;
		document.getElementById("cell-area").value = data.area;

		document.getElementById("cell-type").value = data.functionType || 'Position_cell';
		document.getElementById("cell-direction").value = data.cellDirection;
		// FIXME: appears only when live status is on.
		const targetCell = apiQueryData_.wms.cells.find((c) => c.cell_id == data.id);
		const rtCellLoad = targetCell?.load;
		document.getElementById("cell-load").value = cellLoadAdaptor.toUiCellLoad(rtCellLoad);
		document.getElementById("cell-markers").value = data.markers || 'None';
		document.getElementById("marker-offset").value = data.markerOffset || '0.5, 0.5, 1.57';
		document.getElementById("cell-size").value = data.cellSize || '0.5, 0.5';
		document.getElementById("cell-party").value = data.party || '';

		document.getElementById("cell-applyButton").onclick = self.#onApplyCellNodeData.bind(this, data, callback, self, 'add');
		document.getElementById("cell-createButton").onclick = self.#onSaveCellNodeData.bind(this, data, callback, self, 'edit');
		document.getElementById("cell-cancelButton").onclick = cancelNodeEdit.bind(this, callback);

		document.getElementById("cell-popUp").style.display = 'block';

		updatePopUpPosition('cell');

		document.getElementById('nav-agent').style.display = 'block';
	}

	async #onApplyCellNodeData(data, callback) {
		let map = document.getElementById('map-select').value;
		let targetVal = document.getElementById('cell-load').value;
		// --- FARobot 1 million idea, PATENTED! ---
		targetVal = (targetVal === 'occupied') ? 'rack' : targetVal;
		// console.log(targetVal);
		// --- trench RESTful API ---
		const cellLoad = {
			"cells_to_update": [{
				"map": map,
				"area_id": data.area,
				"cell_id": data.cellId,
				"status": "",
				"load": targetVal,
			}]
		};
		console.log(cellLoad);
		let res = await fetchPutCellLoad(rmtToken_, cellLoad);
		// res = await res.json();

		// ------ response handler ------
		console.log(res.status);
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

	#onSaveCellNodeData(data, callback, self, _mode = 'edit') {
		// --- cancel the manipulation mode ---
		// closeManipulationMode();
		visCanvas.setMode('rdCell');

		// --- start saving Cell operation ---
		const cellIdName = document.getElementById("cell-label").value;
		const oriCellID = data.cellId;

		// --- [protection] prevent from duplcated cell name ---
		const bValidInput = inputCheck(cellIdName);
		document.getElementById("node-label").style["border-color"] = (bValidInput) ? "" : "red";

		if (!bValidInput) {
			notificationMsg(2, 'Input include invalid characters.');
			return;
		}

		const cellAllData = self.dataManager.getAllData();
		const cellLabelArr = cellAllData.map((c) => c.labelFullName);
		const dupCellDisplayName = cellAllData.filter((c) => c.cellId !== oriCellID).find((c) => c.labelFullName == cellIdName);
		// console.log(dupCellDisplayName);

		if (dupCellDisplayName !== undefined) {
			alert('DUPLICATED CELL LABEL!');
			return;
		}

		// === data assignment ===
		const sysX = document.getElementById("cell-x").value;
		const sysY = document.getElementById("cell-y").value;
		const sysPos = new Point2D(sysX, sysY);
		const visPos = tfROS2Canvas(gMapMeta_, sysPos)
		const visR = document.getElementById("cell-r").value;

		const cellArea = document.getElementById("cell-area").value;
		const cellType = document.getElementById("cell-type").value;
		const functionType = document.getElementById("cell-type").value;


		let cellDetectionType = cellTypeAdaptor.toDetectionType2(functionTypes2_, functionType);
		cellDetectionType = (cellDetectionType === null) ? 'None' : cellDetectionType;

		// --- check cell label ---
		if (!inputCheck(cellIdName)) {
			document.getElementById("cell-label").style["border-color"] = "red";
			notificationMsg(2, `Input include invalid characters.`);
			return;
		} else {
			document.getElementById("cell-label").style["border-color"] = "";
		}

		const target = functionTypes2_.find(ct => ct.type === functionType);
		let cellWidth = 1.0;
		let cellLength = 1.0;
		if (target !== undefined) {
			cellWidth = Number(target.width);
			cellLength = Number(target.length);
		}

		const cellDirection = document.getElementById("cell-direction").value;
		const cellLoad = document.getElementById("cell-load").value;

		// --- update modified data here ---
		// data.cellId = cellIdName;
		data.cellId = (_mode === 'add') ? genUuid() : data.cellId;
		data.label = cellIdName;
		data.labelFullName = cellIdName;
		data.area = cellArea;

		data.group = 'wms';
		data.shape = 'circle';
		data.widthConstraint = 25;
		data.size = 20;
		data.borderWidth = 2;
		const tooltip = `<div class="farobot-map-tooltip" 
                      style="color: ${getColor(cellDetectionTypes_, cellDetectionType, 'tooltipColor')}">
                  ${cellIdName}@(${sysPos.x.toFixed(2)}, ${sysPos.y.toFixed(2)})
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
		data.x = Number(visPos.x);
		data.y = Number(visPos.y);
		const rad = Number(cvtDeg2Rad(visR).toFixed(4));
		data.r = (visR === "") ? null : rad;
		// console.log(data.r);
		data.sysPos = sysPos;
		data.w = cellWidth;
		data.l = cellLength;
		data.cellType = cellType || data.cellType;
		data.functionType = functionType || data.functionType;
		data.cellDetectionType = cellDetectionType;
		data.cellDirection = cellDirection;
		data.cellLoad = cellLoad;
		data.labelFullName = cellIdName;

		// --- [trial] updated wms data ---
		let cellSize = document.getElementById('cell-size').value;
		cellSize = cellSize.split(',').map((i) => parseFloat(i));
		data.cellSize = cellSize;

		// FIXME: --- Cell party ---
		const party = document.getElementById('cell-party').value;
		data.party = party;

		const markers = document.getElementById("cell-markers").value;
		data.markers = markers;

		let markerOffset = document.getElementById("marker-offset").value;
		markerOffset = markerOffset.split(",").map((i) => parseFloat(i));
		data.markerOffset = markerOffset;

		// --- [protection] prevent area from empty input ---
		var emptyCheck = cellArea.replace(/\s/g, "");
		if (cellArea === "" || emptyCheck === "") {
			alert('area SHOULD NOT be empty');
			return;
		}

		// === update changes to data manager ===
		// console.log(data)
		clearNodePopUp();
		if (_mode === 'add') {
			self.dataManager.add(data);
		}
		if (_mode === 'edit') {
			self.dataManager.update(data, 'sys');
		}
		// console.log(self.dataManager.getAllData());

		self.canvas.body.data.nodes.update(data); // write-in to the cache
		self.canvas.addNodeMode();

		// --- enable all cells draggable ---
		self.#initCanvasNodes(self.canvas, 'wms');
	}

	#initCanvasNodes(canvas, nodeType) {
		// --- enable all route vertices draggable ---
		const networkNodes = Object.values(canvas.body.nodes);
		networkNodes.filter((node) => node.options.group === nodeType).forEach((node) => {
			node.options.fixed.x = node.options.fixed.y = false;
		});
	}

	// --- update the final pose of the cell ---
	#onDragEndCellNode(params, self) {
		// --- protection ---
		if (!params.nodes.length) { return; }

		// --- get node id ---
		const nodeId = params.nodes[0];

		// --- get node group ---
		const dragNode = visNetwork_.body.nodes[nodeId];
		const group = dragNode.options.group;
		console.log(dragNode);

		// --- update data cache ---
		if (group !== 'wms') { return; }

		// --- update the position after dragging ---
		self.canvas.body.data.nodes.update({ ...dragNode.options, x: dragNode.x, y: dragNode.y }); // write-in to the cache
		const visPos = new Point2D(dragNode.x, dragNode.y);
		// --- perserve vertex shape, the shape mutates by dragging ---
		self.dataManager.update({ ...dragNode.options, visPos: visPos, shape: 'circle', x: dragNode.x, y: dragNode.y }, 'vis');
	}
}


// =================================
//   Cells Layer UI Event-bindings    
// =================================
// let cellDM = new CellDataManager(visNetwork_);
let cellTool;

const cancelCellEdit = document.getElementById('cancel-cell-edit');
const addCell = document.getElementById('add-cell');
const editCell = document.getElementById('edit-cell');
const deleteSelectedCell = document.getElementById('delete-cell');
const undoCell = document.getElementById('undo-cell');
const functionTypeBtn = document.getElementById('function-type-btn');
const liveCellSatus = document.getElementById('set-live-btn');

cancelCellEdit.addEventListener('click', (e) => {
	closeManipulationMode();
	visCanvas.setMode(e.target.id === 'cancel-route-edit' ? 'rdRoute' : 'rdCell');
});

// ------ cell editing ------
addCell.addEventListener('click', () => {
	const cellDM = CellDataManager.getInstance(visNetwork_);
	// --- updated ---
	cellTool = new AddCellTool(visNetwork_, cellDM);
	showManipulationModeText("Add Cell");
});
editCell.addEventListener('click', () => {
	visNetwork_.editNode();
});
deleteSelectedCell.addEventListener('click', () => {
	visNetwork_.deleteSelected(visNetwork_);
	closeManipulationMode();
	showManipulationModeText("Delete Cell");
});

// ------ cell undo ------
undoCell.addEventListener('click', () => {
	closeManipulationMode();
	const cellDM = CellDataManager.getInstance(visNetwork_);
	cellDM.undo();
});

// ------ function type ------
let hiddenFunctionTypes = [];
functionTypeBtn.addEventListener('click', () => {
	const cellTypeDeck = $('#sb-cell-size-deck');
	cellTypeDeck.empty();

	let targetFunctionTypes = functionTypes2_.filter((ft) => ['rack', 'charger'].includes(ft.category));
	hiddenFunctionTypes = functionTypes2_.filter((ft) => !['rack', 'charger'].includes(ft.category));
	// console.log(targetFunctionTypes);
	// console.log(hiddenFunctionTypes);
	targetFunctionTypes.forEach((ft) => {
		const uuid = genUuid().slice(0, 5);
		// ====== updated WMS version ======
		const template = document.querySelector('#sb-cell-type-row2');
		const node = document.importNode(template.content, true);
		let paramNode = node.querySelector('.cell-type-name');
		paramNode.textContent = ft.type;

		paramNode = node.querySelector('.cell-detection-type');
		paramNode.addEventListener('change', updateFunctionTypeParams.bind(paramNode, functionTypes2_));
		paramNode.value = ft.category;

		paramNode = node.querySelector('.device-recognition');

		// --- switch to corresponding recognition option ---
		const recogParentNode = node.querySelector('.device-recognition').parentNode;
		const recogNode = node.querySelector('.device-recognition');
		recogNode.remove();
		let newRecogNode = document.createElement("select");
		newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
		let opt = document.createElement("option");
		const ftType = (ft.category === 'rack') ? 'Rack' : 'Charger';
		opt.text = opt.value = ftType;
		newRecogNode.options.add(opt);
		newRecogNode.disabled = true;
		recogParentNode.append(newRecogNode);

		const extraParamTemplate = document.querySelector(`#sb-cell-type-${ft.category}`)
		const extraParamNodes = document.importNode(extraParamTemplate.content, true);
		paramNode = node.querySelector('.function-type-params');
		paramNode.append(extraParamNodes);

		if (ft.category === 'rack') {
			paramNode = node.querySelector('.device-size');
			paramNode.classList.add("function-type-input-" + uuid);
			paramNode.setAttribute('id', 'sbft-size-' + uuid);
			console.log(ft.size);
			paramNode.value = `${ft.size.width},${ft.size.length},${ft.size.height}`;
			paramNode.disabled = true;

			paramNode = node.querySelector('.device-payload');
			paramNode.classList.add("function-type-input-" + uuid);
			paramNode.setAttribute('id', 'sbft-payload-' + uuid);
			paramNode.value = ft.payload;
			paramNode.disabled = true;
		}
		if (ft.category === 'charger') {
			paramNode = node.querySelector('.device-offset');
			console.log(ft.offset);
			// paramNode.value = ft.offset;
			paramNode.classList.add("function-type-input-" + uuid);
			paramNode.setAttribute('id', 'sbft-offset-' + uuid);
			paramNode.value = `${ft.offset.x},${ft.offset.y},${ft.offset.theta}`;
			paramNode.disabled = true;

			paramNode = node.querySelector('.device-model');
			$(paramNode).select2();

			if (fleetModels_.length) {
				fleetModels_.forEach((fm) => {
					let opt = document.createElement("option");
					opt.text = opt.value = fm; // name equals cell type value
					paramNode.options.add(opt);
				});
			}

			$(paramNode).val(ft.model);
			$(paramNode).trigger('change');

			paramNode.disabled = true;
		}

		let cardNode = node.querySelector('.card-body');

		const editNode = node.querySelector('.edit-cell-size');
		// editNode.setAttribute('id', `ft-edit-${ft.type}`);
		editNode.setAttribute('id', `ft-edit-${uuid}`);
		editNode.addEventListener('click', editCellTypeCb.bind(cardNode));

		const deleteNode = node.querySelector('.delete-cell-size');
		deleteNode.setAttribute('id', `ft-delete-${ft.type}`);
		deleteNode.addEventListener('click', deleteCellTypeCb.bind(cardNode));

		cellTypeDeck.append(node);

		// --- mount the validator: this.value should be either 'rack' or 'charger' ---
		validateFunctionTypeInputEvent2({ type: ft.category, uuid: uuid });
	});


	// --- load cell dectection types on adding a new cell type row ---
	const sbDetectionTypesSel = document.getElementById('sbft-cell-detection-type');
	$(sbDetectionTypesSel).empty();

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
	functionDetectionTypes_.forEach((fdt) => {
		var opt = document.createElement("option");
		opt.text = fdt.label;
		opt.value = fdt.name; // name equals cell type value
		sbDetectionTypesSel.options.add(opt);
	});
	sbDetectionTypesSel.addEventListener('change', updateFunctionTypeParams.bind(sbDetectionTypesSel, functionTypes2_));

	// --- update save button binding ---
	let placeholder = $('#save-placeholder');
	placeholder.empty();
	let saveBtn = document.createElement('button');
	saveBtn.setAttribute('class', 'btn btn-primary btn-lg');
	saveBtn.innerHTML = "Save";
	saveBtn.addEventListener('click', btnSaveFunctionTypes.bind(this));

	placeholder.append(saveBtn);

	applyFontSize(getSavedFontSize(), '#cell-size-sb');
});


let liveOn = false;
liveCellSatus.addEventListener('click', () => {
	if (!liveOn) {
		liveOn = setInterval(function () {
			if (apiQueryData_ !== undefined && apiQueryData_.hasOwnProperty('wms')) {
				updateVisCellStatus(apiQueryData_.wms);
			}
			visNetwork_.redraw();
		}, 450);

		$('#live-btn-text').text('Live Status On');
		$('#live-btn-text').siblings('.fas').attr('class', 'fas fa-eye');
		$('#live-status-text').val('on');
		$('#set-live-btn').text('Set Off');
		rtSetCellConfig(true);

		const cellDM = CellDataManager.getInstance(visNetwork_);
		cellTool = new AddCellTool(visNetwork_, cellDM);
		return;
	}

	clearInterval(liveOn)
	liveOn = null;

	updateVisCellStatus();

	$('#live-btn-text').text('Live Status Off');
	$('#live-btn-text').siblings('.fas').attr('class', 'fas fa-eye-slash');
	$('#live-btn-text').siblings('.fas').attr('style', 'color:darkgray');
	$('#live-status-text').val('off');
	$('#set-live-btn').text('Set On');
	rtSetCellConfig(false);
});

function rtSetCellConfig(_isOn) {
	// ------ switch labels ------
	document.getElementById("cell-operation").innerText = (_isOn) ? "Set Cell Load" : "Edit Cell";

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
	let matches = container.querySelectorAll('table > tbody > tr');
	matches.forEach((el) => {
		console.log(el.className);
		el.style.display = (el.className === 'tr-cell-load') ? rtEl : nonRtEl;
		console.log(persistEls.includes(el.className));
		if (persistEls.includes(el.className)) {
			el.querySelector('input').disabled = (_isOn) ? true : false;
			el.style.display = '';
		}
	});
}

// --- update WMS status ---
function updateVisCellStatus(_msgs) {
	_msgs = _msgs?.cells;

	// --- LIVE-OFF cell status ---
	if (_msgs === undefined) {
		const targetNodes = Object.values(visNetwork_.body.nodes).filter((node) => node.options.group === 'wms');
		const cellNodes = targetNodes.map((node) => ({
			id: node.id,     // cell-id
			color: { border: "rgba(0,0,0,0.6)" }
		}));


		visNetwork_.body.data.nodes.update(cellNodes);
		$('#live-btn-text').siblings('.fas').attr('style', 'color:rgba(0,0,0,0.6)');
		return;
	}

	// --- LIVE-ON cell status ---  
	const selMap = document.getElementById('map-select').value;

	const targetNodes = _msgs.filter((node) => node.map === selMap);
	// console.log(targetNodes);

	const cellDM = CellDataManager.getInstance(visNetwork_);
	const cellNodes = targetNodes.map((node) => {
		const target = cellDM.getTargetByCellId(node.cell_id);
		return {
			id: target.id,   // cell-id
			color: wmsStyles_[node.load] || { border: 'rgba(255,255,0,0.4)' }
		};
	});

	visNetwork_.body.data.nodes.update(cellNodes);
	$('#live-btn-text').siblings('.fas').attr('style', 'color:rgba(0,255,0,0.6)');
};


// --- show corresponding tools, and hide the rest ---
$(document).on('click', '#cell-tool button', function () {
	if (this.id === 'add-cell') {
		document.getElementById("delete-cell").style.display = "none";
	}
});


// =============================
//     Cells Event Callbacks    
// =============================
async function btnSaveCells() {
	const cellDM = CellDataManager.getInstance();
	const systemCellData = cellDM.getAllData4System();
	console.log(systemCellData);

	const content = JSON.stringify(systemCellData);

	// --- update the cells data by map ---
	const selMap = document.getElementById('map-select').value;
	const res = await fetchPutMapCells(rmtToken_, selMap, content);
	// res = await res.json();
	if (res.ok) {
		notificationMsg(1, 'Storage Cells are saved on SUCCESS!');
	} else {
		notificationMsg(3, 'Storage Cells are saved on FAILURE!');
	}

	// --- flush route and cell data   ---
	visNetwork_.body.data.nodes.clear();
	visNetwork_.body.data.edges.clear();

	// --- reload cell data from system ---
	try {
		const res = await fetchGetMapCells(rmtToken_, gMapMeta_.cell);
		const cellData = await res.json();
		console.log(cellData)
		const cellDM = CellDataManager.getInstance(visNetwork_);
		cellDM.loadFromSystem(cellData);

		// --- rendering on UI ---
		const cellAllData = cellDM.getAllData();
		console.log(cellAllData);
		visDrawCells(visNetwork_, cellAllData);
	} catch (err) {
		console.error(err);
	}

	// --- reload graph data from system ---
	try {
		const res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
		const graphData = await res.text();

		// --- global caching ---
		routeDM.loadFromSystem(graphData);

		// --- rendering on UI ---
		const routes = routeDM.getAllData();
		visDrawRoutes(visNetwork_, routes);
	} catch (err) {
		console.error(err);
	}

	visCanvas.setMode('rdCell');
	closeManipulationMode();
}

// --- get the cell type. (default: typeName. if not, look-up the cellTypes) ---
function getCellType(_cellObj, _cellTypes) {
	if (!_cellTypes) { return 'none'; }
	if (_cellObj.typeName) { return _cellObj.typeName; }

	// 1. --- look-up the cell type ---
	const typeTarget = cellTypeAdaptor.toUiCellType2(_cellTypes, {
		'detectionType': _cellObj.type,
		'width': _cellObj.width,
		'length': _cellObj.length
	});
	// 2. --- if the cell type is NOT FOUND, return 'none', else return its name ---
	return (!typeTarget) ? 'none' : typeTarget.name;
}

// -- draw storage cells from json file --
function visDrawCells(_network, cellArr) {
	// --- read the data from WMS json file ---
	const nodes = cellArr.map((co) => {
		// === Data Transformation ===
		// --- functional attributes ---
		// console.log(co);
		const cellIdName = co.labelFullName;
		const visPos = new Point2D(co.x, co.y);
		const pos = tfCanvas2ROS(gMapMeta_, visPos);
		const sysPos = new Point2D(pos.x, pos.y);
		// console.log(sysPos);
		// const r = cvtRad2Deg(Number(co.cell_coordinate[2])).toFixed(2);
		const r = cvtRad2Deg(Number(co.r).toFixed(2));
		const type = getCellType(co, functionTypes2_);
		const cellLoad = cellLoadAdaptor.toUiCellLoad(co.load);

		// --- styling attributes ---
		const targetFT = functionTypes2_.find((ft) => ft.type === co.functionType);
		const styleType = (targetFT?.recognition) ? targetFT.recognition[0] : co.cellTypeDetection;

		const tooltip = `<div class="farobot-map-tooltip"
                          style="color: ${getColor(cellDetectionTypes_, styleType, 'tooltipColor')}">
                      ${cellIdName}@(${sysPos.x.toFixed(2)}, ${sysPos.y.toFixed(2)})
                     </div>`;


		// === Data Assignment ===
		const wmsNode = {};

		// --- functional attributes ---
		wmsNode.group = 'wms';
		wmsNode.x = visPos.x;
		wmsNode.y = visPos.y;
		wmsNode.r = r;
		wmsNode.w = co.w;
		wmsNode.l = co.l;
		wmsNode.id = co.id;
		wmsNode.cellId = co.cellId;
		wmsNode.area = co.area;
		wmsNode.cellLoad = cellLoad;
		wmsNode.labelFullName = co.labelFullName;
		wmsNode.cellType = type;
		wmsNode.cellTypeDetection = co.cellTypeDetection;
		wmsNode.cellSize = co.cellSize;
		wmsNode.cellDirection = co.cellDirection;
		wmsNode.functionType = co.functionType;
		wmsNode.markers = co.markers;
		wmsNode.markerOffset = co.markerOffset;

		// --- styling attributes ---
		wmsNode.title = tooltip;
		wmsNode.label = co.labelFullName.slice(0, 5) + (co.labelFullName.length > 5 ? ".." : "");
		wmsNode.widthConstraint = 25;
		wmsNode.size = 20;
		wmsNode.borderWidth = 2;
		wmsNode.color = {
			highlight: getColor(cellDetectionTypes_, styleType, 'focusColor'),
			background: getColor(cellDetectionTypes_, styleType, 'bgColor'),
			border: getColor(cellDetectionTypes_, styleType, 'borderColor')
		};
		wmsNode.font = {
			size: 7,
			color: getColor(cellDetectionTypes_, co.type, 'color'),
			bold: true
		};
		wmsNode.shape = 'circle';
		wmsNode.party = co.party;

		return wmsNode;
	})

	_network.body.data.nodes.update(nodes);
}

function updateFunctionTypeParams(_funcTypes) {
	const uuid = genUuid().slice(0, 5);
	const paramNode = this.parentNode.parentNode.querySelector('.function-type-params');
	const typeName = this.parentNode.parentNode.querySelector('.cell-type-name').textContent;

	const targetTemplate = document.querySelector(`#sb-cell-type-${this.value}`);
	const targetNodes = document.importNode(targetTemplate.content, true);
	let node = {};

	if (this.value === 'rack') {
		// --- switch to corresponding recognition option ---
		const recogParentNode = paramNode.querySelector('.device-recognition').parentNode;
		const recogNode = paramNode.querySelector('.device-recognition');
		recogNode.remove();
		const newRecogNode = document.createElement("select");
		newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
		const opt = document.createElement("option");
		opt.text = opt.value = 'Rack';
		newRecogNode.options.add(opt);
		recogParentNode.append(newRecogNode);

		// --- flush previous params ---
		paramNode.querySelectorAll('.device-model, .device-offset').forEach((elem) => elem.parentNode.remove());

		// --- append new params --- 
		const target = _funcTypes.filter((ft) => ft.category === this.value).find((ft) => ft.type === typeName);
		const size = (target) ? `${target.size.width},${target.size.length},${target.size.height}` : "";
		// console.log(size);
		const payload = (target) ? target.payload : "";

		node = targetNodes.querySelector('.device-size');
		node.classList.add("function-type-input-" + uuid);
		node.setAttribute('id', 'sbft-size-' + uuid);
		node.value = size;
		node = targetNodes.querySelector('.device-payload');
		node.classList.add("function-type-input-" + uuid);
		node.setAttribute('id', 'sbft-payload-' + uuid);
		node.value = payload;
	}

	if (this.value === 'charger') {
		// --- switch to corresponding recognition option ---
		const recogParentNode = paramNode.querySelector('.device-recognition').parentNode;
		const recogNode = paramNode.querySelector('.device-recognition');
		recogNode.remove();
		const newRecogNode = document.createElement("select");
		newRecogNode.setAttribute('class', 'form-control col-7 device-recognition');
		const opt = document.createElement("option");
		opt.text = opt.value = 'Charger'; // name equals cell type value
		newRecogNode.options.add(opt);
		recogParentNode.append(newRecogNode);

		// --- flush previous params ---
		paramNode.querySelectorAll('.device-size, .device-payload').forEach((elem) => elem.parentNode.remove());

		// --- append new params --- 
		node = targetNodes.querySelector('.device-offset');
		node.classList.add("function-type-input-" + uuid);
		node.setAttribute('id', 'sbft-offset-' + uuid);

		node = targetNodes.querySelector('.device-model');
		$(node).select2();
		if (fleetModels_.length > 0) {
			fleetModels_.forEach((fm) => {
				const opt = document.createElement("option");
				opt.text = opt.value = fm; // name equals cell type value
				node.options.add(opt);
			});
		}
	}

	paramNode.append(targetNodes);

	// --- mount the validator: this.value should be either 'rack' or 'charger' ---
	// console.log(this.value);
	validateFunctionTypeInputEvent2({ type: this.value, uuid: uuid });
}

function deleteCellTypeCb() {
	this.parentNode.remove();
}

function editCellTypeCb() {
	let node = this.querySelector('.cell-detection-type');
	node.disabled = !node.disabled;
	const ftClass = node.value;

	node = this.querySelector('.device-recognition');
	node.disabled = true;

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

	let btnIconNode = this.querySelector('.fas');
	btnIconNode.classList.toggle("fa-pen");
	btnIconNode.classList.toggle("fa-eye");
}

// --- create functionType (cell size) on sidebar ---
function createNewCellSize() {
	console.log('------ create new cell size ------')
	// [protection] if name is empty, return 
	if ($('#sbft-cell-name').val() === "") {
		alert('The cell type name should NOT be empty!');
		return;
	}

	const uuid = genUuid().slice(0, 5);

	const cellTypeName = $('#sbft-cell-name').val();
	const cellDetectionType = $('#sbft-cell-detection-type').val();

	// --- append the value to the cell-size-deck ---
	const cellTypeDeck = $('#sb-cell-size-deck');

	const template = document.querySelector('#sb-cell-type-row2');
	const node = document.importNode(template.content, true);
	let nameNode = node.querySelector('.cell-type-name');
	nameNode.textContent = cellTypeName;

	// --- update cell detection type ---
	const addTypeSel = document.getElementById('sbft-cell-detection-type');
	const selectedType = addTypeSel.value;
	const detectTypesSel = node.querySelector('.cell-detection-type');
	detectTypesSel.addEventListener('change', updateFunctionTypeParams.bind(detectTypesSel, functionTypes2_));
	detectTypesSel.value = selectedType;

	const extraParamTemplate = document.querySelector(`#sb-cell-type-${cellDetectionType}`)
	const extraParamNodes = document.importNode(extraParamTemplate.content, true);
	let paramNode = node.querySelector('.function-type-params');
	paramNode.append(extraParamNodes);

	let userDefinedParams = document.querySelector('#user-defined-params');
	let pNode = {};

	paramNode = node.querySelector('.device-recognition');
	pNode = userDefinedParams.querySelector('.device-recognition');
	paramNode.value = pNode.value;
	$(paramNode).prop('disabled', true);
	pNode.selectedIndex = 0;

	if (cellDetectionType === 'rack') {
		paramNode = node.querySelector('.device-size');
		pNode = userDefinedParams.querySelector('.device-size');
		paramNode.classList.add("function-type-input-" + uuid);
		paramNode.setAttribute('id', 'sbft-size-' + uuid);
		paramNode.value = pNode.value;
		$(paramNode).prop('disabled', true);
		pNode.value = "";

		paramNode = node.querySelector('.device-payload');
		pNode = userDefinedParams.querySelector('.device-payload');
		pNode.classList.add("function-type-input-" + uuid);
		pNode.setAttribute('id', 'sbft-payload-' + uuid);
		paramNode.value = pNode.value;
		$(paramNode).prop('disabled', true);
		pNode.value = "";
	}
	if (cellDetectionType === 'charger') {
		paramNode = node.querySelector('.device-offset');
		pNode = userDefinedParams.querySelector('.device-offset');
		paramNode.classList.add("function-type-input-" + uuid);
		paramNode.setAttribute('id', 'sbft-offset-' + uuid);
		paramNode.value = pNode.value;
		$(paramNode).prop('disabled', true);
		pNode.value = "";

		paramNode = node.querySelector('.device-model');
		$(paramNode).select2();
		pNode = userDefinedParams.querySelector('.device-model');
		if (fleetModels_.length > 0) {
			fleetModels_.forEach((fm) => {
				var opt = document.createElement("option");
				opt.text = opt.value = fm; // name equals cell type value
				paramNode.options.add(opt);
			});
		}

		let selectedModels = $(pNode).select2("data");
		selectedModels = selectedModels.map(s => s.text);
		$(paramNode).val(selectedModels);
		$(paramNode).trigger('change');
		$(paramNode).prop('disabled', true);

		$(pNode).val("");
	}

	const cardNode = node.querySelector('.card-body');
	const editNode = node.querySelector('.edit-cell-size');
	editNode.setAttribute('id', `ft-edit-${cellTypeName}`)
	editNode.addEventListener('click', editCellTypeCb.bind(cardNode));

	const deleteNode = node.querySelector('.delete-cell-size');
	deleteNode.setAttribute('id', `ft-delete-${cellTypeName}`);
	deleteNode.addEventListener('click', deleteCellTypeCb.bind(cardNode));

	cellTypeDeck.append(node);
	validateFunctionTypeInputEvent2({ type: cellDetectionType, uuid: uuid });

	// --- flush the general inputs ---
	$('#sbft-cell-name').val('');
	$('#sbft-cell-detection-type').val('rack');
}

// --- save functionType (cell size) on sidebar ---
async function btnSaveFunctionTypes() {
	const funcTypeCards = document.querySelectorAll('#sb-cell-size-deck > .card');
	console.log(funcTypeCards);

	let functionTypes = Array.from(funcTypeCards).map((ftc) => {
		// ------ updated version ------
		const name = ftc.querySelector('.cell-type-name').textContent;
		const value = ftc.querySelector('.cell-detection-type').value;
		const recog = ftc.querySelector('.device-recognition').value;

		const target = {
			type: name,
			category: value,
			recognition: [recog],
		};

		if (value === 'rack') {
			// --- dummy attribute `offset` for rack --- 
			target['offset'] = { x: 0, y: 0, theta: 0 };

			let size = ftc.querySelector('.device-size').value;
			size = size.split(',');
			target['size'] = {
				width: Number(size[0]),
				length: Number(size[1]),
				height: Number(size[2])
			};

			const payload = ftc.querySelector('.device-payload').value;
			target['payload'] = Number(payload);
		}
		if (value === 'charger') {
			let offset = ftc.querySelector('.device-offset').value;
			offset = offset.split(',');
			target['offset'] = {
				x: Number(offset[0]),
				y: Number(offset[1]),
				theta: Number(offset[2])
			};

			const models = ftc.querySelector('.device-model');
			let selectedModels = $(models).select2("data");
			selectedModels = selectedModels.map((s) => s.text);
			target['model'] = selectedModels;
		}
		return target;
	});
	functionTypes = [...functionTypes, ...hiddenFunctionTypes];
	// console.log(functionTypes);

	// --- put the change to system ---
	const token = await rmtTokenCheck();
	let res = await fetchPutFunctionTypes(token, functionTypes);
	if (res.ok) {
		notificationMsg(1, 'Function Types are saved on SUCCESS!');
	}
	else {
		notificationMsg(3, 'Function Types are saved on FAILURE!');
	}

	// --- get the update from system ---
	res = await fetchGetFunctionTypes(token);
	functionTypes2_ = await res.json();
}

// --- Here is for showing locating vertices/cell by agent pose ---
// --- get selected agent postion as cell position ---
async function navCellAnchor() {
	const selAgent = document.getElementById("nav-agent-select").value;
	let navAgents = await getAvailableRobotFromFleetList();

	const robot = navAgents.find((agent) => agent.robot_id === selAgent);
	if (!robot) {
		notificationMsg(2, `No matched robot in fleet state anchor function`);
		return;
	}

	$("#cell-x").val(robot.location.x);
	$("#cell-y").val(robot.location.y);
}

async function getAvailableRobotFromFleetList() {
	let navAgents = [];

	const res = await fetchFleetStates(rmtToken_, '', 'agent_only');
	fleetData2_ = [];
	if (!res.ok) {
		notificationMsg(2, "No available robot in this map anchor function may not able to use")
		return;
	}

	const queryData = await res.json();
	if (!queryData.hasOwnProperty("fleet_state")) {
		notificationMsg(2, "No available fleet state in this map anchor function may not able to use")
		return [];
	}

	queryData["fleet_state"].forEach(function (fleet_item, fleet_index) {
		fleet_item["robots"].forEach(function (robot_item, robot_index) {
			navAgents.push(robot_item);
		});
	});

	return navAgents;
}

class CellFactory {
	create(obj) {
		const targetFT = functionTypes2_.find((ft) => ft.type === obj.functionType);
		const styleType = (targetFT?.recognition) ? targetFT.recognition[0] : obj?.cellTypeDetection;
		const visPos = new Point2D(obj.x, obj.y);
		const circle = new fabric.Circle({
			radius: 20,
			fill: getColor(cellDetectionTypes_, styleType, 'bgColor'),
			stroke: getColor(cellDetectionTypes_, styleType, 'bgColor'),
			strokeWidth: 1,
			left: visPos.x,
			top: visPos.y,
			originX: 'center',
			originY: 'center',
			objectCaching: false
		});
		const text = new fabric.Text(obj.labelFullName, {
			left: visPos.x,
			top: visPos.y,
			fontSize: 7,
			originX: 'center',
			originY: 'center'
		});
		const group = new fabric.Group([circle, text], {
			opacity: 0.3,
			selectable: false,
			hasBorders: false,
			hasControls: false
		});
		return group;
	}
}

// --- polling WMS status data ----
const apiQueryData_ = { wms: {} };
// const wmsHandler = {
// 	set: (prevData, prop, newData, d) => {
// 		console.log(prop)
// 		prevData[prop] = newData;
// 		cellLoadRender(prevData.wms);
// 		return true;
// 	}
// }

// function cellLoadRender(states) {
// 	console.log(states);
// 	const cellEditPanel = document.getElementById('cell-popUp');
// 	const paneDisplay = cellEditPanel.style.display;
// 	// --- to tell the cell edit panel appears or not ---
// 	if (paneDisplay == 'none' || paneDisplay == '') { return; }

// 	const cellId = cellEditPanel.querySelector('#cell-label').value;
// 	const trDisplay = cellEditPanel.querySelector('.tr-cell-load').style.display;
// 	// --- to tell the cell live-status appears or not ---
// 	if (trDisplay === 'none') {
// 		const targetState = states.cells.find((c) => c.cell_id === cellId);
// 		const cellLoadInput = cellEditPanel.querySelector('#cell-load');
// 		cellLoadInput.value = targetState.load;
// 	}
// }

// const newApiQueryData_ = new Proxy(apiQueryData_, wmsHandler);
function pollWmsStates(_dataObj, _token, _inteval = 1000) {
	setInterval(async function () {
		try {
			const selMap = document.getElementById('map-select').value;
			const res = await fetchWmsStates(_token, selMap);
			if (!res.ok) { return; }
			// --- dealing with fetching data ---
			// newApiQueryData_.wms = await res.json();
			apiQueryData_.wms = await res.json();
		} catch (e) {
			_dataObj['wms'] = undefined;
		}
	}, _inteval);
};
