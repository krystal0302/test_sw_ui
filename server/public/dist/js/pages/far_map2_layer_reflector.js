// ========================
//   Data-Manager Classes    
// ========================
/*
|       Backend      |      Frontend      |
| ------------------ | ------------------ |
| --------- FUNCTIONAL ATTRIBUTES ------- |
| id                 | id                 |
| type               | type               |
| width              | width              |
| coordinates: []    | sysCenter: Point2D |
|       (NULL)       | cvsCenter: Point2D |
*/

// dependenecies: gMapMeta_, tfCanvas2ROS(), tfROS2Canvas()
class ReflectorDataManager {
	constructor() {
		this.dataset = [];
		this.dataCache = new FarCache();
	}

	// status quo: support only Swarm Core system
	loadFromSystem(data) {
		let reflectors = data?.reflectors;
		console.log(reflectors);
		if (!reflectors) {
			alert("Invalid data format from system!");
		}

		// --- the dataset is reset while load from system ---
		this.dataset = reflectors.map(r => {
			const SYS = { x: r.coordinate[0], y: r.coordinate[1] };
			const CVS = tfROS2Canvas(gMapMeta_, { x: SYS.x, y: SYS.y });

			// push into the data store
			const entity = {
				id: r.id,
				type: r.type,
				cvsCenter: new Point2D(Number(CVS.x), Number(CVS.y)),
				sysCenter: new Point2D(Number(SYS.x), Number(SYS.y)),
			};
			return entity;
		});
		console.log(this.dataset);

		// --- the dataCache is clear while load from system ---
		this.dataCache.flush();
	}

	add(data) {
		const CVS = { x: data.x, y: data.y };
		const SYS = tfCanvas2ROS(gMapMeta_, { x: CVS.x, y: CVS.y });

		// push into the data store
		const entity = {
			id: data.id,
			type: data.type,
			width: data.width,
			cvsCenter: new Point2D(Number(CVS.x), Number(CVS.y)),
			sysCenter: new Point2D(Number(SYS.x), Number(SYS.y)),
		};

		// --- cache the change ---
		this.dataCache.put(genUuid(), { action: 'add', ...entity });

		// add the target
		this.dataset.push(entity);
	}

	delete(data) {
		console.log(data)
		if (!data) { return; }

		// find out the target element by id
		const entity = this.dataset.find((d) => d.id === data.id);

		// --- cache the change ---
		this.dataCache.put(genUuid(), { action: 'delete', ...entity });

		// filter out the target 
		this.dataset = this.dataset.filter((d) => d.id !== data.id);
	}

	// --- update markers by manual input ---
	update(change, coord = 'sys') {
		console.log(' --- now running update ---');
		let target = this.dataset.find((d) => d.id === change.id);
		if (!target) { return; }
		if (!['sys', 'cvs'].includes(coord)) {
			console.error(`Unknown coordinate system: ${coord}`)
			return;
		}

		// --- cache the change ---
		this.dataCache.put(genUuid(), { action: 'update', ...target });

		// --- update the change ---
		// --- by manual input ---
		if (coord === 'sys') {
			target.sysCenter = (change?.sysCenter) ? change.sysCenter : target.sysCenter;
			const center = tfROS2Canvas(gMapMeta_, target.sysCenter);
			target.cvsCenter = new Point2D(Number(center.x), Number(center.y));
			return;
		}
		// --- by drag-n-drop ---
		if (coord === 'cvs') {
			target.cvsCenter = (change?.cvsCenter) ? change.cvsCenter : target.cvsCenter;
			const center = tfCanvas2ROS(gMapMeta_, target.cvsCenter);
			target.sysCenter = new Point2D(Number(center.x), Number(center.y));
			return;
		}
	}

	undo() {
		const changeNode = this.dataCache.popHead();
		const change = changeNode?.value;
		if (!change) { return; }

		// --- update changes to data manager ---
		// case 1: add -> delete
		if (change.action === 'add') {
			this.dataset = this.dataset.filter((d) => d.id !== change.id);
		}

		// case 2: delete -> add
		if (change.action === 'delete') {
			this.dataset.push({
				id: change.id,
				type: change.type,
				width: change.width,
				cvsCenter: change.cvsCenter,
				sysCenter: change.sysCenter,
			});
		}

		// case 3: update (latest > prev.) 
		if (change.action === 'update') {
			const target = this.dataset.find((d) => d.id === change.id);
			target.cvsCenter = (change?.cvsCenter) ? change.cvsCenter : target.cvsCenter;
			target.sysCenter = (change?.sysCenter) ? change.sysCenter : target.sysCenter;
		}

		return change;
	}

	getTarget(id) {
		return this.dataset.find((d) => d.id === id);
	}

	clear() {
		// 1. flush the date store 
		this.dataset = [];
		// 2. flush the date cache 
		this.dataCache.flush();
	}

	// ------ helper functions ------
	print() {
		console.log(this.dataset);
	}

	getAllData() {
		return (this.dataset) ? this.dataset : [];
	}

	getDataInSysCoord() {
		// console.log('--- convert dataset to system scheme ---');
		const targetSchemeData = this.dataset.map((d) => ({
			id: d.id,
			type: d.type,
			width: d.width || 0.1,
			coordinate: [d.sysCenter.x, d.sysCenter.y]
		}));

		return { "reflectors": targetSchemeData };
	}

	isExistUnsavedChanges() {
		return (this.dataCache.getSize() > 0) ? true : false;
	}
}


// ========================
//       Tool Classes    
// ========================
// dependencies: gMapMeta_, tfROS2Canvas()
class AddReflectorTool extends Tool {
	constructor(canvas, dm, options) {
		super(canvas);
		super.changeSelectableStatus(false);
		super.unfreezeLayerObjects('rdReflector');
		super.changeCanvasProperty(false, false);

		this.factory = new ReflectorMarkerFactory();
		this.dataManager = dm;
		this.type = options?.type || "cylinder";
		this.radius = 10;
		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;

		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});

		inst.canvas.on('mouse:up', function (opt) {
			inst.#onMouseUp(opt);
		});
	}

	#onMouseUp(opt) {
		const cursor = opt.pointer;
		if (cursor.x < 0 || cursor.y < 0) { return; }

		// --- dragging case ends ---
		if (opt.target && opt.button === 1 && this.canvas.isDragging) {
			const inst = this;

			// --- update the DataManager ---
			const RADIUS = this.radius;
			const center = new Point2D(Number(opt.target.left + RADIUS), Number(opt.target.top + RADIUS));
			this.dataManager.update({
				id: opt.target.id,
				cvsCenter: center,
			}, 'cvs');

			inst.canvas.isDragging = false;
		}
	}

	#onMouseDown(opt) {
		const cursor = opt.pointer;
		if (cursor.x < 0 || cursor.y < 0) { return; }

		const inst = this;
		let vpt = inst.canvas.viewportTransform;

		const clickPoint = opt.absolutePointer;

		// --- create reflector on click ---
		if (!opt.target && opt.button === 1) {
			// --- protection ---
			// let bIsInterfere = false;
			const diameter = this.radius * 2;
			const data = this.dataManager.getAllData();
			console.log(data)
			const interfereOnes = data.filter((d) => distanceBetweenPoints(clickPoint, d.cvsCenter) < diameter);
			console.log(interfereOnes);
			if (interfereOnes.length) { return; }


			// --- create a reflector and do rendering ---
			const reflector = this.factory.create(this.type, { center: clickPoint });
			inst.canvas.add(reflector);

			this.dataManager.add(reflector);
			return;
		}

		// --- dragging case begin ---
		if (opt.target && opt.button === 1) {
			inst.canvas.isDragging = true;
		}

		// --- edit reflector ---
		if (opt.target && opt.button === 3) {
			// --- get the target data ---
			const target = this.dataManager.getTarget(opt.target.id);
			console.log(target);
			if (!target) { return; }

			// --- fill-out the coordinates from data manager ---
			const reflectorConfigTable = document.getElementById('reflector-popUp');
			reflectorConfigTable.querySelector('#reflector-id').value = target.id;
			reflectorConfigTable.querySelector('#reflector-x').value = target.sysCenter.x;
			reflectorConfigTable.querySelector('#reflector-y').value = target.sysCenter.y;
			reflectorConfigTable.querySelector('#reflector-width').value = target.width || 0.1;
			reflectorConfigTable.style.display = 'block';

			const applyButton = document.getElementById("reflector-createButton");
			applyButton.addEventListener('click', this.#commitConfigChanges.bind(this, opt.target));

			document.getElementById("reflector-cancelButton").onclick = function () {
				reflectorConfigTable.style.display = 'none';
			}
		}
	}

	// --- commit the changes ---
	#commitConfigChanges(target) {
		console.log(target);
		const reflectorConfigTable = document.getElementById('reflector-popUp');

		const InputX = reflectorConfigTable.querySelector('#reflector-x').value;
		const InputY = reflectorConfigTable.querySelector('#reflector-y').value;
		const SYS = { x: Number(InputX), y: Number(InputY) };
		const center = tfROS2Canvas(gMapMeta_, { x: SYS.x, y: SYS.y });
		const CVS = new Point2D(Number(center.x), Number(center.y));

		target.width = Number(reflectorConfigTable.querySelector('#reflector-width').value) || 0.1;
		reflectorConfigTable.style.display = 'none';

		const RADIUS = this.radius;
		// TODO: --- update the CanvasView ---
		// target.set({ left: (CVS.x - RADIUS), top: (CVS.y - RADIUS) });
		target.set({ left: (CVS.x), top: (CVS.y - RADIUS) });
		target.setCoords(); // CRITICAL: to update the cursor detection
		this.canvas.renderAll();

		// --- update the DataManager ---
		this.dataManager.update({
			id: target.id,
			sysCenter: new Point2D(SYS.x, SYS.y),
		}, 'sys');
		console.log(JSON.stringify(this.dataManager.getAllData()));

	}
}

class DelReflectorTool extends Tool {
	constructor(canvas, dm, options) {
		super(canvas);
		super.changeSelectableStatus(false);
		super.changeCanvasProperty(false, false);

		this.dataManager = dm;
		this.#init();
	}

	#init() {
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;

		inst.canvas.on('mouse:down', function (opt) {
			console.log(opt)
			inst.#onMouseDown(opt);
		});
	}

	#onMouseDown(opt) {
		const cursor = opt.pointer;
		// --- [protection] ---
		if (cursor.x < 0 || cursor.y < 0) { return; }

		const inst = this;

		// --- delete reflector ---
		if (opt.target && opt.button === 1) {
			inst.canvas.remove(opt.target);
			// --- data synchronization ---
			this.dataManager.delete(opt.target);
			this.dataManager.print();
		}
	}
}


// dependencies: genUuid(), gMapMeta_, tfCanvas2ROS()
class ReflectorMarkerFactory {
	create(type, options) {
		const ID = options.id || genUuid();
		// console.log(ID);
		const RADIUS = options.radius || 10;

		const CVS = { x: options.center.x, y: options.center.y };
		const SYS = tfCanvas2ROS(gMapMeta_, { x: CVS.x, y: CVS.y });

		const commonOptions = {
			id: ID,
			x: CVS.x,
			y: CVS.y,
			type: type,
			stroke: '#FF0000',
			strokeWidth: 4,
			layer: 'rdReflector',
		}

		if (type === "cylinder") {
			return new fabric.Circle({
				radius: RADIUS,
				// fill: 'transparent',
				fill: 'white',
				left: CVS.x,         // cursor x is align the circle center
				top: CVS.y - RADIUS,
				hasBorders: false,
				objectCaching: false,
				hasControls: false,
				selectable: true,
				opacity: 1,
				...commonOptions
			});
		}
		if (type === "board") {
			// --- create board reflector case ---
			return new fabric.Line([CVS.x - RADIUS, CVS.y, CVS.x + RADIUS, CVS.y], {
				hasControls: false,
				selectable: true,
				...commonOptions
			});
		}
	}
}


// =====================================
//   Reflector Layer UI Event-bindings     
// =====================================
const panZoomReflector = document.getElementById('pan-zoom-reflector');
const addCylinderReflector = document.getElementById('add-cylinder-reflector');
const addBoardReflector = document.getElementById('add-board-reflector');
const delReflector = document.getElementById('delete-reflector');
const undoReflector = document.getElementById('undo-reflector');

let reflectorDM = new ReflectorDataManager();
let reflectorTool;

panZoomReflector.addEventListener('click', () => {
	const zoom = fCanvas.width / gMapData_.w;
	reflectorTool = new PanZoom(fCanvas);
	reflectorTool.setMinZoomRatio(zoom);
});

addCylinderReflector.addEventListener('click', () => {
	reflectorTool = new AddReflectorTool(fCanvas, reflectorDM, { type: "cylinder" });
});
addBoardReflector.addEventListener('click', () => {
	reflectorTool = new AddReflectorTool(fCanvas, reflectorDM, { type: "board" });
});
delReflector.addEventListener('click', () => {
	reflectorTool = new DelReflectorTool(fCanvas, reflectorDM);
});
undoReflector.addEventListener('click', () => {
	// 1. undo DataManager 
	reflectorDM.undo();
	// 2. re-render CanvasView 
	const allData = reflectorDM.getAllData();
	canvasView.renderObjects(allData);
})
