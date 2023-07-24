// ========================
//   Data-Manager Classes    
// ========================
class MapImageDataManager {
	constructor(canvas) {
		this.dataset = canvas._objects;
	}

	undo() {
		this.dataset.pop();
	}

	isExistUnsavedChanges() {
		return (this.dataset.length > 0) ? true : false;
	}
}


// ========================
//       Tool Classes    
// ========================
// ------ Pan & Zoom ------ 
class PanZoom extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeCanvasProperty(false, false);

		this.#init();
	}

	#init() {
		this.zoomRange = { min: 0.1, max: 20 }
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;

		inst.canvas.on('mouse:wheel', function (opt) {
			inst.#onMouseWheel(opt);
		});
		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});
		inst.canvas.on('mouse:move', function (opt) {
			inst.#onMouseMove(opt);
		});
		inst.canvas.on('mouse:up', function (opt) {
			inst.#onMouseUp(opt);
		});
		inst.canvas.on('object:moving', function (opt) {
			inst.#disable(opt);
		})
	}

	setMinZoomRatio(zoom) {
		this.zoomRange.min = zoom || 0.1;
	}

	#onMouseWheel(opt) {
		const inst = this;
		let delta = opt.e.deltaY;
		let zoom = inst.canvas.getZoom();
		zoom *= 0.999 ** delta;
		if (zoom > this.zoomRange.max) zoom = this.zoomRange.max;
		if (zoom < this.zoomRange.min) zoom = this.zoomRange.min;
		// -- take mouse cursor as an anchor ---
		inst.canvas.zoomToPoint({ x: opt.e.offsetX, y: opt.e.offsetY }, zoom);

		opt.e.preventDefault();
		opt.e.stopPropagation()
	}

	#onMouseDown(opt) {
		const inst = this;
		let evt = opt.e;
		inst.canvas.isDragging = true;
		inst.canvas.selection = false;
		if (evt instanceof TouchEvent) {
			const { clientX, clientY } = evt.touches[0]
			inst.canvas.lastPosX = clientX
			inst.canvas.lastPosY = clientY
		} else {
			inst.canvas.lastPosX = evt.clientX;
			inst.canvas.lastPosY = evt.clientY;
		}
		if (opt.target === null) return;
		if (opt.target.id === 'mapOrigin') {
			inst.canvas.isDragging = false;
		}
	};

	#onMouseMove(opt) {
		const inst = this;
		if (!inst.canvas.isDragging) { return; }
		const e = opt.e;
		let vpt = inst.canvas.viewportTransform;
		if (e instanceof TouchEvent) {
			const { clientX, clientY } = e.touches[0];
			vpt[4] += (clientX - inst.canvas.lastPosX);
			vpt[5] += (clientY - inst.canvas.lastPosY);
			inst.canvas.requestRenderAll();
			inst.canvas.lastPosX = clientX;
			inst.canvas.lastPosY = clientY;
		} else {
			vpt[4] += (e.clientX - inst.canvas.lastPosX);
			vpt[5] += (e.clientY - inst.canvas.lastPosY);
			inst.canvas.requestRenderAll();
			inst.canvas.lastPosX = e.clientX;
			inst.canvas.lastPosY = e.clientY;
		}
	};

	#onMouseUp(opt) {
		const inst = this;
		inst.canvas.setViewportTransform(inst.canvas.viewportTransform);
		inst.canvas.isDragging = false;
		inst.canvas.selection = true;
	};

	#isEnable() {
		return this.canvas.isDrawingMode;
	}

	#enable() {
		this.canvas.isDrawingMode = true;
	}

	#disable() {
		this.canvas.isDrawingMode = false;
	}
}

// ------ Line ------ 
class Line extends Tool {
	constructor(canvas) {
		super(canvas);
		super.changeSelectableStatus(false);
		super.changeCanvasProperty(false, false);

		this.#init();
	}

	getWidth() { return this.strokeWidth_; }

	setWidth(w) {
		this.strokeWidth_ = w;
	}

	#init() {
		this.strokeWidth_ = 2;
		this.#bindEvents();
	}

	#bindEvents() {
		const inst = this;

		inst.canvas.on('mouse:down', function (opt) {
			inst.#onMouseDown(opt);
		});
		inst.canvas.on('mouse:move', function (opt) {
			inst.#onMouseMove(opt);
		});
		inst.canvas.on('mouse:up', function (opt) {
			inst.#onMouseUp(opt);
		});
		inst.canvas.on('object:moving', function (opt) {
			inst.#disable(opt);
		})
	}

	#onMouseUp(opt) {
		const inst = this;
		if (inst.#isEnable()) {
			inst.#disable();
		}
	};

	#onMouseMove(opt) {
		const inst = this;
		if (!inst.#isEnable()) { return; }

		const pointer = inst.canvas.getPointer(opt.e);
		const activeObj = inst.canvas.getActiveObject();

		activeObj.set({
			x2: pointer.x,
			y2: pointer.y
		});
		activeObj.setCoords();
		inst.canvas.renderAll();
	};

	#onMouseDown(opt) {
		const inst = this;
		inst.#enable();

		const pointer = inst.canvas.getPointer(opt.e);
		const points = [pointer.x, pointer.y, pointer.x, pointer.y];
		const line = new fabric.Line(points, {
			strokeWidth: inst.strokeWidth_,
			stroke: 'black',
			fill: 'black',
			originX: 'center',
			originY: 'center',
			selectable: false,
			hasBorders: false,
			hasControls: false,
			layer: 'rdMapImg',
			selectable: true,
		});
		inst.canvas.add(line).setActiveObject(line);
	};

	#isEnable() {
		return this.canvas.isDrawingMode;
	}

	#enable() {
		this.canvas.isDrawingMode = true;
	}

	#disable() {
		this.canvas.isDrawingMode = false;
	}
}

// --- Eraser ---
class Eraser extends Tool {
	static instance = null;

	constructor(canvas) {
		super(canvas);
		super.changeSelectableStatus(false);
		super.changeCanvasProperty(false, true);

		this.#init();
	}

	static getInstance(canvas) {
		if (Eraser.instance == null) {
			Eraser.instance = new Eraser(canvas);
		} else {
			Eraser.instance.#recoverCanvasState();
		}

		return Eraser.instance;
	}

	#recoverCanvasState() {
		this.canvas.__eventListeners = {};
		this.canvas.forEachObject(function (obj) {
			obj.selectable = false;
		})
		this.canvas.selection = false;
		this.canvas.isDrawingMode = true;
		this.canvas.renderAll();

		this.#updateCursorSize()
	}

	getWidth() { return this.strokeWidth_; }

	setWidth(w) {
		this.strokeWidth_ = Number(w);
		this.canvas.freeDrawingBrush.width = Number(w);

		this.#updateCursorSize()
	}

	#updateCursorSize() {
		const zoom = this.canvas.getZoom();
		const sw = this.strokeWidth_ * zoom;

		this.canvas.freeDrawingCursor = `url(${this.getDrawCursor(sw)}) ${sw / 2} ${sw / 2}, crosshair`;
	}

	#init() {
		this.strokeWidth_ = 5;

		this.canvas.freeDrawingBrush.color = "#FFFFFF";
		this.canvas.freeDrawingBrush.strokeLineCap = 'square';
		this.canvas.freeDrawingBrush.strokeLineJoin = 'bevel';

		this.setWidth(this.strokeWidth_);
	}

	getDrawCursor(brushSize) {
		const square = `
				<svg
					height="${brushSize}"
					fill="white"
					viewBox="0 0 ${brushSize * 2} ${brushSize * 2}"
					width="${brushSize}"
					xmlns="http://www.w3.org/2000/svg"
				>
						<rect x="0" y="0" width="${brushSize * 2}" height="${brushSize * 2}"
			style="fill:white;stroke:black;stroke-width:1;fill-opacity:0.4;stroke-opacity:1.0" />
				</svg>
			`;

		return `data:image/svg+xml;base64,${window.btoa(square)}`;
	};
}

// ============================
//     Supporting Functions    
// ============================
function drawFabricMap(_mapData, _viewSize, _canvas) {
	let img = new Image();
	const mapDataURL = `data:image/png;base64,${_mapData}`;
	img.src = mapDataURL;

	img.onload = function () {
		const maxDim = Math.max(img.width, img.height);
		const zoom = (maxDim >= _viewSize) ? (_viewSize / maxDim) : 1.0;
		const viewWidth = img.width * zoom;
		const viewHeight = img.height * zoom;

		const f_img = new fabric.Image(img);

		_canvas.setWidth(viewWidth);
		_canvas.setHeight(viewHeight);
		_canvas.setZoom(zoom);

		_canvas.setBackgroundImage(f_img).renderAll();
	};
}

// --- object manipulation on fabric ---
let originSprite;
let isOriginSet_ = true;

function btnEditOrigin() {
	isOriginSet_ = false;

	// --- switch off drawing mode ---
	fCanvas.isDrawingMode = false;

	fabric.Image.fromURL(`${getImagePath()}/ucs.png`, function (myImg) {
		// --- get origin position ---
		const currPos = tfROS2Canvas(gMapMeta_, new Point2D(0.0, 0.0));

		const currLeft = (currPos.x - (mapOrigin_.width / 2));
		const currTop = (currPos.y - (mapOrigin_.height / 2));

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

	// --- fabric canvas fit to view port ---
	// --------------------- ([zoom, 0, 0, zoom, panX, panY]) ---
	fCanvas.setViewportTransform([1, 0, 0, 1, 0, 0]);

	const dataInBase64 = fCanvas.toDataURL({
		format: 'png',
		top: 0,
		left: 0,
		width: gMapData_.w,
		height: gMapData_.h
	});
	const mapImageInBase64 = dataInBase64.split(',')[1]; // strip type from base64 string 

	// --- by Swarm APIs ---
	const selMap = document.getElementById('map-select').value;
	const res = await fetchPutMapImage(rmtToken_, selMap, mapImageInBase64);
	const statusText = await res.json();
	if (res.ok) {
		notificationMsg(1, statusText);
	} else {
		notificationMsg(3, statusText);
	}
}


// ===============================
//   Map Layer UI Event-bindings    
// ===============================
const panZoomMap = document.getElementById('pan-zoom-map');
const lineDraw = document.getElementById('line-draw');
const strokeWidthSlider = document.getElementById('inWidthSlider');
const eraseDraw = document.getElementById('erase-draw');

const undoDraw = document.getElementById('undo-draw');
const getOrigin = document.getElementById('get-origin');
const setOrigin = document.getElementById('set-origin');
const cancelOrigin = document.getElementById('cancel-origin');

let mapImageDM = new MapImageDataManager(fCanvas);
let mapTool;

panZoomMap.addEventListener('click', () => {
	document.getElementById('additional-tools').style.display = 'none';
	// --- reset origin related buttons
	cancelOriginEditingMode();

	const zoom = fCanvas.width / gMapData_.w;
	mapTool = new PanZoom(fCanvas);
	mapTool.setMinZoomRatio(zoom);
});

lineDraw.addEventListener('click', function () {
	document.getElementById('additional-tools').style.display = 'block';

	mapTool = new Line(fCanvas);
	document.getElementById('stroke-width-span').innerHTML = 'Line Width';
	document.getElementById('inWidthSlider').value = mapTool.getWidth();
	document.getElementById('outWidthSlider').value = mapTool.getWidth();
})

strokeWidthSlider.addEventListener('click', function () {
	const lineWidth = Number(document.getElementById('outWidthSlider').value);
	mapTool.setWidth(lineWidth);
})

eraseDraw.addEventListener('click', () => {
	document.getElementById('additional-tools').style.display = 'block';

	mapTool = Eraser.getInstance(fCanvas);
	document.getElementById('stroke-width-span').innerHTML = 'Eraser Width';
	document.getElementById('inWidthSlider').value = mapTool.getWidth();
	document.getElementById('outWidthSlider').value = mapTool.getWidth();
});

undoDraw.addEventListener('click', () => {
	// --- protection ---
	if (!mapImageDM.isExistUnsavedChanges()) { return; }

	// --- update DataManager ---
	mapImageDM.undo();

	// --- update CanvasView ---
	canvasView.renderObjects();
});