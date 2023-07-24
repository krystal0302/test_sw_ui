// =================================
//     Cache Mechanisom Classes    
// =================================
// --- Cache for recording User operation ---
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

	getSize() {
		return this.size;
	}
}

// =================================
//       Abstract Tool Classes    
// =================================
class Tool {
	constructor(canvas) {
		this.idCount = 0;
		this.isDrawing = false;
		this.canvas = canvas;
		// --- in-case the event is not initialized ---
		if (canvas?.hasOwnProperty("__eventListeners")) {
			this.#purgeCanvasMouseEvents(this.canvas);
		}
	}

	#purgeCanvasMouseEvents(canvas) {
		// console.log(typeof canvas);
		// TODO: check it's fabric canvas
		// canvas.__eventListeners["mouse:up"] = [];
		// canvas.__eventListeners["mouse:move"] = [];
		// canvas.__eventListeners["mouse:down"] = [];
		// canvas.__eventListeners["object:moving"] = [];
		canvas.__eventListeners = {};
	}

	changeSelectableStatus(bSelectable) {
		this.canvas.forEachObject(function (obj) {
			obj.selectable = bSelectable;
		})
		this.canvas.renderAll();
	}

	unfreezeLayerObjects(layer) {
		this.canvas.forEachObject(function (obj) {
			obj.selectable = (obj.layer === layer) ? true : false;
		})
		this.canvas.renderAll();
	}

	changeCanvasProperty(bSelection, bDrawMode) {
		this.canvas.selection = bSelection;
		this.canvas.isDrawingMode = bDrawMode;
	}
}

class VisTool {
	constructor(canvas) {
		this.canvas = canvas;
		this.#purgeCanvasMouseEvents(this.canvas);
	}

	#purgeCanvasMouseEvents(canvas) {
		// canvas.off();
	}
}


// ===================================
//     Fabric Canvas Instantiation     
// ===================================
let fCanvas = new fabric.Canvas('fabric_canvas', {
	isDrawingMode: false,
	enableRetinaScaling: false,
	preserveObjectStacking: true,
	fireRightClick: true,
	stopContextMenu: true,
});