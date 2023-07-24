// ========================
//   Data-Manager Classes    
// ========================
// === route vertex ===
/*
|     Backend      |    DataManager    |     VisNetwork    |
| ---------------- | ----------------- | ----------------- |
| id               | id                | id                |
|      (NULL)      | x                 | x                 |
|      (NULL)      | y                 | y                 |
|      (NULL)      | visPos.x          | visPos.x          |
|      (NULL)      | visPos.y          | visPos.y          |
| position_x       | sysPos.x          | sysPos.x          |
| position_y       | sysPos.y          | sysPos.y          |
| orientation      | r                 | r                 |
|      (NULL)      | group             | group             |
| label            | label             | label             |
|      (NULL)      | title             | title             |
*/
// === route edge ===
/*
|     Backend      |     Frontend      |
| ---------------- | ----------------- |
|      (NULL)      | id                |
| from             | from              |
| to               | to                |
| weight           | weight            |
| width            | width             |
*/

// dependencies: gMapMeta_, cvtDot2Json(),
class RouteDataManager {
	constructor(canvas) {
		this.canvas = canvas;
		this.dataset = {};
		this.dataCache = new FarCache();
	}

	loadFromSystem(data) {
		this.dataset = cvtDot2Json(data);
		console.log(this.dataset);

		// --- clear the dataCache ---
		this.dataCache.flush();
	}

	addVertex(data, coord = 'sys') {
		// --- cache the operation ---
		let VIS, SYS;
		if (coord === 'sys') {
			SYS = data.sysPos;
			const pos = tfROS2Canvas(gMapMeta_, SYS);
			VIS = new Point2D(pos.x, pos.y);

			// data transformation & update 
			data.x = Number(VIS.x);
			data.y = Number(VIS.y);
			data.visPos = VIS;
		} else {
			VIS = data.visPos;
			const pos = tfCanvas2ROS(gMapMeta_, VIS);
			SYS = new Point2D(pos.x, pos.y);

			// data transformation & update
			data.sysPos = SYS;
		}

		const diffObj = {
			action: 'add',
			type: 'vertex',
			...data
		}
		console.log(diffObj);

		this.dataCache.put(genUuid(), diffObj);

		// --- check the node exist or not ---
		const targetNode = this.dataset.nodes.find((n) => n.id === data.id);
		if (!targetNode) {
			this.dataset.nodes.push(data);
			console.log('add a new vertex');
			return;
		}
	}

	editVertex(change, coord = 'sys') {
		console.log(change);
		// --- cache the operation ---
		const target = this.dataset.nodes.find((d) => d.id === change.id);
		console.log(target);
		this.dataCache.put(genUuid(), { action: 'edit', type: 'vertex', ...target });

		// --- update the dataset ---
		if (coord === 'sys') {
			const SYS = change.sysPos;
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

		this.dataset.nodes = this.dataset.nodes.filter((node) => node.id !== change.id);
		this.dataset.nodes.push(change);
	}

	deleteVertexById(id) {
		// --- get the connections ---
		const connections = visNetwork_.body.nodes[id].edges.map(edge => {
			return {
				id: edge.id,
				from: edge.fromId,
				to: edge.toId,
				weight: "1",
				width: "0.0",
			};
		});
		console.log(connections);
		// console.log(this.canvas.body.data.nodes.get(id));

		// --- cache the operation ---
		console.log(this.dataset);
		console.log(this.dataset.nodes);
		const target = this.dataset.nodes.find((n) => n.id === id);
		console.log(target);
		const diffObj = {
			action: 'delete',
			type: 'vertex',
			...target,
			connections: connections
		};
		console.log('--- deleting the selected vertex ---');
		this.dataCache.put(genUuid(), diffObj);

		// --- remove the vertex from dataset ---
		this.dataset.nodes = this.dataset.nodes.filter((aa) => aa.cell_uuid !== id);
	}

	addEdge(data) {
		// --- cache the operation ---
		const diffObj = {
			action: 'add',
			type: 'edge',
			...data
		};
		// console.log(diffObj);

		this.dataCache.put(genUuid(), diffObj);

		// --- add the edge from dataset ---
		this.dataset.edges.push(data);
	}

	deleteEdge(data) {
		// --- cache the operation ---
		console.log(data);
		const diffObj = {
			action: 'delete',
			type: 'edge',
			...data
		};
		console.log(diffObj);

		this.dataCache.put(genUuid(), diffObj);

		// --- remove the edge from dataset ---
		this.dataset.edges = this.dataset.edges.filter((edge) => !(edge.from === data.id && edge.to === data.id));
	}

	undo() {
		const inst = this;
		let change = inst.dataCache.popHead();
		console.log(change);
		change = change?.value;
		console.log(change);
		if (!change) { return; }

		// case1: action: add. UNDO (add -> delete) ---
		if (change.action === 'add' && change.type === 'vertex') {
			console.log(' --- undo add vertex case --- ');
			// --- update visCache ---
			inst.canvas.body.data.nodes.remove(change.id);
			// --- update rosCache ---
			inst.dataset.nodes = inst.dataset.nodes.filter((ca) => ca.id !== change.id);
			return;
		}

		// case2: action -> edit 
		if (change.action === 'edit' && change.type === 'vertex') {
			// --- update visCache ---
			const inst = this;
			inst.canvas.body.data.nodes.update(change);;
			// --- update data manager ---
			inst.dataset.nodes = inst.dataset.nodes.filter((co) => co.id !== change.id);
			inst.dataset.nodes.push(change);
			return;
		}

		// case3: action: delete. UNDO (delete -> add) ---
		if (change.action === 'delete' && change.type === 'vertex') {
			// console.log(change);
			// --- recover the deleted vertex ---
			const recoverNode = {
				id: change.id,
				label: change.label,
				x: change.x,
				y: change.y,
				group: change.group,
				title: change.title
			}
			inst.canvas.body.data.nodes.update(recoverNode); // vis-canvas view 
			inst.dataset.nodes.push(recoverNode);            // data manager

			// --- recover the deleted vertex connections(edges)  ---
			const recoverEdges = change.connections;
			inst.canvas.body.data.edges.add(recoverEdges);  // vis-canvas view
			inst.dataset.edges.concat(recoverEdges);        // data manager
			return;
		}

		// case4: [edge] action: add. UNDO (add -> delete) ---
		if (change.action === 'add' && change.type === 'edge') {
			// --- update visCache ---
			// --- find the target edge id ---
			const edges = inst.canvas.getConnectedEdges(change.from);
			const targetEdgeId = edges.find(edgeId => {
				const edge = inst.canvas.body.edges[edgeId];
				return edge.fromId === change.from && edge.toId === change.to;
			});
			// console.log(targetEdgeId);

			inst.canvas.body.data.edges.remove(targetEdgeId);
			// --- update rosCache ---
			inst.dataset.edges = inst.dataset.edges.filter((ca) => ca.id !== targetEdgeId);
			return;
		}

		// case5: [edge] action: delete. UNDO (delete -> add) ---
		if (change.action === 'delete' && change.type === 'edge') {
			// console.log(change);
			// --- update visCache ---
			inst.canvas.body.data.edges.update({
				id: change.id,
				from: change.fromId,
				to: change.toId
			});

			// --- update rosCache ---
			inst.dataset.edges.push({
				from: change.fromId,
				to: change.toId,
				weight: "1",
				width: "0.0"
			});
			return;
		}
	}

	clear() {
		this.dataset = {};
		this.dataCache.flush();
	}

	getAllData() {
		return this.dataset;
	}

	getTargetVertex(id) {
		return this.dataset.nodes.find((d) => d.id === id);
	}

	updateVertexPosByLabel(label, pos) {
		console.log(this.dataset);
		const target = this.dataset.nodes.find((n) => n.label === label);
		if (!target) {
			console.error('found no target by label');
			return;
		}
		target.x = pos.x;
		target.y = pos.y;
	}

	getEdgeEndVertex(controlEdge) {
		return this.dataset.nodes.filter((ng) => ng.id === controlEdge.from || ng.id === controlEdge.to);
	}

	getAllVertexLabels() {
		return this.dataset.nodes.map((gn) => gn.label);
	}

	isDuplicatedLabel(label) {
		const existingLabels = this.dataset.nodes.map((gn) => gn.label);
		return existingLabels.includes(label);
	}

	isExistUnsavedChanges() {
		return (this.dataCache.getSize() > 0) ? true : false;
	}
}


// ========================
//       Tool Classes    
// ========================
class AddRouteVertexTool extends VisTool {
	constructor(canvas, dm, options) {
		super(canvas);

		// --- vis-config ---
		this.#initCanvasNodes(this.canvas, 'navnode');

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
		$('#edit-status-badge').text('Add Vertex Mode');
		showCancelEditButton(true);
		clearNodePopUp();

		// --- switch edit edge mode ---
		const options = {
			interaction: {
				dragNodes: true,
			},
		}
		this.canvas.setOptions(options);
	}

	#bindEvents() {
		const inst = this;

		// --- bind mouse events ---

		// --- bind vis-canvas manipulation events ---
		const cfgAddNode = (data, callback) => { return inst.#onAddRouteVertex(data, callback, inst); };
		const cfgEditNode = (data, callback) => { return inst.#onEditRouteVertex(data, callback, inst); };
		const cfgDeleteNode = (data, callback) => { return inst.#onDeleteRouteVertex(data, callback, inst); };
		const options = {
			manipulation: {
				enabled: false,
				addNode: cfgAddNode,
				editNode: cfgEditNode,
				deleteNode: cfgDeleteNode
			}
		};

		const cfgDragEndNode = (params) => { return inst.#onDragEndRouteVertex(params, inst); };
		inst.canvas.off("dragEnd");
		inst.canvas.on("dragEnd", cfgDragEndNode);

		inst.canvas.setOptions(options);
	}

	#onAddRouteVertex(data, callback, self) {
		// --- update View ---
		document.getElementById("node-operation").innerText = "Add Vertex";
		document.getElementById("node-createButton").value = "Create";

		data.id = 'fa' + data.id;

		document.getElementById("node-label").value = data.label;

		// --- update node location from cache ---
		const rosPos = tfCanvas2ROS(gMapMeta_, { x: data.x, y: data.y });
		document.getElementById("node-x").value = rosPos.x;
		document.getElementById("node-y").value = rosPos.y;

		const createBtn = document.getElementById("node-createButton");
		const cancelBtn = document.getElementById("node-cancelButton");
		document.getElementById("node-createButton").onclick = self.#onSaveRouteVertex.bind(createBtn, data, callback, self, 'add');
		document.getElementById("node-cancelButton").onclick = clearNodePopUp.bind(cancelBtn, callback);

		document.getElementById("node-popUp").style.display = "block";
		updatePopUpPosition('node');

		document.getElementById('nav-agent').style.display = 'block';
	}

	#onEditRouteVertex(data, callback, self) {
		// --- filling in the popup DOM elements ---
		document.getElementById("node-operation").innerText = "Edit Vertex";
		document.getElementById("node-createButton").innerText = "OK"; // "Save"

		document.getElementById("node-label").value = data.label;

		// --- edit selected nav-graph node case ---
		const vertexNode = self.dataManager.getTargetVertex(data.id);
		console.log(vertexNode);
		document.getElementById("node-x").value = vertexNode.sysPos.x;
		document.getElementById("node-y").value = vertexNode.sysPos.y;

		const createBtn = document.getElementById("node-createButton");
		const cancelBtn = document.getElementById("node-cancelButton");
		document.getElementById("node-createButton").onclick = self.#onSaveRouteVertex.bind(createBtn, data, callback, self, 'edit');
		document.getElementById("node-cancelButton").onclick = cancelNodeEdit.bind(cancelBtn, callback);

		document.getElementById("node-popUp").style.display = "block";
		updatePopUpPosition('node');

		document.getElementById('nav-agent').style.display = 'block';
	}

	#onDeleteRouteVertex(data, callback, self) {
		const deletedNode = self.canvas.body.data.nodes.get(data.nodes[0]);
		// console.log(deletedNode);

		// --- cache the difference ---
		self.dataManager.deleteVertexById(deletedNode.id);

		// --- [visCache] execute delete the cell node ---
		callback(data);
	}

	#onSaveRouteVertex(data, callback, self, _mode) {
		// --- cancel the manipulation mode ---
		// closeManipulationMode();
		visCanvas.setMode('rdRoute');

		// --- start saving Route Vertex operation ---
		const nodeLabel = document.getElementById("node-label").value;

		// -- [protection] prevent from duplcated vertex name --
		const bValidInput = inputCheck(nodeLabel);
		document.getElementById("node-label").style["border-color"] = (bValidInput) ? "" : "red";

		if (!bValidInput) {
			notificationMsg(2, 'Input include invalid characters.');
			return;
		}

		const allData = self.dataManager.getAllData();
		const allExistingLabels = allData.nodes.map((d) => d.label);

		if (_mode === 'add' && allExistingLabels.includes(nodeLabel)) {
			alert('DUPLICATED VERTEX LABEL!');
			return;
		}
		if (_mode === 'edit' && data.label !== nodeLabel && allExistingLabels.includes(nodeLabel)) {
			alert('DUPLICATED VERTEX LABEL!');
			return;
		}

		// === data assignment ===
		const x = document.getElementById("node-x").value;
		const y = document.getElementById("node-y").value;
		const sysPos = new Point2D(x, y);
		const visPos = tfROS2Canvas(gMapMeta_, sysPos);

		// --- functional attributes ---
		data.id = data.id.replace(/-/g, '');
		data.x = Number(visPos.x);
		data.y = Number(visPos.y);
		data.group = 'navnode';
		data.sysPos = sysPos;

		// --- styling attributes ---
		data.label = document.getElementById("node-label").value;
		data.shape = 'dot';
		data.size = 5;
		const tooltip = `<div class="farobot-map-tooltip" style="color: gray;">
		                   ${data.label}@(${Number(sysPos.x).toFixed(2)}, ${Number(sysPos.y).toFixed(2)})
										 </div>`;
		data.title = tooltip;
		data.color = 'gray';
		data.font = {
			size: 12,
			color: 'black'
		};

		// === update changes to data manager ===
		// console.log(data); 
		clearNodePopUp();
		if (_mode === 'add') {
			self.dataManager.addVertex(data, 'sys');
		}
		if (_mode === 'edit') {
			self.dataManager.editVertex(data);
		}
		// console.log(self.dataManager.getAllData());

		console.log(data);
		self.canvas.body.data.nodes.update(data); // write-in to the cache
		self.canvas.addNodeMode();

		// --- enable all route vertices draggable ---
		self.#initCanvasNodes(self.canvas, 'navnode');
	}

	#initCanvasNodes(canvas, nodeType) {
		// --- enable all route vertices draggable ---
		const networkNodes = Object.values(canvas.body.nodes);
		networkNodes
			.filter((node) => node.options.group === nodeType)
			.forEach((node) => {
				node.options.fixed.x = node.options.fixed.y = false;
			});
	}

	#onDragEndRouteVertex(params, self) {
		// --- protection ---
		if (!params.nodes.length) { return; }

		// --- get node id ---
		const nodeId = params.nodes[0];

		// --- get node group ---
		const dragNode = self.canvas.body.nodes[nodeId];
		const group = dragNode.options.group;
		console.log(dragNode);

		// --- update data cache ---
		if (group !== 'navnode') { return; }

		// --- update the position after dragging ---
		self.canvas.body.data.nodes.update({ ...dragNode.options, x: dragNode.x, y: dragNode.y }); // write-in to the cache

		dragNode.options.visPos = new Point2D(dragNode.x, dragNode.y);
		// --- perserve vertex shape, the shape mutates by dragging ---
		self.dataManager.editVertex({ ...dragNode.options, x: dragNode.x, y: dragNode.y }, 'vis');
	}
}

class AddRouteEdgeTool extends VisTool {
	constructor(canvas, dm, options) {
		super(canvas);

		this.dataManager = dm;
		this.options = options || {};

		this.#init();
	}

	#init() {
		this.#bindEvents();

		// --- legacy ---
		this.canvas.addEdgeMode();
		// document.getElementById('edit-status').style.display = 'block';
		$('#edit-status').show();
		$('#edit-status-title').text('Status.');
		$('#edit-status-badge').text('Add Edge Mode');
		showCancelEditButton(true);
		clearNodePopUp();

		// --- switch edit edge mode ---
		const options = {
			interaction: {
				dragNodes: false,
			},
		}
		this.canvas.setOptions(options);
	}

	#bindEvents() {
		const inst = this;

		// --- bind vis-canvas manipulation events ---
		const cfgAddEdge = (data, callback) => { return inst.#onAddRouteEdge(data, callback, inst); };
		const cfgDeleteEdge = (data, callback) => { return inst.#onDeleteRouteEdge(data, callback, inst); };
		const options = {
			manipulation: {
				enabled: false,
				addEdge: cfgAddEdge,
				deleteEdge: cfgDeleteEdge
			}
		};
		inst.canvas.setOptions(options);

		const cfgDragEndNode = (params) => { return inst.#onDragEndRouteEdge(params, inst); };
		inst.canvas.off("dragEnd");
		inst.canvas.on("dragEnd", cfgDragEndNode);
	}

	#onAddRouteEdge(data, callback, self) {
		console.log(data);
		const allData = self.dataManager.getAllData();
		const vertices = allData.nodes || [];
		// --- [protection] connect vertex itself ---
		//  1. CANNOT connect with itself
		if (data.from === data.to) {
			self.canvas.addEdgeMode();
			return;
		}
		// 2. cell are not allowed to be connected
		const fromEnd = vertices.find((v) => v.id === data.from);
		const toEnd = vertices.find((v) => v.id === data.to);
		if (!fromEnd || !toEnd) {
			return;
		}


		// --- add edge to data manager ---
		self.dataManager.addEdge(data);
		callback(data);

		self.canvas.addEdgeMode();
	}

	#onDeleteRouteEdge(data, callback, self) {
		// console.log(data);
		const delID = data.edges[0];
		const obj = visNetwork_.body.edges[delID];

		routeDM.deleteEdge(obj);

		callback(data);
	}

	#onDragEndRouteEdge(params, self) {
		// ---- [protection] pervent from duplicated callback ----
		console.log(params);
		if (!params?.controlEdge?.from || !params?.controlEdge?.to) { return; }

		// ---- [protection] ensure the both ends of an edge are nav-graph nodes ----
		// const edges = visNetwork_.body.edges;
		const edges = self.canvas.body.edges;
		const edgeArr = Object.values(edges);
		const edgeEnds = routeDM.getEdgeEndVertex(params.controlEdge);
		if (edgeEnds.length < 2) {
			let targetEdge = edgeArr.find((e) => e.fromId === params.controlEdge.from && e.toId === params.controlEdge.to);
			// visNetwork_.body.data.edges.remove([targetEdge.id]);
			// visNetwork_.addEdgeMode();
			self.canvas.body.data.edges.remove([targetEdge.id]);
			self.canvas.addEdgeMode();
			return;
		}

		// ---- [protection] prevent from adding duplicated edges ----
		let duplicatedEdges = edgeArr.filter((edge) => edge.fromId === params.controlEdge.from && edge.toId === params.controlEdge.to);
		if (duplicatedEdges.length > 1) {
			let toDeleteEdges = duplicatedEdges.map((de) => de.id);
			toDeleteEdges?.shift(); // skip the original one
			// visNetwork_.body.data.edges.remove(toDeleteEdges);
			// visNetwork_.addEdgeMode();
			self.canvas.body.data.edges.remove(toDeleteEdges);
			self.canvas.addEdgeMode();
			return;
		}

		// TODO: cache the route edges
	}

}


// =================================
//   Route Layer UI Event-bindings    
// =================================
let routeDM = new RouteDataManager(visNetwork_);
let routeTool;

const cancelRouteEdit = document.getElementById('cancel-route-edit');
const addRouteVertex = document.getElementById('add-node');
const editRouteVertex = document.getElementById('edit-node');
const addRouteEdge = document.getElementById('add-edge');
const editRouteEdge = document.getElementById('edit-edge');
const deleteSelected = document.getElementById('delete-node');
const undoRoute = document.getElementById('undo-route');

cancelRouteEdit.addEventListener('click', (e) => {
	closeManipulationMode();
	visCanvas.setMode(e.target.id === 'cancel-route-edit' ? 'rdRoute' : 'rdCell');
	// visCanvas.setOptions({ manipulation: { enabled: false } });
});

// ------ route editing ------
addRouteVertex.addEventListener('click', () => {
	// --- updated ---
	routeTool = new AddRouteVertexTool(visNetwork_, routeDM);
	showManipulationModeText("Add Vertex")
});
editRouteVertex.addEventListener('click', () => {
	visNetwork_.editNode();
});

addRouteEdge.addEventListener('click', () => {
	// --- updated ---
	routeTool = new AddRouteEdgeTool(visNetwork_, routeDM);
	visCanvas.setMode('rdRoute');
	showManipulationModeText("Add Edge")
});
editRouteEdge.addEventListener('click', () => {
	visNetwork_.editEdgeMode();
});

deleteSelected.addEventListener('click', () => {
	visNetwork_.deleteSelected(visNetwork_);
	showManipulationModeText("Delete Route")
	closeManipulationMode();
});

// ------ route undo ------
undoRoute.addEventListener('click', () => {
	routeDM.undo();
	closeManipulationMode();
});

// --- show corresponding tools, and hide the rest ---
$(document).on('click', '#route-tool button', function () {
	if (this.id === 'add-node' || this.id === 'add-edge') {
		document.getElementById("delete-node").style.display = "none";
	}
});

// =============================
//     Routes Event Callbacks    
// =============================
async function btnSaveRoutes() {
	let graph_nodes_id = [];
	let id_label_table = {};
	let id_connections = {};
	for (i in visNetwork_.body.nodes) {
		if (visNetwork_.body.nodes[i].options.group !== 'navnode') { continue; }

		graph_nodes_id.push(visNetwork_.body.nodes[i].options.id);
		id_label_table[visNetwork_.body.nodes[i].options.id] = visNetwork_.body.nodes[i].options.label;
		var conn = visNetwork_.body.nodes[i].edges.map((edge) => edge.toId);
		conn = conn.filter(c => c !== visNetwork_.body.nodes[i].options.id);
		id_connections[visNetwork_.body.nodes[i].options.id] = conn;
	}

	// --- convert JSON object to array struct ---
	const nodesObj = visNetwork_.getPositions(graph_nodes_id);
	let nodes = Object.entries(nodesObj).map((n) => { n[1].id = n[0]; return n[1]; });

	// ------ [BEGIN] save the result to file [BEGIN] ------
	// --- may cause some mis-aligned transformations ---
	// nodes = _.uniqBy(nodes, function (n) { return n.id; });
	for (n in nodes) {
		const node = nodes[n];
		// console.log(node);

		nodes[n]['label'] = id_label_table[nodes[n].id];
		nodes[n]['connections'] = _.uniq(id_connections[nodes[n].id]);

		// console.log(nodes[n].id);
		const nodeArr = routeDM.getTargetVertex(nodes[n].id)
		const visPos = new Point2D(nodeArr.x, nodeArr.y);
		const pos = tfCanvas2ROS(gMapMeta_, visPos);
		const sysPos = new Point2D(pos.x, pos.y);
		nodes[n].x = sysPos.x;
		nodes[n].y = sysPos.y;

		for (it in node["connections"]) {
			const edge = {};
			edge["from"] = node["id"];
			edge["to"] = node["connections"][it];
			edge["weight"] = "1";
			edge["width"] = "0.0";
		}
	}
	var strGraph = cvtJson2Dot(nodes);

	// --- POST graph data to server side ---
	const selMap = document.getElementById('map-select').value;
	const res = await fetchPutMapGraph(rmtToken_, selMap, strGraph);
	console.log(res);

	if (res.ok) {
		notificationMsg(1, 'Save Routes on Success!');
	} else {
		notificationMsg(3, 'Save Routes on Failure!');
	}

	// --- refresh all the edges ---
	visNetwork_.body.data.edges.clear();

	// CRITICAL! Update graph contents
	try {
		// --- load graph data from back-end ---
		let res = await fetchGetMapGraph(rmtToken_, gMapMeta_.nav_graph);
		let graphData = await res.text();

		// --- global cacheing ---
		routeDM.loadFromSystem(graphData);

		// --- rendering on UI ---
		const routes = routeDM.getAllData();
		visDrawRoutes(visNetwork_, routes);
	} catch (err) {
		console.error(err);
	}

	visCanvas.setMode('rdRoute');
	closeManipulationMode();
}

function cancelNodeEdit(callback) {
	closeManipulationMode();
	callback(null);
}

function editEdgeWithoutDrag(data, callback) {
	// --- filling in the popup DOM elements ---
	document.getElementById("edge-label").value = data.id;
	document.getElementById("edge-weight").value = data.label;
	document.getElementById("edge-createButton").onclick = saveEdgeData.bind(this, data, callback);
	document.getElementById("edge-cancelButton").onclick = cancelEdgeEdit.bind(this, callback);
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
	clearEdgePopUp();
	callback(data);
}

function visDrawRoutes(_network, routes) {
	console.log(routes);
	// --- nodes' coordinates transformation ---
	routes.nodes.forEach((n) => {
		const tooltip = `<div class="farobot-map-tooltip" style="color: gray;">
		                   ${n.label}@(${Number(n.x).toFixed(2)}, ${Number(n.y).toFixed(2)})
										 </div>`;
		const sysPos = new Point2D(n.x, n.y);
		const pos = tfROS2Canvas(gMapMeta_, sysPos);
		const visPos = new Point2D(pos.x, pos.y);
		n.x = Number(pos.x);
		n.y = Number(pos.y);
		n.sysPos = sysPos;
		n.visPos = visPos;
		n.group = 'navnode';
		n.title = tooltip;
	});

	// --- update nodes ---
	_network.body.data.nodes.update(routes.nodes);

	// --- update edges ---
	_network.body.data.edges.update(routes.edges);
};
class RouteFactory {
	createNode(obj) {
		const visPos = new Point2D(obj.x, obj.y);
		const circle = new fabric.Circle({
			radius: 4,
			fill: 'darkgray',
			stroke: 'black',
			strokeWidth: 1,
			opacity: 0.3,
			left: visPos.x,
			top: visPos.y,
			selectable: false,
			hasBorders: false,
			hasControls: false,
			originX: 'center',
			originY: 'center',
			objectCaching: false
		});
		return circle;
	}
	createEdge(obj) {
		const fromX = obj.fromX;
		const fromY = obj.fromY;
		let toX = obj.toX;
		let toY = obj.toY;
		const angle = Math.atan2(toY - fromY, toX - fromX);
		const headLen = 2; // arrow head size
		// --- put the end of the line back at the arrow ---
		toX = toX - 7 * Math.cos(angle);
		toY = toY - 7 * Math.sin(angle);
		const points = [
			{
				x: fromX,
				y: fromY
			}, {
				x: fromX - (headLen / 4) * Math.cos(angle - Math.PI / 2),
				y: fromY - (headLen / 4) * Math.sin(angle - Math.PI / 2)
			}, {
				x: toX - (headLen / 4) * Math.cos(angle - Math.PI / 2),
				y: toY - (headLen / 4) * Math.sin(angle - Math.PI / 2)
			}, {
				x: toX - (headLen) * Math.cos(angle - Math.PI / 2),
				y: toY - (headLen) * Math.sin(angle - Math.PI / 2)
			}, {
				x: toX + (headLen) * Math.cos(angle),
				y: toY + (headLen) * Math.sin(angle)
			}, {
				x: toX - (headLen) * Math.cos(angle + Math.PI / 2),
				y: toY - (headLen) * Math.sin(angle + Math.PI / 2)
			}, {
				x: toX - (headLen / 4) * Math.cos(angle + Math.PI / 2),
				y: toY - (headLen / 4) * Math.sin(angle + Math.PI / 2)
			}, {
				x: fromX - (headLen / 4) * Math.cos(angle + Math.PI / 2),
				y: fromY - (headLen / 4) * Math.sin(angle + Math.PI / 2)
			}, {
				x: fromX,
				y: fromY
			}
		];
		const pline = new fabric.Polyline(points, {
			fill: 'black',
			stroke: 'black',
			strokeWidth: 1,
			opacity: 0.3,
			originX: 'left',
			originY: 'top',
			selectable: false
		});
		return pline;
	}
}

async function navRouteAnchor() {
	const selAgent = document.getElementById("nav-agent-select").value;
	let navAgents = await getAvailableRobotFromFleetList();

	const robot = navAgents.find((agent) => agent.robot_id === selAgent);
	if (!robot) {
		notificationMsg(2, `No matched robot in fleet state anchor function`);
		return;
	}

	$("#node-x").val(robot.location.x);
	$("#node-y").val(robot.location.y);
}