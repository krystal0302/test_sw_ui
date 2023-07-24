let id = document.getElementById("drawflow");
let editor = new Drawflow(id);

editor.reroute = true;
editor.reroute_fix_curvature = true;
editor.force_first_input = false;

editor.start();

document.addEventListener('flowPageDataLoaded', () => {
	renderTriggerSelect();
	initDrawFlow();
})

const renderTriggerSelect = () => {
	const flowType = flowPageData.currentFlowType.type;
	const triggerSelect = document.getElementById('flow_select');

	if (flowType === "Manual") {
		const optionToSelect = triggerSelect.querySelector('#flow-manual');
		optionToSelect.selected = true;
		optionToSelect.textContent = flowType;
	} else if (flowType.includes('Event:')) {
		const optionToSelect = triggerSelect.querySelector('#flow-event');
		optionToSelect.selected = true;
		optionToSelect.textContent = flowType;
	} else if (flowType.includes('Timer:')) {
		const optionToSelect = triggerSelect.querySelector('#flow-timer');
		optionToSelect.selected = true;
		optionToSelect.textContent = flowType;
	} else {
	}
}

const initDrawFlow = () => {
	console.log("Init DrawFlow")
	const initData = flowPageData.drawFlowUIData === undefined? initDrawFlowData(): flowPageData.drawFlowUIData;

	editor.clear();
	editor.import(initData);

	initDrawFlowOption();
	adjustIfConditionLayout();
}

const initDrawFlowData = () => {
	let data_init_drawflow =
	{
		"drawflow": {
			"Home": {
				"data": {
					"2": {
						"id": 2,
						"name": "start",
						"data": {},
						"class": "start",
						"html": "",
						"typenode": false,
						"inputs": {},
						"outputs": {
							"output_1": {
								"connections": []
							}
						},
						"pos_x": 59,
						"pos_y": 304
					},
					"3": {
						"id": 3,
						"name": "finish",
						"data": {},
						"class": "finish",
						"html": "",
						"typenode": false,
						"inputs": {
							"input_1": {
								"connections": []
							}
						},
						"outputs": {},
						"pos_x": 632,
						"pos_y": 300
					}
				}
			}
		}
	};

	const startNodeHTML = document.getElementById("drawflowStartNode").innerHTML;
	const finishNodeHTML = document.getElementById("drawflowFinishNode").innerHTML;

	data_init_drawflow.drawflow.Home.data[2].html = startNodeHTML;
	data_init_drawflow.drawflow.Home.data[3].html = finishNodeHTML;

	return data_init_drawflow
}

const initDrawFlowOption = () => {
	console.log("Init DrawFlow Option")
	$('#Task').empty();

	const optionTemplate = document.getElementById("drawflowDragOption");
	const roles = Object.keys(flowPageData.currentFleetSetting.roles);
	// console.log(roles)

	roles.forEach(role => {
		let node = document.importNode(optionTemplate.content, true);
		let span = node.querySelector('span');
		let div = node.querySelector('.drag-drawflow');

		span.textContent = role;
		div.dataset.node = role;

		$('#Task').append(node)
	});
}

/* DRAG EVENT */
var elements = document.getElementsByClassName('drag-drawflow');
for (var i = 0; i < elements.length; i++) {
	elements[i].addEventListener('touchend', drop, false);
	elements[i].addEventListener('touchmove', positionMobile, false);
	elements[i].addEventListener('touchstart', drag, false );
}

var mobile_item_selec = '';
var mobile_last_move = null;

function positionMobile(ev) {
	mobile_last_move = ev;
}

function allowDrop(ev) {
	ev.preventDefault();
}

function drag(ev) {
	if (ev.type === "touchstart") {
		mobile_item_selec = ev.target.closest(".drag-drawflow").getAttribute('data-node');
	} else {
		ev.dataTransfer.setData("node", ev.target.getAttribute('data-node'));
	}
}

function drop(ev) {
	if (ev.type === "touchend") {
		var parentdrawflow = document.elementFromPoint( mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY).closest("#drawflow");
		if(parentdrawflow != null) {
			addNodeToDrawFlow(mobile_item_selec, mobile_last_move.touches[0].clientX, mobile_last_move.touches[0].clientY);
		}
		mobile_item_selec = '';
	} else {
		ev.preventDefault();
		var data = ev.dataTransfer.getData("node");
		addNodeToDrawFlow(data, ev.clientX, ev.clientY);
	}

}

function addNodeToDrawFlow(name, pos_x, pos_y) {
	if(editor.editor_mode === 'fixed') {
	return false;
	}
	pos_x = pos_x * ( editor.precanvas.clientWidth / (editor.precanvas.clientWidth * editor.zoom)) - (editor.precanvas.getBoundingClientRect().x * ( editor.precanvas.clientWidth / (editor.precanvas.clientWidth * editor.zoom)));
	pos_y = pos_y * ( editor.precanvas.clientHeight / (editor.precanvas.clientHeight * editor.zoom)) - (editor.precanvas.getBoundingClientRect().y * ( editor.precanvas.clientHeight / (editor.precanvas.clientHeight * editor.zoom)));

	console.log('ASDASDADASDASD')
	console.log(name)

	switch (name) {
		case 'logical_if':
			let ifconditionNodeTemplat = document.getElementById("drawflowIfConditionNode");
			let if_id = editor.addNode('if-condition', 1, 2, pos_x, pos_y, 'if-condition', {}, ifconditionNodeTemplat.innerHTML );
		break;

		case 'logical_if_error_handle':
			let ifconditionErrorNodeTemplat = document.getElementById("drawflowIfConditionErrorHandleNode");
			let if_error_id = editor.addNode('if-condition-error-handle', 1, 2, pos_x, pos_y, 'if-condition', {}, ifconditionErrorNodeTemplat.innerHTML );
		break;

	default:
		let normalNodeTemplat = document.getElementById("drawflowNode");
		let node = document.importNode(normalNodeTemplat.content, true);
		let span = node.querySelector('span');
		span.textContent = adjustTextMaxLength(name);
		span.title = name;
		span.dataset.rolename = name;
		var className = name.replace(/\s/g, '_');

		// add behavior info to drawflow
		let behaviorItem = document.getElementById("behaviorListItem");
		let behaviorCollapseDiv = node.querySelector('.behavior-list');
		let behaviors = flowPageData.currentFleetSetting.roles[name];

		behaviors.forEach((behaviorObj, behaviorIdx) => {
			const behaviorName = behaviorObj.title_name;
			let behaviorItemNode = document.importNode(behaviorItem.content, true);
			let behaviorItemSpan = behaviorItemNode.querySelector('span');
			let behaviorNumber = adjustNumberDisplayText(behaviorIdx+1);
			behaviorItemSpan.textContent = `${behaviorNumber}  ${behaviorName}`;
			behaviorCollapseDiv.appendChild(behaviorItemNode)
		});

		// change link text
		let behaviorTextlink = node.querySelector('.drawflow-show-behavior-list-info');
		behaviorTextlink.textContent = `Expand ${behaviors.length} Behaviors  ▾`

		var fake_div = document.createElement('div');
		fake_div.appendChild(node.cloneNode(true));

		let newNodeId = editor.addNode(name, 1, 2, pos_x, pos_y, className, {}, fake_div.innerHTML );

		// $(`#node-${newNodeId}`).parent().find(".outputs > .output_1").attr("title", "success");
		// $(`#node-${newNodeId}`).parent().find(".outputs > .output_2").attr("title", "failure");

		// console.log(editor)
		break
	}

	adjustIfConditionLayout();
	setUnSaveChange(true);
}

// editor Event
editor.on('zoom', function(zoom) {
	console.log("zoom " + zoom);
	setTimeout(() => {
		adjustIfConditionLayout()
	}, 500);
})

editor.on('nodeMoved', function(id) {
	console.log("Node moved " + id);
	// setCurrentSelectedNode(id);
	setUnSaveChange(true);
})

editor.on('nodeSelected', function(id) {
	console.log("Node selected " + id);
	setCurrentSelectedNode(id);
	setUnSaveChange(true);
})

editor.on('nodeUnselected', function(nodeUnselect) {
	console.log("Node unselected " + nodeUnselect);
	resetCurrentSelectedNode();
})

editor.on('nodeRemoved', function(removeNodeId) {
	console.log("Node Removed " + removeNodeId);
	deleteNodeDataInCurrentSavedData(`node-${removeNodeId}`);
	resetCurrentSelectedNodeWhenRemove();
	closeFlowSidebar();
})

editor.on('mouseMove', function(position) {
	// console.log('Position mouse x:' + position.x + ' y:'+ position.y);
	// console.log(editor)

	// window.setInterval(() => {
	// }, 500);
})

editor.on('connectionCreated', function(output_id, input_id, output_class, input_class) {
	console.log(output_id, input_id, output_class, input_class)
})

const setCurrentSelectedNode = (id) => {
	let nodeInfo = editor.getNodeFromId(id);
	// console.log(nodeInfo)
	currentSelectedNode.node_id = id;

	if (nodeInfo.class === 'if-condition') {
		currentSelectedNode.nodType = 'if-condition';
		const targetNodeID = translateToNodeElementID(id);

		// show add condition button
		let targetIfConditonNode = document.getElementById(targetNodeID);
		let addConditionBtn = targetIfConditonNode.querySelector(".add-condition");
		addConditionBtn.classList.add("active");

		let elseCondition = targetIfConditonNode.querySelector(".if-condition-else");
		elseCondition.style.marginTop = "10px";

		editor.updateConnectionNodes(targetNodeID);
	} else if (nodeInfo.class === 'start' || nodeInfo.class === 'finish') {
		currentSelectedNode.nodType = 'default';
	} else {
		currentSelectedNode.nodType = 'normal';
	}
	adjustIfConditionLayout();
}

const resetCurrentSelectedNode = () => {
	if (currentSelectedNode.nodType === 'if-condition') {
		const targetNodeID = translateToNodeElementID(currentSelectedNode.node_id);
		let targetIfConditonNode = document.getElementById(targetNodeID);

		let addConditionBtn = targetIfConditonNode.querySelector(".add-condition");
		addConditionBtn.classList.remove("active");

		let elseCondition = targetIfConditonNode.querySelector(".if-condition-else");
		elseCondition.style.marginTop = "0px";

		editor.updateConnectionNodes(targetNodeID);
	}

	currentSelectedNode.node_id = undefined;
	currentSelectedNode.nodType = 'normal';
	adjustIfConditionLayout();
}

const resetCurrentSelectedNodeWhenRemove = () => {
	currentSelectedNode.node_id = undefined;
	currentSelectedNode.nodType = 'normal';
}

$(document).on('click', '.add-condition', function (e) {
	// Get condition template
	let conditionTemplate = document.getElementById('conditionGroup');
	let condition = document.importNode(conditionTemplate.content, true);

	// target node
	const elementID = $(this).parent().closest('.drawflow-node').attr('id');
	const drawFlowNodeID = translateToNodeDrawFlowID(elementID);

	// Add Output
	editor.addNodeOutput(drawFlowNodeID);

	// Handle add condition
	let targetIfConditonNode = document.getElementById(elementID);
	let numberOfCondition = targetIfConditonNode.querySelectorAll(".condition-group").length;
	let refNode = targetIfConditonNode.querySelector(".add-condition")

	// change condition placeholder
	condition.querySelector('.condition-input').setAttribute('value',`Condition ${numberOfCondition+1}`);
	condition.querySelector('.condition-input').setAttribute('disabled', true);
	refNode.parentNode.insertBefore(condition, refNode);

	// update connection
	editor.updateConnectionNodes(elementID);

	// Update Drawwflow Content
	const tmpCpoy = targetIfConditonNode.cloneNode(true);
	let tmpNodePlaceholder = document.createElement("div");
	tmpNodePlaceholder.appendChild(tmpCpoy.querySelector('.drawflow_content_node'));

	// clear style and expand css
	let addConditionBtn = tmpNodePlaceholder.querySelector(".add-condition");
	addConditionBtn.classList.remove("active");
	let elseCondition = tmpNodePlaceholder.querySelector(".if-condition-else");
	elseCondition.style = null;

	editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = tmpNodePlaceholder.firstChild.innerHTML;

	adjustIfConditionLayout()

	// check if  sidebar open
	const sidebarDrawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	if (sidebarDrawFlowNodeID === undefined || `${sidebarDrawFlowNodeID.trim()}`.length === 0){
		// sidebar close
	} else {
		// sidebar open
		if (sidebarDrawFlowNodeID === drawFlowNodeID) {
			const addConditionBtn = document.querySelector('.sidebar-add-condition');

			const eventObj = {
				isTriggerFromDrawflowAdd: true
			}
			$(addConditionBtn).trigger('click', eventObj);
		}
	}
});

$(document).on('change', '.drawflow-task-priority-select', function (e) {
    let selected_val = $(this).val();
	const elementID = $(this).parent().closest('.drawflow-node').attr('id');
    const drawFlowNodeID = translateToNodeDrawFlowID(elementID);
    let component_html = editor.getNodeFromId(drawFlowNodeID).html;
    let component_select = $(component_html).find(".drawflow-task-priority-select");
    component_select.find("option").removeAttr('selected')
    component_select.find(`option[value="${selected_val}"]`).attr('selected', true);
	let backgroundColor = '#00284D';

	switch (`${selected_val}`) {
		case '5':
			backgroundColor = '#0029FF';
			break;
		case '4':
			backgroundColor = '#0029FF';
			break;
		default:
			backgroundColor = '#00284D';
			break;
	}
	// change html css color
	document.getElementById(elementID).getElementsByClassName("drawflow-task-priority-select")[0].setAttribute('style',`background-color: ${backgroundColor};`);

    const placeholder = document.createElement("div");
    placeholder.innerHTML = component_html;
    let node = placeholder.firstElementChild;
    let current_select = node
    current_select.getElementsByTagName('select')[0].innerHTML = component_select.html()
	current_select.getElementsByTagName('select')[0].setAttribute('style',`background-color: ${backgroundColor};`);
    // console.log(current_select.outerHTML)
    // console.log(component_html)
    editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = current_select.outerHTML

    // console.log(editor.getNodeFromId(drawFlowNodeID).html)

	// change sidebar general setting if open
	let sidebarPrioritySelect = document.getElementsByClassName('general-setting-priority-select');
	if (sidebarPrioritySelect.length > 0) {
		const sidebarPrioritySelectVal = `${sidebarPrioritySelect[0].value}`;
		let sidebarPriorityOptions = sidebarPrioritySelect[0].querySelectorAll('option');
		sidebarPriorityOptions.forEach(option => {
			if (selected_val === `${option.value}`) {
				option.selected = true
			} else {
				option.selected = false
			}
		});
	}
});

const adjustIfConditionLayout = () => {
	let ifNodes = document.getElementById('drawflow').querySelectorAll('.drawflow-node.if-condition');

	ifNodes.forEach(ifnode => {
		let outputs = ifnode.querySelectorAll('.output');
		let inputs = ifnode.querySelectorAll('.input');
		let conditions = ifnode.querySelectorAll('.condition-group');
		let elseCondition = ifnode.querySelectorAll('.else-condition-group');
		let addBtnActive = ifnode.querySelector('.add-condition').classList.contains('active');

		// css style value param
		const defaultConditionNumber = 1;
		const targetPositionType = 'absolute';
		const targetRight = '-10px';
		const titlebias = 50;
		const r = 7.5;
		const isExpand = addBtnActive? true: false;

		// for normal condition input
		inputs.forEach((condition, index) => {
			let input_idx = 1;
			let targetInput = ifnode.querySelector(`.input_${input_idx}`);

			const nodePading = window.getComputedStyle(ifnode.querySelector('.if-condition-group'), null).getPropertyValue('padding').replace('px', '') * 1;

			let newTop = (r + nodePading ) + (isExpand? titlebias: 0);

			targetInput.style.position = targetPositionType;
			targetInput.style.right = targetRight;
			targetInput.style.top = `${newTop}px`;
		});

		// for normal condition output
		conditions.forEach((condition, index) => {
			let output_idx = index + 1;
			let targetOutput = ifnode.querySelector(`.output_${output_idx}`);

			const deleteMargin = 3*(((index+1)-defaultConditionNumber) < 0? 0: (index+1)-defaultConditionNumber);
			const inputWidthbias = 30;

			const nodePading = window.getComputedStyle(ifnode.querySelector('.if-condition-group'), null).getPropertyValue('padding').replace('px', '') * 1;
			const conditionGroupMarginBottom = window.getComputedStyle(conditions[0], null).getPropertyValue('margin-bottom').replace('px', '') * 1;

			let valueBetween2Output = conditionGroupMarginBottom + inputWidthbias;
			let newTop = ((r + nodePading) + valueBetween2Output*index) + deleteMargin + (isExpand? titlebias: 0);

			targetOutput.style.position = targetPositionType;
			targetOutput.style.right = targetRight;
			targetOutput.style.top = `${newTop}px`;
		});

		// for else condition output
		elseCondition.forEach((condition, index) => {
			let output_idx = outputs.length;
			let targetOutput = ifnode.querySelector(`.output_${output_idx}`);

			const deleteMargin = 1.1*(conditions.length-defaultConditionNumber);
			const addBtnHeigh = 27;

			const nodePading = 0; //window.getComputedStyle(ifnode.querySelector('.if-condition-group'), null).getPropertyValue('padding').replace('px', '') * 1;
			const conditionGroupMarginBottom = 2; //window.getComputedStyle(conditions[0], null).getPropertyValue('margin-bottom').replace('px', '') * 1;
			const elseConditoinMarginTop = window.getComputedStyle(ifnode.querySelector('.if-condition-else'), null).getPropertyValue('margin-top').replace('px', '') * 1;

			let conditionPaddingBotton = nodePading;
			let elsePaddingTop = nodePading;
			let expandbias = titlebias + (addBtnHeigh + conditionPaddingBotton + elseConditoinMarginTop + elsePaddingTop);

			let valueBetween2Output = conditionGroupMarginBottom + 30;
			let newTop = ((r + nodePading) + valueBetween2Output*(output_idx-1)) + deleteMargin + (isExpand? expandbias: 0);

			targetOutput.style.position = targetPositionType;
			targetOutput.style.right = targetRight;
			targetOutput.style.top = `${newTop}px`;
		});

		editor.updateConnectionNodes(ifnode.id);
	});
}

const drawFlowUnselectNode = (targetNode) => {
	let parent = targetNode.closest('.drawflow-node');

	// reset to unselect style
	parent.classList.remove('selected');
	editor.node_selected = null;
	resetCurrentSelectedNode();
}

// edit behavior
$(document).on("click", ".more-info-edit", (e) => {
	let currentEditInfo = e.currentTarget;
	let parent = currentEditInfo.closest('.drawflow-node');

	openFlowSidebar();
	drawFlowUnselectNode(parent);
})

$(document).on("click", ".more-info-delete", (e) => {
	console.log(e)
	let anchor = e.currentTarget;
	let closestNode = anchor.closest('.drawflow-node');
	let nodeId= closestNode.id;
	editor.removeNodeId(nodeId);
})

// drawflow expand node behaviors
$(document).on("click", ".drawflow-show-behavior-list-info", function () {
	let parentNode = this.closest('.drawflow-node');
	let nodeID = parentNode.id;

	let vehicleDiv = parentNode.getElementsByClassName('vehicle-info')[0];
	vehicleDiv.classList.toggle('active');
	let collapseDiv = parentNode.getElementsByClassName('collapse')[0];
	$(collapseDiv).collapse('toggle');
})

$(document).on('shown.bs.collapse', '.collapse.behavior-list', function () {
	let parentNode = this.closest('.drawflow-node');
	let nodeID = parentNode.id;

	editor.updateConnectionNodes(nodeID);
})

$(document).on('hidden.bs.collapse', '.collapse.behavior-list', function () {
	let parentNode = this.closest('.drawflow-node');
	let nodeID = parentNode.id;

	editor.updateConnectionNodes(nodeID);
})

const deleteNodeDataInCurrentSavedData = (nodeID) => {
	if (flowPageData.currentSavedData.hasOwnProperty(nodeID)) {
		delete flowPageData.currentSavedData[nodeID];
		console.log(`Delete ${nodeID} saved data`)
	} else {
		console.log(`${nodeID} have no saved data to delete`)
	}
}

const checkBehaviorDataAreValidate = () => {
	let sidebar = document.getElementById(sidebarID);
	const notValidateBehaviors  = sidebar.querySelectorAll('.behaviorDataValidate:not([style="display: none;"])').length;

	if (notValidateBehaviors !== 0) {
		console.log('Still have behavior not finish')
		return false
	} else {
		console.log('Behaviors are ok')
		return true
	}
}

const setDrawFlowNodeRoleEditStatus = (elementID, isAllBehaviorValidate) => {
	const drawFlowNodeID = translateToNodeDrawFlowID(elementID);
	const targetNode = document.getElementById(elementID);
	const targetInfo = targetNode.querySelector('.drawflow-behavior-undefind-info');
	const isIfCondiotnNode = targetNode.classList.contains('if-condition');

	if (!isIfCondiotnNode) {
		// Update drawflow info
		let component_html = editor.getNodeFromId(drawFlowNodeID).html;

		const placeholder = document.createElement("div");
		placeholder.innerHTML = component_html;
		let node = placeholder.firstElementChild;
		let current_select = node;
		const drawflowTargetInfo = current_select.querySelector('.drawflow-behavior-undefind-info');

		if (isAllBehaviorValidate) {
			targetInfo.classList.add('validate');
			drawflowTargetInfo.classList.add('validate');
		} else {
			targetInfo.classList.remove('validate');
			drawflowTargetInfo.classList.remove('validate');
		}

		editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = current_select.outerHTML
	}
}

const setDrawFlowNodeConditionEditStatus = (elementID, conditionIdx, isValidate) => {
	const drawFlowNodeID = translateToNodeDrawFlowID(elementID);
	const targetNode = document.getElementById(elementID);
	const targetInfo = targetNode.querySelectorAll('.drawflow-behavior-undefind-info')[conditionIdx];
	const isIfCondiotnNode = targetNode.classList.contains('if-condition');

	// console.log(targetNode)
	// console.log(conditionIdx)
	// console.log(targetInfo)

	if (isIfCondiotnNode) {
		// Update drawflow info
		let component_html = editor.getNodeFromId(drawFlowNodeID).html;

		const placeholder = document.createElement("div");
		placeholder.innerHTML = component_html;
		let node = placeholder.firstElementChild;
		let current_select = node;
		const drawflowTargetInfo = current_select.querySelectorAll('.drawflow-behavior-undefind-info')[conditionIdx];

		if (isValidate) {
			targetInfo.classList.add('validate');
			drawflowTargetInfo.classList.add('validate');
		} else {
			targetInfo.classList.remove('validate');
			drawflowTargetInfo.classList.remove('validate');
		}

		editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = current_select.outerHTML
	}
}

window.addEventListener('scroll', function() {
	let pageOffset = window.pageYOffset;
	let elementTop = 56;
	this.document.getElementById('control_sidebar').style.top = `${elementTop- pageOffset}px`;
	// console.log(window.pageYOffset)
});

const changeMode = (option) => {
    lock.style.display = (option === 'lock') ? 'none' : 'block';
    unlock.style.display = (option === 'lock') ? 'block' : 'none';
}