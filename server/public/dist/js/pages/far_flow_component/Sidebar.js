const sidebarID = 'control_sidebar_content_div';

const getCurrentSideBarSelectNode = () => {
	let control_sidebar = document.getElementById('control_sidebar');
	const selNode = control_sidebar.dataset.currentSelectedNode;
	const selNodeType = control_sidebar.dataset.currentSelectedNodeType;

	let currentSelectedNode = {
		node_id: selNode,
		nodType: selNodeType
	}

	return currentSelectedNode;
}

const isSideBarOpen = () => {
	return document.body.classList.contains('control-sidebar-slide-open');
}

const openFlowSidebar = () => {
	if (currentSelectedNode.nodType === 'default') {
		// do nothing
	} else {
        const currentNodeID = currentSelectedNode.node_id;
		const currentNodeType = currentSelectedNode.nodType;

		let control_sidebar = document.getElementById('control_sidebar');
		control_sidebar.style.display = 'block';
		control_sidebar.dataset.currentSelectedNode = currentNodeID;
		control_sidebar.dataset.currentSelectedNodeType = currentNodeType;
		document.body.classList.add('control-sidebar-slide-open');

		if (currentNodeType === 'if-condition') {
			handleSidbarContentIfConditionNode();
		} else {
			handleSidbarContentNormalNode();
			handleSidbarContentHeaderNode();
			handleControlSidebarGeneralSettingNode();
		}

		handleSidbarContentFooterNode();
	}
}

const closeFlowSidebar = () => {
	const currenSelectNode = getCurrentSideBarSelectNode();
	if (currenSelectNode.nodType === 'default') {
		// do nothing
	} else {
		let control_sidebar = document.getElementById('control_sidebar')
		control_sidebar.style.display = 'none';
		control_sidebar.dataset.currentSelectedNode = '';
		control_sidebar.dataset.currentSelectedNodeType = '';
		document.body.classList.remove('control-sidebar-slide-open');

		// remove sidebar content
		let sidebar = document.getElementById(sidebarID);
		let sidebarChildList = [];
		sidebar.replaceChildren(...sidebarChildList);
	}
}

const handleControlSidebarGeneralSettingNode = async () => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);
	let sidebar = document.getElementById(sidebarID);

	// Get available robot
	let availableRobot = {};
	let scanRobotRes = await fetchScanRobots2(rmtToken_);
	if (scanRobotRes.total > 0) {
		scanRobotRes.robots.forEach(robot => {
			availableRobot[robot.robot_id] = robot.robot_name
		});
	}
	console.log(availableRobot)

	// relate role
	let selectedRole = document.getElementById(elementID).querySelector('.normal-node-text').dataset.rolename;
	let selectedPriority = document.getElementById(elementID).querySelector('.drawflow-task-priority-select').value;
	let selectedVehicle = document.getElementById(elementID).querySelector('.vehicle-info').textContent.replace('Vehicle:', '').trim();

	console.log(selectedRole)
	console.log(selectedPriority)
	console.log(selectedVehicle)

	const generalSettingTmplate = document.getElementById('controlSidebarGeneralSetting');
	let  generalSettingnode = document.importNode(generalSettingTmplate.content, true);
	let generalSettingDiv = generalSettingnode.querySelector('.control-sidebar-general-setting');

	// change show role name
	let displyHeader = generalSettingnode.querySelector('.general-setting-role-name');
	displyHeader.textContent = selectedRole;

	// change priority
	let generalSettingPrioritySelect = generalSettingnode.querySelector('.general-setting-priority-select');
	let priorityOptions = generalSettingPrioritySelect.querySelectorAll('option');
	priorityOptions.forEach(option => {
		if (`${selectedPriority}` === `${option.value}`) {
			option.selected = true
		} else {
			option.selected = false
		}
	});

	// add option
	let generalSettingVehicleSelect = generalSettingnode.querySelector('.general-setting-vehicle-select');
	for (const [robot_id, robot_name] of Object.entries(availableRobot)) {
		let newVehicleOption = document.createElement('option');
		newVehicleOption.value = robot_name;
		newVehicleOption.textContent = robot_name;
		newVehicleOption.dataset.robotId = robot_id;
		generalSettingVehicleSelect.appendChild(newVehicleOption);
	}


	// change select vehicle
	let vehicleOptions = generalSettingVehicleSelect.querySelectorAll('option');
	vehicleOptions.forEach(option => {
		if (`${selectedVehicle}` === `${option.textContent}`) {
			option.selected = true
		} else {
			option.selected = false
		}
	});

	sidebar.insertBefore(generalSettingDiv, sidebar.firstChild)
}

const handleSidbarContentHeaderNode = () => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	let sidebar = document.getElementById(sidebarID);

	const headerTmplate = document.getElementById('controlSidebarHeader');
	let headernode = document.importNode(headerTmplate.content, true);
	let headerDiv = headernode.querySelector('.control-sidebar-header');

	if (haveSaveCachePageData(drawFlowNodeID)) {
		const elementID = translateToNodeElementID(drawFlowNodeID);
		const error_type = flowPageData.currentSavedData[elementID].error_handle.error_type;
		const failure_timeout = flowPageData.currentSavedData[elementID].error_handle.failure_timeout;
		const retry_limit = flowPageData.currentSavedData[elementID].error_handle.retry_limit;
		headerDiv.querySelector('#error-type-input').value = error_type;
		headerDiv.querySelector('#failure-timeout-input').value = failure_timeout;
		headerDiv.querySelector('#retry-limit-input').value = retry_limit;
	}

	sidebar.insertBefore(headerDiv, sidebar.firstChild)
}

const handleSidbarContentFooterNode = () => {
	let sidebar = document.getElementById(sidebarID);

	const footerTmplate = document.getElementById('controlSidebarFooter');
	let footernode = document.importNode(footerTmplate.content, true);
	let footerDiv = footernode.querySelector('.control-sidebar-footer');

	sidebar.appendChild(footerDiv)
}

const handleSidbarContentNormalNode = () => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);
	let sidebar = document.getElementById(sidebarID);
	let sidebarChildList = [];

	// relate behavior number
	let selectedRole = document.getElementById(elementID).querySelector('.normal-node-text').dataset.rolename;
	let behaviorsInRole = flowPageData.currentFleetSetting.roles[selectedRole];
	console.log(selectedRole)
	console.log(behaviorsInRole)

	// fake node
	const behaviorInfoTmplate = document.getElementById('behaviorInfo');
	let infonode = document.importNode(behaviorInfoTmplate.content, true);
	let targetspan = infonode.querySelector('.targetNodeName');
	targetspan.textContent = elementID;
	targetspan.dataset.rolename = selectedRole;
	let infospan = infonode.querySelector('.behaviorTotal');
	infospan.textContent = `${elementID}: ${behaviorsInRole.length} Behaviors`;
	sidebarChildList.push(infonode)

	behaviorsInRole.forEach((behavior, index) => {
		let behavior_index = index + 1;
		let title_name = behavior.title_name;
		let title_content = behavior.title_content
		let type = title_name === 'Artifact'? behavior.type: '';
		let service = title_name === 'Artifact'? behavior.service: '';

		console.log(title_name, title_content, type, service)
		let node = document.createElement('div');

		switch (title_name) {
			case "Nav2Client":
				node = handleShowSideBarCard(elementID, 'Nav2Client', 'Move', behavior_index);
				break;

			case "Dock":
				node = handleShowSideBarCard(elementID, 'Dock', 'Dock', behavior_index);
				break;

			case "Undock":
				node = handleShowSideBarCard(elementID, 'Undock', 'Undock', behavior_index);
				break;

			case "Rotate":
				node = handleShowSideBarCard(elementID, 'Rotate', 'Rotate', behavior_index);
				break;

			case "Charge":
				node = handleShowSideBarCard(elementID, 'Charge', 'Charge', behavior_index);
            	break;

			case "Artifact":
				node = handleShowSideBarCard(elementID, 'Artifact', `Artifact`, behavior_index);
				break;

			case "WaitAction":
				node = handleShowSideBarCard(elementID, 'WaitAction', 'Wait', behavior_index);
				break;

			case "DockingArtifact":
				node = handleShowSideBarCard(elementID, 'DockingArtifact', `DockingArtifact`, behavior_index);
				break;

			case "Report":
				node = handleShowSideBarCard(elementID, 'Report', `Report`, behavior_index);
				break;


			default:
				break;
		}
		sidebarChildList.push(node)
	});

	sidebar.replaceChildren(...sidebarChildList);
}

const handleSidbarContentIfConditionNode = () => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);
	let sidebar = document.getElementById(sidebarID);
	let sidebarChildList = [];
	let conditions = document.getElementById(elementID).querySelectorAll('.condition-group');

	// fake node
	const conditionInfoTmplate = document.getElementById('IfConditionInfo');
	let infonode = document.importNode(conditionInfoTmplate.content, true);
	let targetspan = infonode.querySelector('.targetNodeName');
	targetspan.textContent = elementID;
	const isConditionErrorHandle = editor.getNodeFromId(drawFlowNodeID).name.includes('error-handle')
	let infospan = infonode.querySelector('.conditionTotal');
	infospan.textContent = `${elementID}: ${conditions.length} Conditions ${isConditionErrorHandle? "(Error Handle)": ''}`;
	sidebarChildList.push(infonode)

    // add sidebar card
	conditions.forEach((condition, conditionIdx) => {
		let conditionName = condition.querySelector('.condition-input').value;
		let node = handleShowIfConditionSideBarCard(conditionName, conditionIdx)
		sidebarChildList.push(node);
	})

	sidebar.replaceChildren(...sidebarChildList);
}

$(document).on('click', '.sidebar-add-condition', function (e, data) {
	const currenSelectNode = getCurrentSideBarSelectNode();
	const isTriggerFromDrawflowAdd = data !== undefined? data.isTriggerFromDrawflowAdd: false;

	// target node
	const drawFlowNodeID = currenSelectNode.node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);
	let targetIfConditonNode = document.getElementById(elementID);

	let numberOfCondition = targetIfConditonNode.querySelectorAll(".condition-group").length;
	let conditionIdx = isTriggerFromDrawflowAdd? numberOfCondition -1: numberOfCondition;
	let conditionName = `Condition ${conditionIdx+1}`;

	if (!isTriggerFromDrawflowAdd) {
		// Get condition template
		let conditionTemplate = document.getElementById('conditionGroup');
		let condition = document.importNode(conditionTemplate.content, true);

		// Add Output
		editor.addNodeOutput(drawFlowNodeID);

		// Handle add condition
		let refNode = targetIfConditonNode.querySelector(".add-condition")

		// change condition placeholder
		condition.querySelector('.condition-input').setAttribute('value', conditionName);
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

		drawFlowUnselectNode(targetIfConditonNode);
	}

	// Insert sidebar card before footer
	let sidebar = document.getElementById(sidebarID);
	let targetRefNode = sidebar.querySelector('.control-sidebar-footer');
	let node = handleShowIfConditionSideBarCard(conditionName, conditionIdx);
	sidebar.insertBefore(node, targetRefNode);
});

$(document).on('click', '.conditionDeleteBtn', function (e) {
	const currenSelectNode = getCurrentSideBarSelectNode();

	// target node
	const drawFlowNodeID = currenSelectNode.node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	let currentDeleteBtn = e.currentTarget;
	let parent = currentDeleteBtn.closest('.condition-card')
	let currentConditionIdx = parseInt(parent.querySelector('.conditionCollapseBtn').id.replace('conditionBtn_', ''));

	let node = document.getElementById(elementID);

	// remove item
	let targetDeleteCondition = node.querySelectorAll('.condition-group')[currentConditionIdx];
	targetDeleteCondition.remove();
	editor.removeNodeOutput(drawFlowNodeID, `output_${1 + currentConditionIdx}`);

	// Update Drawwflow Content
	const tmpCpoy = node.cloneNode(true);
	let tmpNodePlaceholder = document.createElement("div");
	tmpNodePlaceholder.appendChild(tmpCpoy.querySelector('.drawflow_content_node'));

	// clear style and expand css
	let addConditionBtn = tmpNodePlaceholder.querySelector(".add-condition");
	addConditionBtn.classList.remove("active");
	let elseCondition = tmpNodePlaceholder.querySelector(".if-condition-else");
	elseCondition.style = null;

	editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = tmpNodePlaceholder.firstChild.innerHTML;

	drawFlowUnselectNode(node.querySelector('.else-condition-group'));

	// delete sidebar element
	let sidebar = document.getElementById(sidebarID);
	let conditionCards = sidebar.querySelectorAll('.condition-card');

	conditionCards.forEach((element, idx) => {
		if (idx < currentConditionIdx) {
			// Do nothing
		} else if (idx == currentConditionIdx) {
			// Remove Element later
		} else {
			// Chnage attr
			let cardHeaderBtn = element.querySelector('.card-header .conditionCollapseBtn');
			let cardContent = element.querySelector('.conditionParamContent');

			// console.log(cardHeaderBtn, cardContent)

			const conditionIdx = idx - 1;

			cardHeaderBtn.id = `conditionBtn_${conditionIdx}`;
			cardHeaderBtn.dataset.target = `#condition_${conditionIdx}`;
			cardContent.id = `condition_${conditionIdx}`;
			console.log(element)

			// update codemirrorconditionidx
			let targetCodeMirrorEditor = element.querySelector('.condition-expression .CodeMirror').CodeMirror;
			targetCodeMirrorEditor.options.conditionIndex = conditionIdx;
		}
	});
	conditionCards[currentConditionIdx].closest('.col-12').remove();
})

$(document).on('change', '.general-setting-priority-select', function (e) {
	const currenSelectNode = getCurrentSideBarSelectNode();

	// target node
	const drawFlowNodeID = currenSelectNode.node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	const sidebarPrioritySelectVal = `${$(this).val()}`;
	const drawflowPrioritySelect = document.getElementById(elementID).querySelector('.drawflow-task-priority-select');

	let drawflowPriorityOptions = drawflowPrioritySelect.querySelectorAll('option');
	drawflowPriorityOptions.forEach(option => {
		if (sidebarPrioritySelectVal === `${option.value}`) {
			option.selected = true
		} else {
			option.selected = false
		}
	});

	$(drawflowPrioritySelect).trigger('change');
})

$(document).on('change', '.general-setting-vehicle-select', function (e) {
	const currenSelectNode = getCurrentSideBarSelectNode();

	// target node
	const drawFlowNodeID = currenSelectNode.node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	const sidebarPrioritySelectVal = $(this).val();
	const sidebarPrioritySelectDataRobotID = $(this).find('option:selected').data('robot-id');

	const drawflowVehicleInfo = document.getElementById(elementID).querySelector('.vehicle-info');
	drawflowVehicleInfo.textContent = `Vehicle: ${sidebarPrioritySelectVal}`;
	drawflowVehicleInfo.dataset.robotId = sidebarPrioritySelectDataRobotID;
})

$(document).on('click', '.a-collapse-all', function (e) {
	$('.multi-collapse').collapse('hide')
	const anchor = $(this)[0];
	$(this)[0].classList.replace('a-collapse-all', 'a-expand-all');
	$(this)[0].textContent = 'Expand All';
});

$(document).on('click', '.a-expand-all', function (e) {
	$('.multi-collapse').collapse('show')
	const anchor = $(this)[0];
	$(this)[0].classList.replace('a-expand-all', 'a-collapse-all');
	$(this)[0].textContent = 'Collapse All';
});