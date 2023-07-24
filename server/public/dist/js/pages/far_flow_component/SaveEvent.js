// save edit param
$(document).on("click", "#comfirmLeaveEditBehavior", () => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	if (haveSaveCachePageData(drawFlowNodeID)) {
		setDrawFlowNodeRoleEditStatus(elementID, true)
	} else {
		setDrawFlowNodeRoleEditStatus(elementID, false)
	}
	closeFlowSidebar();
	$('#cancelEditBehaviorModal').modal('hide');
})

$(document).on("click", ".sidebar-cancel", function () {
	console.log('sidebar-cancel')
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	if (!checkBehaviorDataAreValidate()) {
		if (haveSaveCachePageData(drawFlowNodeID)) {
			setDrawFlowNodeRoleEditStatus(elementID, true);
		} else {
			setDrawFlowNodeRoleEditStatus(elementID, false);
		}
	} else {
		setDrawFlowNodeRoleEditStatus(elementID, true)
	}

	closeFlowSidebar();

	// if (!checkBehaviorDataAreValidate()) {
	// 	let comfirmModal = document.getElementById('cancelEditBehaviorModal');
	// 	let comfirmModalP = comfirmModal.querySelector('.modal-body p');
	// 	comfirmModalP.textContent = "You have undifined behavior, your change may not be save. Do you want to cancel edit behavior?";
	// 	$('#cancelEditBehaviorModal').modal('show');
	// } else {
	// 	let comfirmModal = document.getElementById('cancelEditBehaviorModal');
	// 	let comfirmModalP = comfirmModal.querySelector('.modal-body p');
	// 	comfirmModalP.textContent = "You change will not be saved, are you sure you want to cancel?";
	// 	$('#cancelEditBehaviorModal').modal('show');
	// }
})

$(document).on("click", ".sidebar-save", function () {
	console.log('sidebar-save')
	const currentSidebarNodeType = getCurrentSideBarSelectNode().nodType;

	if (currentSidebarNodeType === 'if-condition') {
		handleIfConditionSaveSidebarData();
	} else {
		handleNonIfConditionSaveSidebarData();
	}
})

const handleIfConditionSaveSidebarData = () => {
	console.log('handleIfConditionSaveSidebarData');
	let sidebar = document.getElementById(sidebarID);

	// Get Target node
	let tagetNode = sidebar.querySelector('.targetNodeName');
	const nodeID = tagetNode.textContent;
	console.log(nodeID)

	let tmpConditions = {};

	// Get All Conditions
	let conditions = sidebar.querySelectorAll('.condition-card')
	conditions.forEach((condition, idx) => {
		let targetCodeMirrorEditor = condition.querySelector('.condition-expression .CodeMirror').CodeMirror;
		let conditionName = condition.querySelector('.conditionCollapseBtn').textContent;
		let conditionValue = targetCodeMirrorEditor.getValue();
		console.log(conditionName, conditionValue)
		console.log(JSON.stringify(conditionValue))
		tmpConditions[conditionName] = conditionValue;
	});

	let cloneSaveData = _.cloneDeep(conditionSaveData);
	console.log(cloneSaveData)
	cloneSaveData.nodeId = nodeID;
	cloneSaveData.roleName = '';
	cloneSaveData.conditions = tmpConditions;
	cloneSaveData.error_handle.error_type = '';
	cloneSaveData.error_handle.failure_timeout = '';
	cloneSaveData.error_handle.retry_limit = '';
	cloneSaveData.type = 'condition';

	flowPageData.currentSavedData[nodeID] = cloneSaveData;
	closeFlowSidebar();
}

const handleNonIfConditionSaveSidebarData = () => {
	console.log('handleNonIfConditionSaveSidebarData');
	let sidebar = document.getElementById(sidebarID);
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);

	// check all data is ok
	if (!checkBehaviorDataAreValidate()) {
		// notificationMsg(3, `Still have undifined behavior, please check again !`);
		setDrawFlowNodeRoleEditStatus(elementID, false)
		// return
	} else {
		setDrawFlowNodeRoleEditStatus(elementID, true)
	}

	// Get Target node
	let tagetNode = sidebar.querySelector('.targetNodeName');
	const nodeID = tagetNode.textContent;
	const roleNmae= tagetNode.getAttribute('data-rolename');
	let tmpRoleObject = _.cloneDeep(flowPageData.currentFleetSetting.roles[roleNmae]);
	console.log(nodeID, roleNmae)
	console.log(tmpRoleObject)

	// Get Error handle value
	const errotTypeVal = sidebar.querySelector('#error-type-input').value;
	const failureTimeOutVal = sidebar.querySelector('#failure-timeout-input').value;
	const retryLimitVal = sidebar.querySelector('#retry-limit-input').value;
	console.log(errotTypeVal, failureTimeOutVal, retryLimitVal)

	// Get All behavior content
	const behaviorParamContents = sidebar.querySelectorAll('.behaviorParamContent');

	behaviorParamContents.forEach(paramContent => {
		let contentID = paramContent.id;
		let behaviorType = contentID.split('_')[0];
		let behaviorIndex = parseInt(contentID.split('_')[1], 10) - 1;
		let targetBehavior = tmpRoleObject[behaviorIndex].title_content;
		console.log(behaviorType, behaviorIndex)

		if (behaviorType === "DockingArtifact") {
			behaviorType = "Artifact";
		}

		switch (behaviorType) {
			case "Nav2Client":
				const mapVal = paramContent.querySelector('.Nav2Client-map-select').value;
				const areaVal = paramContent.querySelector('.Nav2Client-area-select').value;
				const cellVal = paramContent.querySelector('.Nav2Client-cell-select').value;
				console.log(mapVal, areaVal, cellVal)

				for (const [key, value] of Object.entries(targetBehavior)) {
					targetBehavior[key] = `${mapVal}@${areaVal}@${cellVal}`
				}
				break;

			case "Rotate":
				const rotateMode = paramContent.querySelector('.rotate-mode-radio:checked').value;
				const isRotateByCell = rotateMode === '1'? true : false;
				console.log(isRotateByCell);

				if (isRotateByCell) {
					const mapVal = paramContent.querySelector('.Nav2Client-map-select').value;
					const areaVal = paramContent.querySelector('.Nav2Client-area-select').value;
					const cellVal = paramContent.querySelector('.Nav2Client-cell-select').value;
					console.log(mapVal, areaVal, cellVal)

					for (const [key, value] of Object.entries(targetBehavior)) {
						if (key.includes("goal")) {
							targetBehavior[key] = `${mapVal}@${areaVal}@${cellVal}`;
						}
					}
				} else {
          let direction = paramContent.querySelector(
            ".rotate-angle-radio:checked"
          ).value;
          // NOTE: follow the  Cartesian system, clockwise is negative
          let clockwise = direction === "clockwise" ? -1 : 1;
          let rotateAngle = paramContent.querySelector(
            ".rotate-angle-input"
          ).value;

          for (const [key, value] of Object.entries(targetBehavior)) {
            if (key.includes("angle")) {
              if (rotateAngle.trim() === "") {
                targetBehavior[key] = "undefined";
              } else {
                targetBehavior[key] = `${clockwise * rotateAngle}`;
              }
            }
          }
        }
				break;

			case "Charge":
				const chargeMapVal = paramContent.querySelector('.Nav2Client-map-select').value;
				const chargeAreaVal = paramContent.querySelector('.Nav2Client-area-select').value;
				const chargeCellVal = paramContent.querySelector('.Nav2Client-cell-select').value;
				const chargeVal = paramContent.querySelector('.percentage-input').value;
				for (const [key, value] of Object.entries(targetBehavior)) {
					if (key.includes("goal")) {
						targetBehavior[key] = `${chargeMapVal}@${chargeAreaVal}@${chargeCellVal}`;
					} else if (key.includes("percentage")) {
						targetBehavior[key] = `${chargeVal}`;
					}
				}
            	break;

			case "Artifact":
				const selectedArtifactID = paramContent.querySelector('.artifact-select').value;
				let artifactVal = {}
				const artifactParams = paramContent.querySelectorAll('.artifact-service-param');
				artifactParams.forEach(paramsNode => {
					if (paramsNode.type === 'radio' && !paramsNode.checked) {
						return;
					}
					const paramVal = paramsNode.value;

					// get service name
					const closestDivRow = paramsNode.closest('.row');
					const closestLabel = closestDivRow.querySelector('label');
					const paramType = closestLabel.textContent;
					const dataType = closestLabel.dataset.paramType;

					artifactVal[paramType] = (dataType === "double" || dataType === "int")? `${Number(paramVal)}`: paramVal;
				});
				for (const [key, value] of Object.entries(targetBehavior)) {
					if (key.includes("artifact_id")) {
						targetBehavior[key] = `${selectedArtifactID}`;
					} else if (key.includes("value_")) {
						targetBehavior[key] = artifactVal;
					}
				}
				break;

			case "WaitAction":
				const durationVal = paramContent.querySelector('.waitaction-input').value;
				for (const [key, value] of Object.entries(targetBehavior)) {
					if (durationVal.trim() === "") {
						targetBehavior[key] = 'undefined';
					} else {
						targetBehavior[key] = `${durationVal}`;
					}
				}
				break;

			case "Report":
				const reportMagVal = paramContent.querySelector('.report-input').value;
				for (const [key, value] of Object.entries(targetBehavior)) {
					if (reportMagVal.trim() === "") {
						targetBehavior[key] = 'undefined';
					} else {
						targetBehavior[key] = `${reportMagVal}`;
					}
				}
				break;

			default:
				break;
		}
	});

	let cloneSaveData = _.cloneDeep(saveData);
	console.log(cloneSaveData)
	cloneSaveData.nodeId = nodeID;
	cloneSaveData.roleName = roleNmae;
	cloneSaveData.behavior = tmpRoleObject;
	cloneSaveData.error_handle.error_type = errotTypeVal;
	cloneSaveData.error_handle.failure_timeout = failureTimeOutVal;
	cloneSaveData.error_handle.retry_limit = retryLimitVal;
	cloneSaveData.type = 'task';

	flowPageData.currentSavedData[nodeID] = cloneSaveData;
	console.log(flowPageData)
	console.log(tmpRoleObject)

	closeFlowSidebar();
}


// Save flow
const save = () => {
	// is Empty flow
	const isEmptyFlow = document.querySelectorAll('.drawflow-node:not(.start):not(.finish)').length >0? false: true;
	if (isEmptyFlow) {
		notificationMsg(3, `You have to add role into flow, otherwise please cancel it.`);
		return;
	}

	const isStillEditRoleName = document.getElementById('edit-flowname-switch').querySelector('i').classList.contains('fa-eye');
	console.log(isStillEditRoleName)
	if (isStillEditRoleName) {
		notificationMsg(3, `You have to finish rename flow name, otherwise please cancel it.`);
		return;
	}

	// have undefined behavior
	// if (haveUndefinedBehavior()) {
	// 	notificationMsg(3, `You have undefined role, please check it.`);
	// 	return;
	// }

	const fleetName = flowPageData.currentFleet;
	const flowName = getCurrentFlowName();
	console.log(fleetName, flowName)

	// flow type
	const flowType = flowPageData.currentFlowType.type;

	if (flowType === "Manual") {
		// pass
		console.log('flow save type manual')
	} else if (flowType.includes('Event:')) {
		// pass
		console.log('flow save type event')
	} else if (flowType.includes('Timer:')) {
		// pass
		console.log('flow save type timer')
	} else {
		notificationMsg(3, `Please choose flow trigger before save it!`);
		return
	}

	const drawflowInfo = editor.export();
	console.log(drawflowInfo);

	const flowSavedData = processFlowSaveData(flowName);
	flowSavedData.flow_type.type = flowType;
	flowSavedData.flow_type.start_time = flowPageData.currentFlowType.start_time;
	flowSavedData.flow_type.end_time = flowPageData.currentFlowType.end_time;
	flowSavedData.flow_type.interval = flowPageData.currentFlowType.interval;
	console.log(flowSavedData)

	// Save ui need file
	save_flow_ui_setting(fleetName, flowName,  JSON.stringify(drawflowInfo, null, 2));
	save_flow_setting(fleetName, flowName,  JSON.stringify(flowSavedData, null, 2))

	setUnSaveChange();
}

const processFlowSaveData = (flowName) => {
	let saveDate = {
		flow_id: "",
		flow_name: flowName,
		flow_type: {
			type: '',
			start_time: '',
			end_time: '',
			interval: ''
		},
		group: [],
		flow_data: []
	};

	let currentUISavedData = flowPageData.currentSavedData;

	// Add Start Node
	let startNodeId = `${editor.getNodesFromName("start")[0]}`;
	let startNodeOutput = editor.getNodeFromId(startNodeId).outputs;
	let startData = {
		node_id: startNodeId,
		node_name: 'start',
		role_name: 'start',
		type: 'default',
		priority: '',
		error_handle: {
			error_type: '',
			failure_timeout: '',
			retry_limit: ''
		},
		relation: {
			// output
			default: [],
			error_handle: []
		},
		task_param: {}
	}
	for (const [outputName, outputData] of Object.entries(startNodeOutput)) {
		const connections = outputData.connections;
		connections.forEach(connection => {
			const outputNodeId = connection.node;
			startData.relation.default.push(outputNodeId);
		});
	}
	saveDate.flow_data.push(startData)

	// Add Finish Node
	let finishNodeId = `${editor.getNodesFromName("finish")[0]}`;
	let finishNodeOutput = editor.getNodeFromId(finishNodeId).outputs;
	let finishData = {
		node_id: finishNodeId,
		node_name: 'finish',
		role_name: 'finish',
		type: 'default',
		priority: '',
		error_handle: {
			error_type: '',
			failure_timeout: '',
			retry_limit: ''
		},
		relation: {
			// output
			default: [],
			error_handle: []
		},
		task_param: {}
	}
	for (const [outputName, outputData] of Object.entries(finishNodeOutput)) {
		const connections = outputData.connections;
		connections.forEach(connection => {
			const outputNodeId = connection.node;
			finishData.relation.default.push(outputNodeId);
		});
	}
	saveDate.flow_data.push(finishData)

	for (const [nodeId, nodeData] of Object.entries(currentUISavedData)) {
		console.log(nodeId, nodeData);
		let roleData = {
			node_id: nodeId.replace('node-', ''),
			node_name: nodeData.roleName,
			role_name: nodeData.roleName,
			type: nodeData.type,
			priority: '',
			relation: {}
		}

		// node output
		let nodeOutput = editor.getNodeFromId(nodeId.replace('node-', '')).outputs;
		let lastOutput = Object.keys(nodeOutput).pop();

		if (nodeData.type === 'task') {
			// init task type data
			roleData.relation = {
				// output
				default: [],
				error_handle: []
			};
			roleData.task_param = {};

			// handle type: task
			// error handel
			let error_handle = {
				error_type: nodeData.error_handle.error_type,
				failure_timeout: nodeData.error_handle.failure_timeout,
				retry_limit: nodeData.error_handle.retry_limit
			}
			roleData.error_handle = error_handle;

			// priority
			let prioritySelect = document.getElementById(nodeId).querySelector('.drawflow-task-priority-select');
			roleData.priority = prioritySelect.value;

			// relation
			console.log(lastOutput)
			for (const [outputName, outputData] of Object.entries(nodeOutput)) {
				const connections = outputData.connections;
				connections.forEach(connection => {
					const outputNodeId = connection.node;
					if (outputName === lastOutput){
						roleData.relation.error_handle.push(outputNodeId);
					} else {
						roleData.relation.default.push(outputNodeId);
					}
				});
			}

			// task param
			nodeData.behavior.forEach(behavior => {
				const behaviorParam = behavior.title_content;
				for (const [paramKey, paramVal] of Object.entries(behaviorParam)) {
					roleData.task_param[paramKey] = paramVal;
				}
			});
		} else {
			// init condition type data
			roleData.conditions = {};

			// handle type: condition
			// condition
			const conditions = nodeData.conditions;
			roleData.conditions = conditions;

			// relation
			const conditionNameList = Object.keys(conditions);

			let idx_output = 0;
			for (const [outputName, outputData] of Object.entries(nodeOutput)) {
				const connections = outputData.connections;
				const conditionKeyName = conditionNameList[idx_output];
				connections.forEach(connection => {
					const outputNodeId = connection.node;

					if (outputName === lastOutput){
						// else
						if (roleData.relation.hasOwnProperty('else')) {
							roleData.relation.else.push(outputNodeId);
						} else {
							roleData.relation['else'] = [outputNodeId];
						}
					} else {
						// if & else if
						if (roleData.relation.hasOwnProperty(`${conditionKeyName}`)) {
							roleData.relation[`${conditionKeyName}`].push(outputNodeId);
						} else {
							roleData.relation[conditionKeyName] = [outputNodeId];
						}
					}
				});
				idx_output++;
			}
		}

		if (nodeData.type === 'condition') {
			const isConditionErrorHandle = editor.getNodeFromId(nodeId.replace('node-', '')).name.includes('error-handle')
			if (isConditionErrorHandle) {
				roleData.type = 'condition_error'
			}
		}

		saveDate.flow_data.push(roleData)
	}

	console.log(saveDate)

	return saveDate;
}

const save_flow_ui_setting = async (_fleetname, _flowname, _data) => {
	await restPostFlowUIData(_data, _fleetname, _flowname);
	notificationMsg(0, `${_flowname} panel saved`);
}

const save_flow_setting = async (_fleetname, _flowname, _data) => {
	var res = await restPostFlowData(_data, _fleetname, _flowname);
}

// save flow for testing
const getSaveDataForTesting = (tmpFlowName) => {
	// is Empty flow
	const isEmptyFlow = document.querySelectorAll('.drawflow-node:not(.start):not(.finish)').length >0? false: true;
	if (isEmptyFlow) {
		notificationMsg(3, `You have to add role into flow, otherwise please cancel it.`);
		return;
	}

	const fleetName = flowPageData.currentFleet;
	const flowName = tmpFlowName;
	console.log(fleetName, flowName)

	// flow type
	const flowType = flowPageData.currentFlowType.type;

	if (flowType === "Manual") {
		// pass
		console.log('flow save type manual')
	} else if (flowType.includes('Event:')) {
		// pass
		console.log('flow save type event')
	} else if (flowType.includes('Timer:')) {
		// pass
		console.log('flow save type timer')
	} else {
		notificationMsg(3, `Please choose flow trigger before save it!`);
		return
	}

	let flowSavedData = processFlowSaveData(flowName);
	flowSavedData.flow_type.type = flowType;
	flowSavedData.flow_type.start_time = flowPageData.currentFlowType.start_time;
	flowSavedData.flow_type.end_time = flowPageData.currentFlowType.end_time;
	flowSavedData.flow_type.interval = flowPageData.currentFlowType.interval;

	// adjust assign robot
	let nodesCanAssignRobot = document.querySelectorAll('.drawflow-node:not(.start):not(.finish):not(.if-condition)');

	nodesCanAssignRobot.forEach(node => {
		const elementID = node.id
		const drawFlowNodeID = translateToNodeDrawFlowID(elementID);
		const vehicleInfo =  document.getElementById(elementID).querySelector('.vehicle-info');
		const assignRobotVal = vehicleInfo.dataset.robotId;

		console.log(elementID, drawFlowNodeID, assignRobotVal)
		flowSavedData.flow_data.forEach(data => {
			const dataType = data.type;
			const dataNodeID = data.node_id;

			if (dataType === 'task' && dataNodeID === drawFlowNodeID) {
				data.assigned_robot = assignRobotVal
			}
		});
	});

	console.log(flowSavedData)
	// ================ tmp for trigger test by api ================
	// return flowSavedData
	// ================ tmp for trigger test by api ================

	// Save tmp file
	save_flow_setting(fleetName, flowName,  JSON.stringify(flowSavedData, null, 2));

	return new Promise((resolve, reject) => {
		setTimeout(() => {
		  resolve('tmp save ok');
		}, 500);
	});
}