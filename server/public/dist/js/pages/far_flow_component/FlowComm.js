let flowPageData = {
	currentFleet: "default",
	currentFleetSetting: {
		agents: [],
		artifacts: [],
		maps: {},
		roles: [],
        conditions: {},
		artifactsTemplates: {}
	},
	currentSavedData: {},
	currentFlowType: {
		type: "",
		start_time: '',
		end_time: '',
		interval: ''
	},
	langTemplateObj_: {},
	drawFlowUIData: undefined,
	haveUnSaveChange: false
};

let currentSelectedNode = {
	node_id: undefined,
	nodType: 'normal'
};

const saveData = {
	nodeId: '',
	roleName: '',
	behavior: '',
	error_handle: {
		error_type: '',
		failure_timeout: '',
		retry_limit: ''
	}
}

const conditionSaveData = {
	nodeId: '',
	roleName: '',
	behavior: '',
	error_handle: {
		error_type: '',
		failure_timeout: '',
		retry_limit: ''
	},
	conditions: {}
}

let flowPageDataLoadedEvent = new Event('flowPageDataLoaded');

if (document.readyState === 'loading') {
	console.log('loading')
	document.addEventListener('readystatechange', (event) => stateHandler(event));
}

const stateHandler = async (event) => {
	if (document.readyState === 'interactive') {
		console.log('interactive')
		pageRender();
		await initDataAsync();
		flowPageData.currentFleet = getSelectedFleet();
		await dataInitialize();
		await initDatePicker();
		await loadSavedData();
        document.dispatchEvent(flowPageDataLoadedEvent);
	} else if (document.readyState === 'complete') {
		console.log('complete');
	}
}

const dataInitialize = async () => {
	let fleetData = await restGetFleetSettings(flowPageData.currentFleet);
	// console.log(fleetData)
	await rmtTokenCheck();
    await initAPIConfig(rmtToken_);
	// console.log(rmtToken_)
	const currentFleetSettings = fleetData[flowPageData.currentFleet];

	flowPageData.currentFleetSetting.agents = currentFleetSettings.agents;
	flowPageData.currentFleetSetting.artifacts = await updateCurrentFleetArtifacts(currentFleetSettings);
	flowPageData.currentFleetSetting.roles = await updateCurrentFleetRoles(currentFleetSettings);
	flowPageData.currentFleetSetting.maps = await updateCurrentFleetMaps(currentFleetSettings);
    flowPageData.currentFleetSetting.conditions = await updateFlowConditions();
}

const updateCurrentFleetRoles = async (currentFleetSettings) => {
	let currentRoles = {};
	if (currentFleetSettings.roles.length === 0) {
		notificationMsg(3, "No role in fleet, flow function may have some issue.")
	} else {
		// let rolesMappingData = await restGetRolesParamData(flowPageData.currentFleet);
		// currentRoles = rolesMappingData;

		await restGetRolesParamData(flowPageData.currentFleet).then(function(response) {
			// console.log("GetRole Response:", response);
			currentRoles = response;
		}).catch(function(error) {
			console.error("GetRole Error:", error);
		});
	}

	return currentRoles;
}

const updateCurrentFleetMaps = async (currentFleetSettings) => {
	let currentMaps = {};
	if (currentFleetSettings.maps.length === 0) {
		notificationMsg(3, "No Map in fleet, flow function may have some issue.")
	} else {
		for (let index = 0; index < currentFleetSettings.maps.length; index++) {
			const mapName = currentFleetSettings.maps[index];
			let res = await fetchWmsStates(rmtToken_, mapName);
			if (res.ok) {
				let mapData = await res.json();
				let map_info = {};

				mapData.cells.forEach(({ area_id, cell_id, display_name }) => {
					if (!Object.hasOwn(map_info, area_id)){
						map_info[area_id] = [];
					}
					map_info[area_id].push({id: cell_id, displayName: display_name});
				})

				currentMaps[mapName] = map_info;
			} else {
				console.error(res.status, res.body)
			}
		}
	}
	// console.log(currentMaps)
	return currentMaps;
}

const updateCurrentFleetArtifacts = async (currentFleetSettings) => {
	let agentArtifacts = currentFleetSettings.artifacts.agent;
	let externalArtifacts = currentFleetSettings.artifacts.external;

	const artifactTypeList = await fetchGetArtifactTypes(rmtToken_);
	let artifactTypeDict = {};

	artifactTypeList.forEach(artfact => {
		let art_type = artfact.type;
		let art_category = artfact.category;
		let art_services = artfact.services;
		let services_params = [];

		if (art_category !=='Planning') {
			let servicesDict = {};
			art_services.forEach(paramsInfo => {
				let service = paramsInfo.service;
				let service_param = paramsInfo.parameter;

				service_param.forEach(params => {
					params.default = translateArtifactServiceDataToUI(params.default);
				});

				servicesDict[service] = service_param;
			})
			services_params.push(servicesDict);

			artifactTypeDict[art_type] = servicesDict;
		}
	});

	let currnetArtifactTypeDict = {};

	agentArtifacts.forEach(artfact => {
		for (const [agent_id, agent_artifacts] of Object.entries(artfact)) {
			agent_artifacts.forEach(agent_artfact => {
				const artifactType = agent_artfact.split('@')[0];
				const artifactID = agent_artfact.split('@')[1];

				if (currnetArtifactTypeDict.hasOwnProperty(artifactType)){
					currnetArtifactTypeDict[artifactType].availableIDs.push(artifactID);
				} else {
					currnetArtifactTypeDict[artifactType] = { availableIDs: [artifactID], params: artifactTypeDict[artifactType] };
				}
			})
		}
	})

	externalArtifacts.forEach(artfact => {
		const artifactType = artfact.split('@')[0];
		const artifactID = artfact.split('@')[1];

		if (currnetArtifactTypeDict.hasOwnProperty(artifactType)){
			currnetArtifactTypeDict[artifactType].availableIDs.push(artifactID);
		} else {
			currnetArtifactTypeDict[artifactType] = { availableIDs: [artifactID], params: artifactTypeDict[artifactType] };
		}
	})

	flowPageData.currentFleetSetting.artifactsTemplates = artifactTypeDict;

	return currnetArtifactTypeDict;
}

const initDataAsync = async () => {
	// ------ register user activity detector ------
	userActivityDetector();

	// ------ set up sidebar fleet switching callback function ------
	// setSwitchFleetCallback(switchFleetCallback);

	// ------ get login status ------
	var statusData = await restLoginStatus();
	await getLoginStatus(statusData, 'operation');

	// wait until sidebar fleet is generated
	// await sleep(1000); // sleep 1s

	// var selectedFleet = getSelectedFleet();
	// await getFleetSettingsData(selectedFleet + '.yaml');

	// for get rmt scan robot
	// await getRMTscanRobot(selectedFleet);

	// jQuery.ajaxSetup({
	//   async: false
	// });
	// getFlowData(selectedFleet);
	// jQuery.ajaxSetup({
	//   async: true
	// });

	// ------ [lang] switch language ------
	await initLanguageSupport();

	let lng = getSetLang() || 'en';
	flowPageData.langTemplateObj_ = await restGetTemplateLang(lng, 'flow_config');

	$('#available-items-title').html(flowPageData.langTemplateObj_.list.artf.lbl_Title);
}

const initDatePicker = async () => {
	await rmtTokenCheck();
	console.log(rmtToken_);
	var sys_date_millisec = await fetchGetSystemTime(rmtToken_);
	var sys_date = new Date(sys_date_millisec);
	console.log(' ----- data picker ------- ')
	console.log(sys_date_millisec)
	console.log(new Date(`${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`))
	$(".datepicker").datepicker({
	  clearBtn: true,
	  format: "yyyy/mm/dd",
	  startDate: new Date(`${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`)
	});

	$(".datepicker").each(function () {
	  $(this).datepicker('setDate', `${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`);
	});
}

// for assigned robot
const rmtTokenCheck = async () => {
  if (!rmtToken_) {
    console.log(rmtToken_);
    try {
      console.log('------ ###### rmt token re-fetch #### ------')
      rmtToken_ = await fetchToken();
    } catch (err) {
      console.error(err);
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

const updateFlowConditions = async () => {
	let currentConditions = {};
    let conditionsData = await fetchFlowConditions(rmtToken_);
    if (conditionsData.system_status_code === 5550000) {
        currentConditions = conditionsData.conditions;
    }

	return currentConditions;
}

const pageRender = () => {
	$("#flow_dashboard").hide();
	$("#flow_edit_dashboard").show();
	$("#back-div").show();
	$("#save-flow-div").show();

	$("#flow-deck .editing-flowname").text(getCurrentFlowName());

	$('#flow-profile-tab').css('display', 'none');
    $('#flow-home-tab').tab('show');
    $('#flow-home-tab').parent().css("width", "100%");
    $('#flow-home-tab').css("border-top-right-radius", "15px");
    $('#flow-home-tab').css('padding', '0px');
    $('#flow-home-tab').removeAttr("data-toggle");
    $('#flow-home-tab').removeAttr("href");

	// document.body.classList.add("sidebar-collapse");
}

const loadSavedData = async () => {
	const flowName = getCurrentFlowName();
	const flowFileName = `${flowPageData.currentFleet}-flow-${flowName}`;
	const flowUIFileName = `${flowPageData.currentFleet}-flowui-${flowName}`;

	// saved data
	let getData = undefined;
	await restGetFlowData(flowName, flowFileName).then(function(response) {
		// console.log("GetFlow Response:", response);
		getData = response;
	}).catch(function(error) {
		console.error("GetFlow Error:", error);
	});

	if (getData !== undefined && getData !== 'undefined') {
		updateSavedData(JSON.parse(getData));
	}

	// saved drawflow data
	let getUIData = undefined;
	await restGetFlowData(flowName, flowUIFileName).then(function(response) {
		// console.log("GetFlowUI Response:", response);
		getUIData = response;
	}).catch(function(error) {
		console.error("GetFlowUI Error:", error);
	});

	if (getUIData !== undefined && getUIData !== 'undefined') {
		flowPageData.drawFlowUIData = JSON.parse(getUIData);
	}
}

const updateSavedData = (loadData) => {
	console.log(loadData)

	// load flow type
	const flowType = loadData.flow_type.type;
	const flowInterval = loadData.flow_type.interval;
	const flowStartTime = loadData.flow_type.start_time;
	const flowEndtTime = loadData.flow_type.end_time;
	flowPageData.currentFlowType.type = flowType;
	flowPageData.currentFlowType.interval = flowInterval;
	flowPageData.currentFlowType.start_time = flowStartTime;
	flowPageData.currentFlowType.end_time = flowEndtTime;

	// load flow data
	const flowDatas = loadData.flow_data;
	console.log(flowDatas)

	flowDatas.forEach(data => {
		const dataType = data.type;
		const nodeID = data.node_id;
		const elementID = translateToNodeElementID(nodeID);

		let cloneSaveData = undefined;

		switch (dataType) {
			case "condition":
				cloneSaveData = _.cloneDeep(conditionSaveData);

				cloneSaveData.nodeId = elementID;
				cloneSaveData.conditions = data.conditions;
				cloneSaveData.type = "condition";

				flowPageData.currentSavedData[elementID] = cloneSaveData;
				break;
			case "condition_error":
				cloneSaveData = _.cloneDeep(conditionSaveData);

				cloneSaveData.nodeId = elementID;
				cloneSaveData.conditions = data.conditions;
				cloneSaveData.type = "condition";

				flowPageData.currentSavedData[elementID] = cloneSaveData;
				break;
			case "task":
				cloneSaveData = _.cloneDeep(saveData);
				const roleNmae = data.role_name;
				const taskParam = data.task_param;
				let tmpRoleObject = _.cloneDeep(flowPageData.currentFleetSetting.roles[roleNmae]);

				tmpRoleObject.forEach(behavior => {
					let title_content_params = Object.keys(behavior.title_content);
					title_content_params.forEach(paramKey => {
						if (taskParam.hasOwnProperty(paramKey)) {
							behavior.title_content[paramKey] = taskParam[paramKey];
						}
					});
				});

				cloneSaveData.nodeId = elementID;
				cloneSaveData.roleName = roleNmae;
				cloneSaveData.behavior = tmpRoleObject;
				cloneSaveData.error_handle.error_type = data.error_handle.error_type;
				cloneSaveData.error_handle.failure_timeout = data.error_handle.failure_timeout;
				cloneSaveData.error_handle.retry_limit = data.error_handle.retry_limit;
				cloneSaveData.type = "task";

				flowPageData.currentSavedData[elementID] = cloneSaveData;
				break;

			default:
				break;
		}
	});
}

const setUnSaveChange = (haveUnSaveData=false) => {
	flowPageData.haveUnSaveChange = haveUnSaveData;
}

const resetFlowPageSavedData = () => {
	flowPageData.currentSavedData = {};
	flowPageData.drawFlowUIData = undefined;
	setUnSaveChange()

	currentSelectedNode = {
		node_id: undefined,
		nodType: 'normal'
	};
}