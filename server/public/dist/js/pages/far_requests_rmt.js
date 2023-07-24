/*
 * Author: John Wu
 * Date: 08 Sep. 21,
 * Description:
 *   Expectation: variables independent, commonly-used
 **/

const swarmHostName = `${window.location.hostname}`;
const swarmPort = '5000';
const swarmHost = `http://${swarmHostName}:${swarmPort}`;
const smrPort = '4000';

// ======== Fetch Token ===========================
function fetchToken() {
	return fetch(`${swarmHost}/login/access-token/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			"Content-Type": "application/x-www-form-urlencoded",
		},
		body: "username=root&password=root@farobot",
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

// ======== Scan ===========================
function fetchScanRobots(_token) {
	return fetch(`${swarmHost}/robots/scan/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchScanArtifacts(_token) {
	return fetch(`${swarmHost}/artifacts/scan/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchScanRobots2(_token) {
	return fetch(`${swarmHost}/v2/robots/scan/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchRobotsTemplates2(_token) {
	return fetch(`${swarmHost}/v2/robots/templates/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => { return response; });
}

function fetchScanArtifacts2(_token) {
	return fetch(`${swarmHost}/v2/artifacts/scan/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchGetArtifactTypes(_token) {
	return fetch(`${swarmHost}/v2/artifacts/type-list`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchGetArtifactsProperty(_token) {
	return fetch(`${swarmHost}/v2/artifacts/property`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchRobots(_token) {
	return fetch(`${swarmHost}/robots/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchMaps(_token) {
	return fetch(`${swarmHost}/maps/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchRoles(_token) {
	return fetch(`${swarmHost}/roles/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// ======== Version ========================
function fetchGetSwVerList(_token) {
	return fetch(`${swarmHost}/sw_update/get_sw_list`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPostAgentSwVer(_token, _agents) {
	return fetch(`${swarmHost}/sw_update/get_agent_sw_version/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json" // CRITICAL!
		},
		body: `{ "robot_list": ${_agents} }`,
	})
		.then((response) => { return response; });
}

function fetchPutAgentSwVer(_token, agentStr, _version) {
	let data = `{ "robot_list": ${agentStr}, "agent_sw_version": "${_version}"  }`;
	console.log('data: ', data);
	return fetch(
		`${swarmHost}/sw_update/update_agent_sw/`,
		{
			method: 'PUT',
			headers: {
				accept: 'application/json',
				Authorization: `${_token.token_type} ${_token.access_token}`,
				'Content-Type': 'application/json',
			},
			body: data,
		}
	)
		.then((response) => {
			return response;
		});
}

function fetchGetCoreSwVer(_token) {
	// --- real request ---
	return fetch(`${swarmHost}/sw_update/get_core_sw_version`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPutCoreSwVer(_token, _version) {
	return fetch(
		`${swarmHost}/sw_update/update_core_sw/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "core_sw_version": "${_version}"  }`,
	}
	)
		.then((response) => {
			return response;
		});
}

function fetchPutAddDebianPackage(_token, _formData) {
	return fetch(
		`${swarmHost}/v2/sw_update/add_debian_package/`,
		{
			method: 'PUT',
			headers: {
				accept: 'application/json',
				Authorization: `${_token.token_type} ${_token.access_token}`
			},
			body: _formData,
		}
	)
		.then((response) => response)
		.then((data) => {
			return data;
		});
}

function fetchPutArtifactSwVer(_token, agentStr, _version) {
	let data = `{"artifact_list": ${agentStr}, "artifact_sw_version": "${_version}" }`;
	console.log('data: ', data);

	return fetch(
		`${swarmHost}/v2/sw_update/update_artifact_sw/`,
		{
			method: 'PUT',
			headers: {
				accept: 'application/json',
				Authorization: `${_token.token_type} ${_token.access_token}`,
				'Content-Type': 'application/json',
			},
			body: data,
		}
	)
		.then((response) => {
			return response;
		});
}

function fetchOtaUpdateTarget(_token) {
	return fetch(`${swarmHost}/v2/sw_update/get_ota_update_target_core`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchAgentOtaSystemStatus(_token, _agents) {
	return fetch(
		`${swarmHost}/v2/sw_update/get_ota_sys_status_agent`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ${_agents} }`,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPutOtaSw(_token) {
	return fetch(
		`${swarmHost}/v2/sw_update/ota_update_core`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPutAgentOtaSw(_token, _agents) {
	return fetch(
		`${swarmHost}/v2/sw_update/ota_update_agent`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ${_agents} }`,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchOtaStatus(_token) {
	return fetch(`${swarmHost}/v2/sw_update/get_ota_status_core`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchAgentOtaStatus(_token, _agents) {
	return fetch(
		`${swarmHost}/v2/sw_update/get_ota_update_status_agent`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ${_agents} }`,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}


// ======== Map Deployment =================
function fetchPostFleetMapDeplyment(_token, _robots, _map) {
	return fetch(
		`${swarmHost}/maps/deploy`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ${_robots}, "map_name": "${_map}" }`,
	}
	)
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// ======== Settings =======================
function fetchAgentSettings2(_token, _agents) {
	return fetch(
		`${swarmHost}/settings/get_agent_settings/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ${_agents} }`,
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchAgentSettings(_token, _agent) {
	return fetch(
		`${swarmHost}/settings/get_agent_settings/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": [ "${_agent}" ] }`,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchAgentMetadata(_token, _agent) {
	return fetch(
		`${swarmHost}/settings/get_agent_metadata/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": [ "${_agent}" ] }`,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPutAgentSettings(_token, _robotId, _config) {
	return fetch(
		`${swarmHost}/settings/set_agent_settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": ["${_robotId}"], "agent_settings": {"wifi":"", "nav_conf": "${_config}"}  }`,
	})
		.then((response) => response.json())
		.then((data) => {
			console.log(data);
			return data;
		});
}

function fetchResetAgentSettings(_token, _agent) {
	return fetch(
		`${swarmHost}/v2/settings/reset-agent-settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "robot_list": [ "${_agent}" ] }`,
	})
		.then((response) => response.json())
		.then((data) => {
			console.log(data);
			return data;
		});
}

function fetchArtifactSettings(_token, _artifact) {
	return fetch(
		`${swarmHost}/settings/get_artifact_settings/`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "artifact_list": [ "${_artifact}" ] }`,
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchPutArtifactSettings(_token, _artifactId, _config) {
	return fetch(
		`${swarmHost}/settings/set_artifact_settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "artifact_list": ["${_artifactId}"], "artifact_settings": {"artifact_conf": "${_config}"}  }`,
	})
		.then((response) => response.json())
		.then((data) => {
			// console.log(data);
			return data;
		});
}

function fetchResetArtifactSettings(_token, _artifact) {
	return fetch(
		`${swarmHost}/v2/settings/reset-artifact-settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "artifact_list": [ "${_artifact}" ] }`,
	})
		.then((response) => response.json())
		.then((data) => {
			console.log(data);
			return data;
		});
}

function fetchPutCoreSettings(_token, _config) {
	return fetch(
		`${swarmHost}/settings/set_core_settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: `{ "core_settings": "${_config}" }`,
	})
		.then((response) => response.json())
		.then((data) => {
			console.log(data);
			return data;
		});
}

function fetchGetCoreSettings(_token) {
	return fetch(
		`${swarmHost}/settings/get_core_settings/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchResetCoreSettings(_token) {
	return fetch(
		`${swarmHost}/v2/settings/reset-core-settings/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchCoreMetadata(_token) {
	return fetch(
		`${swarmHost}/settings/get_core_metadata/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchCoreSettingsPackage(_token) {
	return fetch(
		`${swarmHost}/v2/settings/core-setting-package/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
	})
		.then(response => response.blob())
		.then((data) => {
			return data;
		});
}

function fetchPutCoreSettingsPackage(_token, _formData) {
	return fetch(
		`${swarmHost}/v2/settings/core-setting-package/`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
		body: _formData,
	})
		.then((response) => response)
		.then((data) => {
			return data;
		});
}

// ======== Swarm Update ===================
function fetchGetCoreSwUpdateStatus(_token) {
	// --- real request ---
	return fetch(`${swarmHost}/sw_update/get_core_sw_update_status`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// ======== Artifact Update ===================
function fetchPostArtifactSwUpdateStatus(_token, _agentStr) {
	let data = `{
		"artifact_list": ${_agentStr}
		}`;

	return fetch(
		`${swarmHost}/v2/sw_update/get_artifact_sw_update_status`,
		{
			method: 'POST',
			headers: {
				accept: 'application/json',
				Authorization: `${_token.token_type} ${_token.access_token}`,
				'Content-Type': 'application/json',
			},
			body: data,
		}
	)
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// ======== Agents Update ===================
function fetchPostAgentsSwUpdateStatus(_token, _agentStr) {
	let data = `{
		"robot_list": ${_agentStr}
		}`;

	return fetch(
		`${swarmHost}/sw_update/get_agent_sw_update_status`,
		{
			method: 'POST',
			headers: {
				accept: 'application/json',
				Authorization: `${_token.token_type} ${_token.access_token}`,
				'Content-Type': 'application/json',
			},
			body: data,
		}
	)
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}



// ======== License Validation =============
function fetchGetLicenseValidation(_token) {
	return fetch(`${swarmHost}/v2/license/check_result`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetLicenseContainerInfo(_token) {
	return fetch(`${swarmHost}/v2/license/cmcontainer`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetLicenseInfo(_token) {
	return fetch(`${swarmHost}/v2/license/license?product_code=2000`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetLicenseFootprint(_token) {
	return fetch(`${swarmHost}/v2/license/footprint`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPostLicense(_token, _formData) {
	return fetch(
		`${swarmHost}/v2/license/update`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
		body: _formData,
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetSLAConfirmation(_token) {
	return fetch(`${swarmHost}/v2/license/sla-confirmation`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPostSLAConfirmation(_token) {
	return fetch(`${swarmHost}/v2/license/sla-confirmation`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; })
}

// ======== Tasks Manipulation =============
function fetchFlowList(_token, _fleetName) {
	var queryURL = (_fleetName === '') ? '' : `?fleet_name=${_fleetName}`;
	return fetch(`${swarmHost}/v2/flows${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchPostFlow(_token, _flowName, _options) {
	return fetch(`${swarmHost}/v2/flows/${_flowName}`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
		body: `{ "args": ${JSON.stringify(_options.args)}}`
	})
		.then((response) => { return response; })
}

function fetchPostTask(_token, _flowId, _flowName, _options) {
	return fetch(`${swarmHost}/v2/flows/trigger?flow_id=${_flowId}&flow_name=${_flowName}`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
		body: `{ "args": ${JSON.stringify(_options.args)}}`
	})
		.then((response) => { return response; })
}

function fetchDeleteFlow(_token, _flowId) {
	return fetch(`${swarmHost}/v2/flows/${_flowId}`, {
		method: "DELETE",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchDeleteTask(_token, _taskId) {
	return fetch(`${swarmHost}/v2/flows/tasks/${_taskId}`, {
		method: "DELETE",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchPutFlow(_token, _flowId, _action = "pause") {
	return fetch(`${swarmHost}/v2/flows/${_flowId}?action=${_action}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchPutTask(_token, _taskId, _action = "pause") {
	return fetch(`${swarmHost}/v2/flows/tasks/${_taskId}?action=${_action}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}


// ======== Tasks Manipulation =============
function fetchWmsStates(_token, _mapName = '') {
	var queryURL = (_mapName === '') ? '' : `?map=${_mapName}`;
	return fetch(`${swarmHost}/v2/wms${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchFleetStates(_token, _fleetName = '', _mode = '') {
	var queryURL = (_fleetName === '') ? '' : `?fleet_name=${_fleetName}`;

	if (_mode != '') {
		if (queryURL === '') {
			queryURL = `?mode=${_mode}`;
		} else {
			queryURL += `&mode=${_mode}`;
		}
	}
	return fetch(`${swarmHost}/v2/fleets/status${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchFleetStatus(_token, _fleetName = '') {
	var queryURL = (_fleetName === '') ? '' : `?fleet_name=${_fleetName}`;
	return fetch(`${swarmHost}/v2/fleets/status${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchArtifactStatus(_token, _fleetName = '') {
	var queryURL = (_fleetName === '') ? '' : `?fleet_name=${_fleetName}`;
	return fetch(`${swarmHost}/v2/artifacts/status${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchGetTimerFlowStatus(_token, _flow_id = '') {
	var queryURL = (_flow_id === '') ? '' : `?flow_id=${_flow_id}`;
	return fetch(`${swarmHost}/v2/flows/timer-flow-status${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchGetFlowStates(_token, _fleetName = '') {
	var queryURL = (_fleetName === '') ? '' : `?fleet_name=${_fleetName}`;
	return fetch(`${swarmHost}/v2/flows/status${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchPaths(_token) {
	return fetch(`${swarmHost}/v2/paths`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchPathMarkers(_token, _mapName) {
	return fetch(`${swarmHost}/v2/paths/markers?ns=${_mapName}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchHikForbiddenArea(_token) {
	return fetch(`${swarmHost}/v2/hik/forbidden-area`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchRobotStatus(amr_ip) {
	const smrUrl = `http://${amr_ip}:${smrPort}/status/get_robot_status`
	return fetch(smrUrl, {
		method: "GET",
	});
}

function fetchGmappingStatus(amr_ip) {
	const smrUrl = `http://${amr_ip}:${smrPort}/status/get_gmapping_status`
	return fetch(smrUrl, {
		method: "GET",
	});
}

function fetchCreateMapImage(amr_ip) {
	const smrUrl = `http://${amr_ip}:${smrPort}/status/get_create_map`
	return fetch(smrUrl, {
		method: "GET",
	});
}

function putEnableSLAM(amr_ip) {
	const smrUrl = `http://${amr_ip}:${smrPort}/mode/SLAM`
	return fetch(smrUrl, {
		method: "PUT",
	});
}

function putDisnableSLAM(amr_ip) {
	const smrUrl = `http://${amr_ip}:${smrPort}/mode/end_SLAM`
	return fetch(smrUrl, {
		method: "PUT",
	});
}

function putSetManualLock(amr_ip, lockStatus) {
	const smrUrl = `http://${amr_ip}:${smrPort}/mode/set_manual_lock/${lockStatus}`
	return fetch(smrUrl, {
		method: "PUT",
	});
}

function putOpenLoop(amr_ip, vx, w) {
	const smrUrl = `http://${amr_ip}:${smrPort}/control/open_loop/${vx}/${w}`
	return fetch(smrUrl, {
		method: "PUT",
	});
}

function fetchSelectedMap(amr_ip, mapName) {
	const smrUrl = `http://${amr_ip}:${smrPort}/status/specific_map_data/${mapName}`
	return fetch(smrUrl, {
		method: "GET",
	});
}

function postSaveSlamMap(amr_ip, mapName, degree) {
	const smrUrl = `http://${amr_ip}:${smrPort}/fio/save_slam_map/${mapName}/${degree}`
	return fetch(smrUrl, {
		method: "POST",
	});
}

function fetchRobotMaps(_token, _robot_id) {
	return fetch(`${swarmHost}/v2/maps/robot_maps?robot_id=${_robot_id}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchUploadAgentMap(_token, _agent, _map_name) {
	return fetch(
		`${swarmHost}/v2/maps/upload_robot_map`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
		body: JSON.stringify({ "robot_id": _agent, "map_name": _map_name }),
	})
		.then(function (response) {
			console.log(response.status)
			if (response.status !== 201) {
				return false;
			} else {
				return true;
			}
		})
	// .then((data) => {
	// 	// console.log(data);
	// 	return data;
	// });
}

function fetchGetSystemTime(_token) {
	var time = "";
	$.ajax({
		url: `/getTime`,
		type: 'GET',
		headers: {
			// "Authorization": `bearer ${_token.access_token}`
		},
		async: false,
		success: function (data) {
			let mili_date = JSON.parse(data)['mili'];
			console.log(`fetchMappingData success: ${mili_date}`)
			// var sysdate_milli = Date.parse(mili_date);
			var current_date = new Date(mili_date);
			current_date.setHours(0, 0, 0, 0);
			time = current_date.getTime();
			sys_time = time;
			console.log(`fetchMappingData success: ${current_date}`)
		},
		error: function (data) {
			// console.log(`fetchMappingData Error: ${data}`)
			// console.log(data.getResponseHeader('Date'))
			// var sysdate_milli = Date.parse(data.getResponseHeader('Date'));
			// var current_date = new Date(sysdate_milli);
			// current_date.setHours(0, 0, 0, 0);
			// time = current_date.getTime();
			// sys_time = sysdate_milli;
		}
	});
	return time;
}

// ======== artifact reboot =============
function fetchGetSwarmCoreLiveliness(_token) {
	return fetch(`${swarmHost}/docs`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
	})
		.then((response) => { return response; });
}

function fetchPutRebootSwarmCore(_token) {
	return fetch(`${swarmHost}/v2/restart/core`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
	})
		.then((response) => { return response; });
}

function fetchPutRebootAgent(_token, _agentId) {
	console.log(_agentId);
	return fetch(`${swarmHost}/v2/restart/robot`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "robot_list": ["${_agentId}"]}`,
	})
		.then((response) => { return response; });
}

function fetchPutRebootArtifact(_token, _artifactId) {
	console.log(_artifactId);
	return fetch(`${swarmHost}/v2/restart/artifact`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "artifact_list": ["${_artifactId}"]}`,
	})
		.then((response) => { return response; });
}

// ======== artifact reboot =============
function fetchGetFleets(_token) {
	return fetch(`${swarmHost}/v2/fleets`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetFleetConfigs(_token, _fleetName) {
	return fetch(`${swarmHost}/v2/fleets/${_fleetName}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchPostFleetConfigs(_token, _fleetName) {
	return fetch(`${swarmHost}/v2/fleets/${_fleetName}`, {
		method: "POST",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; })
}

function fetchPutFleetConfigs(_token, _fleetName, _fleetConfig) {
	var _fleetConfig = (typeof _fleetConfig == 'object') ? JSON.stringify(_fleetConfig) : _fleetConfig;
	return fetch(`${swarmHost}/v2/fleets/${_fleetName}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: _fleetConfig
	})
		.then((response) => { return response; });
}

function fetchDeleteFleetConfigs(_token, _fleetName) {
	return fetch(`${swarmHost}/v2/fleets/${_fleetName}`, {
		method: "DELETE",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchGetAllRoles(_token) {
	return fetch(`${swarmHost}/v2/roles/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchGetAllMaps(_token) {
	return fetch(`${swarmHost}/v2/maps/`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchGetMapMeta(_token, _mapName) {
	console.log(_mapName);
	return fetch(`${swarmHost}/v2/maps/${_mapName}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutMapNameAlias(_token, _mapName, _newMapName) {
	return fetch(`${swarmHost}/v2/maps/${_mapName}?new_alias_name=${_newMapName}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
	})
		.then((response) => { return response; });
}

function fetchPutMapImage(_token, _mapName, _mapImage) {
	return fetch(`${swarmHost}/v2/maps/${_mapName}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "new_map_image_data": { "content": "${_mapImage}"} }`
	})
		.then((response) => { return response; });
}

function fetchPutMapMeta(_token, _mapName, _mapMeta) {
	// _mapMeta = (typeof _mapMeta == 'object') ? JSON.stringify(_mapMeta) : _mapMeta;
	console.log(_mapMeta);
	let putMapMeta = {
		"image": `${_mapName}.png`,
		"mode": _mapMeta.mode,
		"resolution": _mapMeta.resolution,
		"origin_x": _mapMeta.origin_x,
		"origin_y": _mapMeta.origin_y,
		"origin_theta": _mapMeta.origin_theta,
		"negate": _mapMeta.negate,
		"occupied_thresh": _mapMeta.occupied_thresh,
		"free_thresh": _mapMeta.free_thresh,
		"nav_graph": `${_mapName}.dot`,
		"cell": `${_mapName}.json`,
		"triton": `${_mapName}.amf`
	};
	return fetch(`${swarmHost}/v2/maps/${_mapName}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "new_map_info_data": ${JSON.stringify(putMapMeta)} }`
	})
		.then((response) => { return response; });
}

function fetchDeleteMap(_token, _mapName) {
	return fetch(`${swarmHost}/v2/maps/${_mapName}`, {
		method: "DELETE",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchGetMapGraph(_token, _mapName) {
	_mapName = _mapName.split('.').shift();
	return fetch(`${swarmHost}/v2/maps/${_mapName}/graph`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutMapGraph(_token, _mapName, _mapGraph) {
	return fetch(`${swarmHost}/v2/maps/${_mapName}/graph`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "content": "${_mapGraph}" }`
	})
		.then((response) => { return response; });
}

function fetchGetMapCells(_token, _mapName) {
	_mapName = _mapName.split('.').shift();
	return fetch(`${swarmHost}/v2/maps/${_mapName}/cells`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutMapCells(_token, _mapName, _mapCells) {
	var _mapCells = (typeof _mapCells == 'object') ? JSON.stringify(_mapCells) : _mapCells;
	return fetch(`${swarmHost}/v2/maps/${_mapName}/cells`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: `{ "override": ${_mapCells} }`
	})
		.then((response) => { return response; });
}

// ======== Logs =======================
const LogType = {
	Battery: "battery",
	Debug: "debug",
	Event: "event"
}

function fetchLogPackage(_token, _logType) {
	return fetch(`${swarmHost}/v2/logs/${_logType}-log`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`
		},
	})
		.then(response => response.blob())
		.then((data) => {
			return data;
		});
}

function fetchLogPackageProgress(_token, _logType) {
	return fetch(`${swarmHost}/v2/logs/${_logType}-log/progress`, {
		method: 'GET',
		headers: {
			accept: 'application/json',
			Authorization: `${_token.token_type} ${_token.access_token}`
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetCellRelation(_token) {
	return fetch(`${swarmHost}/v2/maps/cell_relationships`, {
		method: 'GET',
		headers: {
			accept: 'application/json',
			Authorization: `${_token.token_type} ${_token.access_token}`
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// function fetchPutCellRelation(_token) {
// 	return fetch(`${swarmHost}/v2/maps/cell_relationships`, {
// 		method: 'PUT',
// 		headers: {
// 			accept: 'application/json',
// 			Authorization: `${_token.token_type} ${_token.access_token}`
// 		}
// 	})
// 		.then((response) => response.json())
// 		.then((data) => {
// 			return data;
// 		});
// }

function fetchPutCellRelation(_token, _cellRelation) {
	let strCellRelation = (typeof _cellRelation == 'object') ? JSON.stringify(_cellRelation) : _cellRelation;
	return fetch(`${swarmHost}/v2/maps/cell_relationships`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: strCellRelation
	})
		.then((response) => { return response; });
}

// ======== Function Type ==============
function fetchGetFunctionTypes(_token) {
	return fetch(`${swarmHost}/v2/wms/function-type?mode=raw_data`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchGetSupportingFunctionTypes(_token) {
	return fetch(`${swarmHost}/v2/wms/function-type/supported-recog`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutFunctionTypes(_token, _functionTypes) {
	_functionTypes = (typeof _functionTypes === 'object') ? JSON.stringify(_functionTypes) : _functionTypes;
	console.log(_functionTypes);
	return fetch(`${swarmHost}/v2/wms/function-type`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: _functionTypes
	})
		.then((response) => { return response; });
}

function fetchPutCellLoad(_token, _cellLoad) {
	let strCellLoad = (typeof _cellLoad == 'object') ? JSON.stringify(_cellLoad) : _cellLoad;
	return fetch(`${swarmHost}/v2/wms`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: strCellLoad
	})
		.then((response) => { return response; });
}

function fetchHistorySchedule(_token, _startTime, _endTime, _fleetName, _robots, _flows) {
	var queryURL = `?start_time=${encodeURIComponent(_startTime)}&end_time=${encodeURIComponent(_endTime)}&fleet_name=${_fleetName}`;
	if (_robots.length > 0) {
		_robots.map(r => queryURL += `&robots=${r}`);
	}
	if (_flows.length > 0) {
		_flows.map(f => queryURL += `&flow_name=${f}`);
	}
	// console.log(queryURL);
	return fetch(`${swarmHost}/v2/schedule/history${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchEstSchedule(_token, _startTime, _endTime, _fleetName, _robots, _flows) {
	var queryURL = `?start_time=${encodeURIComponent(_startTime)}&end_time=${encodeURIComponent(_endTime)}&fleet_name=${_fleetName}`;
	if (_robots.length > 0) {
		_robots.map(r => queryURL += `&robots=${r}`);
	}
	if (_flows.length > 0) {
		_flows.map(f => queryURL += `&flow_name=${f}`);
	}
	// console.log(queryURL);
	return fetch(`${swarmHost}/v2/schedule${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json'
		},
	})
		.then((response) => { return response; });
}

function fetchFlowConditions(_token) {
	return fetch(`${swarmHost}/v2/flows/conditions`, {
		method: 'GET',
		headers: {
			accept: 'application/json',
			Authorization: `${_token.token_type} ${_token.access_token}`
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchArtifactTemplate(_token) {
	return fetch(`${swarmHost}/v2/artifacts/templates`, {
		method: 'GET',
		headers: {
			accept: 'application/json',
			Authorization: `${_token.token_type} ${_token.access_token}`
		}
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

// ======== Zone =======================
function fetchZoneMetadata(_token) {
	return fetch(
		`${swarmHost}/v2/zone/get_zone_metadata`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			"Content-Type": "application/json"
		},
	})
		.then((response) => response.json())
		.then((data) => {
			return data;
		});
}

function fetchGetZoneConfig(_token, _mapName) {
	var queryURL = (_mapName === '') ? '' : `?map_name=${_mapName}`;
	return fetch(`${swarmHost}/v2/zone/config${queryURL}`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutZoneConfig(_token, _mapName, _zoneConfigs) {
	var queryURL = (_mapName === '') ? '' : `?map_name=${_mapName}`;
	var _zoneConfigs = (typeof _zoneConfigs == 'object') ? JSON.stringify(_zoneConfigs) : _zoneConfigs;
	return fetch(`${swarmHost}/v2/zone/config${queryURL}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: _zoneConfigs
	})
		.then((response) => { return response; });
}

function fetchPutZoneActivation(_token, _mapName, _zoneId, _isActivate) {
	var queryURL = `?map_name=${_mapName}&uuid=${_zoneId}&activate=${_isActivate}`;
	return fetch(`${swarmHost}/v2/zone/activation${queryURL}`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; });
}

// ============ Reflector ===================
function fetchGetReflectorConfig(_token, _mapName) {
	// var queryURL = (_mapName === '') ? '' : `?map_name=${_mapName}`;
	return fetch(`${swarmHost}/v2/maps/${_mapName}/reflector-data`, {
		method: "GET",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		}
	})
		.then((response) => { return response; })
}

function fetchPutReflectorConfig(_token, _mapName, _reflectorConfigs) {
	_reflectorConfigs = (typeof _reflectorConfigs == 'object') ? JSON.stringify(_reflectorConfigs) : _reflectorConfigs;
	return fetch(`${swarmHost}/v2/maps/${_mapName}/reflector-data`, {
		method: "PUT",
		headers: {
			accept: "application/json",
			Authorization: `${_token.token_type} ${_token.access_token}`,
			'Content-Type': 'application/json',
		},
		body: _reflectorConfigs
	})
		.then((response) => { return response; });
}