const apiLogStyle = 'background: #222; color: #bada55';

function initAPIConfig(token) {
	axios.defaults.baseURL = `http://${window.location.hostname}:5000`;
	axios.defaults.headers = {
		accept: "application/json",
		Authorization: `${token.token_type} ${token.access_token}`,
		'Content-Type': 'application/json',
	};
	axios.interceptors.response.use(
		response => response,
		error => {
			const res = error.response;
			console.log(`%c[API ISSUE] ${res.config.method} ${res.config.url}`, apiLogStyle, res);
		});
}

async function faApiGetFunctionTypes() {
	const config = {
		method: 'GET',
		url: '/v3/wms/function-type?mode=raw_data',
	}

	return await axios(config);
}

async function faApiGetRobotsTemplates() {
	const config = {
		method: 'GET',
		url: '/v2/robots/templates/',
	}

	return await axios(config);
}


async function faApiGetScanRobot() {
	const config = {
		method: 'GET',
		url: '/v2/robots/scan/',
	}

	return await axios(config);
}

async function faApiPostFleetMapDeployment(robots, map) {
	const config = {
		method: 'POST',
		url: '/vPilot_forklift/maps/deploy/to-robots/',
		data: { "robot_list": robots, "map_name": map },
	}

	return await axios(config);
}

// Live Map
async function faApiGetInternalLiveMapData(currentFleet, currenMap) {
	let queryURL = (currentFleet === '' || currenMap === '') ? '' : `?fleet_name=${currentFleet}&map_name=${currenMap}`;
	const config = {
		method: 'GET',
		url: `/v2/internal/live-view${queryURL}`,
	}

	return await axios(config);
}

// Flow
async function faApiPostFlow(flowName, flowArgs) {
	const config = {
		method: 'POST',
		url: `/v2/flows/${flowName}`,
		data: flowArgs,
	}

	return await axios(config);
}

// ------ Forklist pilot ------
async function faApiGetAllEnvs() {
	const config = {
		method: 'GET',
		url: '/vPilot_forklift/maps',
	}

	return await axios(config);
}

async function pilotFetchGetAllEnvMapList(envName) {
	const config = {
		method: 'GET',
		url: `/vPilot_forklift/maps/${envName}/image-list`,
	}

	return await axios(config);
}

async function pilotFetchGetFilterRelateMap(envName) {
	const config = {
		method: 'GET',
		url: `/vPilot_forklift/maps/${envName}/filter-relate-map`,
	}

	return await axios(config);
}

async function pilotFetchGetAllEnvMapData(envName) {
	const config = {
		method: 'GET',
		url: `/vPilot_forklift/maps/${envName}/image-data`,
	}

	return await axios(config);
}

async function pilotFetchGetEnvMapGraph(envName) {
	const config = {
		method: 'GET',
		url: `/vPilot_forklift/maps/${envName}/graph`,
	}

	return await axios(config);
}

async function pilotFetchGetEnvMapCell(envName) {
	const config = {
		method: 'GET',
		url: `/vPilot_forklift/maps/${envName}/cells`,
	}

	return await axios(config);
}

async function pilotFetchDeleteEnvMap(envName) {
	const config = {
		method: 'DELETE',
		url: `/vPilot_forklift/maps/${envName}`,
	}

	return await axios(config);
}

async function pilotFetchDeleteAdditionalMap(envName, mapImage) {
	const config = {
		method: 'DELETE',
		url: `/vPilot_forklift/maps/${envName}/additional-map?additional_map=${mapImage}`,
	}

	return await axios(config);
}

// ------ Battery log ------
async function faApiGetLogJob(logType) {
	const config = {
		method: 'GET',
		url: `/v2/logs/${logType}-log/job`,
	}

	return await axios(config);
}

async function faApiPostLogJob(logType) {
	const config = {
		method: 'POST',
		url: `/v2/logs/${logType}-log/job`,
	}

	return await axios(config);
}

async function faApiGetLogProgress(logType) {
	const config = {
		method: 'GET',
		url: `/v2/logs/${logType}-log/progress`,
	}

	return await axios(config);
}

async function faApiGetLogPackageResult(logType) {
	const config = {
		method: 'GET',
		url: `/v2/logs/${logType}-log/result`,
		responseType: 'blob'
	}

	return await axios(config);
}