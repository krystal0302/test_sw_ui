function isEmptyString(str) {
    return (!str || str.length === 0 );
}

const adjustNumberDisplayText = (displayNumber) => {
	return `${displayNumber}`.length < 2? `0${displayNumber}`: `${displayNumber}`;
}

const haveSaveCachePageData = (nodeId) => {
	const nodeID = nodeId.includes('node-')? nodeId: `node-${nodeId}`;
	return flowPageData.currentSavedData[nodeID] === undefined? false : true;
}

const isNotCompleteSaveData = (data) => {
	let istemp = true

	if (typeof data === 'object' && data !== null) {
		istemp = false;
	} else if (typeof data === 'object' && data === null) {
		istemp = true;
	} else {
		istemp = (data === undefined || data === 'undefined' || isEmptyString(data) || data.includes('no-data'))? true: false;
	}

	return istemp
}

const setBehaviorDataVlidate = (elements, setType) => {
	let closestParent = elements.closest('.behavior-card');
	let vlidatElement = closestParent.getElementsByClassName('behaviorDataValidate')[0];

	if (setType === 'show') {
		vlidatElement.style.display = 'block';
	} else {
		vlidatElement.style.display = 'none';
	}
}

const haveUndefinedBehavior = () => {
	let undefinedCount = document.querySelectorAll('.drawflow-behavior-undefind-info:not(.validate)').length;

	if (undefinedCount > 0) {
		return true
	} else {
		// check all node have save value
		const allNodeID = [...document.querySelectorAll('.drawflow-node:not(.start):not(.finish)')].map(node => node.id);
		const savedDataCheckList = allNodeID.map(id => Object.keys(flowPageData.currentSavedData).includes(id));
		if (savedDataCheckList.includes(false)) {
			return true;
		} else {
			return false;
		}
	}
}

const generateUUID = () => {
    let uuid = "", i, random;
    for (i = 0; i < 32; i++) {
      random = (Math.random() * 16) | 0;
      if (i === 8 || i === 12 || i === 16 || i === 20) {
        uuid += "-";
      }
      uuid += (i === 12 ? 4 : i === 16 ? (random & 3) | 8 : random).toString(16);
    }
    return uuid;
}

const adjustTextMaxLength = (text) => {
	const maxLength = 20;

	if (text.length > maxLength) {
		return `${text.slice(0, maxLength)}...`;
	} else {
		return text
	}
}

const roundDecimal = function (val, precision) {
	return Math.round(Math.round(val * Math.pow(10, (precision || 0) + 1)) / 10) / Math.pow(10, (precision || 0));
}

const translateToNodeElementID = (nodeid) => {
	const nodeID = nodeid.includes('node-')? nodeid: `node-${nodeid}`;
	return nodeID
}

const translateToNodeDrawFlowID = (nodeid) => {
	const nodeID = nodeid.includes('node-')? nodeid.replace('node-', ''): `${nodeid}`;
	return nodeID
}

const getCurrentFlowName = () => {
	const queryString = window.location.search;
	const urlParams = new URLSearchParams(queryString);
	const flowName = urlParams.get('flowName');
	return flowName;
}

const setCurrentFlowName = (newFlowName) => {
	const queryString = window.location.search;
	const urlParams = new URLSearchParams(queryString);
	const flowName = urlParams.set('flowName', newFlowName);
	// window.location.replace(`${window.location.pathname}?${urlParams}`);
	window.history.pushState(null, null, `${window.location.pathname}?${urlParams}`);
}

const translateArtifactServiceDataToUI = (artifactDataVal) => {
	if (artifactDataVal === "True") {
		return true;
	} else if (artifactDataVal === "False") {
		return false;
	} else if (!isNaN(artifactDataVal)) {
		return Number(artifactDataVal);
	} else {
		return artifactDataVal
	}
}