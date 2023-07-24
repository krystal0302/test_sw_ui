const handleArtifactTypeParam = (saveData, templateData, behaviorIdx) => {
    const ArtifactTypeTmplate = document.getElementById('Artifact');
    let artifactNode = document.importNode(ArtifactTypeTmplate.content, true);

	// artifact info
	const artifactInfo = templateData.artifactInfo;
	const artifactType = artifactInfo.artifactType;
	const artifactService = artifactInfo.artifactService;

	artifactNode.querySelector('.show-artifact-type').textContent = artifactType;
	artifactNode.querySelector('.show-artifact-service').textContent = artifactService;

	// load saved data
    const haveSaveData = saveData === undefined? false: true;
	let selectedArtifactID = undefined;

    if (haveSaveData) {
        // load area
		for (const [key, value] of Object.entries(saveData)) {
			if (key.includes("artifact_id")) {
				selectedArtifactID = value;
			} else if (key.includes("value_")) {
				let updateTargetServiceData = templateData.data.params[artifactService];
				updateTargetServiceData.forEach(serviceData => {
					const paramName = serviceData.param_name
					serviceData.default = translateArtifactServiceDataToUI(value[paramName]);
				});
			}
		}
    }

	// Update Available artifact id
	const availableIDs = templateData.data.availableIDs;
	let artifactSelect = artifactNode.querySelector('.artifact-select');
	availableIDs.forEach(id => {
		let newOption = document.createElement('option');
		newOption.textContent = id;
		newOption.value = id;
		if (haveSaveData && selectedArtifactID === id) {
			newOption.selected = true;
		}
		artifactSelect.appendChild(newOption)
	});

	// Get relate service
	const targetServiceData = templateData.data.params[artifactService];

	targetServiceData.forEach(serviceData => {
		const serviceNode = getServiceNode(serviceData, behaviorIdx);

		if (serviceNode !== undefined) {
			artifactNode.appendChild(serviceNode);
		}
	});

    return artifactNode;
}

const getServiceNode = (serviceData, behaviorIdx) => {
	let paramNode = undefined;

	const serviceName = serviceData.param_name;
	const dataRange = serviceData.data_range;
	const dataType = serviceData.data_type;
	const dataDefault = serviceData.default;

	if (dataType ==='bool') {
		const ArtifactRadioTypeTmplate = document.getElementById('ArtifactTypeRadios');
		paramNode = document.importNode(ArtifactRadioTypeTmplate.content, true);

		let paramLabel = paramNode.querySelector('label');
		paramLabel.textContent = serviceName;
		paramLabel.dataset.dataRange = "";
		paramLabel.dataset.paramType = dataType;

		paramNode.querySelectorAll('.artifact-radio').forEach(radioInput => {
			let closestParent = radioInput.closest('div');
			let closestLabel = closestParent.querySelector('.form-check-label');

			const closestLabelText = closestLabel.textContent.trim();
			const currentName = radioInput.name;
			const changeID = `${currentName}${closestLabelText}_${behaviorIdx}`;

			if (dataDefault === true) {
				if (closestLabelText === 'True') {
					radioInput.checked = true;
				}
			} else {
				if (closestLabelText === 'False') {
					radioInput.checked = true;
				}
			}

			radioInput.id = changeID;
			radioInput.name = `${currentName}_${behaviorIdx}`;

			closestLabel.setAttribute("for", changeID);
		});
	} else if (dataType ==='select') {
		const ArtifactSelectTypeTmplate = document.getElementById('ArtifactTypeSelect');
		paramNode = document.importNode(ArtifactSelectTypeTmplate.content, true);

		let paramLabel = paramNode.querySelector('label');
		paramLabel.textContent = serviceName;
		paramLabel.dataset.dataRange = "";
		paramLabel.dataset.paramType = dataType;

		let paramSelect = paramNode.querySelector('select');
		const availableParams = JSON.parse(dataRange.replace(/'/g, '"'));

		availableParams.forEach(param => {
			let newOption = document.createElement('option');
			newOption.textContent = param;
			newOption.value = param;
			newOption.selected = dataDefault === param? true: false;
			paramSelect.appendChild(newOption)
		});
	} else if (dataType ==='string' || dataType ==='int' || dataType ==='double') {
		const ArtifactInputTypeTmplate = document.getElementById('ArtifactTypeInput');
		paramNode = document.importNode(ArtifactInputTypeTmplate.content, true);

		let paramLabel = paramNode.querySelector('label');
		paramLabel.textContent = serviceName;
		paramLabel.dataset.dataRange = dataRange.replace(':', ',');
		paramLabel.dataset.paramType = dataType;
		const strDataRange = dataRange.replace(':', ',');
		const listDataRange = JSON.parse(strDataRange);

		let paramInput = paramNode.querySelector('input');

		// handle int, double type
		if (dataType === "string") {
			paramInput.type = "text";
			paramInput.classList.add('artifact-input-string');
			paramInput.placeholder = 'Please enter string value.';

			paramInput.setAttribute('minlength', listDataRange[0]);
			paramInput.setAttribute('maxlength', listDataRange.pop());
		} else {
			paramInput.type = "number";
			paramInput.classList.add(`artifact-input-${dataType}`);
			paramInput.placeholder = `Please enter ${dataType} value.`;

			paramInput.min = listDataRange[0];
			paramInput.max = listDataRange.pop();
			paramInput.step = dataType === "int"? 1: 0.0001;
		}

		paramInput.value = dataDefault;
	}

	return paramNode;
}

$(document).on("keyup", ".artifact-input-double", (inputE) => {
	let inputDouble = inputE.currentTarget;
	let inpuVal = inputDouble.value;
	const doubleRegex = /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/;
	let isOkDouble = doubleRegex.test(inpuVal);

	const minVal = Number(inputDouble.min);
	const maxVal = Number(inputDouble.max);

	if (isOkDouble && inpuVal >= minVal && maxVal >= inpuVal) {
		setBehaviorDataVlidate(inputDouble, 'hide');
		inputDouble.style.border = "none";
		inputDouble.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputDouble, 'show');
		inputDouble.style.border = "2px solid red";
		inputDouble.setAttribute("title", `Please Enter double value between ${minVal} ~ ${maxVal}`);
	}
})

$(document).on("keyup", ".artifact-input-int", (inputE) => {
	let inputInt = inputE.currentTarget;
	let inpuVal = inputInt.value;
	const intRegex = /^-?\d+$/;
	let isOkInt = intRegex.test(inpuVal);

	const minVal = Number(inputInt.min);
	const maxVal = Number(inputInt.max);

	if (isOkInt && inpuVal >= minVal && maxVal >= inpuVal) {
		setBehaviorDataVlidate(inputInt, 'hide');
		inputInt.style.border = "none";
		inputInt.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputInt, 'show');
		inputInt.style.border = "2px solid red";
		inputInt.setAttribute("title", `Please Enter integer value between ${minVal} ~ ${maxVal}`);
	}
})

$(document).on("keyup", ".artifact-input-string", (inputE) => {
	let inputString = inputE.currentTarget;
	let inpuVal = inputString.value;
	// const stringRegex = /^[a-zA-Z]+$/;
	// let isOkString = stringRegex.test(inpuVal);
	let isOkString = typeof inpuVal === "string"

	const closestDivRow = inputString.closest('.row');
	const closestLabel = closestDivRow.querySelector('label');
	const dataRange = JSON.parse(closestLabel.dataset.dataRange);

	const minVal = Number(dataRange[0]);
	const maxVal = Number(dataRange.pop());

	if (isOkString && inpuVal.length >= minVal && maxVal >= inpuVal.length) {
		setBehaviorDataVlidate(inputString, 'hide');
		inputString.style.border = "none";
		inputString.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputString, 'show');
		inputString.style.border = "2px solid red";
		inputString.setAttribute("title", `Please Enter string length between ${minVal} ~ ${maxVal}`);
	}
})