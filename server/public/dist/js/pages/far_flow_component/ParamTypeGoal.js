const handleGoalTypeParam = (saveData, isMoving=true, isUseWithOtherParam=false) => {
    const GoalTypeTmplate = document.getElementById('Nav2Client');
    let goalNode = document.importNode(GoalTypeTmplate.content, true);

	if (isUseWithOtherParam) {
		let tmpTargetSpanDiv = goalNode.querySelector('span').closest('div');
		tmpTargetSpanDiv.style.display = 'none';
		// let tmpClosestCellSelect = goalNode.querySelector('.Nav2Client-cell-select');
		// tmpClosestCellSelect.classList.add('waitOtherParamComplete')
	}

	// set is moving or not
	const isMovingVal = isMoving? 'Y': 'N';
	let cellSelect = goalNode.querySelector('.Nav2Client-cell-select');
	cellSelect.dataset.isMoving = isMovingVal;

    // append map option
    let mapSelect = goalNode.querySelector('.Nav2Client-map-select');
    let availableMaps = Object.keys(flowPageData.currentFleetSetting.maps);
    const optionNav2ClientTmplate = document.getElementById('optionNav2Client');
    let optionNode = document.importNode(optionNav2ClientTmplate.content, true);

    // load saved data
    const haveSaveData = saveData === undefined? false: true;
    const mapDataArr = haveSaveData? saveData.split('@'): undefined;

    availableMaps.forEach(mapName => {
        const tmpNode = optionNode.cloneNode(true);
        let option = tmpNode.querySelector('option');

        option.textContent = mapName;
        option.className = 'Nav2Client-map-option';
        option.value = mapName;

        console.log(mapDataArr)
        console.log(mapName)
        if (haveSaveData && mapName === mapDataArr[0]) {
            option.selected = true
        }

        mapSelect.appendChild(option);
    });

    if (haveSaveData) {
        // load area
		const savedAreaData = flowPageData.currentFleetSetting.maps[mapDataArr[0]];
		if (savedAreaData !== undefined) {
			let closestAreaSelect = goalNode.querySelector('.Nav2Client-area-select');
			let availableAreas = Object.keys(savedAreaData);

			closestAreaSelect.disabled = false;
			availableAreas.forEach(areaName => {
				const tmpNode = optionNode.cloneNode(true);
				let option = tmpNode.querySelector('option');

				option.textContent = areaName;
				option.className = 'Nav2Client-area-option';
				option.value = areaName;

				if (areaName === mapDataArr[1]) {
					option.selected = true
				}

				closestAreaSelect.appendChild(option);
			});

			// load cell
			const savedCellData = savedAreaData[mapDataArr[1]];

			if (savedCellData !== undefined) {
				let closestCellSelect = goalNode.querySelector('.Nav2Client-cell-select');
				let availableCells = savedCellData;
				closestCellSelect.disabled = false;

				// Add auto option
				addAutoOption(closestCellSelect, optionNode, mapDataArr[2]);

				availableCells.forEach(cell => {
					const cellId = cell.id;
					const cellDisplayName = cell.displayName;
					const tmpNode = optionNode.cloneNode(true);
					let option = tmpNode.querySelector('option');

					option.textContent = cellDisplayName;
					option.className = 'Nav2Client-cell-option';
					option.value = cellId;

					if (cellId === mapDataArr[2]) {
						option.selected = true
					}

					closestCellSelect.appendChild(option);
				});
			}
		}
    }

    return goalNode;
}

// area select
$(document).on("change", ".Nav2Client-map-select", function () {
	let mapSelect = this;
	let closestParent = mapSelect.closest('.card-body');
	let closestAreaSelect = closestParent.getElementsByClassName('Nav2Client-area-select')[0];
	let closestCellSelect = closestParent.getElementsByClassName('Nav2Client-cell-select')[0];
	let mapSelectValue = mapSelect.value;

	// clearChild
	let areaNotDefaultOptions = closestAreaSelect.querySelectorAll('option:not([value="no-data"])');
	areaNotDefaultOptions.forEach(option => {
		option.remove();
	});
	let cellNotDefaultOptions = closestCellSelect.querySelectorAll('option:not([value="no-data"])');
	cellNotDefaultOptions.forEach(option => {
		option.remove();
	});

	if (mapSelectValue === 'no-data') {
		closestAreaSelect.disabled = true;
		closestCellSelect.disabled = true;
	} else {
		closestAreaSelect.disabled = false;

		let availableAreas = Object.keys(flowPageData.currentFleetSetting.maps[mapSelectValue]);
		const optionNav2ClientTmplate = document.getElementById('optionNav2Client');
		let optionNode = document.importNode(optionNav2ClientTmplate.content, true);

		availableAreas.forEach(areaName => {
			const tmpNode = optionNode.cloneNode(true);
			let option = tmpNode.querySelector('option');

			option.textContent = areaName;
			option.className = 'Nav2Client-area-option';
			option.value = areaName;
			closestAreaSelect.appendChild(option);
		});
	}

	setBehaviorDataVlidate(mapSelect, 'show');
});

// cell select
$(document).on("change", ".Nav2Client-area-select", function () {
	let areaSelect = this;
	let closestParent = areaSelect.closest('.card-body');
	let closestCellSelect = closestParent.getElementsByClassName('Nav2Client-cell-select')[0];
	let mapSelectValue = closestParent.getElementsByClassName('Nav2Client-map-select')[0].value;
	let areaSelectValue = areaSelect.value;

	// clearChild
	let notDefaultOptions = closestCellSelect.querySelectorAll('option:not([value="no-data"])');
	notDefaultOptions.forEach(option => {
		option.remove();
	});

	if (areaSelectValue === 'no-data') {
		closestCellSelect.disabled = true;
	} else {
		closestCellSelect.disabled = false;

		let availableCells = flowPageData.currentFleetSetting.maps[mapSelectValue][areaSelectValue];
		const optionNav2ClientTmplate = document.getElementById('optionNav2Client');
		let optionNode = document.importNode(optionNav2ClientTmplate.content, true);

		// Add auto option
		addAutoOption(closestCellSelect, optionNode);

		availableCells.forEach(cell => {
			const cellId = cell.id;
			const cellDisplayName = cell.displayName;
			const tmpNode = optionNode.cloneNode(true);
			let option = tmpNode.querySelector('option');

			option.textContent = cellDisplayName;
			option.className = 'Nav2Client-cell-option';
			option.value = cellId;
			closestCellSelect.appendChild(option);
		});
	}

	setBehaviorDataVlidate(areaSelect, 'show');
});

$(document).on("change", ".Nav2Client-cell-select", function () {
	let cellSelect = this;
	let cellSelectValue = cellSelect.value;
	const isWaitOtherParamComple = cellSelect.classList.contains('waitOtherParamComplete');

	if (isWaitOtherParamComple) {
		setBehaviorDataVlidate(cellSelect, 'show');
	} else {
		if (cellSelectValue === 'no-data') {
			setBehaviorDataVlidate(cellSelect, 'show');
		} else {
			setBehaviorDataVlidate(cellSelect, 'hide');
		}
	}
})

const addAutoOption = (AppendCellSelect, optionNode, selectedOptionVal='') => {
	const isMoving = AppendCellSelect.dataset.isMoving === 'Y'? true: false;

	// Add auto option
	if (isMoving) {
		const autoOption = {
			auto_empty: "Auto Empty",
			auto_occupied: "Auto Occupied"
		}

		for (const [optionVal, optionText] of Object.entries(autoOption)) {
			const tmpNode = optionNode.cloneNode(true);
			let option = tmpNode.querySelector('option');

			option.textContent = optionText;
			option.className = 'Nav2Client-cell-option';
			option.value = optionVal;
			if (selectedOptionVal === optionVal) {
				option.selected = true;
			}
			AppendCellSelect.appendChild(option);
		}
	} else {
		// add Auto for others
		const tmpNode = optionNode.cloneNode(true);
		let option = tmpNode.querySelector('option');

		option.textContent = 'Auto';
		option.className = 'Nav2Client-cell-option';
		option.value = 'auto';
		if (selectedOptionVal === 'auto') {
			option.selected = true;
		}
		AppendCellSelect.appendChild(option);
	}
}