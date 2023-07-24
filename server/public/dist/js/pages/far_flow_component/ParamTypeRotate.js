const handleRotateTypeParam = (saveData, behaviorIdx) => {
    const RotateTypeTmplate = document.getElementById('Rotate');
    let rotateNode = document.importNode(RotateTypeTmplate.content, true);

    // Update radio input and label id
    let rotateModeRadio = rotateNode.querySelector('.div-rotate-mode-radio');
    let modeFormChecks = rotateModeRadio.querySelectorAll('div.form-check');

    const isRotateByCell = saveData === undefined? true: saveData.isRotateByCell;
    const isClockwise = saveData === undefined? true: saveData.clockwise;
    const rotateSavedData = saveData === undefined? undefined: saveData.data;

    modeFormChecks.forEach((div, idx) => {
        let radioInput = div.querySelector('.form-check-input');
        let radioLabel = div.querySelector('.form-check-label');
        const new_id = `rotateMode-${idx}-${behaviorIdx}`;
        const new_name = `rotateModeRadio-${behaviorIdx}`;

        radioInput.id = new_id;
        radioInput.name = new_name;
        radioLabel.htmlFor = new_id;

        // add rotate by cell layout
        if (radioInput.value === '1') {
            let goalsavedData = (isRotateByCell && rotateSavedData!== undefined)? rotateSavedData: undefined;
            let goalnode = handleGoalTypeParam(goalsavedData, false, true);
            rotateNode.querySelectorAll('.tab-pane')[0].appendChild(goalnode);
        }

        if (isRotateByCell && radioInput.value === '1') {
            radioInput.checked = true;
        } else if (!isRotateByCell && radioInput.value === '2') {
            radioInput.checked = true;
            switchPanel(radioInput);
        } else {
            radioInput.checked = false;
        }
    });

    let rotateAngleRadio = rotateNode.querySelector('.div-rotate-angle-radio');
    let angleFormChecks = rotateAngleRadio.querySelectorAll('div.form-check');

    angleFormChecks.forEach((div, idx) => {
        let radioInput = div.querySelector('.form-check-input');
        let radioLabel = div.querySelector('.form-check-label');
        const new_id = `rotateAngle-${idx}-${behaviorIdx}`;
        const new_name = `rotateAngle-${behaviorIdx}`;

        radioInput.id = new_id;
        radioInput.name = new_name;
        radioLabel.htmlFor = new_id;

        if (!isRotateByCell) {
            if (isClockwise && radioInput.value === 'clockwise') {
                radioInput.checked = true;
            } else if (!isClockwise && radioInput.value === 'counterclockwise') {
                radioInput.checked = true;
            } else {
                radioInput.checked = false;
            }
        }
    });

    if (!isRotateByCell && rotateSavedData != undefined) {
        let angleInput = rotateNode.querySelector('.rotate-angle-input');
        angleInput.value = Math.abs(rotateSavedData);
    }

    return rotateNode;
}

const rotateDataProcess = (data) => {
    let after = {
        isRotateByCell: true,
        clockwise: true,
        data: undefined
    }
    for (const [key, value] of Object.entries(data)) {
        if (key.includes('goal') && value !== 'undefined') {
            after.isRotateByCell = true;
            after.data = value;
        } else if (key.includes('angle') && value !== 'undefined') {
          after.isRotateByCell = false;
          after.data = value;
          // NOTE: follow the Cartesian system, negative is clockwise
          after.clockwise = parseInt(value, 10) < 0;
        }
    }
    return after;
}

// rotate mode switch
const switchPanel = (radioInput) => {
	let parent = radioInput.closest('.rotate-content-setion');

	let tabPanes = parent.querySelectorAll('.tab-pane');
	let tabPaneindex = parseInt(radioInput.value) - 1;
	console.log($(tabPanes[tabPaneindex]))
	for (let index = 0; index < tabPanes.length; index++) {
		if (index === tabPaneindex) {
			$(tabPanes[index]).tab('show');
		} else {
			$(tabPanes[index]).removeClass('active');
		}
	}
};

const selectRotateMode = (radioInput) => {
	switchPanel(radioInput);

	let parent = radioInput.closest('.card-body');
	let tabPaneindex = parseInt(radioInput.value) - 1;
	let isRotateByCell = tabPaneindex === 0? true: false;

	if (isRotateByCell) {
		let mapSelectVal = parent.querySelector('.Nav2Client-map-select').value;
		let areaSelectVal = parent.querySelector('.Nav2Client-area-select').value;
		let cellSelectVal = parent.querySelector('.Nav2Client-cell-select').value;
		console.log(mapSelectVal, areaSelectVal, cellSelectVal)
		console.log(mapSelectVal !== 'no-data',areaSelectVal !== 'no-data',cellSelectVal !== 'no-data')
		if(mapSelectVal !== 'no-data' && areaSelectVal !== 'no-data' && cellSelectVal !== 'no-data') {
			setBehaviorDataVlidate(parent, 'hide');
		} else {
			setBehaviorDataVlidate(parent, 'show');
		}
	} else {
		let inputAngle = parent.querySelector('.rotate-angle-input');
		let roateAngleValue = inputAngle.value;
		const numberRegex = /^\d+$/;
		let isOkAngle = numberRegex.test(roateAngleValue);

		if (isOkAngle && roateAngleValue >= 0 && 180 >= roateAngleValue) {
      setBehaviorDataVlidate(parent, "hide");
      inputAngle.style.border = "none";
      inputAngle.removeAttribute("title");
    } else {
      setBehaviorDataVlidate(parent, "show");
      if (roateAngleValue != "") {
        inputAngle.style.border = "2px solid red";
        inputAngle.setAttribute("title", "Please Enter number between 0 ~ 180");
      }
    }
	}
}

$(document).on("keyup", ".rotate-angle-input", (inputE) => {
	let inputAngle = inputE.currentTarget;
	let inpuVal = inputAngle.value;
	const numberRegex = /^\d+$/;
	let isOkAngle = numberRegex.test(inpuVal)
	if (isOkAngle && inpuVal >= 0 && 360 >= inpuVal) {
		setBehaviorDataVlidate(inputAngle, 'hide');
		inputAngle.style.border = "none";
		inputAngle.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputAngle, 'show');
		inputAngle.style.border = "2px solid red";
		inputAngle.setAttribute("title", "Please Enter number between 0 ~ 360");
	}
})