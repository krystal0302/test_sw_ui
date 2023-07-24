const handleChargeTypeParam = (saveData) => {
    const ChargeTypeTmplate = document.getElementById('Charge');
    let chargeNode = document.importNode(ChargeTypeTmplate.content, true);

    // load saved data
    const haveSaveData = saveData === undefined? false: true;

	// add Nav2Client
	let goalsavedData = haveSaveData? saveData.goal: undefined;
    let goalnode = handleGoalTypeParam(goalsavedData, false, true);

    if (haveSaveData) {
		// load percentage data
		let targetDuration = chargeNode.querySelector('.percentage-input');
		targetDuration.value = saveData.percentage;
		goalnode.querySelector('.Nav2Client-cell-select').classList.remove('waitOtherParamComplete')
    } else {
		// add validate tag
		goalnode.querySelector('.Nav2Client-cell-select').classList.add('waitOtherParamComplete')
	}

	chargeNode.appendChild(goalnode);

    return chargeNode;
}

$(document).on("keyup", ".percentage-input", (inputE) => {
	let inputPercentage = inputE.currentTarget;
	let inpuVal = inputPercentage.value;
	const numberRegex = /^\d+$/;
	let isOkPercentage = numberRegex.test(inpuVal)

	const closestCardParent = inputPercentage.closest('.behavior-card');
	const closestCellSelect = closestCardParent.querySelector('.Nav2Client-cell-select');
	const containWaitOtherParamComple = closestCellSelect.classList.contains('waitOtherParamComplete');

	if (isOkPercentage && inpuVal >= 0 && 100 >= inpuVal) {
		inputPercentage.style.border = "none";
		inputPercentage.removeAttribute("title");

		if (containWaitOtherParamComple) {
			closestCellSelect.classList.remove('waitOtherParamComplete');
		}

		if (closestCellSelect.value === 'no-data') {
			setBehaviorDataVlidate(inputPercentage, 'show');
		} else {
			setBehaviorDataVlidate(inputPercentage, 'hide');
		}
	} else {
		setBehaviorDataVlidate(inputPercentage, 'show');
		inputPercentage.style.border = "2px solid red";
		inputPercentage.setAttribute("title", "Please Enter integer value between 0 ~ 100");

		if (!containWaitOtherParamComple) {
			closestCellSelect.classList.add('waitOtherParamComplete');
		}
	}
})

const chargeDataProcess = (data) => {
    let after = {
        percentage: undefined,
        goal: undefined
    }
    for (const [key, value] of Object.entries(data)) {
        if (key.includes('goal') && value !== 'undefined') {
            after.goal = value;
        } else if (key.includes('percentage') && value !== 'undefined') {
            after.percentage = value;
        }
    }
    return after;
}