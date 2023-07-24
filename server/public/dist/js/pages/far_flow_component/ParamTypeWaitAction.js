const handleWaitActionTypeParam = (saveData) => {
    const WaitActionTypeTmplate = document.getElementById('WaitAction');
    let waitNode = document.importNode(WaitActionTypeTmplate.content, true);

    // load saved data
    const haveSaveData = saveData === undefined? false: true;

    if (haveSaveData) {
        // load area
		let targetDuration = waitNode.querySelector('.waitaction-input');
		targetDuration.value = saveData;
    }

    return waitNode;
}

$(document).on("keyup", ".waitaction-input", (inputE) => {
	let inputDuration = inputE.currentTarget;
	let inpuVal = inputDuration.value;
	const numberRegex = /^\d+$/;
	let isOkDuration = numberRegex.test(inpuVal)
	if (isOkDuration && inpuVal >= 0) {
		setBehaviorDataVlidate(inputDuration, 'hide');
		inputDuration.style.border = "none";
		inputDuration.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputDuration, 'show');
		inputDuration.style.border = "2px solid red";
		inputDuration.setAttribute("title", "Please Enter integer");
	}
})