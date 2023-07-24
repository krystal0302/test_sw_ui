const handleReportTypeParam = (saveData) => {
    const ReportTypeTmplate = document.getElementById('Report');
    let reportNode = document.importNode(ReportTypeTmplate.content, true);

    // load saved data
    const haveSaveData = saveData === undefined? false: true;

    if (haveSaveData) {
        // load area
		let targetReportMsg = reportNode.querySelector('.report-input');
		targetReportMsg.value = saveData;
    }

    return reportNode;
}

$(document).on("keyup", ".report-input", (inputE) => {
	let inputReportMsg = inputE.currentTarget;
	let inpuVal = inputReportMsg.value;
	const numberRegex = /^[a-zA-Z\d]+$/;
	console.log(inpuVal)
	let isOkDuration = numberRegex.test(inpuVal)
	console.log(isOkDuration)
	if (isOkDuration && inpuVal.length >= 0 && inpuVal.length <= 20) {
		setBehaviorDataVlidate(inputReportMsg, 'hide');
		inputReportMsg.style.border = "none";
		inputReportMsg.removeAttribute("title");
	} else {
		setBehaviorDataVlidate(inputReportMsg, 'show');
		inputReportMsg.style.border = "2px solid red";
		inputReportMsg.setAttribute("title", "Please Enter integer");
	}
})