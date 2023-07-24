
let gmappingState = false;

$('#toggle-gmapping').on('click', async function () {
	console.log('toggle gampping!');
	var targetAgent = document.getElementById('sel-gmapping-agent').value;
	console.log(targetAgent)
	gmappingState = !gmappingState;
	var state = (gmappingState) ? "ON" : "OFF";
	let result = await restPutParam({ agent: targetAgent, msg: state });
	if (result.gmappingState !== gmappingState) {
		notificationMsg(3, 'failed to set gmapping');
		// promptBox('error', 'Error: failed to set gmapping');
	}
	if (result.gmappingState) {
		notificationMsg(1, 'gmapping ACTIVATED!');
		// promptBox('success', 'Success: gmapping ACTIVATED!');
	}
	else {
		notificationMsg(1, 'gmapping DE-ACTIVATED!');
		// promptBox('success', 'Success: gmapping DE-ACTIVATED!');
	}
});

