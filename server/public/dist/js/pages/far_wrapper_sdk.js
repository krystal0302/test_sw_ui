/*
 * Author: John Wu
 * Date: 05 Oct. 21,
 * Description:
 **/

// ========================
//     Glocal Variables 
// ========================
// let lsArtifacts;
let lsArtifacts = [
	{
		type: 'conveyor',
		id: 'conveyor01',
		wrapper: 'wrapperB'
	},
];


// ======================
//       Load Ready 
// ======================
$(function () {
	initRundownAsync();
});

let wrapperCache_;
let wrapperTypes_;
async function initRundownAsync() {
	// ------ sidebar generation ------
	// ------ register user activity detector ------
	userActivityDetector();

	// ------ get login status ------
	var statusData = await restLoginStatus();
	getLoginStatus(statusData, 'logs');

	// ------ start to play ------
	var myWrappers = await cliListWrappers();
	myWrappers = myWrappers.output;
	console.log(myWrappers);
	// --- sort the wrappers by name ---
	wrapperCache_ = myWrappers.sort((w1, w2) => {
		if (w1.name < w2.name) { return -1; }
		if (w1.name > w2.name) { return 1; }
		return 0;
	});
	loadWrappers(myWrappers)

	// lsArtifacts = await cliListArtifacts();
	// lsArtifacts = lsArtifacts.output;
	console.log(lsArtifacts);
	loadArtifacts(lsArtifacts);

	wrapperTypes_ = await cliListWrapperTypes();
	wrapperTypes_ = wrapperTypes_.output;
	console.log(wrapperTypes_);

	var sel = document.getElementById('sel-wrapper-type');
	for (var key in wrapperTypes_) {
		var opt = document.createElement("option");
		opt.text = wrapperTypes_[key];
		sel.options.add(opt);
	}

	// toggle color theme again after DOM elements are all generated
	var isDarkMode = getSavedTheme() === 'dark';
	toggleContentDarkTheme(isDarkMode);
}

function loadArtifacts(_artifacts) {
	console.log(_artifacts);
	var $artDeck = $('#artifacts-deck');
	// console.log($artDeck);
	$artDeck.empty();
	_artifacts.forEach((art) => {
		console.log(art);
		var node = createArtifactsWithWrappersCardView(art);
		$artDeck.append(node);
	});
}

function createArtifactsWithWrappersCardView(_artObj) {
	const template = document.querySelector('#artifact-card');
	const node = document.importNode(template.content, true);

	var avatarNode = node.querySelector('.card-img-top');
	avatarNode.src = `dist/img/sprites/${_artObj.type}-440x220.png`;

	var titleNode = node.querySelector('.artifact-id');
	titleNode.textContent = _artObj.id;

	var cardNode = node.querySelector('.card');

	// --- warpper services edit event callback ---
	var wrpSelNode = node.querySelector('.wrappers-sel');
	$(wrpSelNode).empty();
	// --- update the wrapper option from artifacts ---
	for (var key in wrapperCache_) {
		var opt = document.createElement("option");
		opt.text = wrapperCache_[key].name;
		if (opt.text === _artObj.wrapper) {
			opt.selected = 'selected';
		}
		wrpSelNode.options.add(opt);
	}

	// --- artifact wrapper deploy callback ---
	var wrpDeployNode = node.querySelector('.wrapper-deploy');
	wrpDeployNode.addEventListener('click', btnDeployWrapperCb.bind(cardNode, _artObj.id));

	return node;
}

function btnDeployWrapperCb(_artId) {
	console.log(_artId);

	var wrpSel = this.querySelector('.wrappers-sel');
	var wrapperName = wrpSel.options[wrpSel.selectedIndex].value;

	// console.log(`deploy ${wrapperName} wrapper on ${_artId} artifact!`);
	// --- run the deployment --- 
	var retMsg = cliDeployWrapperOnArtifact(_artId, wrapperName);
	console.log(retMsg);

	// TODO: --- parse the return message ---

	notificationMsg(0, `> python3 wrapper deploy ${_artId} ${wrapperName}`);
	// toast(`> python3 wrapper deploy ${_artId} ${wrapperName}`);
	// toast(`${wrapperName} wrapper is deployed to ${_artId}`);
}

function loadWrappers(_wrappers) {
	console.log(_wrappers);
	var $wrpList = $('#wrappers-list');
	$wrpList.empty();
	_wrappers.forEach((w) => {
		var node = createWrapperCardView(w);
		$wrpList.append(node);
	});
}

function createWrapperCardView(_wrpObj) {
	const template = document.querySelector('#wrapper-card');
	const node = document.importNode(template.content, true);

	var titleNode = node.querySelector('.wrapper-name');
	titleNode.textContent = _wrpObj.name;

	var cardNode = node.querySelector('.card');

	// --- warpper services edit event callback ---
	var configNode = node.querySelector('.config-role');
	configNode.addEventListener('click', loadSidebarCb.bind(cardNode, _wrpObj));

	// --- role remove event callback ---
	var cardNode2 = node.querySelector('.form-inline');
	var removeNode = node.querySelector('.remove-role');
	removeNode.addEventListener('click', removeWrapperCb.bind(cardNode2, _wrpObj.name));

	return node;
}

function loadSidebarCb(_wrpObj) {
	// --- update wrapper name ---
	$.cookie('wrapper_name', _wrpObj.name, {
		expires: 1,
		path: '/wrapper_dev.html'
	});

	// --- update wrapper name ---
	document.getElementById('sb-wrapper-name').innerHTML = _wrpObj.name;
	document.getElementById('sb-wrapper-type').innerHTML = _wrpObj.type;

	// --- get wrapper content(services) from back-end. ---
	var $serviceDeck = $('#service-deck');
	$serviceDeck.empty();
	var node;
	_wrpObj.services.forEach(srv => {
		node = createServiceCardView(srv);
		$serviceDeck.append(node);
	});
}

function createServiceCardView(_serviceName) {
	// --- get service card template ---
	const template = document.querySelector('#wrapper-service-card');
	const node = document.importNode(template.content, true);

	// --- update the information ---
	var serviceName = node.querySelector(".service-name");
	serviceName.value = _serviceName;

	var serviceEdit = node.querySelector(".service-edit");
	serviceEdit.addEventListener('click', () => {
		console.log(_serviceName);
		$.cookie('wrapper_service', _serviceName, {
			expires: 1,
			path: '/wrapper_dev.html'
		});
		window.location.href = "wrapper_dev.html"
	});

	return node;
}

async function removeWrapperCb(_wrapperName) {
	if (confirm("Remove the Wrapper?")) {
		console.log('press ok')
		// --- Delete the wrapper by RESTful ---
		await cliDeleteWrapper(_wrapperName);

		//  --- remove agent card from the deck ---
		// $(this).CardWidget('remove');
		$(this).remove();

		// --- update the data from back-end ---
		var wrapperList = await cliListWrappers();
		wrapperCache_ = wrapperList.output;

		// --- update the select DOM elements of artifacts ---
		$("#artifacts-deck").find("select").each(function () {
			var selWrapper = $(this).find("option:selected").text();

			$(this).empty();
			for (var key in wrapperCache_) {
				var opt = document.createElement("option");
				opt.text = wrapperCache_[key].name;

				if (opt.text === selWrapper) {
					opt.selected = 'selected';
				}
				$(this).append(opt);
			}
		});

	} else {
		console.log('press cancel')
	}
}

async function createWrapper() {
	var wrapperName = $("#wrapper-name").val();
	if (wrapperName === "") {
		alert('wrapper name is necessary!');
		return;
	}
	// console.log(wrp_name)
	var sel = document.getElementById('sel-wrapper-type');
	var wrapperType = sel.options[sel.selectedIndex].value;
	// console.log(`selected wrapper type: ${wrapperType}`);

	// --- create a wrapper by RESTful ---
	await cliCreateWrapper(wrapperName, wrapperType);

	// --- update wrappers on view ---
	var wrapperList = await cliListWrappers();
	wrapperCache_ = wrapperList.output;
	loadWrappers(wrapperCache_)

	// --- update the select DOM elements of artifacts ---
	$("#artifacts-deck").find("select").each(function () {
		var selWrapper = $(this).find("option:selected").text();

		$(this).empty();
		for (var key in wrapperCache_) {
			var opt = document.createElement("option");
			opt.text = wrapperCache_[key].name;

			if (opt.text === selWrapper) {
				opt.selected = 'selected';
			}
			$(this).append(opt);
		}
	});

	$("#wrapper-modal").modal('hide');
}


// ============================
//     Widgets Manipulation 
// ============================
$('#hide-sidebar').on('click', function () {
	$('.control-sidebar').ControlSidebar('toggle');
});


// ============================
//     CLI Tool Requests 
// ============================
async function cliListWrapperTypes() {
	var command = { "cmd": "type", "subcmd": "list", "args": "" };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// --- data adaptor ---
	retMsg = { "status": "success", "output": retMsg };
	// console.log(retMsg);
	return retMsg;
}

async function cliListWrappers() {
	var command = { "cmd": "wrapper", "subcmd": "list", "args": "" };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// --- data adaptor ---
	retMsg = { "status": "success", "output": retMsg };
	// console.log(retMsg);
	return retMsg;
}

async function cliCreateWrapper(_wrapperName, _wrapperType) {
	var command = { "cmd": "wrapper", "subcmd": "create", "args": `${_wrapperName} ${_wrapperType}` };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// console.log(retMsg);
	return retMsg;
}

async function cliDeleteWrapper(_wrapperName) {
	var command = { "cmd": "wrapper", "subcmd": "delete", "args": `${_wrapperName}` };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// console.log(retMsg);
	return retMsg;
}

async function cliListArtifacts() {
	var command = { "cmd": "artifact", "subcmd": "list", "args": "" };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// --- data adaptor ---
	retMsg = { "status": "success", "output": retMsg };
	// console.log(retMsg);
	return retMsg;
}

async function cliDeployWrapperOnArtifact(_deviceId, _wrapperName) {
	var command = { "cmd": "artifact", "subcmd": "deploy", "args": `${_deviceId} ${_wrapperName}` };
	var retMsg = await restPostCli(command);
	retMsg = JSON.parse(retMsg);
	// console.log(retMsg);
	return retMsg;
}

// ============================
//     RESTful APIs 
// ============================
function restPostCli(_cmd) {
	return $.ajax({
		url: '/wrapper/sdk/cli',
		type: 'POST',
		data: _cmd,
		success: function (result) {
			console.log("--- POST CLI Command ---");
		}
	});
}