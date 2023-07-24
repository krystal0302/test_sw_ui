// =======================================
//     Connected-Cells Event Callbacks    
// =======================================

// --- WRAP-UP edited connections ---
async function btnSaveConnCells() {
	// 1. get all the connections
	// 2. convert data into object 
	const connCellCards = document.querySelectorAll('#cc-connections > .conn-card');

	let connDataObj = {};
	connCellCards.forEach((conn) => {
		// --- connected cell tag ---
		const name = conn.querySelector('.conn-name').textContent;

		// --- connected cell values ---
		const selNode = conn.querySelector('select');
		const selVal2 = $(selNode).find(':selected');

		const connections = [];
		for (let opt of selVal2) {
			let c = $(opt).data();
			c = c.connCell.split(',');
			connections.push({ "map_name": c[0].trim(), "transition_cell": c[1].trim() });
		};

		connDataObj[name] = connections;
	});


	// --- send to the server ---
	const res = await fetchPutCellRelation(rmtToken_, connDataObj);
	if (res.ok) {
		notificationMsg(1, 'Save Cell Connections on Success!');
	} else {
		notificationMsg(3, 'Save Cell Connections on Failure!');
	}
}

// --- CREATE a new connection ---
function btnCreateCellConn() {
	$('#name-cell-conn').modal('show');
}

$('#confirm-conn-name').on('click', function () {
	let newConnName = $('#new-conn-name').val();

	// --- get all the cells data from local storage ---
	let fullConnData = getSavedCellRelation();
	fullConnData = JSON.parse(fullConnData);

	// --- invalid input check ---
	if (newConnName == '') {
		$('#new-conn-name').val('');
		alert('The Connection name should not be empty!');
		return;
	}

	// --- duplication check ---
	let isDup = testDupConnName(newConnName);
	if (isDup) {
		$('#new-conn-name').val('');
		alert('The Connection name is duplcated!');
		return;
	}

	genCellConnectionDom(newConnName, fullConnData, '');
	// --- reset procedure ---
	$('#name-cell-conn').modal('hide');
});

function cbEditCellConn() {
	this.toggleAttribute('disabled');
	let iconNode = $(this).parent().parent().find('.edit-cell-conn > i');
	iconNode.toggleClass('fas fa-pen');
	iconNode.toggleClass('fas fa-eye');
}

function cbDeleteCellConn() {
	this.remove();
}

function genCellConnectionDom(_connName, _fullConnData, _selOptions) {
	let template = document.querySelector('#tmpl-cell-conn');

	const node = document.importNode(template.content, true);
	let mainCard = node.querySelector('.conn-card');
	let nameNode = node.querySelector('.conn-name');

	nameNode.textContent = _connName;

	let selNode = node.querySelector('.conn-combination');
	$(selNode).attr('id', _connName);

	// --- add data into drop-down menu with format ---
	let dataCells = [];
	_fullConnData.forEach((fcd) => {
		const elConnCell = {
			id: fcd.cell_id,
			text: fcd.display_name,
			parent: fcd.map
		};

		let mapObj = dataCells.find((dc) => dc.text === fcd.map);
		//  --- case: exist ---
		if (mapObj) {
			mapObj.children.push(elConnCell);
			return;
		}
		//  --- case: not-exist ---
		dataCells.push({ id: fcd.map, text: fcd.map, children: [elConnCell] });
	});

	function formatState(data, container) {
		opt = $(data.element);
		og = opt.closest('optgroup').attr('label');
		$(data.element).attr('data-conn-cell', og + ', ' + data.id)
		return og + ', ' + data.text;
	};

	$(selNode).select2({
		placeholder: 'Select Connected Cells',
		width: "100%",
		multiple: true,
		templateSelection: formatState,
		data: dataCells,
	}).val(_selOptions).trigger('change');

	// TODO: refactor the mechanism
	$(selNode).on('change', function (e) {
		const selId = e.target.id;
		const selVal2 = $(`#${selId}`).find(':selected');
		let connections = [];
		for (let opt of selVal2) {
			let c = $(opt).data();
			c = c.connCell.split(',');
			connections.push({ "map_name": c[0].trim(), "transition_cell": c[1].trim() });
		};
	});

	$(selNode).on('select2:select', function (e) {
		const selId = e.target.id;
		const selConnCellId = $(this).val();
		const selCells = selConnCellId.map((sc) => (sc.includes('-')) ? sc.split('-')[1] : sc);
		const validResult = testDupConnCell(selId, selCells);

		if (validResult.isValid) {
			alert(`The CELL ${e.params.data.text} is used by other connections!!`);
			$(selNode).val(selConnCellId).trigger('change');
		}
	})

	// --- event bindings ---
	const editCellConn = node.querySelector('.edit-cell-conn');
	editCellConn.addEventListener('click', cbEditCellConn.bind(selNode));

	const delCellConn = node.querySelector('.delete-cell-conn');
	delCellConn.addEventListener('click', cbDeleteCellConn.bind(mainCard));

	$('#cc-connections').append(node);
}


// ======== Connected Cells ========
// --- LOAD existing connections ---
async function loadConnCells() {
	// --- swarm_core-wise ---
	const res = await fetchGetAllMaps(rmtToken_);
	const mapObj = await res.json();
	const allMaps = Object.keys(mapObj)

	// --- fetch all the options ---
	const asyncGetFleetCells = async (mapArray) => {
		let result = [];
		try {
			for (const m of mapArray) {
				let res = await fetchGetMapCells(rmtToken_, m);
				res = await res.json();
				res = (typeof res === 'string') ? JSON.parse(res) : res;
				for (let a of Object.keys(res)) {
					res[a].forEach(c => {
						if (c.cell_coordinate.length < 3 || c?.cell_coordinate[2] === null) { return; }
						result.push({ map: m, area: a, cell_id: c.cell_id, display_name: c.display_name });
					});
				}
				await sleep(1000); // sleep 1s
			}
		} catch (e) { }
		return result;
	};

	let fullConnData = await asyncGetFleetCells(allMaps);

	setLocalStorageByKey('connCells', JSON.stringify(fullConnData));

	// --- fetch all the connections ---
	// ------ get the connected cells history from API ------
	const apiCellConn = await fetchGetCellRelation(rmtToken_);
	console.log(apiCellConn);

	// --- hide overlay ---
	$('#preload-overlay').hide();

	// --- render the history with option ---
	$('#cc-connections').empty();

	if (typeof apiCellConn !== 'object') { return; }
	for (let connName of Object.keys(apiCellConn)) {
		const historyConn = apiCellConn[connName];
		const historyConnCells = historyConn.map((c) => c.transition_cell);

		// --- start to fill the options ---
		genCellConnectionDom(connName, fullConnData, historyConnCells);
	}
}

function testDupConnCell(_selId, _arrCells) {
	if (!_arrCells.length) { return; }
	// 1. get all cells
	const connCellCards = document.querySelectorAll('#cc-connections > .conn-card');

	let connDataObj = [];
	connCellCards.forEach((conn) => {
		// --- connected cell tag ---
		let name = conn.querySelector('.conn-name').textContent;
		if (name === _selId) { return; }

		// --- connected cell values ---
		let selNode = conn.querySelector('select');
		let selVal2 = $(selNode).val();
		connDataObj.push(...selVal2);
	});

	let res = connDataObj.filter((cdo) => _arrCells.includes(cdo));
	let data = _arrCells.filter((ac) => !connDataObj.includes(ac));
	return (res.length) ? { isValid: true, data: data } : { isValid: false, data: data };
}

function testDupConnName(_newConnName) {
	const connCellCards = document.querySelectorAll('#cc-connections > .conn-card');

	let arrConnName = [];
	connCellCards.forEach((conn) => {
		// --- connected cell tag ---
		let name = conn.querySelector('.conn-name').textContent;
		arrConnName.push(name);
	});

	return arrConnName.includes(_newConnName) ? true : false;
}


