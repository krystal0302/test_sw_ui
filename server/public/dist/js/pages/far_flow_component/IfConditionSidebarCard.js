const handleShowIfConditionSideBarCard = (conditionName, conditionIdx) => {
	const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
	const elementID = translateToNodeElementID(drawFlowNodeID);
    const conditionNameStr = `${conditionName}`;

    const IfConditionSideBarCardTmplate = document.getElementById('IfConditionSidebarCard');
    let node = document.importNode(IfConditionSideBarCardTmplate.content, true);
    let span = node.querySelector('.conditionIndex');
    let collapesBtn = node.querySelector('#conditionCollapseBtn');
    let collapesAnchor = node.querySelector('#conditionCollapseAnchor');
    let collapesContent = node.querySelector('#conditionCollapseContent');
    let deleteBtn = node.querySelector('.conditionDeleteBtn');

    if (conditionIdx === 0) {
        deleteBtn.style.display = 'none';
        deleteBtn.disabled = true;
    }

    deleteBtn.dataset.nodeId = drawFlowNodeID;

    span.textContent = '';

    collapesBtn.id = `conditionBtn_${conditionIdx}`;
    collapesBtn.textContent = conditionNameStr;
    collapesBtn.dataset.target = `#condition_${conditionIdx}`;

    collapesAnchor.id = `conditionAnchor_${conditionIdx}`;
    collapesAnchor.href = `#condition_${conditionIdx}`;
    collapesAnchor.dataset.target = `#condition_${conditionIdx}`;

    // for hidden edit condition name
    let editConditionInput = node.querySelector('.editConditionNameInput');
    editConditionInput.value = conditionNameStr;
    let editConditionAppendSpan = node.querySelector('.editConditionNameAppendSpan');
    editConditionAppendSpan.textContent = `${conditionNameStr.length}/25`;

    collapesContent.id = `condition_${conditionIdx}`;
    // accordion type collapse
    // collapesContent.dataset.parent = `#${sidebarID}`;

    let contentElement = undefined;
    // const dataIdx = behaviorIndex - 1;
    const haveSavedData = haveSaveCachePageData(drawFlowNodeID);
    console.log(haveSavedData, drawFlowNodeID, flowPageData.currentSavedData[elementID])
    const conditionSavedVal = haveSavedData? flowPageData.currentSavedData[elementID].conditions[conditionNameStr]: undefined;

    // load condition content
    const conditionContentTmplate = document.getElementById('IfConditionContent');
    let conditionContent = document.importNode(conditionContentTmplate.content, true);

    // load condtion dropdown
    const conditionDropdownItemTmplate = document.getElementById('conditionDropdownItem');
    let dropDownItem = document.importNode(conditionDropdownItemTmplate.content, true);

    // change codemirror text area
    const newTextAreaID = `code_${conditionIdx}`;
    let targetAreaElement = conditionContent.getElementById("code");
    targetAreaElement.id = newTextAreaID;

    let editor = CodeMirror.fromTextArea(conditionContent.getElementById(newTextAreaID), {
        mode: "javascript",
        lineNumbers: true,
        theme: "monokai",
        lineWrapping: true,
        elementID: elementID,
        conditionIndex: conditionIdx,
    });

    editor.on("change", function() {
        editor.refresh();
    });

    editor.on("changes", function() {
        const currentVal = editor.getValue();
        const elementID = editor.options.elementID;
        const conditionIdx = editor.options.conditionIndex;

        if (currentVal.trim().length === 0){
            console.log('Is empty input')
            setDrawFlowNodeConditionEditStatus(elementID, conditionIdx, false);
        } else {
            console.log('Is ok input')
            setDrawFlowNodeConditionEditStatus(elementID, conditionIdx, true);
        }
    });

    editor.on('codeMirrorFocus', (data) => {
        editor.focus();
        editor.setCursor(editor.lineCount(), 0);
        editor.refresh();
    })

    if (haveSavedData && conditionSavedVal != undefined) {
        editor.getDoc().setValue(conditionSavedVal);
    }

    // condition-dropdown-groups
    let targetDropdownGroups = conditionContent.querySelector('.api-conditions-menu');

    for (const [condition, conditionValue] of Object.entries(flowPageData.currentFleetSetting.conditions)) {
        console.log(`${condition}: `, conditionValue);
        let tmpDropDownItem = dropDownItem.cloneNode(true).querySelector('.condition-item');

        const optionDisplyText = conditionValue.option_text;
        const optionValue = conditionValue.option_value

        tmpDropDownItem.dataset.condtionData = JSON.stringify(optionValue);
        tmpDropDownItem.textContent = optionDisplyText;
        targetDropdownGroups.appendChild(tmpDropDownItem);
    }

    contentElement = conditionContent;

    if (contentElement !== undefined) {
        let cardBody = document.createElement('div');
        cardBody.classList = 'card-body';
        cardBody.appendChild(contentElement);
        const replaceTarget = node.querySelector('.card-body');
        collapesContent.replaceChild(cardBody, replaceTarget);
    }

    return node;
}

$(document).on('click', '.condition-item', function (e) {
    let currentConditionItem = this;
    let insertVal = currentConditionItem.dataset.condtionData;
    let parentNode = currentConditionItem.closest('.card-body');

    let targetCodeMirrorEditor = parentNode.querySelector('.condition-expression .CodeMirror').CodeMirror;

    insertTextToCodeMirror(targetCodeMirrorEditor, insertVal);
})

const insertTextToCodeMirror = (codeMirrorEditor, insertText) => {
    let doc = codeMirrorEditor.getDoc();
    let cursor = doc.getCursor();
    let line = doc.getLine(cursor.line);

    let pos = {
		line: cursor.line,
        ch: cursor.ch
	};

    let spaceAtFront = codeMirrorEditor.getValue().slice(-1) === " " ? "": " ";

	if (line.length === 0) {
		// check if the line is empty
		// add the data
		doc.replaceRange(insertText, pos);
	} else {
		// add a new line and the data
		doc.replaceRange(`${spaceAtFront}${insertText} `, pos);
	}
}

$(document).on('click', '.dropdown-submenu .operator-item' , function (e) {
    let allSubmenu = this.closest('.dropdown-menu').querySelectorAll('dropdown-menu');
    console.log(allSubmenu)
    $(this).next('div').toggle();
    e.stopPropagation();
    e.preventDefault();
});

$(document).on('click', '.operator' , function (e) {
    $(this).closest('.dropdown-menu').toggle();

    // clear first dropdown
    let parentDropDown = this.closest('.dropdown');
    parentDropDown.classList.remove('show');

    let btnDropdownToggle = parentDropDown.querySelector('.dropdown-toggle');
    btnDropdownToggle.setAttribute('aria-expanded', false);

    let mainDropDownMenu = parentDropDown.querySelector('.dropdown-menu');
    $(mainDropDownMenu).toggle();
    mainDropDownMenu.classList.remove('show');
    mainDropDownMenu.setAttribute('style', '');

    let parentCardBody = this.closest('.card-body');
    let targetCodeMirrorEditor = parentCardBody.querySelector('.condition-expression .CodeMirror').CodeMirror;
    let insertVal = this.dataset.operatorData;

    insertTextToCodeMirror(targetCodeMirrorEditor, insertVal)
});

$(document).on('click', '.conditionCollapseBtn', function (e) {
	let currentNodeType = getCurrentSideBarSelectNode().nodType;
	if (currentNodeType === 'if-condition') {
		const parentNode = this.closest('.condition-card')
		let targetCodeMirrorEditor = parentNode.querySelector('.condition-expression .CodeMirror').CodeMirror;
		CodeMirror.signal(targetCodeMirrorEditor, "codeMirrorFocus", "focus event");
	}
})

$(document).on('click', '.conditionCollapseAnchor', function (e) {
	let currentNodeType = getCurrentSideBarSelectNode().nodType;
	if (currentNodeType === 'if-condition') {
		const parentNode = this.closest('.condition-card')
		let targetCodeMirrorEditor = parentNode.querySelector('.condition-expression .CodeMirror').CodeMirror;
		CodeMirror.signal(targetCodeMirrorEditor, "codeMirrorFocus", "focus event");
	}
})

$(document).on('click', '.editConditionNameBtn', function (e) {
	let paraentCardHeader = this.closest('.card-header');
    let conditionDisplayDiv = paraentCardHeader.querySelector('.conditionDisplay');
    let conditionEditDiv = paraentCardHeader.querySelector('.conditionEdit');

    toggleDisplay(conditionDisplayDiv);
    toggleDisplay(conditionEditDiv);
})

$(document).on('click', '.saveEditConditionNameBtn', function (e) {
	let paraentCardHeader = this.closest('.card-header');
    let conditionDisplayDiv = paraentCardHeader.querySelector('.conditionDisplay');
    let conditionEditDiv = paraentCardHeader.querySelector('.conditionEdit');
    let editInput = conditionEditDiv.querySelector('.editConditionNameInput');
    let conditionCollapseBtn = conditionDisplayDiv.querySelector('.conditionCollapseBtn');

    let editConditionVal = editInput.value;
    const conditionIdx = parseInt(conditionCollapseBtn.id.replace('conditionBtn_', ''));

    console.log(editConditionVal, conditionIdx)

    if (editConditionVal.trim().length === 0) {
        // empty text
        editInput.style.border = "2px solid red";
		editInput.setAttribute("title", "Please Enter Condition name");
    } else if (editConditionVal.length > 25) {
        // empty text
        editInput.style.border = "2px solid red";
		editInput.setAttribute("title", "Please Enter Condition name length less than 25");
    } else {
        // save change to collapse btn
        conditionCollapseBtn.textContent = editConditionVal;

        // save change to drawflow
        const drawFlowNodeID = getCurrentSideBarSelectNode().node_id;
        const elementID = translateToNodeElementID(drawFlowNodeID);

        // Update Drawwflow Content
        let targetIfConditonNode = document.getElementById(elementID);
        let targetIfConditonInput = targetIfConditonNode.querySelectorAll('.condition-input')[conditionIdx];
        targetIfConditonInput.setAttribute('value', editConditionVal);

        // Update Drawwflow Content Data
	    const tmpCpoy = targetIfConditonNode.cloneNode(true);
        let tmpNodePlaceholder = document.createElement("div");
        tmpNodePlaceholder.appendChild(tmpCpoy.querySelector('.drawflow_content_node'));

        editor.drawflow.drawflow.Home.data[drawFlowNodeID].html = tmpNodePlaceholder.firstChild.innerHTML;

		editInput.style.border = "none";
		editInput.removeAttribute("title");
        toggleDisplay(conditionDisplayDiv);
        toggleDisplay(conditionEditDiv);
    }
})

$(document).on("keyup", ".editConditionNameInput", (inputE) => {
	let inputConditionName = inputE.currentTarget;
    let parantInputGroup = inputConditionName.closest('.input-group');
	let inpuVal = inputConditionName.value;
	// const numberRegex = /^\d+$/;
	// let isOkPercentage = numberRegex.test(inpuVal)

    let editConditionAppendSpan = parantInputGroup.querySelector('.editConditionNameAppendSpan');
    editConditionAppendSpan.textContent = `${inpuVal.length}/25`;

	if (inpuVal.trim().length === 0) {
		parantInputGroup.style.border = "2px solid red";
		inputConditionName.setAttribute("title", "Please Enter Condition name");
	} else if (inpuVal.length > 25) {
        // empty text
        parantInputGroup.style.border = "2px solid red";
		inputConditionName.setAttribute("title", "Please Enter Condition name length less than 25");
    } else {
        parantInputGroup.style.border = "none";
		inputConditionName.removeAttribute("title");
	}
})

const toggleDisplay = (htmlElement) => {
    htmlElement.style.display = htmlElement.style.display === 'none' ? '' : 'none';
}