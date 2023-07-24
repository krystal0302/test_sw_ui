// ==============================
//     Input Validation 
// ==============================
// --- add input values validator ---
function validateVertexInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const vertex_validatorManager = new ValidatorMananger();

  const nodeLabelRule = new Rule('node-label', nodeLabelValidation);
  const nodeXRule = new Rule('node-x', numberValueValidation);
  const nodeYRule = new Rule('node-y', numberValueValidation);

  vertex_validatorManager.addValidator(nodeLabelRule);
  vertex_validatorManager.addValidator(nodeXRule);
  vertex_validatorManager.addValidator(nodeYRule);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {
    const validator = vm.getValidator(this.id);
    if (validator == undefined) { return; }

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---
    if (res.bValid) {
      $(this).css('box-shadow', '');
      $(this).css('border-color', "");
      $(this).css('outline-color', "");
    } else {
      $(this).css('box-shadow', '0 0 10px #CC0000');
      $(this).css('border-color', "red");
      $(this).css('outline-color', "red");
    }

    $(this).data('powertip', res.strMsg).powerTip();
    $.powerTip.show($(this));

    // --- [UX] Interaction Logics ---
    $('#node-createButton').prop('disabled', !res.bValid);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('node-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, vertex_validatorManager));
  }
}

// --- add input values validator ---
function validateCellInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();

  const cellLabelRule = new Rule('cell-label', textNameValidation);
  const cellAreaRule = new Rule('cell-area', textNameValidation);
  const cellXRule = new Rule('cell-x', numberValueValidation);
  const cellYRule = new Rule('cell-y', numberValueValidation);
  const markerOffsetRule = new Rule('marker-offset', markerOffsetValidation);
  const cellSizeRule = new Rule('cell-size', cellSizeValidation);

  validatorManager.addValidator(cellLabelRule);
  validatorManager.addValidator(cellAreaRule);
  validatorManager.addValidator(cellXRule);
  validatorManager.addValidator(cellYRule);
  validatorManager.addValidator(markerOffsetRule);
  validatorManager.addValidator(cellSizeRule);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {
    const validator = vm.getValidator(this.id);
    if (validator == undefined) { return; }

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---
    if (res.bValid) {
      $(this).css('box-shadow', '');
      $(this).css('border-color', "");
      $(this).css('outline-color', "");
    } else {
      $(this).css('box-shadow', '0 0 10px #CC0000');
      $(this).css('border-color', "red");
      $(this).css('outline-color', "red");
    }

    $(this).data('powertip', res.strMsg).powerTip();
    $.powerTip.show($(this));

    // --- [UX] Interaction Logics ---
    $('#cell-createButton').prop('disabled', !res.bValid);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('cell-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

// ====== customized page-related validation rules ======
function markerOffsetValidation(inputVal) {
  const isThreeParams = (inputVal.split(',').length === 3);
  const bRes = isThreeParams;
  let strMsg = "";
  strMsg += (isThreeParams) ? strMsg : '- the number of delimiter(,) SHOULD BE 2';

  return { bValid: bRes, strMsg: strMsg }
}

function cellSizeValidation(inputVal) {
  const isTwoParams = (inputVal.split(',').length === 2);
  const bRes = isTwoParams;
  let strMsg = "";
  strMsg += (isTwoParams) ? strMsg : '- the number of delimiter(,) SHOULD BE 1';

  return { bValid: bRes, strMsg: strMsg }
}

function nodeLabelValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$]+$/;                   // forbidden characters \ / : * ? " < > |
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names
  const reg4 = /\s/;                                      // space
  const reg5 = /^\d[0-9a-zA-Z]*$/;                        // start with number

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && !reg5.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (!reg5.test(inputVal)) ? '' : '- cannot start with number';
  strMsg.push(msg5);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// --- add input values validator ---
function validateZoneInputEvent(inputNode) {
  function debounce(func, delay = 200) {
    let timer = null;

    return () => {
      let context = this;
      let args = arguments;

      clearTimeout(timer);
      timer = setTimeout(() => {
        func.apply(context, args);
      }, delay)
    }
  }

  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#save-changes');

  const zonenameRule = new Rule('zone-name', textNameValidation, interaction_element);

  validatorManager.addValidator(zonenameRule);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {

    let validator;
    if (this.id === 'zone-name') {
      validator = vm.getValidator(this.id);
    } else {
      validator = new Rule(this.id, zoneConfigValidation, interaction_element);
    }

    if (validator == undefined) { return; }

    validator.run(this.value);
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('zone-input');
  for (let el of targets) {
    el.addEventListener("input", validationFlow.bind(el, validatorManager));
  }

  if (inputNode.id === 'zone-name') return;
  if (!inputNode.classList.contains("input-handler")) {
    inputNode.classList.add("input-handler");
    inputNode.addEventListener("input", debounce(validationFlow.bind(inputNode)));
  }
}

function zoneConfigValidation(inputVal) {
  // console.log(inputVal);
  let bRes = true;
  let strMsg = "";
  let configs = zoneDM.getConfigs();
  let type = configs[this._name].valid_value.data_type;
  let range = configs[this._name].valid_value.data_range.replace(/[[\]]/g, '');

  switch (type) {
    case 'double': {
      var minVal = parseFloat(range.split(":")[0]);
      var maxVal = parseFloat(range.split(":")[1]);
      var doubleVal = parseFloat(inputVal);
      if (!$.isNumeric(doubleVal)) {
        strMsg = "value type should be " + type;
      }
      if (doubleVal > maxVal || doubleVal < minVal) {
        strMsg = "value must be in the range of " + range.replace(':', '~');
      }
      bRes = $.isNumeric(doubleVal) && doubleVal <= maxVal && doubleVal >= minVal;
      break;
    }
    case 'int': {
      var minVal = parseInt(range.split(":")[0]);
      var maxVal = parseInt(range.split(":")[1]);
      var intVal = parseInt(inputVal);
      if (!Number.isInteger(Number(inputVal))) {
        strMsg = "value type should be " + type;
      }
      if (intVal > maxVal || intVal < minVal) {
        strMsg = "value must be in the range of " + range.replace(':', '~');
      }
      bRes = Number.isInteger(Number(inputVal)) && intVal <= maxVal && intVal >= minVal;
      break;
    }
  }
  return { bValid: bRes, strMsg: strMsg }
}

// --- add input values validator ---
function validateRenameInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#rename_confirm');

  const mapnameRule = new Rule('new-mapname', reNameMapValidation, interaction_element);

  validatorManager.addValidator(mapnameRule);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {
    const validator = vm.getValidator(this.id);
    if (validator == undefined) { return; }

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---

    // --- [UX] Interaction Logics ---
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('rename-map-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

function reNameMapValidation(inputVal) {
  const mapName = document.getElementById('map-select').text;

  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  const mapNameNoChange = (mapName === inputVal ? true : false);
  let mapAliasExists = false;
  if (inputVal == "" || mapNameNoChange) {
    // pass
  } else {
    let data = restMapAliasExistence(inputVal);
    if (data == null) {
      mapAliasExists = false;
    } else {
      if (mapAliasExists === undefined) {
        mapAliasExists = false;
      } else {
        mapAliasExists = data.responseJSON;
      }
    }
  }

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && !mapNameNoChange && !mapAliasExists;
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $ + - = ~ ` # %';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (reg5.test(inputVal)) ? '' : '- must end with number or A-Za-z';
  strMsg.push(msg5);
  if (mapNameNoChange) {
    const msg6 = (!mapNameNoChange) ? '' : '- map name no change';
    strMsg.push(msg6);
  } else {
    const msg6 = (!mapAliasExists) ? '' : '- map name exist';
    strMsg.push(msg6);
  }

  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// ------ generic validation rules ------
function cellNameValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  let cell_name_duplicate = false;

  // === gather all cell id === 
  const cellDM = CellDataManager.getInstance(visNetwork_);
  const cellAllData = cellDM.getAllData();
  let cellIdArr = cellAllData.map((c) => c.cell_id);

  if (editCellMode === 'add' && cellIdArr.includes(inputVal)) {
    cell_name_duplicate = true;
  }
  if (editCellMode === 'edit' && editCellData.labelFullName !== inputVal && cellIdArr.includes(inputVal)) {
    cell_name_duplicate = true;
  }

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && !cell_name_duplicate;
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $ + - = ~ ` # %';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (reg5.test(inputVal)) ? '' : '- must end with number or A-Za-z';
  strMsg.push(msg5);
  const msg6 = (!cell_name_duplicate) ? '' : '- duplicate cell name';
  strMsg.push(msg6);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

// --- add FunctionType input values validator ---
function validateFunctionTypeInputEvent(type) {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#sbft-add-btn');

  if (type === 'rack') {
    const functiontypenameRule = new Rule('sbft-cell-name', textNameValidation, interaction_element);
    const sizeRule = new Rule('sbft-size', sizeValueValidation, interaction_element);
    const payloadRule = new Rule('sbft-payload', payloadValueValidation, interaction_element);

    validatorManager.addValidator(functiontypenameRule);
    validatorManager.addValidator(sizeRule);
    validatorManager.addValidator(payloadRule);
  }
  if (type === 'charger') {
    const functiontypenameRule = new Rule('sbft-cell-name', textNameValidation, interaction_element);
    const offsetRule = new Rule('sbft-offset', offsetValueValidation, interaction_element);

    validatorManager.addValidator(functiontypenameRule);
    validatorManager.addValidator(offsetRule);
  }

  /*
  // --- [CONFIG] 2. Conduct Event-Bindings ---
  //                  * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('function-type-input');
  for (let el of targets) {
    el.addEventListener("input", validationFlowFunctionType.bind(el, validatorManager));
  }
  // return validatorManager;
}

function validateFunctionTypeInputEvent2(validateObj) {
  console.log(validateObj);
  const uuid = validateObj.uuid;
  console.log(uuid)
  const type = validateObj.type;
  console.log(type);
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#sbft-add-btn');

  if (type === 'rack') {
    const functiontypenameRule = new Rule('sbft-cell-name', textNameValidation, interaction_element);
    const sizeRule = new Rule('sbft-size-' + uuid, sizeValueValidation, interaction_element);
    const payloadRule = new Rule('sbft-payload-' + uuid, payloadValueValidation, interaction_element);

    validatorManager.addValidator(functiontypenameRule);
    validatorManager.addValidator(sizeRule);
    validatorManager.addValidator(payloadRule);
  }
  if (type === 'charger') {
    const functiontypenameRule = new Rule('sbft-cell-name', textNameValidation, interaction_element);
    const offsetRule = new Rule('sbft-offset-' + uuid, offsetValueValidation, interaction_element);

    validatorManager.addValidator(functiontypenameRule);
    validatorManager.addValidator(offsetRule);
  }

  /*
  // --- [CONFIG] 2. Conduct Event-Bindings ---
  //                  * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('function-type-input-' + uuid);
  for (let el of targets) {
    el.addEventListener("input", validationFlowFunctionType.bind(el, validatorManager));
  }
}

function validationFlowFunctionType(vm) {
  const validator = vm.getValidator(this.id);
  if (validator == undefined) { return; }
  validator.run(this.value, false);

  if (!validator._interaction_element) { return; }
  vm.makeNonClickableIfOneOfInputsFailed(validator._interaction_element);
  // --- [STYLING] reflect validation result ---
  // --- [UX] Interaction Logics ---
}

function sizeValueValidation(inputVal) {
  // --- dimensions should not be negative and followed by form "width,length,height" ---
  const reg1 = /^((0|[1-9]\d*)(\.\d+)?)\,((0|[1-9]\d*)(\.\d+)?)\,((0|[1-9]\d*)(\.\d+)?)$/;
  const bRes = reg1.test(inputVal);

  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid number or includes invalid charaters or invalid format';
  strMsg.push(msg1);

  return { bValid: bRes, strMsg: strMsg }
}

function offsetValueValidation(inputVal) {
  const reg1 = /^(-?(0|[1-9]\d*)(\.\d+)?)\,(-?(0|[1-9]\d*)(\.\d+)?)\,(-?(0|[1-9]\d*)(\.\d+)?)$/;
  const bRes = reg1.test(inputVal);

  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid number or includes invalid charaters or invalid format';
  strMsg.push(msg1);

  return { bValid: bRes, strMsg: strMsg }
}

function payloadValueValidation(inputVal) {
  const reg1 = /^[0-9.]+$/;                // (+|-)0-9
  const bRes = reg1.test(inputVal);
  let strMsg = "";
  strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters';

  return { bValid: bRes, strMsg: strMsg }
}