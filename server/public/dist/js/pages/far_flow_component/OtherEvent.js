// back btn
const homeUri = '/operation.html';
$(document).on('click', '#flow-back', function (e) {
  if (flowPageData.haveUnSaveChange) {
    var yes = confirm('Are you sure you want to leave page? you change may not save.');

    if (yes) {
      window.location.href = homeUri;
    }
  } else {
    window.location.href = homeUri;
  }
});

// Edit flow name
let cachedFleetname_;
$('#edit-flowname-switch').on('click', async function () {
  var target = this.querySelector('.fa');

  if (target.classList.contains('fa-pen')) {
    target.setAttribute("class", "fa fa-eye");
    var targetNode = document.getElementsByClassName('editing-flowname');
    // console.log($(targetNode).text());
    var inputNode = document.createElement('input');
    inputNode.setAttribute("id", 'editing-flowname-input');
    inputNode.setAttribute("class", 'editing-flowname flow-input');
    inputNode.value = $(targetNode).text();

    // --- chore: cache the state ---
    cachedFleetname_ = $(targetNode).text();

    $(targetNode).replaceWith($(inputNode));
    validateFlowNameInputEvent();
    return;
  }

  if (target.classList.contains('fa-eye')) {
    var targetNode = document.getElementsByClassName('editing-flowname');
    var inputNode = document.createElement('span');
    inputNode.setAttribute("id", 'editing-flowname-span');
    inputNode.setAttribute("class", 'editing-flowname flow-input');

    // --- chore: comparison with original and modified ones ---
    var tmpName = cachedFleetname_;
    var modifiedName = $(targetNode).val();

    if (tmpName == modifiedName) {
      target.setAttribute("class", "fa fa-pen");
      $(targetNode).css("border-color", '');
      inputNode.textContent = tmpName;
      $(targetNode).replaceWith($(inputNode));
    } else {
      if (!inputCheck(modifiedName)) {
        $(targetNode).css("border-color", 'red');
        notificationMsg(2, `Flow name include invalid characters`);
      } else {

        let flowFileExist = await restOperationFileExistence("flow", cachedFleetname_);
        let duplicateres = await checkDuplicateFlowname(getSelectedFleet(), cachedFleetname_, modifiedName);
        duplicateres = JSON.parse(duplicateres);

        if (!flowFileExist.fileExists) {
          // New flow Rename
          if (duplicateres.statusCode === 200) {
            target.setAttribute("class", "fa fa-pen");
            $(targetNode).css("border-color", '');

            if (cachedFleetname_ !== modifiedName) {
              console.log('rename new the flee now!');
              tmpName = modifiedName;
              inputNode.textContent = tmpName;
              $(targetNode).replaceWith($(inputNode));
              style = 1;
              // --- update global flow name ---
              flow_name = tmpName;
              setCurrentFlowName(flow_name);
            }
          } else {
            target.setAttribute("class", "fa fa-eye");
            notificationMsg(3, duplicateres.message);
            return
          }
        } else {
          // Old flow Rename
          let rename_needed = false;

          if (duplicateres.statusCode === 200) {
            target.setAttribute("class", "fa fa-pen");
            $(targetNode).css("border-color", '');

            if (cachedFleetname_ !== modifiedName) {
              if (confirm('rename the flow?')) {
                console.log('rename the flee now!');
                tmpName = modifiedName;
                rename_needed = true;
              }
            }
          } else {
            target.setAttribute("class", "fa fa-eye");
            notificationMsg(3, duplicateres.message);
            return
          }

          if (rename_needed) {
            let res = await restRenameFlowname(getSelectedFleet(), cachedFleetname_, modifiedName);
            res = JSON.parse(res);
            if (res.statusCode === 200) {
              inputNode.textContent = tmpName;
              $(targetNode).replaceWith($(inputNode));
              style = 1;
              // --- update global flow name ---
              flow_name = tmpName;
              setCurrentFlowName(flow_name);
              // restDeleteFlowData(getSelectedFleet(), cachedFleetname_);
            } else {
              style = 3;
            }

            notificationMsg(style, res.message);
          } else {
            inputNode.textContent = tmpName;
            $(targetNode).replaceWith($(inputNode));
            notificationMsg(0, 'Rename Flow Canceled');
          }
        }
      }
    }
  }
})

// Flow trigger select
$(".trigger-select").on('click', function () {
    const currentFlowName = getCurrentFlowName();

    if (isTouchDevice()) return;
    console.log(`touch device? ${isTouchDevice()}`);
    var platform = navigator.platform;
    console.log(platform + " click trigger type");

    var show_name = "";
    if ($(this).attr("id") == "flow_select") {
      show_name = currentFlowName;
    } else {
      show_name = currentFlowName;
    }

    if ($(this).val() === 'manual') {
        resetTaskTriggerSelectOptions("flow");
        changeTriggerText("Manual", 'manual');
    } else if ($(this).val() === 'event') {
        $("#event-label").html(show_name + ' event:');
        $("#event-trigger-modal").modal('show');
    } else if ($(this).val() === 'timer') {
        $("#timer-label").html(show_name + ' Timer:');
        $("#timer-trigger-modal").modal('show');
    }

    setUnSaveChange(true);
});

$('.trigger-select').on('change', function () {
    const currentFlowName = getCurrentFlowName();

    if (!isTouchDevice()) return;
    console.log(`touch device? ${isTouchDevice()}`);
    var platform = navigator.platform;
    console.log(platform + " change trigger type");
    var show_name = "";
    if ($(this).attr("id") == "flow_select") {
        show_name = currentFlowName;
    } else {
        show_name = currentFlowName;
    }
    if ($(this).val() === 'manual') {
        resetTaskTriggerSelectOptions("flow");
        changeTriggerText("Manual", 'manual');
    } else if ($(this).val() === 'event') {
        $("#event-label").html(show_name + ' event:');
        $("#event-trigger-modal").modal('show');
    } else if ($(this).val() === 'timer') {
        $("#timer-label").html(show_name + ' Timer:');
        $("#timer-trigger-modal").modal('show');
    }
});

$("#add-event").click(function () {
    var eventID = $("#event-id").val();
    const changeText = `Event: ${eventID}`;

    resetTaskTriggerSelectOptions("flow");
    changeTriggerText(changeText, 'event');

    $('#event-trigger-modal').modal('hide');
});

$("#add-timer").click(function () {
    var needRepeat = $('#repeatCheckbox').is(':checked');
    var regex = /^([0-9]|0[0-9]|1[0-9]|2[0-3]):[0-5][0-9]$/
    if ($("#startDate").val() === "") {
        alert('Please fill in task timer start date !');
    } else if ($("#startTime").val() === "") {
        alert('Please fill in task timer start time !');
    } else if (!regex.test($("#startTime").val())) {
        alert('Task timer start time format is invalid!');
    } else if (needRepeat && $("#endDate").val() === "") {
        alert('Please fill in task timer end date !')
    } else if (needRepeat && $("#endTime").val() === "") {
        alert('Please fill in task timer end time !');
    } else if (needRepeat && !regex.test($("#endTime").val())) {
        alert('Task timer end time format is invalid!');
    } else {
        var startTimeString = $("#startTime").val();
        resetTaskTriggerSelectOptions("flow");

        get_time_trigger_event(startTimeString, $("#endTime").val(), $("#cycle-time").val(), needRepeat, $('#startDate').val(), $('#endDate').val());

        startTimeString += needRepeat ? " (Rep)" : "";

        const changeText = `Timer: ${startTimeString}`;
        changeTriggerText(changeText, 'timer');
        $('#timer-trigger-modal').modal('hide');

        var startDate = $("#startDate").val();
        timerInfoString = startDate + " " + $("#startTime").val();
        if (needRepeat) {
            var endDate = $("#endDate").val();
            timerInfoString = timerInfoString.concat(";", $("#cycle-time").val(), ";", endDate, " ", $("#endTime").val());
        }
    }
});

const resetTaskTriggerSelectOptions = () => {
    const modeName = "flow"
    $(`#${modeName}-event`).text('Event');
    $(`#${modeName}-timer`).text('Timer');
}

const changeTriggerText = (_text, _type) => {
    var modeName = "flow-";
    $(`.trigger-select option[id=${modeName}${_type}]`).text(_text);
    flowPageData.currentFlowType.type = _text;
}

function get_time_trigger_event(start, end, duration, isrepeat, startdate, enddate) {
	time_start_times = [];
	start_time = start;
	end_time = end;
	repeat_time = duration;
	start_date = startdate;
	end_date = enddate;

	console.log(' ---- get_time_trigger_event ---- ')
	console.log(start_date, start_time, end_date, end_time, repeat_time)

	saved_start_time = new Date(start_date)
	saved_end_time = new Date(end_date)
	saved_start_time.setHours(start_time.split(':')[0])
	saved_start_time.setMinutes(start_time.split(':')[1])
	saved_end_time.setHours(end_time.split(':')[0])
	saved_end_time.setMinutes(end_time.split(':')[1])

	saved_start_time_iso = moment(saved_start_time).format()
	saved_end_time_iso = moment(saved_end_time).format()

	console.log(saved_start_time, saved_end_time)
	console.log(saved_start_time_iso, saved_end_time_iso)

	flowPageData.currentFlowType.start_time = saved_start_time_iso;

	if (isrepeat) {
		var start_time_sec = start.split(':')[0] * 60 * 60 + start.split(':')[1] * 60;
		var end_time_sec = end.split(':')[0] * 60 * 60 + end.split(':')[1] * 60;
		var repeat_time_sec = duration * 60;

		start_time_sec = new Date(startdate).getTime() * 0.001 + start_time_sec;
		end_time_sec = new Date(enddate).getTime() * 0.001 + end_time_sec;

		var time_diff = (end_time_sec - start_time_sec) / repeat_time_sec;
		// console.log(time_diff)
		time_send_times = time_diff;
		for (var i = 0; i <= time_diff; i++) {
			var time_sec = start_time_sec + repeat_time_sec * i;
			time_start_times.push(time_sec);
		}

        flowPageData.currentFlowType.end_time = saved_end_time_iso;
        flowPageData.currentFlowType.interval = `PT${repeat_time}M`;
	} else {
		var start_time_sec = start.split(':')[0] * 60 * 60 + start.split(':')[1] * 60;
		time_send_times = 1;
		time_start_times.push(new Date(startdate).getTime() * 0.001 + start_time_sec);

        flowPageData.currentFlowType.end_time = saved_start_time_iso;
        flowPageData.currentFlowType.interval = ``;
	};
}

// --- add input values validator ---
function validateFlowNameInputEvent(rule_list = []) {
    /*
    // --- [CONFIG] 1. Setup Validation Configuration       ------
    //                  * create validatorManager and Rules
    //                  * add rules into validatorManager   -
    **/
    const validatorManager = new ValidatorMananger();
    const interaction_element = $('#save-flow-btn');

    const flowNameRule = new Rule('flow-name', textNameValidation, $('#add-flow'));
    const editflowNameRule = new Rule('editing-flowname-input', textNameValidation, interaction_element);
    const failuretimeoutRule = new Rule('failure_timeout', positivenumberValueValidation, interaction_element);
    const errorTypeRule = new Rule('error_type', textNameValidation, interaction_element);
    const retryLimitRule = new Rule('retry_limit', positivenumberValueValidation, interaction_element);
    const eventNameRule = new Rule('event-id', textNameValidation, $('#add-timer'));
    const startTimeRule = new Rule('startTime', startTimeValidation, $('#add-timer'));
    const endTimeRule = new Rule('endTime', endTimeValidation, $('#add-timer'));

    validatorManager.addValidator(flowNameRule);
    validatorManager.addValidator(editflowNameRule);
    validatorManager.addValidator(failuretimeoutRule);
    validatorManager.addValidator(errorTypeRule);
    validatorManager.addValidator(retryLimitRule);
    validatorManager.addValidator(eventNameRule);
    validatorManager.addValidator(startTimeRule);
    validatorManager.addValidator(endTimeRule);

    rule_list.forEach(element_rule => {
      validatorManager.addValidator(element_rule);
    });
    // console.log(role_validatorManager);

    /*
    // --- [CONFIG] 2. Define Validation Flow ---
    //                  * getValidator(): get the corresponding validator,
    //                  * run()         : run the valiations,
    //                  * do the styling and interaction logics
    **/
    function validationFlow(vm) {
      const validator = vm.getValidator(this.id);
      if (validator == undefined) { return; }

      const res = validator.run(this.value, false);

      // --- [STYLING] reflect validation result ---

      let switch_id = this.id;
      if (this.id.includes("input-name-")) {
        switch_id = 'input-name';
      }

      // --- [UX] Interaction Logics ---
      switch (switch_id) {
        case 'flow-name':
          $('#add-flow').prop('disabled', !res.bValid);
          break;
        case 'editing-flowname-input':
          $('#save-flow-btn').prop('disabled', !res.bValid);
          break;
        case 'event-id':
          $('#add-event').prop('disabled', !res.bValid);
          break;
        case 'startTime':
          if ($("#endTime").css("border-color") == 'rgb(255, 0, 0)') {
            break;
          }
          $('#add-timer').prop('disabled', !res.bValid);
          break;
        case 'endTime':
          if ($("#startTime").css("border-color") == 'rgb(255, 0, 0)') {
            break;
          }
          $('#add-timer').prop('disabled', !res.bValid);
          break;
        case 'failure_timeout':
          $('#save-flow-btn').prop('disabled', !res.bValid);
          validate_res_dict[this.id] = res.bValid;
          break;
        case 'error_type':
          $('#save-flow-btn').prop('disabled', !res.bValid);
          validate_res_dict[this.id] = res.bValid;
          break;
        case 'retry_limit':
          $('#save-flow-btn').prop('disabled', !res.bValid);
          validate_res_dict[this.id] = res.bValid;
          break;
        case 'input-name':
          $('#save-flow-btn').prop('disabled', !res.bValid);
          let geer_icon = `detailid_${this.id.replace("input-name-", "")}`;
          $(`#${geer_icon}`).prop('disabled', !res.bValid)
          break;

        default:
          $('#save-flow-btn').prop('disabled', !res.bValid);
          break;
      }

    }

    /*
    // --- [CONFIG] 3. Conduct Event-Bindings ---
    //                 * bind the validation procecedure to the target elements
    **/
    const targets = document.getElementsByClassName('flow-input');
    for (let el of targets) {
      el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
    }
}

// Start Time
function startTimeValidation(inputVal) {
    const reg1 = /(^([01]\d|2[0-3]):([0-5]\d)$)|(^24:00$)/;     // 00:00 - 24:00

    let bRes = reg1.test(inputVal);
    let strMsg = [];
    const msg1 = (reg1.test(inputVal)) ? '' : '- invalid time format, avaliable 00:00 - 24:00';
    strMsg.push(msg1);
    strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

    return { bValid: bRes, strMsg: strMsg }
  }

  // End Time
  function endTimeValidation(inputVal) {
    const reg1 = /(^([01]\d|2[0-3]):([0-5]\d)$)|(^24:00$)/;     // 00:00 - 24:00

    let bRes = reg1.test(inputVal);
    let strMsg = [];
    const msg1 = (reg1.test(inputVal)) ? '' : '- invalid time format, avaliable 00:00 - 24:00';
    strMsg.push(msg1);
    strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

    if (inputVal.length == 0) {
      bRes = true;
      strMsg = [];
    }

    return { bValid: bRes, strMsg: strMsg }
}

const init_panel = () => {
    if (!confirm('Clear the current flow draft?')) { return; }

    resetFlowPageSavedData();
    initDrawFlow();
}

const runTestFlow = () => {
    let comfirmModal = document.getElementById('runTestFlowModal');
    $('#runTestFlowModal').modal('show');
}

const trggerTestFlow = async () => {
  console.log('---- Trigger Test flow ----');
  const fleetName = flowPageData.currentFleet;
	const flowName = `${getCurrentFlowName()}_${genUuid().replace(/-/g, '')}`;
	// const flowName = getCurrentFlowName();
  console.log(`Temp flow name: ${flowName}`)

  getSaveDataForTesting(flowName).then(function () {
		sendFlowTask(flowName, false).then(function () {
      restDeleteFlowData(fleetName, flowName);
      notificationMsg(0, `tmp filw:${flowName} is deleted`);
    });
    $('#runTestFlowModal').modal('hide');
	})

  // ================ tmp for trigger test by api ================
  // let flowTriggerTemplat = {
  //   args: {
  //     start_time: '',
  //     end_time: '',
  //     interval: '',
  //     params: {
  //       // "node-name": {
  //       //   "local-attribute-key": "local-attribute-value",
  //       //   "assigned_robot": "robot-id"
  //       // }
  //     }
  //   }
  // }

  // const processedData = getSaveDataForTesting(flowName)
  // console.log('@@@@@@@@@@@@@@@@@@@@@')
  // console.log(processedData)

  // processedData.flow_data.forEach(nodeData => {
  //   const nodeID = `node-${nodeData.node_id}`;
  //   const nodeType = nodeData.type;

  //   if (nodeType === 'task') {
  //     flowTriggerTemplat.args.params[nodeID] = nodeData.task_param;
  //     flowTriggerTemplat.args.params[nodeID].assigned_robot = nodeData.assigned_robot;
  //   } else if (nodeType === 'condition') {
  //     flowTriggerTemplat.args.params[nodeID] = nodeData.conditions;
  //   }
  // });

  // console.log(flowTriggerTemplat)
  // let a = await faApiPostFlow(flowName, flowTriggerTemplat)
  // console.log(a)
  // ================ tmp for trigger test by api ================
}