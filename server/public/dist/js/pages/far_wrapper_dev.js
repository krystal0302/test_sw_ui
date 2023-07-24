/*
 * Author: John Wu
 * Date: 05 Oct. 21,
 * Description:
 */

// ========================
//     Glocal Variables 
// ========================
let editor;
let currentFilename = "";

let currWrapper;
let currService;

let cmds_ = ['help', 'type', 'wrapper', 'artifact'];
let subcmds_ = ['list', 'create', 'delete', 'list_service', 'get_service', 'save_service', 'build', 'deploy', 'run', 'run_log'];

// ======================
//       Load Ready 
// ======================
$(function () {
  initRundownAsync();
})

async function initRundownAsync() {
  // ------ sidebar generation ------
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  getLoginStatus(statusData, 'logs');

  // ------ start to play ------
  currWrapper = $.cookie('wrapper_name');
  currService = $.cookie('wrapper_service');

  $("#wrapper-name").text(currWrapper);
  $("#service-name").text(currService);

  // --- init code editor ---
  editor = CodeMirror.fromTextArea(document.getElementById('editor'), {
    lineNumbers: true,
    mode: "python",
    lint: true,
    indentWithTabs: false,
    indentUnit: 4,
    smartIndent: true,
    matchBrackets: true,
    theme: "monokai"
  });

  // --- fetch wrapper service code snippet from back-end. ---
  var content = await cliGetWapperServicesCode(currWrapper, currService);
  editor.setValue(content.output);

  document.getElementsByTagName('form')[0].onsubmit = function (evt) {
    evt.preventDefault();
    processCommand();
  }

  // --- get the text from the input ---
  var textInputValue = document.getElementById('terminalTextInput').value.trim();

  // --- scroll to the bottom of the results ---
  scrollToBottomOfResults();

  // --- get the list of keywords for help & posting it to the screen ---
  var postHelpList = function () {
    // --- help keywords array ---
    var helpKeyWords = [
      "- type + list : to list wrapper-types",
      "- wrapper + list : to list wrappers",
      "- wrapper + create <wrapper_name> <wrapper_type> : to create wrapper",
      "- wrapper + delete <wrapper_name> : to delete wrapper",
      "- wrapper + list_service <wrapper_name> : to list service",
      "- wrapper + get_service <wrapper_name> <service_name> : to get code from service",
      "- wrapper + save_service <wrapper_name> <service_name> <code> : to save code to service",
      "- wrapper + build <wrapper_name> : to build wrapper",
      "- artifact + list : to list artifacts",
      "- artifact + deploy <artifact_id> <wrapper_name> : to deploy artifacts",
      "- artifact + run <artifact_id> <wrapper_name> <service_name> : to run artifacts",
      "- artifact + run_log <artifact_id> : to get artifact run log",
    ].join('<br>');
    appendMessagesToConsole(helpKeyWords);
  }

  var helpResponses = function (_cmd) {
    clearInput();
    switch (_cmd) {
      case "help":
      case "?":
        postHelpList();
        break;

      case "clear":
        clearConsole();
        break;

      default:
        appendMessagesToConsole("<p><i>The command " + "<b>" + textInputValue + "</b>" + " was not found. Type <b>Help</b> to see all commands.</i></p>");
        break;
    }
  }

  var commandValidator = function (_cmd) {
    var isValid = true;
    var jsonCmd = { "cmd": "", "subcmd": "", "args": "" };

    var isValidCmd = cmds_.includes(_cmd[0]);
    var isValidSubcmd = subcmds_.includes(_cmd[1]);

    if (isValidCmd || isValidSubcmd) {
      isValid = false;
      return { isValid, jsonCmd };
    }

    // TODO: confirm the number is align with subcommand.

    var argsArr = _cmd.slice(2, -1);
    jsonCmd.cmd = _cmd[0];
    jsonCmd.subcmd = _cmd[1];
    jsonCmd.args = argsArr.join(' ');

    return { isValid, jsonCmd };
  }

  // --- check the entered commands and assign it to the correct function ---
  var processCommand = async function () {
    var textInput = document.getElementById('terminalTextInput').value.trim();
    textInput = textInput.toLowerCase();
    var cmdCombination = textInput.split(' ');

    const { isValid, cmdReq } = commandValidator(cmdCombination);

    if (isValid) {
      // -- print out the input command as reference ---
      clearInput();
      appendMessagesToConsole("<p class='userEnteredText'>> " + textInputValue + "</p>");

      var retMsg = await restPostCli(cmdReq);
      appendMessagesToConsole(`<i>${retMsg.output}</i>`);

    } else {
      helpResponses(cmdRe[0]);
    }
  };

  // --- list the artifact data ---
  lsArtifacts = await cliListArtifacts();
  console.log(lsArtifacts);
  lsArtifacts = lsArtifacts.output;
  console.log(lsArtifacts);
  loadTestRunArtifacts(lsArtifacts);

}


function btnRunServiceCb() {
  // --- get the the device id --- 
  var deviceSel = document.getElementById('wrapper-opts');
  // console.log(deviceSel);
  var deviceId = deviceSel.options[deviceSel.selectedIndex].value;
  // console.log(deviceId);

  // --- run the wrapper on artifact ---
  var retMsg = cliRunWrapperOnArtifact(deviceId, currWrapper, currService);
  console.log(retMsg);

  // TODO: --- parse the return message ----
  // - dummy behavior -
  appendMessagesToConsole(`> python3 artifact run ${deviceId} ${currWrapper}, ${currService}`);
}

function loadTestRunArtifacts(_artifacts) {
  var artifactIDs = _artifacts.map(a => a.id)

  // --- warpper services edit event callback ---
  console.log(_artifacts);
  var wrpSelNode = $('#wrapper-opts');
  wrpSelNode.empty();

  // --- update the wrapper option from artifacts ---
  for (var key in artifactIDs) {
    var opt = document.createElement("option");
    // opt.text = wrapperCache_[key].name;
    opt.text = artifactIDs[key];
    wrpSelNode.append(opt);
  }

  // --- add select onchange event ---
  wrpSelNode.on('change', function () {
    var selId = $(this).find(":selected").val();
    var art = _artifacts.filter(a => a.id === selId)[0]
    $(".card-img-top").attr("src", `dist/img/sprites/${art.type}-440x220.png`);
  })
}

async function btnSaveServiceCb() {
  // -- get the edited data ---
  var text = editor.getValue();
  text = JSON.stringify(text);

  // --- save the wrapper service --- 
  var retMsg = await cliSaveWapperServicesCode(currWrapper, currService, text);
  var notificationType = retMsg.status === 'error' ? 3 : 1;
  notificationMsg(notificationType, retMsg.output);
  // toast(retMsg.output);

  // --- build the wrapper service --- 
  var retMsg = await cliBuildWrapper(currWrapper);
  console.log(retMsg);
  // - dummy behavior -
  appendMessagesToConsole(`> python3 wrapper build ${currWrapper}`);

  notificationMsg(0, 'Wrapper service is saved!');
  // toast('Wrapper service is saved!');
}


// ============================
//     Widgets Manipulation 
// ============================
// ------ clear editor context ------
function clearEditor() {
  editor.setValue("");
}

// --- clear terminal result ---
var clearConsole = function (textToAdd) {
  document.getElementById('terminalResultsCont').innerHTML = "";
  // scrollToBottomOfResults();
}

// --- clear command input ---
var clearInput = function () {
  document.getElementById('terminalTextInput').value = "";
}

// --- scroll to the bottom of the results ---
var scrollToBottomOfResults = function () {
  var terminalResultsDiv = document.getElementById('terminalResultsCont');
  terminalResultsDiv.scrollTop = terminalResultsDiv.scrollHeight;
}

// --- add text to the results ---
var appendMessagesToConsole = function (textToAdd) {
  document.getElementById('terminalResultsCont').innerHTML += "<p>" + textToAdd + "</p>";
  scrollToBottomOfResults();
}

// ------ Notification Utility ------
function toast(message) {
  Toastify({
    text: message,
    duration: 3000,
    close: false,
    gravity: "top",
    position: "right",
    backgroundColor: "#666",
    stopOnFocus: true,
  }).showToast();
}

// ------ toggle sidebar ------------
$('#hide-sidebar').on('click', function () {
  $('.control-sidebar').ControlSidebar('toggle');
});


// ============================
//     CLI Tool Requests 
// ============================
async function cliGetWapperServicesCode(_wrapperName, _serviceName) {
  var command = { "cmd": "wrapper", "subcmd": "get_service", "args": `${_wrapperName} ${_serviceName}` };
  var retMsg = await restPostCli(command);
  retMsg = JSON.parse(retMsg);
  // console.log(retMsg);
  return retMsg;
}

async function cliSaveWapperServicesCode(_wrapperName, _serviceName, _code) {
  var command = { "cmd": "wrapper", "subcmd": "save_service", "args": `${_wrapperName} ${_serviceName} ${_code}` };
  var retMsg = await restPostCli(command);
  retMsg = JSON.parse(retMsg);
  // console.log(retMsg);
  return retMsg;
}

async function cliBuildWrapper(_wrapperName) {
  var command = { "cmd": "wrapper", "subcmd": "build", "args": `${_wrapperName}` };
  var retMsg = await restPostCli(command);
  retMsg = JSON.parse(retMsg);
  // console.log(retMsg);
  return retMsg;
}

async function cliListArtifacts() {
  var command = { "cmd": "artifact", "subcmd": "list", "args": "" };
  var retMsg = await restPostCli(command);
  retMsg = JSON.parse(retMsg);
  retMsg = { "status": "success", "output": retMsg };
  // console.log(retMsg);
  return retMsg;
}

async function cliRunWrapperOnArtifact(_deviceId, _wrapperName, _serviceName) {
  var command = { "cmd": "artifact", "subcmd": "run", "args": `${_deviceId} ${_wrapperName} ${_serviceName}` };
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

