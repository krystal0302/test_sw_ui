/*
 * Author: Angela Kao
 * Date: 9 July 13
 * Description:
 **/

var dataTable;
var descArray = new Array("time the event was recorded", "sample description",
  "sample description", "sample description",
  "sample description", "sample description",
  "sample description", "sample description");
var isExporting = false;

const LOGJOBERROR = "error";
const LOGJOBIDLE = "initialized";
const LOGJOBINPROGRESS = "in-progress";
const LOGJOBFINISHED = "finished";

$(function () {

  initDataAsync();

  advancedOptionsToggle();
  // setUpLevelMenu();
  // chkclearSelectAll();
  setUpDateTimePicker();
  // addTableHeaderTooltip(); // --- hide for UX concern ---
  setUpLogTable();

  bindButtonEvents();
  validateLogInputEvent();
});

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'logs');

  // ------ fetch RMT token initially ------
  await rmtTokenCheck();
  initAPIConfig(rmtToken_);

  // ------ language switch ------
  await initLanguageSupport();

  addDownloadLogLink(LogType.Battery);
  await checkLogJobProgress(LogType.Battery);
}

async function rmtTokenCheck() {
  if (rmtToken_ === undefined) {
    try {
      console.log('--- re-fetch rmt token ---')
      rmtToken_ = await fetchToken();
    } catch (err) {
      console.error(err);
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

function bindButtonEvents() {
  // $("#selectAll").click(selectAllLevel);
  // $("#apply-level").click(applyLevel);
  $("#full-text-search").click(searchKeyword);
  $("#apply-options").click(advancedSearch);
  $("#clear-options").click(clearSearch);
  // $("#export-battery-log").click(function () { exportLogFiles($(this), LogType.Battery); });
  $("#export-battery-log").click(function () { startLogJob($(this), LogType.Battery) });
  $("#export-debug-log").click(function () { exportLogFiles($(this), LogType.Debug); });
  $('#search-keyword').on('input', function (e) {
    if ($("#search-keyword").val() === "") {
      searchKeyword();
    }
  });
}

function addTableHeaderTooltip() {
  $("#log-table thead th").each(function (index) {
    var title = descArray[index];
    infoIcon = $('<i class="far fa-question-circle" data-toggle="tooltip" data-placement="bottom" title="' + title + '"></i>');
    infoIcon.tooltip();
    infoIcon.click(function (e) {
      e.stopPropagation();
    });
    $(this).append(infoIcon);
  });
}

function setUpLogTable() {
  dataTable = $('#log-table').DataTable({
    "processing": true,
    "serverSide": true,
    "paging": true,
    "ordering": true,
    "searching": true,
    "autoWidth": false,
    "order": [[0, "desc"]],
    ajax: {
      url: "/testdb/logs",
      // dataSrc: '',
      type: "POST",
      dataFilter: function (data) {
        var json = jQuery.parseJSON(data);
        return JSON.stringify(json);
      }
    },
    "columns": [
      { "data": "TIME" },
      { "data": "event_type" },
      { "data": "event_id" },
      { "data": "module_pub" },
      { "data": "module_sub" },
      { "data": "level" },
      { "data": "msg_cont" },
      { "data": "robot_id" }
    ],
    "columnDefs": [{
      "defaultContent": "-",
      "targets": "_all"
    }]
  });

  // insert buttons
  new $.fn.dataTable.Buttons(dataTable, {
    buttons: [
      {
        text: 'Download CSV',
        className: 'btn btn-secondary farobot-lg-btn d-flex align-items-center justify-content-center',
        // extend: 'csv',
        // filename: 'log_results',
        // fieldSeparator: ';',
        action: function (e, dt, button, config) {
          addDownloadLoader(button);
          console.log(dt.ajax.params())
          var dataObj = dt.ajax.params();
          dataObj.page = "all";
          $.ajax({
            url: "/testdb/logs",
            type: "POST",
            data: dataObj,
            success: function (res, status, xhr) {
              console.log(`--- download event log ---`);
              window.location = '/testdb/downloadEventLogs';
              removeDownloadLoader(button);
              // console.log(res);
              // var csvData = new Blob([res], { type: 'text/csv;charset=utf-8;' });
              // var csvURL = window.URL.createObjectURL(csvData);
              // var tempLink = document.createElement('a');
              // tempLink.href = csvURL;
              // tempLink.setAttribute('download', 'log_results.csv');
              // tempLink.click();
            },
            error: function (err_data) {
              console.log(`--- download event log error ---`);
              console.log(err_data);
              removeDownloadLoader(button);
              setTimeout(function () {
                alert('Download log CSV error!');
              }, 10);
            }
          });
        }
      }
    ]
  });

  dataTable.buttons(0, null).container().appendTo('#export-div');

  // hide search field on card body
  $("#log-table_filter").hide();

  // language switch
  $('#export-div > div button > span').attr('data-i18n', 'log.btn_DownloadCSV');
}

function resetSearchKeyword(isAdvanceSearch = false) {
  dataTable.search('')
  dataTable.columns().every(function () {
    this.search('')
  });
  if (isAdvanceSearch) {
    $("#search-keyword").val('');
  } else {
    clearAdvancedOptions();
    // $.fn.dataTable.ext.search.pop();
  }
}

function clearAdvancedOptions() {
  $("div#options-content input").filter(function () {
    return $(this).val() != ""
  }).val('');
}

function searchKeyword() {
  resetSearchKeyword();
  var keyword = $("#search-keyword").val();
  dataTable.search(keyword).draw();
}

function advancedSearch() {
  if (!checkDateRangeisVaild()) { return; }
  resetSearchKeyword(true);

  var searchObjArray = [];
  var dateArray = [];
  $("div#options-content input").each(function () {
    var inputID = $(this).attr('id');
    var inputVal = $(this).val();

    if (isEmptyString(inputVal)) { return; }

    switch (inputID) {
      case "event-id":
        searchObjArray.push({ colNum: 2, searchText: inputVal });
        break;
      case "event-type":
        searchObjArray.push({ colNum: 1, searchText: inputVal });
        break;
      case "module-pub":
        searchObjArray.push({ colNum: 3, searchText: inputVal });
        break;
      case "module-sub":
        searchObjArray.push({ colNum: 4, searchText: inputVal });
        break;
      case "level":
        searchObjArray.push({ colNum: 5, searchText: inputVal });
        break;
      case "device-id":
        searchObjArray.push({ colNum: 7, searchText: inputVal });
        break;
      case "msg_content":
        searchObjArray.push({ colNum: 6, searchText: inputVal });
        break;
      default:
        dateArray.push({ [inputID]: inputVal });
        break;
    }
  });

  $.each(searchObjArray, function (index, obj) {
    dataTable.columns(obj.colNum).search(obj.searchText)
  });
  if (dateArray.length > 0) {
    var dateRange = getDateRange();
    dataTable.columns(0).search(dateRange);
  }
  dataTable.draw();
}

function clearSearch() {
  clearAdvancedOptions();
  advancedSearch();

  let clear_element_id = ["#event-id", "#event-type", "#module-pub", "#module-sub", "#level", "#device-id", "#msg_content", "#startDate", "#startTime", "#endDate", "#endTime"];

  clear_element_id.forEach(element_id => {
    $.powerTip.destroy($(element_id));
    $(element_id).css('box-shadow', '');
    $(element_id).css('border-color', "");
    $(element_id).css('outline-color', "");
  });
}

function checkDateRangeisVaild() {
  var startDate = $('#startDate').val();
  var endDate = $('#endDate').val();
  var startTime = $('#startTime').val();
  var endTime = $('#endTime').val();
  if ((!isEmptyString(startTime) || !isEmptyString(endTime)) && (isEmptyString(startDate) && isEmptyString(endDate))) {
    alert("Please fill date!");
    return false;
  }
  return true;

}

function getDateRange() {
  var minDate, maxDate, minTime, maxTime;
  var startDateVal = $('#startDate').val();
  var endDateVal = $('#endDate').val();
  var startTimeVal = $('#startTime').val();
  var endTimeVal = $('#endTime').val();

  if (isEmptyString(startDateVal)) {
    minDate = endDateVal;
    maxDate = endDateVal;
  } else if (isEmptyString(endDateVal)) {
    minDate = startDateVal;
    maxDate = startDateVal;
  } else {
    var startDate = new Date(startDateVal);
    var endDate = new Date(endDateVal);
    if (startDate > endDate) {
      minDate = endDateVal;
      maxDate = startDateVal;
    } else {
      minDate = startDateVal;
      maxDate = endDateVal;
    }
  }

  if (isEmptyString(startTimeVal) && isEmptyString(endTimeVal)) {
    minTime = "00:00";
    maxTime = "23:59";
  } else if (isEmptyString(startTimeVal)) {
    minTime = endTimeVal;
    maxTime = endTimeVal;
  } else if (isEmptyString(endTimeVal)) {
    minTime = startTimeVal;
    maxTime = startTimeVal;
  } else {
    var startTime = new Date(startTimeVal);
    var endTime = new Date(endTimeVal);
    if (startTime > endTime) {
      minTime = endTimeVal;
      maxTime = startTimeVal;
    } else {
      minTime = startTimeVal;
      maxTime = endTimeVal;
    }
  }

  var minDateAndTime = minDate + ' ' + minTime;
  var maxDateAndTime = maxDate + ' ' + maxTime;
  return minDateAndTime + ';' + maxDateAndTime;
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function getLogJobStatus(logType) {
  let status;
  await faApiGetLogJob(logType)
    .then(function (res) {
      // console.log(res);
      const data = res.data;
      status = data.value?.status;
    })
    .catch(function (error) {
      console.log(error);
      status = LOGJOBERROR;
    })

  return status;
}

async function checkLogJobProgress(logType) {
  const targetBtn = $(`#export-${logType}-log`);
  const jobStatus = await getLogJobStatus(logType);
  console.log(`${logType} log job status: ${jobStatus}`);
  if (jobStatus === LOGJOBINPROGRESS) {
    lockExportButtonAndAddLoader(targetBtn, logType);
    updateLogProgress(targetBtn, logType);
  } else if (jobStatus === LOGJOBFINISHED) {
    updateLogProgress(targetBtn, logType);
  }
}

async function startLogJob(thisBtn, logType) {
  const jobStatus = await getLogJobStatus(logType);
  console.log(`${logType} log job status: ${jobStatus}`);
  if (jobStatus !== LOGJOBIDLE && jobStatus !== LOGJOBFINISHED) { return; }
  lockExportButtonAndAddLoader(thisBtn, logType);
  await faApiPostLogJob(logType);
  updateLogProgress(thisBtn, logType);
}

const updateLogProgress = async (targetBtn, logType) => {
  await faApiGetLogProgress(logType)
    .then(async function (res) {
      const data = res.data;
      console.log(data);
      let percent = data.value?.percent;
      if (!Number.isInteger(percent)) {
        resetLogProgress(targetBtn, logType);
        return;
      }
      logFileName[logType] = data.value?.packageName || `${logType}-log.zip`;

      updateProgressBar(percent, logType);
      if (percent >= 100) {
        unlockExportButtonAndRemoveLoader(targetBtn, logType);
        enableDownloadLogLink(logType);
        return;
      }

      await sleep(1000); // sleep 1s
      updateLogProgress(targetBtn, logType);
    })
    .catch(function (error) {
      console.log(error);
      resetLogProgress(targetBtn, logType);
    });
};

function addDownloadLogLink(logType) {
  let downloadLink = document.createElement('a');
  downloadLink.className = 'text-info';
  downloadLink.innerHTML = 'Download';
  downloadLink.style.pointerEvents = 'none';
  $(`#export-${logType}-log`).after(downloadLink);
}

function enableDownloadLogLink(logType) {
  let downloadLink = document.querySelector(`#export-${logType}-log-card a`);
  downloadLink.style.pointerEvents = 'auto';
  downloadLink.style.cursor = 'default';
  downloadLink.href = `javascript:getLogResult('${logType}');`;
}

async function getLogResult(logType) {
  await faApiGetLogPackageResult(logType)
    .then(function (res) {
      // console.log(res);

      if (logFileName[logType] === undefined) {
        console.log('get file name error.');
        notificationMsg(3, `Can not download ${logType} log!`);
        return;
      }
      console.log('download file name: ', logFileName[logType]);
      const dataType = res.headers['content-type'];
      let blob = new Blob([res.data], { type: dataType });
      let blobURL = URL.createObjectURL(blob);
      let a = document.createElement('a');
      document.body.appendChild(a);
      a.style = 'display: none';
      a.href = blobURL;
      a.download = logFileName[logType];
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(blobURL);

    })
    .catch(function (error) {
      console.log(error);
      notificationMsg(3, `Can not download ${logType} log!`);
    })
}

function lockExportButtonAndAddLoader(targetBtn, logType) {
  let $loader = $(`#${logType}-log-loader`).detach();
  let lang = getSetLang() || 'en';
  targetBtn.prop('disabled', true).text(langExportLog[lang].run).prepend($loader);
  $(`#${logType}-log-loader`).show();
}

function unlockExportButtonAndRemoveLoader(targetBtn, logType) {
  let $loader = $(`#${logType}-log-loader`).detach();
  let lang = getSetLang() || 'en';
  targetBtn.prop('disabled', false).text(langExportLog[lang].done).prepend($loader);
  $(`#${logType}-log-loader`).hide();
}

function resetLogProgress(targetBtn, logType) {
  updateProgressBar(0);
  unlockExportButtonAndRemoveLoader(targetBtn, logType);
}

async function exportLogFiles(thisBtn, logType) {
  removeDownloadLogLink(logType);
  addExportLoader(thisBtn, logType);
  // -- track export progress to completion --
  updateExportProgress(thisBtn, logType);
  // -- export debug log package --
  let blobData = await fetchLogPackage(rmtToken_, logType);
  updateDownloadLogLink(blobData, thisBtn, logType);

  // // -- export debug log tar file --
  // restGetDebugLogTarFile().then(data => {
  //   console.log(data);
  //   if (data.hasOwnProperty("errors")) {
  //     resetProgress($(this));
  //     alert('Export debug log error!');
  //   }
  // }).catch(error => {
  //   console.log('export debug log unexceptional error:');
  //   console.log(error);
  //   resetProgress($(this));
  //   alert('Export debug log error!');
  // });

  // // -- detect export progress bar percentage --
  // checkExportProgress($(this));
}

// const checkExportProgress = async (targetBtn) => {
//   let progressData = await restGetDebugLogTarFileProgress()
//   console.log(progressData);
//   if (progressData.percent >= 100) {
//     progressData.percent = 100;
//     removeExportLoader(targetBtn);
//     updateDownloadLogLink();
//   }
//   updateProgressBar(progressData.percent);

//   if (progressData.errorMsg.length > 0) {
//     console.log(progressData.errorMsg);
//     resetProgress(targetBtn);
//     alert('Can\'t generate tar.gz file!');
//     return;
//   }
//   if (!isExporting) return;
//   checkExportProgress(targetBtn);
// }

var logFileName = {};
const updateExportProgress = async (targetBtn, logType) => {
  try {
    let progressData = await fetchLogPackageProgress(rmtToken_, logType);
    console.log(progressData);
    let percent = progressData.precent || 0;
    logFileName[logType] = progressData.packageName || `swr-${logType}-log.tar.gz`;

    updateProgressBar(percent, logType);
    if (percent === 100) {
      removeExportLoader(targetBtn);
      // debugLogFileName = progressData.packageName;
    }
  } catch (err) {
    console.error(err);
  }
  if (!isExporting) return;
  updateExportProgress(targetBtn, logType);
};

function advancedOptionsToggle() {
  $("#search-advanced").click(function () {
    $("#options-container").show();
  });

  $("#cancel-options").click(function () {
    $("#options-container").hide();
  });
}

function setUpDateTimePicker() {
  $(".datepicker").datepicker({
    clearBtn: true,
    format: "yyyy/mm/dd"
  }).on("changeDate", function (e) {
    let clear_element_id = ["#startDate", "#endDate"];

    clear_element_id.forEach(element_id => {
      let input_color = $(element_id).css("border-color");
      if (input_color == 'rgb(255, 0, 0)') {
        $.powerTip.destroy($(element_id));
        $(element_id).css('box-shadow', '');
        $(element_id).css('border-color', "");
        $(element_id).css('outline-color', "");
      }
    });

    $('#apply-options').prop('disabled', false);
  });

  $("#startTime").timepicker({
    minuteStep: 1,
    defaultTime: false,
    // showMeridian: false,
    showInputs: false
  });

  $("#endTime").timepicker({
    minuteStep: 1,
    defaultTime: false,
    // showMeridian: false,
    showInputs: false
  });

  $(".timepicker").click(function () {
    var upClass = document.getElementsByClassName('icon-chevron-up');
    var downClass = document.getElementsByClassName('icon-chevron-down');

    while (upClass.length) {
      upClass[0].className = "fas fa-chevron-up";
    }
    while (downClass.length) {
      downClass[0].className = "fas fa-chevron-down";
    }
  });
}

function setUpLevelMenu() {
  $(".dropdown-menu.keep-open").on('click', function (e) {
    e.stopPropagation();
  });
}

function selectAllLevel() {
  $("input:checkbox[name='chk-level']").prop('checked', $(this).prop('checked'));
}

function chkclearSelectAll() {
  $("input:checkbox[name='chk-level']").change(function () {
    if (this.id != 'selectAll' && $("#selectAll").is(':checked') && $(this).is(':checked') == false) {
      $("#selectAll").prop('checked', false);
    }
  })
}

function applyLevel() {
  var levelArray = [];
  var levelText;

  $("input:checkbox[name='chk-level']:checked").each(function () {
    if (this.id != 'selectAll') {
      levelArray.push($.trim($(this).parent().text()));
    }
  });

  if (levelArray.length > 0) {
    levelText = array2String(levelArray);
    $("#level-dropdown").html("<span style='color: black;'>" + levelText) + "</span>";
  } else {
    $("#level-dropdown").html("<span>Level</span>");
  }

  $("#level-dropdown").dropdown('toggle');
}

function array2String(arr) {
  if (arr.length === 1) return arr[0];
  var firstArray = arr.slice(0, arr.length - 1);
  var last = arr[arr.length - 1];
  return firstArray.join(', ') + ", " + last;
}

function addProgressBarAnimation(logType) {
  $(`#export-${logType}-progressbar`).addClass('progress-bar-animated');
}

function removeProgressBarAnimation(logType) {
  $(`#export-${logType}-progressbar`).removeClass('progress-bar-animated');
}

const langExportLog = {
  'en': {
    'run': "Exporting...",
    'done': "Export",
  },
  'zht': {
    'run': "匯出中...",
    'done': "匯出",
  },
  'zh': {
    'run': "导出中...",
    'done': "导出 CSV",
  },
};

function addExportLoader(selElement, logType) {
  var $loader = $(`#${logType}-log-loader`).detach();
  // selElement.prop('disabled', true).text('Exporting...').prepend($loader);
  let lang = getSetLang() || 'en';
  selElement.prop('disabled', true).text(langExportLog[lang].run).prepend($loader);
  $(`#${logType}-log-loader`).show();
  isExporting = true;
}

function removeExportLoader(selElement, logType) {
  var $loader = $(`#${logType}-log-loader`).detach();
  // selElement.prop('disabled', false).text('Export').prepend($loader);
  let lang = getSetLang() || 'en';
  selElement.prop('disabled', false).text(langExportLog[lang].done).prepend($loader);
  $(`#${logType}-log-loader`).hide();
  isExporting = false;
}

const langDownloadCsv = {
  'en': {
    'run': "Downloading...",
    'done': "Download CSV",
  },
  'zht': {
    'run': "下載中...",
    'done': "下載 CSV",
  },
  'zh': {
    'run': "下载中...",
    'done': "下载 CSV",
  },
};

function addDownloadLoader(selElement) {
  var loader = `<div class="farobot-loader" style="margin-right: 5px;"></div>`;
  // selElement.prop('disabled', true).html(`${loader} Downloading...`);
  let lang = getSetLang() || 'en';
  selElement.prop('disabled', true).html(`${loader} ${langDownloadCsv[lang].run}`);
}

function removeDownloadLoader(selElement) {
  // selElement.prop('disabled', false).text('Download CSV');
  let lang = getSetLang() || 'en';
  selElement.prop('disabled', false).text(langDownloadCsv[lang].done);
}

// async function updateDownloadLogLink() {
//   var fileName = await restGetDebugLogTarFileName();
//   console.log(fileName);
//   if (fileName.length > 0) {
//     var downloadLink = document.createElement('a');
//     downloadLink.href = '#';
//     downloadLink.className = 'text-info';
//     downloadLink.innerHTML = 'Download';
//     downloadLink.setAttribute('download', fileName);
//     $('#export-debug-log').after(downloadLink);
//   }
// }

function updateDownloadLogLink(_data, _exportBtn, logType) {
  console.log('download file name: ', logFileName[logType]);
  if (!_data instanceof Blob || logFileName[logType] === undefined) {
    notificationMsg(3, `Export ${logType} log error!`);
    return;
  }
  completeProgress(_exportBtn);

  let blob = new Blob([_data], { type: _data.type });
  let blobURL = URL.createObjectURL(blob);
  let downloadLink = document.createElement('a');
  downloadLink.href = blobURL;
  downloadLink.className = 'text-info';
  downloadLink.innerHTML = 'Download';
  downloadLink.download = logFileName[logType];
  $(`#export-${logType}-log`).after(downloadLink);
}

function removeDownloadLogLink(logType) {
  $(`#export-${logType}-log-card a`).each(function () {
    $(this).remove();
  });
}

function updateProgressBar(percent, logType) {
  $(`#export-${logType}-progressbar`).show();
  $(`#export-${logType}-progressbar`).width(percent + '%');
  $(`#export-${logType}-progressbar`).html(percent + '%');

  if (percent > 0 && percent < 100) {
    addProgressBarAnimation(logType);
  } else {
    removeProgressBarAnimation(logType);
  }
}

function completeProgress(exportBtn) {
  updateProgressBar(100);
  removeExportLoader(exportBtn);
}

// function resetProgress(exportBtn) {
//   updateProgressBar(0);
//   removeExportLoader(exportBtn);
// }

// $(document).on('click', '#export-log-card a', async function (e) {
//   e.preventDefault();
//   let fileName = await restGetDebugLogTarFileName();
//   if (fileName.length > 0) {
//     window.location = `/log/downloadTarFile/${fileName}`;
//     removeDownloadLogLink();
//   }
// });

$(document).on('click', '.sidebar .nav-link', function (e) {
  var url = $(this).attr("href");
  if (url == "" || url == "#") return;
  if (isExporting) {
    if (!confirm('Still exporting. Are you sure to leave?')) {
      e.preventDefault();
    }
  }
});

// --- add input values validator ---
function validateLogInputEvent(rule_list = []) {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#apply-options');

  const searchRule = new Rule('search-keyword', logsearchValidation, $('#full-text-search'));

  const eventIDRule = new Rule('event-id', logsearchValidation, interaction_element);
  const eventTypeRule = new Rule('event-type', logsearchValidation, interaction_element);
  const pubRule = new Rule('module-pub', logsearchValidation, interaction_element);
  const subRule = new Rule('module-sub', logsearchValidation, interaction_element);
  const levelRule = new Rule('level', logsearchValidation, interaction_element);
  const deviceIdRule = new Rule('device-id', logsearchValidation, interaction_element);
  const msgRule = new Rule('msg_content', logsearchValidation, interaction_element);

  const startDateRule = new Rule('startDate', logDateValidation, interaction_element);
  const startTimeRule = new Rule('startTime', logTimeValidation, interaction_element);
  const endDateRule = new Rule('endDate', logDateValidation, interaction_element);
  const endTimeRule = new Rule('endTime', logTimeValidation, interaction_element);

  validatorManager.addValidator(searchRule);

  validatorManager.addValidator(eventIDRule);
  validatorManager.addValidator(eventTypeRule);
  validatorManager.addValidator(pubRule);
  validatorManager.addValidator(subRule);
  validatorManager.addValidator(levelRule);
  validatorManager.addValidator(deviceIdRule);
  validatorManager.addValidator(msgRule);

  validatorManager.addValidator(startDateRule);
  validatorManager.addValidator(startTimeRule);
  validatorManager.addValidator(endDateRule);
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
    let checkAllRes = vm.checkAllRuleClear();
    console.log(checkAllRes)

    // --- [STYLING] reflect validation result ---

    let switch_id = this.id;

    // --- [UX] Interaction Logics ---
    switch (switch_id) {
      case 'search-keyword':
        $('#full-text-search').prop('disabled', !res.bValid);
        break;

      default:
        let err_still = []
        for (const [key, value] of Object.entries(checkAllRes)) {
          if (key != 'search-keyword') {
            err_still.push(value);
          }
        }
        if (err_still.includes(false)) {
          interaction_element.prop('disabled', true);
        } else {
          interaction_element.prop('disabled', false);
        }
        break;
    }
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('log-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

function logsearchValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z

  let bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal);
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
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  if (inputVal.length == 0) {
    bRes = true;
    strMsg = [];
  }

  return { bValid: bRes, strMsg: strMsg }
}

function logDateValidation(inputVal) {
  const reg1 = /([12]\d{3}\/(0[1-9]|1[0-2])\/(0[1-9]|[12]\d|3[01]))/;     // yyyy/MM/dd

  let bRes = reg1.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- Invalid date format, format: yyyy/MM/dd';
  strMsg.push(msg1);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  if (inputVal.length == 0) {
    bRes = true;
    strMsg = [];
  }

  return { bValid: bRes, strMsg: strMsg }
}

function logTimeValidation(inputVal) {
  const reg1 = /^([0-9]|0[0-9]|1[0-9]|2[0-3]):([0-5][0-9])\s{1}([AP][M])$/;     // HH:MM AM/PM

  let bRes = reg1.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- Invalid date format, format: HH:MM AM/PM';
  strMsg.push(msg1);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  if (inputVal.length == 0) {
    bRes = true;
    strMsg = [];
  }

  return { bValid: bRes, strMsg: strMsg }
}