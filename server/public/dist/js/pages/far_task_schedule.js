/*
 * Author: Angela
 * Date: 01 Dec 2022
 * Description:
 **/

var sys_date;
var curFleet;
var taskChart;
var chartCache_ = [];
var dateRangeCache_ = { start: null, end: null };
let langTemplateObj_ = {};

// ======================
//       Load Ready 
// ======================
$(async function () {
  'use strict'
  initDataAsync();
})

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ set up sidebar fleet switching callback function ------
  setSwitchFleetCallback(switchFleetCallback);

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'taskSchedule');

  // ------ language switch ------
  await initLanguageSupport();

  let lng = getSetLang() || 'en';
  langTemplateObj_ = await restGetTemplateLang(lng, 'task_schedule_viewer');

  await rmtTokenCheck();
  curFleet = document.getElementById('fleet-select').value;
  await initDatetimePicker();
  await updateFilterPickers(curFleet);
  await initTaskChart();
  selectAllAgents();
  pollTaskSchedule();
  // testGanttChart();
}

async function switchFleetCallback() {
  curFleet = document.getElementById('fleet-select').value;
  console.log(`--- switch fleet: ${curFleet} ---`);
  await updateFilterPickers(curFleet);
  selectAllAgents();
  // Agents
  let agents = $('#agent-select').selectpicker('val');
  // Flows
  let flows = $('#flow-select').selectpicker('val');
  chartCache_ = await getChartData(cvt2ISODateString(dateRangeCache_.start), cvt2ISODateString(dateRangeCache_.end), curFleet, agents, flows);
  resetChartData(chartCache_.data);
  resetChartYAxis(chartCache_.categorys);
}

async function initDatetimePicker() {
  var sys_date_millisec = await fetchGetSystemTime(rmtToken_);
  sys_date = new Date(sys_date_millisec);
  $(".datepicker").datepicker({
    // forceParse: false,
    autoclose: true,
    format: "yyyy/mm/dd"
  });

  $(".datepicker").each(function () {
    $(this).datepicker('setDate', `${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`);
  });

  $(".datepicker").datepicker().on("changeDate", function (e) {
    if (dateReset) return;
    // --- restrict date range at most 1 month ---
    let selectedDate = e.date;
    if (selectedDate === undefined) return;
    var date = new Date(selectedDate.getTime());
    let datepickerId = e.target.firstElementChild.id;
    if (datepickerId === 'startDate') {
      var maxDate = new Date(date.setMonth(date.getMonth() + 1));
      let endDate = document.getElementById('endDate').value;
      endDate = cvtString2Date(endDate);
      if (endDate > maxDate || endDate < selectedDate) {
        $('#endDatePicker').datepicker('setDate', null);
      }
      $('#endDatePicker').datepicker('setStartDate', selectedDate);
      $('#endDatePicker').datepicker('setEndDate', maxDate);
    } else {
      var minDate = new Date(date.setMonth(date.getMonth() - 1));
      let startDate = document.getElementById('startDate').value;
      startDate = cvtString2Date(startDate);
      if (startDate < minDate || startDate > selectedDate) {
        $('#startDatePicker').datepicker('setDate', null);
      }
      $('#startDatePicker').datepicker('setStartDate', minDate);
      $('#startDatePicker').datepicker('setEndDate', selectedDate);
    }
  });

  var curTime = new Date(),
    sTime = new Date(curTime),
    eTime = new Date(curTime);
  sTime.setMinutes(curTime.getMinutes() - 30);
  eTime.setMinutes(curTime.getMinutes() + 30);
  sTime = sTime.toLocaleString('en-US', { hour: 'numeric', minute: 'numeric', hour12: true });
  eTime = eTime.toLocaleString('en-US', { hour: 'numeric', minute: 'numeric', hour12: true });

  $("#startTime").timepicker({
    minuteStep: 1,
    defaultTime: sTime,
    showInputs: false
  });

  $("#endTime").timepicker({
    minuteStep: 1,
    defaultTime: eTime,
    showInputs: false
  });

  $('#startTime').timepicker('setTime', sTime);
  $('#endTime').timepicker('setTime', eTime);

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

async function updateFilterPickers(fleet_) {
  let agentSelect = document.getElementById("agent-select");
  let flowSelect = document.getElementById("flow-select");
  agentSelect.length = 0;
  flowSelect.length = 0;

  let robotIds = await getAvailableRobotsID(fleet_);
  if (robotIds !== undefined && robotIds.length > 0) {
    robotIds.forEach((id) => {
      var option = document.createElement("option");
      option.text = id;
      option.value = id;
      agentSelect.appendChild(option);
    });
  }

  let flowNames = await getAvailableFlowsName(fleet_);
  if (flowNames !== undefined && flowNames.length > 0) {
    flowNames.forEach((name) => {
      var option = document.createElement("option");
      option.text = name;
      option.value = name;
      flowSelect.appendChild(option);
    });
  }

  $('.selectpicker').selectpicker('refresh');
}

let dateReset = false;
function resetDateRange() {
  dateReset = true;
  $(".datepicker").each(function () {
    $(this).datepicker('setDate', `${sys_date.getFullYear()}/${sys_date.getMonth() + 1}/${sys_date.getDate()}`);
    $(this).datepicker('setStartDate', null);
    $(this).datepicker('setEndDate', null);
  });
  $('#startTime').timepicker('setTime', '12:00 AM');
  $('#endTime').timepicker('setTime', '11:59 PM');
  applyDateRange();
  dateReset = false;
}

// function selectAllFilterOptions() {
//   $('.selectpicker').selectpicker('selectAll');
// }

function selectAllAgents() {
  $('#agent-select').selectpicker('selectAll');
}

async function initTaskChart() {
  // Date
  const [start, end] = getDateRange();
  // Fleet
  let curFleet = document.getElementById('fleet-select').value;
  // Agents
  let agents = $('#agent-select').selectpicker('val');
  // Flows
  let flows = $('#flow-select').selectpicker('val');
  // Scale
  const [unit, number] = document.getElementById('scale-select').value.split("-");

  chartCache_ = await getChartData(cvt2ISODateString(start), cvt2ISODateString(end), curFleet, agents, flows);

  if (!isEmpty(langTemplateObj_)) {
    let chartObj = langTemplateObj_.chart;
    Object.keys(chartObj).forEach(function (key) {
      chartObj[key.replace('ldn_', '').toLowerCase()] = chartObj[key];
      delete chartObj[key];
    });
    _.forOwn(chartObj, function (value, key) { amchartsLegends[key].name = value });
    console.log(amchartsLegends);
  }

  var task_init_data = {
    element_id: 'chartdiv',
    is_dark_theme: getSavedTheme() === 'dark',
    category_list: chartCache_.categorys,
    legend_list: amchartsLegends,
    scale_unit: unit,
    scale_num: number
  };
  var taskGanttChart = new GanttChart(task_init_data);
  taskGanttChart.startDate = start;
  taskGanttChart.endDate = end;
  taskGanttChart.chartData = chartCache_.data;
  const [chart, series] = taskGanttChart.generateGanttAMCharts();
  taskChart = chart;
  chart.appear();
  series.appear(1000, 100);
}

function pollTaskSchedule(_interval = 1000) {
  setInterval(async function () {
    if (chartCache_.data.length === 0) return;

    // Agents
    let agents = $('#agent-select').selectpicker('val');
    // Flows
    let flows = $('#flow-select').selectpicker('val');

    let estData = await getChartData(cvt2ISODateString(dateRangeCache_.start), cvt2ISODateString(dateRangeCache_.end), curFleet, agents, flows, 'e');
    estData = estData.data;
    let overTimeEstData = _.filter(estData, ({ end_time }) => (end_time * 1000) < new Date().getTime());
    let overTimeTaskIds = _.map(overTimeEstData, 'task_id');
    let hisCacheData = _.clone(chartCache_.data);
    _.remove(hisCacheData, obj => obj.isHisData === false);

    // --- re-fetch history task schedule data if needed ---
    if (overTimeTaskIds.length > 0) {
      // console.log(overTimeTaskIds);
      let hisData = await getChartData(cvt2ISODateString(dateRangeCache_.start), cvt2ISODateString(dateRangeCache_.end), curFleet, agents, flows, 'h');
      hisData = hisData.data;
      const filteredHisData = hisData.filter((obj) => {
        return overTimeTaskIds.indexOf(obj.task_id) >= 0;
      });
      Array.prototype.push.apply(hisCacheData, filteredHisData);
      // test only
      // _.remove(estData, obj => overTimeTaskIds.includes(obj.task_id));
    }
    Array.prototype.push.apply(hisCacheData, estData);
    if (JSON.stringify(chartCache_.data) === JSON.stringify(hisCacheData)) return;
    console.log(hisCacheData);
    chartCache_.data = hisCacheData;
    resetChartData(hisCacheData);
  }, _interval);
}

function getDateRange() {
  let startDate = document.getElementById('startDate').value;
  let startTime = document.getElementById('startTime').value;
  startTime = cvtTime12hTo24h(startTime);
  startDate = startDate.concat(" ", startTime);
  let endDate = document.getElementById('endDate').value;
  let endTime = document.getElementById('endTime').value;
  endTime = cvtTime12hTo24h(endTime);
  endDate = endDate.concat(" ", endTime);
  dateRangeCache_.start = startDate;
  dateRangeCache_.end = endDate;
  return [startDate, endDate];
}

function checkDateIsVaild() {
  let startDate = document.getElementById('startDate').value;
  let startTime = document.getElementById('startTime').value;
  let endDate = document.getElementById('endDate').value;
  let endTime = document.getElementById('endTime').value;
  if (isEmptyString(startDate) || isEmptyString(startTime) || isEmptyString(endDate) || isEmptyString(endTime)) {
    alert("Please fill date and time!");
    return false;
  } else if (cvtTime12hTo24h(startTime) > cvtTime12hTo24h(endTime)) {
    alert("Start time is later than end time!");
    return false;
  }
  return true;
}


// ======================
//     Event Callbacks
// ======================
async function applyDateRange() {
  if (!checkDateIsVaild()) return;
  // Date
  const [start, end] = getDateRange();
  console.log(`start date: ${start}, end date: ${end}`);
  let startDate = cvtString2Date(start);
  let endDate = cvtString2Date(end);
  // Agents
  let agents = $('#agent-select').selectpicker('val');
  // Flows
  let flows = $('#flow-select').selectpicker('val');
  chartCache_ = await getChartData(cvt2ISODateString(start), cvt2ISODateString(end), curFleet, agents, flows);
  resetChartXAxisRange(startDate, endDate);
  resetChartData(chartCache_.data);
}

function applyScale(el) {
  var selScale = el.value;
  const [unit, number] = selScale.split("-");
  console.log(`unit: ${unit}, count: ${number}`);
  resetChartXAxisInterval(unit, number);
  resetChartData(chartCache_.data);
}

async function filterAgents(el, evt) {
  var selAgents = $(el).selectpicker('val');
  if (selAgents.length == 0) {
    chartCache_.data = [];
    resetChartData();
    resetChartYAxis();
    return;
  }
  console.log(selAgents);
  // Flows
  let flows = $('#flow-select').selectpicker('val');
  chartCache_ = await getChartData(cvt2ISODateString(dateRangeCache_.start), cvt2ISODateString(dateRangeCache_.end), curFleet, selAgents, flows);
  resetChartData(chartCache_.data);
  resetChartYAxis(chartCache_.categorys);
}

async function filterFlows(el, evt) {
  var selFlows = $(el).selectpicker('val');
  if (selFlows.length == 0) {
    chartCache_.data = [];
    resetChartData();
    return;
  }
  console.log(selFlows);
  // Agents
  let agents = $('#agent-select').selectpicker('val');
  chartCache_ = await getChartData(cvt2ISODateString(dateRangeCache_.start), cvt2ISODateString(dateRangeCache_.end), curFleet, agents, selFlows);
  resetChartData(chartCache_.data);
}

function resetFilters() {
  resetDateRange();
  var scaleSel = document.getElementById('scale-select');
  if (scaleSel.selectedIndex != 0) {
    scaleSel.selectedIndex = 0;
    scaleSel.onchange();
  }
  selectAllAgents();
}


// ======================
//      API Requests    
// ======================
async function rmtTokenCheck() {
  if (!rmtToken_) {
    try {
      console.log('------ ###### rmt token re-fetch #### ------')
      rmtToken_ = await fetchToken();
    } catch (err) {
      console.error(err);
      console.log(rmtToken_);
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

async function getChartData(startDate_, endDate_, fleet_, agents_, flows_, type_ = 'all') {
  let chartData = { categorys: [], data: [] };
  let comData = [];
  if (type_ === 'e' || type_ === 'all') {
    let estData = await getEstSchedule(startDate_, endDate_, fleet_, agents_, flows_);
    estData = _.map(estData, o => _.extend({ isHistory: false }, o));
    Array.prototype.push.apply(comData, estData);
  }
  if (type_ === 'h' || type_ === 'all') {
    let hisData = await getHistorySchedule(startDate_, endDate_, fleet_, agents_, flows_);
    hisData = _.map(hisData, o => _.extend({ isHistory: true }, o));
    Array.prototype.push.apply(comData, hisData);
  }

  // if (comData.length === 0) return;
  // console.log(comData);

  let categorys = [];
  let dataArray = [];
  categorys = comData.map(r => ({ category: r.robot_id }));
  categorys = _.uniqBy(categorys, obj => obj.category);
  chartData.categorys = categorys;
  // console.log(categorys);

  for (var key in comData) {
    var obj = comData[key];
    var rid = obj.robot_id;
    var isHisData = obj.isHistory;
    var tasks = obj.tasks;
    for (var key in tasks) {
      let taskObj = tasks[key];
      let state = parseInt(taskObj.state);
      let wasFailed = parseInt(taskObj.was_failed);
      // let isHistory = (taskObj.end_time * 1000) < new Date().getTime();
      if (isHisData) {
        if (state === 2 && wasFailed === 0) {
          taskObj.columnSettings = { fill: amchartsLegends.complete.color };
        } else {
          taskObj.columnSettings = { fill: amchartsLegends.fail.color };
        }
      } else {
        if (state === 4) {
          taskObj.columnSettings = { fill: amchartsLegends.pause.color };
        } else if (state === 1 || state === 3) {
          taskObj.columnSettings = { fill: amchartsLegends.active.color };
        } else {
          taskObj.columnSettings = { fill: amchartsLegends.queue.color };
        }
      }
      taskObj.category = rid;
      taskObj.isHisData = isHisData;
      taskObj.start = taskObj.start_time * 1000;
      taskObj.end = taskObj.end_time * 1000;
      dataArray.push(taskObj);
    }
  }
  chartData.data = dataArray;
  console.log(dataArray);
  return chartData;
}

async function getHistorySchedule(startDate_, endDate_, fleet_, agents_, flows_) {
  var queryData = [];

  let res = await fetchHistorySchedule(rmtToken_, startDate_, endDate_, fleet_, agents_, flows_);
  if (!res.ok) return queryData;

  queryData = await res.json();
  if (queryData.hasOwnProperty("robots")) {
    queryData = queryData.robots;
  }

  return queryData;
}

async function getEstSchedule(startDate_, endDate_, fleet_, agents_, flows_) {
  var queryData = [];

  let res = await fetchEstSchedule(rmtToken_, startDate_, endDate_, fleet_, agents_, flows_);
  if (!res.ok) return queryData;

  queryData = await res.json();
  if (queryData.hasOwnProperty("robots")) {
    queryData = queryData.robots;
  }

  return queryData;
}

async function getAvailableRobotsID(fleet_) {
  var agentIds = [];

  var res = await fetchFleetStates(rmtToken_, fleet_, 'agent_only');
  if (!res.ok) return;

  var queryData = await res.json();
  if (queryData.hasOwnProperty("fleet_state")) {
    queryData["fleet_state"].forEach((fleet_item) => {
      fleet_item["robots"].forEach((robot_item) => {
        agentIds.push(robot_item["robot_id"]);
      });
    });
  }

  return agentIds;
}

async function getAvailableFlowsName(fleet_) {
  var flowNames = [];

  var res = await fetchFlowList(rmtToken_, fleet_);
  if (!res.ok) return;

  var queryData = await res.json();
  if (queryData.hasOwnProperty("swarm_data") && queryData.swarm_data.length > 0) {
    let swarm_data = queryData.swarm_data[0]; // suppose only one key-value pair
    flowNames = swarm_data["flows"];
  }

  return flowNames;
}


// ======================
//      Gantt Chart
// ======================

function resetChartData(data_ = []) {
  taskChart.data.setAll(data_);
}

function resetChartYAxis(data_ = []) {
  taskChart._settings.yAxis.data.setAll(data_);
}

function resetChartXAxisRange(minDate_, maxDate_) {
  taskChart._settings.xAxis._settings.min = minDate_.getTime();
  taskChart._settings.xAxis._settings.max = maxDate_.getTime();
}

function resetChartXAxisInterval(timeUnit_, count_) {
  taskChart._settings.xAxis._settings.baseInterval = { timeUnit: timeUnit_, count: count_ };
}

function testGanttChart() {
  // am5.ready(function () {

  // Create root element
  var root = am5.Root.new("chartdiv");
  root.dateFormatter.setAll({
    dateFormat: "yyyy/MM/dd HH:mm:ss",
    dateFields: ["valueX", "openValueX"]
  });

  // Set themes
  var isDarkMode = getSavedTheme() === 'dark';
  if (isDarkMode) {
    root.setThemes([
      am5themes_Dark.new(root)
    ]);
  } else {
    root.setThemes([
      am5themes_Animated.new(root)
    ]);
  }

  // Create chart
  var chart = root.container.children.push(am5xy.XYChart.new(root, {
    panX: false,
    // panY: false,
    // wheelX: "panX",
    wheelY: "panX",
    // layout: root.verticalLayout
  }));

  // Data
  var data = [{
    category: "fb_0",
    start: new Date(2022, 11, 6, 0, 0, 0).getTime(),
    end: new Date(2022, 11, 6, 0, 20, 0).getTime(),
    columnSettings: {
      fill: am5.color(0xff0000),
    },
    taskName: "test task"
  }];

  // Create axes
  var yAxis = chart.yAxes.push(
    am5xy.CategoryAxis.new(root, {
      categoryField: "category",
      renderer: am5xy.AxisRendererY.new(root, {}),
      tooltip: am5.Tooltip.new(root, {})
    })
  );

  yAxis.data.setAll([
    { category: "fb_4" },
    { category: "fb_3" },
    { category: "fb_2" },
    { category: "fb_1" },
    { category: "fb_0" }
  ]);

  let scale_val = document.getElementById('scale-select').value;
  var scale_unit = "minute";
  var scale_num = 1;
  // var gridIntervals = [
  //   { timeUnit: "minute", count: 1 },
  //   { timeUnit: "hour", count: 1 }
  // ];
  if (scale_val === 'auto') {
    //TODO: display optimized granularity automatically
  } else {
    scale_unit = scale_val.split("-")[0];
    scale_num = parseInt(scale_val.split("-")[1]);
  }

  let startDate = document.getElementById('startDate').value;
  let endDate = document.getElementById('endDate').value;
  let startTime = document.getElementById('startTime').value;
  let endTime = document.getElementById('endTime').value;
  startTime = cvtTime12hTo24h(startTime);
  endTime = cvtTime12hTo24h(endTime);
  startDate = startDate.concat(" ", startTime);
  endDate = endDate.concat(" ", endTime);
  startDate = cvtString2Date(startDate);
  endDate = cvtString2Date(endDate);
  console.log(`Date from ${startDate} to ${endDate}`);
  console.log(startDate.toISOString());
  console.log(endDate.toISOString());

  var xAxis = chart.xAxes.push(
    am5xy.DateAxis.new(root, {
      min: startDate.getTime(),
      max: endDate.getTime(),
      baseInterval: { timeUnit: scale_unit, count: scale_num },
      // gridIntervals: gridIntervals,
      renderer: am5xy.AxisRendererX.new(root, {
        // minGridDistance: 75
      })
    })
  );

  xAxis.get("dateFormats")["second"] = "MMM dd HH:mm:ss";
  xAxis.get("dateFormats")["minute"] = "MMM dd HH:mm";
  xAxis.get("dateFormats")["hour"] = "MMM dd HH:mm";

  // // Stroke current date line
  // var currentDate = new Date();
  // // if (currentDate <= endDate) {
  // var rangeDataItem = xAxis.makeDataItem({
  //   value: currentDate,
  //   above: true
  // });

  // xAxis.createAxisRange(rangeDataItem);

  // rangeDataItem.get("grid").setAll({
  //   stroke: am5.color(0xff0000),
  //   strokeOpacity: 1
  // });

  // rangeDataItem.get("label").setAll({
  //   fill: am5.color(0xff0000),
  //   text: 'Now',
  //   dy: 25
  // });
  // // }

  // Add series
  var series = chart.series.push(am5xy.ColumnSeries.new(root, {
    xAxis: xAxis,
    yAxis: yAxis,
    openValueXField: "start",
    valueXField: "end",
    categoryYField: "category"
  }));

  series.columns.template.setAll({
    templateField: "columnSettings",
    strokeOpacity: 0,
    height: am5.percent(50),
    tooltipText:
      `Task ID:\nTask name: {taskName}\nPriority:\nStart time: [bold]{openValueX}[/]\nEnd time: [bold]{valueX}[/]\nExecution time:`
  });

  // series.events.on("datavalidated", function (ev) {
  //   //TODO: depends on scale
  //   ev.target.get("xAxis").zoomToDates(new Date(2022, 11, 1), new Date(2022, 11, 1, 0, 10));
  // });

  series.data.setAll(data);

  // Add scrollbars #1
  chart.set("scrollbarX", am5.Scrollbar.new(root, {
    orientation: "horizontal",
    height: 50
  }));

  // let scrollbarX = chart.get("scrollbarX");

  // scrollbarX.thumb.setAll({
  //   fill: am5.color(0xff0000),
  //   fillOpacity: 0.1,
  //   minWidth: 50
  // });

  // scrollbarX.startGrip.setAll({
  //   visible: false
  // });

  // scrollbarX.endGrip.setAll({
  //   visible: false
  // });

  // Add scrollbars #2
  // var scrollbarX = am5xy.XYChartScrollbar.new(root, {
  //   orientation: "horizontal",
  //   height: 50
  // });

  // chart.set("scrollbarX", scrollbarX);

  // not work
  // scrollbarX.events.on("validated", function () {
  //   scrollbarX.scrollbarChart.xAxes.getIndex(0).gridIntervals.setAll([{ timeUnit: scale_unit, count: scale_num }]);
  // });

  // Make stuff animate on load
  series.appear();
  chart.appear(1000, 100);
  // }); // end am5.ready()
}

const amchartsLegends = {
  complete: {
    name: 'Normal',
    color: am5.color(0x92d050)
  },
  queue: {
    name: 'Forecast',
    color: am5.color(0x808080)
  },
  fail: {
    name: 'Failed/Canceled/Manual/Intervened',
    color: am5.color(0xff0000)
  },
  active: {
    name: 'Ongoing',
    color: am5.color(0x00a0ff)
  },
  pause: {
    name: 'Paused',
    color: am5.color(0xf39c12)
  }
}