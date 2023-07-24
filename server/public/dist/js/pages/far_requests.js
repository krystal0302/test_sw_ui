/*
 * Author: John Wu
 * Date: 06 Sep. 21,
 * Description:
 *   Expectation: variables independent, commonly-used
 **/

function restAsset() {
  let response = {};
  $.ajax({
    url: `/assets`,
    type: 'GET',
    async: false,
    success: function (data) {
      console.log("--- restAsset success ---");
      console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- restAsset error: ${err_data} ---`);
    }
  });
  return response;
}

// ======== Maps ================================

// --- Maps List ------------
function restGetMapFilelist() {
  return $.get("/maps", function (data, status) {
    if (status === "success") {
      console.log("--- maps list got ---");
    }
  });
}

function restDeleteMapData(_filename) {
  return $.ajax({
    url: `/maps/${_filename}`,
    type: 'DELETE',
    success: function (result) {
      console.log("--- map data deleted ---");
    }
  });
}

// --- Maps Image -----------
function restGetMapImg(_mapName) {
  _mapName = _mapName.split('.').shift();
  return $.get(`/maps/${_mapName}/image`, function (data, status) {
    if (status === "success") {
      console.log("--- map image data got ---");
    }
  });
}

function restPostMapImage(_mapName, _data) {
  $.post(`/maps/${_mapName}/image`, {
    data: _data
  }, function (data, status) {
    if (status === "success") {
      console.log("--- map image posted ---");
    }
  })
}

function restGetMapThumbnail(_filename) {
  return $.get(`/map/${_filename}/thumbnail`, function (data, status) {
    if (status === "success") {
      console.log("--- map-thumbnails got ---");
    }
  });
}

// --- Maps Metadata --------
function restGetMapMetaInfo(_mapName) {
  return $.get(`/maps/${_mapName}/info`, function (data, status) {
    if (status === "success") {
      console.log('--- map meta info got ---');
    }
  });
}

function restPostMapMetaInfo2(_mapName, _content) {
  return $.post(`/maps/${_mapName}/info`, {
    content: _content
  }, function () {
    console.log('post request return');
    console.log('--- map meta info posted ---');
  });
}

// --- Maps Graph -----------
function restGetMapGraph(_fn) {
  // var mapName = _fn.split('.').slice(0, -1).join('.');
  var mapName = _fn.split('.').shift();
  return $.get(`/maps/${mapName}/graph`, function (data, status) {
    if (status === "success") {
      console.log("--- map graph got ---");
    }
  });
}

function restPostMapGraph(_mapName, _content) {
  return $.post(`/maps/${_mapName}/graph`, {
    content: _content,
  }, function () {
    console.log("--- map graph posted ---");
  });
}

// --- Maps Cells -----------
function restGetMapCells(_mapName) {
  _mapName = _mapName.split('.').shift();
  return $.get(`/maps/${_mapName}/cells`, function (data, status) {
    if (status === "success") {
      console.log('--- map cells got ---');
    }
  });
}

function restPostMapCells(_mapName, _content) {
  _mapName = _mapName.split('.').shift();
  // _mapName = _mapName+'_new';
  console.log(_content);
  return $.post(`/maps/${_mapName}/cells`, {
    content: _content,
  }, function () {
    console.log('--- map cells posted ---');
  });
}

// --- Maps table data ---
function restGetAllMapData() {
  return $.ajax({
    url: '/testdb/getMapData',
    type: 'GET',
    success: function (data) {
      console.log("--- map data got ---");
    },
    error: function (err_data) {
      console.log(`---  map data got error: ${err_data} ---`);
    }
  });
}

function restDeleteMapAlias(_mapFileName) {
  return $.ajax({
    url: `/testdb/deleteMapData`,
    type: 'DELETE',
    data: {
      mapFileName: _mapFileName
    },
    success: function (data) {
      console.log(data.message);
    }
  });
}

function restPostMapAlias(_mapFileName, _newMapName) {
  return $.post('/testdb/updateMapAlias', {
    mapFileName: _mapFileName,
    newMapName: _newMapName
  }, function (data, status) {
    if (status === "success") {
      console.log("--- map alias posted ---");
    }
  });
}

function restMapAliasExistence(_mapName) {
  return $.ajax({
    url: `/testdb/maps/${_mapName}/chkAlias`,
    type: 'GET',
    async: false,
    success: function (data) {
      console.log("--- check map alias existence success ---");
    },
    error: function (err_data) {
      console.log(`--- check map alias existence error: ${err_data} ---`);
    }
  });
}

function restMapFileNameExistence(_mapName) {
  return $.ajax({
    url: `/testdb/maps/${_mapName}/chkFileName`,
    type: 'GET',
    async: false,
    success: function (data) {
      console.log("--- check map file name existence success ---");
    },
    error: function (err_data) {
      console.log(`--- check map file name existence error: ${err_data} ---`);
    }
  });
}

// --- Titon Maps -----------
function restGetTritonMap() {
  return $.get("/triton/maps", function (data, status) {
    if (status === "success") {
      console.log('--- Triton map got ---');
    }
  });
}

// ======== Fleets ==============================
function restGetFleets() {
  return $.get("/fleets", function (data, status) {
    if (status === "success") {
      console.log('--- fleet list got ---');
    }
  });
}

function restGetFleetSettings(_fleetName) {
  return $.ajax({
    url: `/fleets/${_fleetName}/settings`,
    type: 'GET',
    success: function (data) {
      console.log('--- fleet settings got ---');
    }
  });
}

function restPostFleetSettings(_fleetName, _data, _oldFleetName = '') {
  return $.post(`/fleets/${_fleetName}/settings`, {
    content: _data,
    oldFilename: _oldFleetName
  }, function (data, status) {
    console.log("--- fleet setings posted ---");
  });
}

function restDeleteFleetConfData(_filename) {
  return $.ajax({
    url: `/fleets/${_filename}`,
    type: 'DELETE',
    success: function (result) {
      console.log("--- fleet data deleted ---");
      wsRemoveFleet(_filename.replace(".yaml", ""), "all");
    }
  });
}

function restGetFleetArtifacts(_fleetName) {
  return $.ajax({
    url: `/fleets/${_fleetName}/settings/artifacts`,
    type: 'GET',
    async: false,
    success: function (data) {
      console.log(`--- fleet artifacts got ---`);
    },
    error: function (err_data) {
      console.log(`--- fleet artifacts got error: ${err_data} ---`);
    }
  });
}

// ======== Artifacts ===========================
function restArtifactsServiceTypes() {
  return $.get(`/artifact/types`, function (data, status) {
    if (status === "success") { } else {
      return 'no artifacts';
    }
  });
}

// ======== Roles ==============================
function restGetBtXsd() {
  return $.get(`/roles/behavior-tree/schema`, function (data, status) {
    if (status === "success") { }
  });
}

function restGetBrXsd() {
  return $.get(`/roles/behaviors/schema`, function (data, status) {
    if (status === "success") { }
  });
}

function restGetBehaviorData(_roleName) {
  _roleName = _roleName.split('.').shift();
  return $.get(`/roles/${_roleName}/behaviors`, function (data, status) {
    if (status === "success") {
      console.log('--- role behaviors got ---');
    }
  });
}

function restPostBehaviorData(_roleName, _data) {
  _roleName = _roleName.split('.').shift();
  return $.post(`/roles/${_roleName}/behaviors`, {
    content: _data
  }, function (data, status) {
    console.log("--- role data posted ---");
  });
}

function restDeleteBehaviorData(_roleName) {
  // _roleName = _roleName.split('.').slice(0, -1).join('.');
  _roleName = _roleName.split('.').shift();
  return $.ajax({
    url: `/roles/${_roleName}/behaviors`,
    type: 'DELETE',
    success: function (result) {
      console.log("--- behavior data deleted ---");
    }
  });
}

function restGetRoleData(_roleName) {
  _roleName = _roleName.split('.').shift();
  return $.get(`/roles/${_roleName}/capabilities`, function (data, status) {
    if (status === "success") { } else {
      return 'no role capabilities';
    }
  });
}

function restPostRoleData(_roleName, _data) {
  return $.post(`/roles/${_roleName}/capabilities`, {
    content: _data
  }, function (data, status) {
    console.log("--- role capabilities data posted ---");
  });
}

function restDeleteRoleData(_filename) {
  return $.ajax({
    url: `/roles/${_filename}/capabilities`,
    type: 'DELETE',
    success: function (result) {
      console.log("--- role data deleted ---");
    }
  });
}

function restRoleFileExistence(_filename) {
  return $.ajax({
    url: `/roles/${_filename}/chkFileNameExists`,
    type: 'GET',
    success: function (data) {
      console.log("--- get role file name existence check ---");
    },
    error: function (err_data) {
      console.log(`--- get role file name existence check error: ${err_data} ---`);
    }
  });
}

function restGetRolesMappingData(_fleetName) {
  return $.ajax({
    url: `/roles/mapping/${_fleetName}`,
    type: 'GET',
    success: function (data) {
      console.log('--- roles mapping got ---');
    },
    error: function (err_data) {
      console.log(`--- roles mapping got error: ${err_data} ---`);
    }
  });
}

function restGetRolesParamData(_fleetName) {
  var deferred = $.Deferred();

  $.ajax({
    url: `/roles/roleParamList/${_fleetName}`,
    type: 'GET',
    success: function (data) {
      console.log('--- roles roleParamList got ---');
      deferred.resolve(data);
    },
    error: function (xhr, status, error) {
      console.log(`--- roles roleParamList got error: ${error} ---`);
      deferred.reject(error);
    }
  });

  return deferred.promise();
}

function restGetRoleBT(_roleName) {
  return $.get(`/role_BT/${_roleName}`, function (data, status) {
    if (status === "success") {
      console.log('--- role-BT got ---');
    } else {
      console.log('no role-BT');
    }
  });
}

// ======== Settings ===========================
function restPostUISettings(_data) {
  return $.post('/settings/settings-ui', {
    content: _data,
  }, function (data, status) {
    console.log("--- UI settings post ---");
  });
}

function restGetSettings() {
  return $.get('/settings/all', function (data, status) {
    if (status === "success") {
      console.log('--- settings got ---');
    }
    else {
      console.log('no settings');
    }
  });
}

// function restPostSettings(_data) {
// 	return $.post('/settings/cell-types', {
// 		content: _data,
// 	}, function (data, status) {
// 		console.log("--- cell-types got ---");
// 	});
// }

// function restPutSettings(_data) {
// 	// return $.post('/settings/put/cell-types', {
// 	// 	content: _data,
// 	// }, function (data, status) {
// 	// 	console.log("--- cell-types put ---");
// 	// });
// 	return $.ajax({
// 		url: '/settings/cell-types',
// 		type: 'PUT',
// 		contentType: 'application/json',
// 		data: JSON.stringify(_data),
// 		success: function (data) {
// 			console.log("--- cell type settings put ---");
// 		}
// 	});
// }

function restGetNotifyGroups() {
  return $.ajax({
    url: '/settings/notify-groups',
    type: 'GET',
    contentType: 'application/json',
    success: function (data) {
      console.log("--- notification groups sent ---");
    }
  });
}

function restPutNotifyGroups(_data) {
  console.log(typeof _data);
  console.log(_data);
  return $.ajax({
    url: '/settings/notify-groups',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify(_data),
    // data: _data,
    success: function (data) {
      console.log("--- notification groups updatation sent ---");
    }
  });
}

function restGetCellTypes() {
  return $.get('/testdb/getCellTypes', function (data, status) {
    if (status === "success") {
      console.log('--- cell-types got ---');
    }
    else {
      console.log('no cell types');
    }
  });
}

function restGetCellTypes(_data) {
  return $.get('/swarm/functionTypes', {
    content: _data,
  }, function (data, status) {
    console.log("--- function-types get ---");
  });
}

function restGetFunctionTypes(_data) {
  return $.get('/swarm/functionTypes', {
    content: _data,
  }, function (data, status) {
    if (status === "success") {
      console.log("--- function-types post ---");
    } else {
      console.log('no cell types');
    }
  });
}

function restPostFunctionTypes(_data) {
  console.log(_data);
  return $.ajax({
    url: '/swarm/functionTypes',
    type: 'POST',
    contentType: 'application/json',
    async: true,
    data: JSON.stringify(_data),
    success: function (data) {
      console.log(data);
    }
  });
}

// ======== Software update ===========================
function restUploadStatus(_fileNameArray) {
  return $.ajax({
    url: '/sw-update/getUploadStatus',
    type: 'POST',
    data: {
      fileNameArray: _fileNameArray,
    },
    success: function (data) {
      // console.log("--- get package upload status ---");
    },
    error: function (err_data) {
      console.log(`--- get package upload status error: ${err_data} ---`);
    }
  });
}

function restDeleteUploadPackage(_filename) {
  return $.ajax({
    url: '/sw-update/removeUploadPkg',
    type: 'POST',
    data: {
      filename: _filename,
    },
    success: function (data) {
      console.log("--- delete upload package ---");
    },
    error: function (err_data) {
      console.log(`--- delete upload package error: ${err_data} ---`);
    }
  });
}

// ======== Operations ==========================
function restGetTasks(_fleet) {
  return $.get(`/operations/${_fleet}/tasks`, function (data, status) {
    if (status === "success") {
      console.log("--- get operation tasks ---");
    }
  });
}

function restGetFlows(_fleet) {
  var deferred = $.Deferred();

  $.ajax({
    url: `/operations/${_fleet}/flows`,
    type: 'GET',
    success: function (data) {
      console.log('--- restGetFlows got ---');
      deferred.resolve(data);
    },
    error: function (xhr, status, error) {
      console.log(`--- restGetFlows got error: ${error} ---`);
      deferred.reject(error);
    }
  });

  return deferred.promise();

  // return $.get(`/operations/${_fleet}/flows`, function (data, status) {
  //   if (status === "success") {
  //     console.log("--- get operation flows ---");
  //   }
  // });
}

function restGetTaskData(fleet, task_name, isAsync = true) {
  return $.ajax({
    url: `/operations/task/${fleet}-task-${task_name}`,
    type: 'GET',
    async: isAsync,
    success: function (data) {
      console.log("--- get operation task : " + task_name + " ---");
    }
  });
}

function restPostTaskData(data, fleet, taskName) {
  return $.post(`/operations/task/${fleet}-task-${taskName}`, {
    content: data,
    fleetname: fleet,
    taskname: taskName
  }, function (data, status) {
    console.log(`--- ${data} ---`);
  });
}

function restDeleteTaskData(fleet, taskName, isAsync = true) {
  return $.ajax({
    url: `/operations/task/${fleet}-task-${taskName}`,
    type: 'DELETE',
    async: isAsync,
    data: {
      fleetname: fleet,
      taskname: taskName
    },
    success: function (data) {
      console.log(`--- ${data} ---`);
    }
  });
}

function restGetFlowData(flow_name, file_name, isAsync = true) {
  var deferred = $.Deferred();

  $.ajax({
    url: `/operations/flow/${file_name}`,
    type: 'GET',
    timeout: 500,
    async: isAsync,
    success: function(response) {
      deferred.resolve(response);
    },
    error: function(xhr, status, error) {
      deferred.reject(error);
    }
  });

  return deferred.promise();
}

function restPostFlowData(data, fleet, flowName) {
  return $.post(`/operations/flow/${fleet}-flow-${flowName}`, {
    content: data,
    flowname: flowName,
    fleetname: fleet
  }, function (data, status) {
    console.log(`--- ${data} ---`);
  });
}

function restPostFlowUIData(data, fleet, flowName) {
  return $.post(`/operations/flowui/${fleet}-flowui-${flowName}`, {
    content: data
  }, function (data, status) {
    console.log(`--- ${data} ---`);
  });
}

function restDeleteFlowData(fleet, flowName, isAsync = true) {
  return $.ajax({
    url: `/operations/flow/${flowName}`,
    type: 'DELETE',
    async: isAsync,
    data: {
      fleetname: fleet
    },
    success: function (data) {
      console.log(`--- ${data} ---`);
    }
  });
}

function restOperationFileExistence(_modename, _filename) {
  return $.post('/operations/chkFileNameExists', {
    eventType: _modename,
    eventName: _filename
  }, function (data, status) {
    if (status === "success") {
      console.log("--- get file name existence check ---");
    }
  });
}

function restRoleInUsedOperations(_fileNameArray, _deletedRole, isAsync = true) {
  return $.ajax({
    url: '/operations/chkRoleInUsed',
    type: 'POST',
    async: isAsync,
    data: {
      fileNameArray: _fileNameArray,
      deleteRoleArray: _deletedRole
    },
    success: function (data) {
      console.log("--- get role in-used operations check ---");
    }
  });
}

function restFleetOperations(fleet, isAsync = true) {
  return $.ajax({
    url: `/operations/${fleet}`,
    type: 'GET',
    async: isAsync,
    success: function (data) {
      console.log("--- get fleet operations ---");
    }
  });
}

// ======== Logs ==========================
function restGetDebugLogTarFile() {
  return $.ajax({
    url: '/log/exportDebugLog',
    type: 'GET',
    success: function (data) {
      console.log('--- get debug log tar file ---');
    }
  });
}

function restGetDebugLogTarFileName() {
  return $.ajax({
    url: '/log/debugTarFileName',
    type: 'GET',
    success: function (data) {
      console.log('--- get debug log tar file name ---');
    }
  });
}

function restGetDebugLogTarFileProgress() {
  return $.ajax({
    url: `/log/exportDebugLogProgress`,
    type: 'GET',
    success: function (data) {
      console.log('--- get export debug log tar file progress ---');
    }
  });
}

// ======== Utils================================
function restGetUIVersion() {
  return $.ajax({
    url: `/version`,
    type: 'GET',
    success: function (data) {
      console.log('--- get UI version ---');
    }
  });
}

// ======== User Table Manipulations ============
function restGetCurrentUserData() {
  return $.get('/testdb/getCurrentUserInfo', function (data, status) {
    if (status === "success") {
      console.log("--- get current user info ---");
    }
  });
}

function restPostUsersRoleData(_usersData) {
  return $.post('/testdb/updateUsersRole', {
    usersData: _usersData
  }, function (data, status) {
    if (status === "success") {
      console.log("--- users role data posted ---");
    }
  });
}

function restDeleteUserData(_deleteUsers) {
  return $.ajax({
    url: '/testdb/deleteUsersData',
    type: 'DELETE',
    data: {
      deleteUsers: _deleteUsers
    },
    success: function (result) {
      console.log("--- users data deleted ---");
    }
  });
}

// ======== History Password ============
function restPasswordExistence(_password) {
  return $.ajax({
    url: '/testdb/chkIsRepeatedPassword',
    type: 'POST',
    async: false,
    data: {
      modifiedPwd: _password
    },
    success: function (data) {
      console.log("--- password existence check ---");
    }
  });
}

// ======== Login & Logout ======================
function restLoginStatus() {
  return $.get('/testdb/getLoginStatus', function (data, status) {
    if (status === "success") {
      console.log("--- get login status ---");
    }
  });
}

function restLogout() {
  return $.get('/testdb/logout', function (data, status) {
    if (status === "success") {
      console.log("--- logout success ---");
      setLoginStatus(JSON.stringify({"userID":'logout', 'loginStatus': false}));
      window.location.href = "login.html";
      removeSelectedFleet();
    } else {
      console.log("--- logout failed ---");
    }
  });
}

// ======== ROS Service Request ======================
// function restDeleteExecutingTask(_type, _task) {
//   return $.ajax({
//     url: '/ros2/tasks/',
//     type: 'DELETE',
//     data: {
//       type: _type,
//       content: _task
//     },
//     error: function (jqXXHR, textSTatus) {
//       if (textSTatus === 'timeout') {
//         alert('failed from timeout')
//       }
//     },
//     success: function (result) {
//       console.log(result);
//     },
//     timeout: 3000
//   });
// }

// function restFlowTaskOp(_job, _options) {
//   var reqType = "";
//   console.log('operation type: ' + _options.op);
//   if (_options.op === "create") {
//     reqType = 'PUT';
//   } else if (_options.op === "remove") {
//     reqType = 'DELETE';
//   } else if (_options.op === "") {
//     return {
//       status_code: 400,
//       message: "EMPTY task operation type"
//     };
//   } else {
//     return {
//       status_code: 400,
//       message: "UNKNOWN task operation type"
//     };
//   }
//   console.log('request type: ' + reqType);

//   return $.ajax({
//     url: '/ros2/flow_tasks/',
//     type: reqType,
//     data: {
//       op: _options.op,
//       "id": _job.id,
//       "name": _job.name,
//       "args": _options.args
//     },
//     error: function (jqXXHR, textSTatus) {
//       if (textSTatus === 'timeout') {
//         alert('failed from timeout')
//       }
//     },
//     success: function (result) {
//       console.log(result);
//     },
//     timeout: 3000
//   });
// }

function restGetParam(_data) {
  return $.ajax({
    url: 'ros2/gmapping/param',
    type: 'GET',
    contentType: 'application/json',
    data: JSON.stringify(_data),
    success: function (data) {
      console.log("--- get gmapping state ---");
      console.log(data);
    }
  });
}

function restPutParam(_data) {
  return $.ajax({
    url: 'ros2/gmapping/param',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify(_data),
    success: function (data) {
      // console.log("--- put gmapping state successfully ---");
      console.log(data);
    }
  });
}

function sendFlowRequest(_data) {
  return $.ajax({
    url: 'ros2/flow_tasks',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify(_data),
    success: function (data) {
      console.log("--- get flow send state ---");
      console.log(data);
    }
  });
}

// ======== Statistics from DB ============
function restPostStatistics(isAsync = true) {
  return $.ajax({
    url: '/swarm/statistics',
    type: 'POST',
    async: isAsync,
    data: {},
    success: function (data) {
      console.log(data);
      return data;
    }
  });
}

function restGetLicenseCheck() {
  return $.ajax({
    url: '/license-check',
    type: 'GET',
    contentType: 'application/json',
    success: function (data) {
      console.log(data);
    }
  });
}

function restDeleteLicense() {
  return $.ajax({
    url: '/remove-license',
    type: 'DELETE',
    success: function (result) {
      console.log("--- license is deleted ---");
    }
  });
}

function restGetLicExpiryDate() {
  return $.ajax({
    url: '/license-expiry',
    type: 'GET',
    contentType: 'application/json',
    success: function (data) {
      console.log(data);
    }
  });
}

function restGetHardwareSignature() {
  return $.ajax({
    url: '/hardware-signature',
    type: 'GET',
    contentType: 'application/json',
    success: function (data) {
      console.log(data);
    }
  });
}

// Plan artifact
function restGetFlattenArtifactConf(data, isAsync = true) {
  let request_res = undefined;
  $.ajax({
    url: `/operations/flattenPlanArtifactConf`,
    type: 'POST',
    async: isAsync,
    data: {
      artifact_conf: data
    },
    success: function (data) {
      // console.log(`--- ${data} ---`);
      request_res = data;
    }
  });
  // console.log(request_res)
  return request_res;
}

function restGetUnFlattenArtifactConf(data, isAsync = true) {
  let request_res = undefined;
  $.ajax({
    url: `/operations/unflattenPlanArtifactConf`,
    type: 'POST',
    async: isAsync,
    data: {
      artifact_conf: data
    },
    success: function (data) {
      // console.log(`--- ${data} ---`);
      request_res = data;
    }
  });
  // console.log(request_res)
  return request_res;
}

function getUnflattenObj(flatten_obj) {
  let response_json = undefined;
  $.ajax({
    type: "POST",
    url: '/fleet/getFlatten',
    async: false,
    data: {
      flatten_object: flatten_obj
    },
    success: function (response) {
      // console.log(response);
      response_json = response;
    },
    error: function (thrownError) {
      alert(`Can't get flat obj!`);
    }
  });
  return response_json;
}

function restPostFleetPlanArtifactSettings(_fleetName, _data, isAsync = true) {
  let request_res = undefined;
  $.ajax({
    url: `/fleets/${_fleetName}/addPlanArtifact`,
    type: 'POST',
    async: isAsync,
    data: {
      artifact_data: JSON.stringify(_data)
    },
    success: function (data) {
      console.log(`--- ${data} ---`);
      request_res = data;
    }
  });
  // console.log(request_res)
  return request_res;
}

function restDeleteFleetPlanArtifactSettings(_fleetName, _artifact_name, isAsync = true) {
  let request_res = undefined;
  $.ajax({
    url: `/deletePlanArtifact/${_fleetName}`,
    type: 'DELETE',
    async: isAsync,
    data: {
      artifact_name: JSON.stringify(_artifact_name)
    },
    success: function (result) {
      console.log("--- map data deleted ---");
      console.log(result);
      request_res = result;
    }
  });
  return request_res;
}

function checkDuplicateFlowname(_fleet, _original, _modified) {
  return $.ajax({
    url: '/flows/checkDuplicate',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify({ fleet: _fleet, original: _original, modified: _modified }),
    success: function (data) {
      console.log(data);
    }
  });
}

function restRenameFlowname(_fleet, _original, _modified) {
  return $.ajax({
    url: 'flows/rename',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify({ fleet: _fleet, original: _original, modified: _modified }),
    success: function (data) {
      console.log(data);
    }
  });
}

function restRenameRoleInFlows(_fileNameArray, _original, _modified) {
  return $.ajax({
    url: 'roles/renameRoleInFlows',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify({ fileNameArray: _fileNameArray, original: _original, modified: _modified }),
    async: false,
    success: function (data) {
      console.log(data);
    }
  });
}

function restRenameRoleConfig(_fileNameArray, _original, _modified) {
  return $.ajax({
    url: 'roles/rename',
    type: 'PUT',
    contentType: 'application/json',
    data: JSON.stringify({ fileNameArray: _fileNameArray, original: _original, modified: _modified }),
    success: function (data) {
      console.log(data);
    }
  });
}

function restGetTooltips() {
  let response = {};
  $.ajax({
    url: `/tooltips`,
    type: 'GET',
    async: false,
    success: function (data) {
      // console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- restGetTooltips error: ${err_data} ---`);
    }
  });
  return response;
}

function restGetLangObj(_lang) {
  // console.log(_lang);
  let response = {};
  $.ajax({
    url: `/lang/${_lang}`,
    type: 'GET',
    async: false,
    success: function (data) {
      // console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- Get Language Error : ${err_data} ---`);
    }
  });
  return response;
}

function restGetTemplateLang(_lang, category) {
  // console.log(_lang);
  let response = {};
  $.ajax({
    url: `/swarm_core/lang/${_lang}/category/${category}`,
    type: 'GET',
    async: false,
    success: function (data) {
      // console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- Get Language Error : ${err_data} ---`);
    }
  });
  return response;
}

function restGetLangMsg(_lang) {
  // console.log(_lang);
  let response = {};
  $.ajax({
    url: `/swarm_core/lang/${_lang}`,
    type: 'GET',
    async: false,
    success: function (data) {
      // console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- Get Language Error : ${err_data} ---`);
    }
  });
  return response;
}

function restGetAgentParamTranslation(_lang) {
  // console.log(_lang);
  let response = {};
  $.ajax({
    url: `/agent_settings/lang/${_lang}`,
    type: 'GET',
    async: false,
    success: function (data) {
      // console.log(data);
      response = data;
    },
    error: function (err_data) {
      console.log(`--- Get Language Error : ${err_data} ---`);
    }
  });
  return response;
}