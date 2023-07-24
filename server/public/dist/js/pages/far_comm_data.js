/*
 * Author: Angela Kao
 * Date: 5 Oct 2022
 * Description: Data sources shared by each page and test file
 **/

// --------------------------------
//   Sidebar Function Permissions
// --------------------------------

let userAuthFunctions_ = {
  "admin": {
    "dashboard": {
      "functionName": "Dashboard",
      "dirPath": "index.html",
      "imgClass": "fas fa-tachometer-alt",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "liveMap": {
      "functionName": "Live view",
      "dirPath": "map_live.html",
      "imgClass": "fas fa-map-marked-alt",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "fleet": {
      "functionName": "Fleet configuration",
      "dirPath": "fleet.html",
      "imgClass": "fas fa-th",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "operation": {
      "functionName": "Flow configuration",
      "dirPath": "operation.html",
      "imgClass": "fas fa-edit",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "taskSchedule": {
      "functionName": "Scheduled task viewer",
      "dirPath": "task_schedule.html",
      "imgClass": "fas fa-stream",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "taskTrigger": {
      "functionName": "Flow trigger",
      "dirPath": "#",
      "lastItemUnderFleetHrchy": true,
      "isTreeView": true
    },
    "map": {
      "functionName": "Map",
      "dirPath": "#",
      "imgClass": "fas fa-map",
      "isTreeView": true,
      "treeItem": {
        "Create Map": "slam.html",
        "Edit Map": "map.html"
      }
    },
    "role": {
      "functionName": "Role",
      "dirPath": "role.html",
      "imgClass": "fas fa-users",
      "isTreeView": false
    },
    "settings": {
      "functionName": "System settings",
      "dirPath": "settings2.html",
      "imgClass": "fas fa-cog",
      "isTreeView": false
    },
    "logs": {
      "functionName": "Log",
      "dirPath": "log.html",
      "imgClass": "fas fa-clipboard-list",
      "isTreeView": false
    },
    "userManagement": {
      "functionName": "User management",
      "dirPath": "manage_user.html",
      "imgClass": "fas fa-user",
      "isTreeView": false
    },
    "wrapperSdk": {
      "functionName": "Wrapper SDK",
      "dirPath": "wrapper.html",
      "imgClass": "fas fa-user",
      "isTreeView": false
    },
    "help": {
      "functionName": "API info",
      "dirPath": "#",
      "imgClass": "fas fa-info-circle",
      "isTreeView": false
    }
  },
  "general": {
    "dashboard": {
      "functionName": "Dashboard",
      "dirPath": "index.html",
      "imgClass": "fas fa-tachometer-alt",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "liveMap": {
      "functionName": "Live view",
      "dirPath": "map_live.html",
      "imgClass": "fas fa-map-marked-alt",
      "lastItemUnderFleetHrchy": false,
      "isTreeView": false
    },
    "taskTrigger": {
      "functionName": "Flow trigger",
      "dirPath": "#",
      "lastItemUnderFleetHrchy": true,
      "isTreeView": true
    },
    "settings": {
      "functionName": "System settings",
      "dirPath": "settings2.html",
      "imgClass": "fas fa-cog",
      "isTreeView": false
    },
    "logs": {
      "functionName": "Log",
      "dirPath": "log.html",
      "imgClass": "fas fa-clipboard-list",
      "isTreeView": false
    }
  }
}

function getAdminUserFunctionNames() {
  return getUserFunctionNames('admin');
}

function getGeneralUserFunctionNames() {
  return getUserFunctionNames('general');
}

function getUserFunctionNames(role_name) {
  var functionNameArr = [];
  let func = Object.values(userAuthFunctions_[role_name]);
  for (const key in func) {
    if (func[key].hasOwnProperty('treeItem')) {
      functionNameArr.push(...Object.keys(func[key].treeItem));
    } else {
      functionNameArr.push(func[key].functionName);
    }
  }
  return functionNameArr;
}

if (typeof module !== 'undefined') {
  module.exports = {
    getAdminUserFunctionNames,
    getGeneralUserFunctionNames
  }
}