import { faker } from '@faker-js/faker';

let jsonData = require('./test_settings.json');

function getHostIP() {
    const interfaces = require('os').networkInterfaces();
    for (let devName in interfaces) {
        let networkInterface = interfaces[devName];
        for (let i = 0; i < networkInterface.length; i++) {
            let info = networkInterface[i];
            if (info.family === 'IPv4' && info.address !== '127.0.0.1' && !info.internal) {
                return info.address;
            }
        }
    }
}

function processFilename(filename){
    return filename.replace(/(\.\.)|(\\)|(\/)/g, '');
}

function getTestSetting(test_item="", load_setting_path="") {
    if (load_setting_path != "" && load_setting_path.length > 0){
        jsonData = require(load_setting_path);
    }

    let test_data = []

    if (jsonData.hasOwnProperty(test_item)){
        test_data = jsonData[test_item];

        let data = test_data["data"];
        
        switch (test_item) {
            case 'role':
                if (typeof data == "string"){
                    data = generateRoleData(data);
                }
                break;
            case 'map':
                for (const [data_type, data_val] of Object.entries(data)) {
                    if (typeof data_val == "string") {
                        switch (data_type) {
                            case "vertex":
                                data['vertex'] = generateVertexData(data['vertex']);
                                break;
                            case "function_type":
                                data['function_type'] = generateFunctionTypeData(data['function_type']);
                                break;
                            case "cell":
                                data['cell'] = generateCellData(data['cell']);
                                break;
                        
                            default:
                                break;
                        }
                    }
                }                
                break;
            case 'operation':
                if (typeof data == "string"){
                    data = generateFlowData(data);
                }
                break;
            case 'fleet':
                if (typeof data == "string"){
                    data = generateFleetData(data);
                }
                break;
        
            default:
                break;
        }
        
        test_data = data;
    }

    let default_setting = {
        "homeUrl": `http://${getHostIP()}:3000`, 
        "login_account": jsonData["user_account"],
        "login_pwd": jsonData["user_password"],
        "repeat_times": 1,
        "test_data": test_data,
        "element_wait_second": 10,
        "data_path": jsonData["app_data_path"],
        "special_character": jsonData["special_character"]
    };
    return default_setting;
}

function getRandomInt(max) {
    return Math.floor(Math.random() * max);
}

function generateRoleData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    for (let i = 1; i <= total_gen; i++) {
        let role_name = faker.system.fileName({ extensionCount: 0 });

        let role_dict = {
            "text_val": role_name,
            "move_menu": [
                "move",
                "charge"
            ],
            "logic_menu": [
                "wait"
            ],
            "others_menu": [
                "artifact"
            ]
        };
        
        data.push(role_dict);
    }

    return data;
}

function generateVertexData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    for (let i = 1; i <= total_gen; i++) {
        let vertex_name = faker.finance.account(5);

        let vertex_dict = {
            "text_val": `_${vertex_name}`,
            "move_to": { 
                offsetX: 100*i, 
                offsetY: 100*i 
            }
        };
        
        data.push(vertex_dict);
    }

    return data;
}

function generateFunctionTypeData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    let DETECTION = {
        Rack: 0,
        Charger: 1
    };
    
    let RECOG = {
        Rack: 0
    };

    for (let i = 1; i <= total_gen; i++) {
        let functionType_name = faker.finance.account(5);

        let functionType_dict = {
            "text_val": `testType_${functionType_name}`,
            "detectType": DETECTION.Rack,
            "recogType": RECOG.Rack,
            "offset": `${i},${i},${i}`,
            "size": `${i},${i},${i}`,
            "payload": `${300*i}`
        };
        
        data.push(functionType_dict);
    }

    return data;
}

function generateCellData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    for (let i = 1; i <= total_gen; i++) {
        let cell_name = faker.finance.account(5);

        let cell_dict = {
            "text_val": `_${cell_name}`,
            "move_to": { 
                offsetX: 200*i, 
                offsetY: 200*i 
            }
        };
        
        data.push(cell_dict);
    }

    return data;
}

function generateFlowData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    let flow_type = ["manual", "event", "timer"];
    let random_choose = getRandomInt(flow_type.length);

    for (let i = 1; i <= total_gen; i++) {
        let flow_name = faker.system.fileName({ extensionCount: 0 });

        let flow_dict = {
            "text_val": flow_name,
            "flow_type": flow_type[random_choose],
            "roles": []
        };
        
        data.push(flow_dict);
    }

    return data;
}

function generateFleetData(random_str){
    let total_gen = parseInt(random_str.split('@')[1]);
    let data = [];

    for (let i = 1; i <= total_gen; i++) {
        let fleet_name = faker.system.fileName({ extensionCount: 0 });

        let fleet_dict = {
            "text_val": fleet_name
        };
        
        data.push(fleet_dict);
    }

    return data;
}

module.exports = {
    getHostIP,
    processFilename,
    getTestSetting,
    getRandomInt
};