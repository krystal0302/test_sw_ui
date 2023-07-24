'use strict';
const express = require('express');
const router = express.Router();
const fs = require('fs');
const ini = require('ini');
const processFilename = require('../util').processFilename;
const path = require('path');
const sizeOf = require('../node_modules/image-size');
const yaml = require('js-yaml');
const xmlConverter = require('../node_modules/xml-js');
var formatter = require('../node_modules/xml-formatter');
const csv2json = require('../node_modules/csvtojson');
const json2csv = require('json2csv');
const resizeImg = require('resize-img');
const process = require('process');
const axios = require('axios')
var packageJSON = require('../package.json');
const logManager = require('../services/log-manager');
var flatten = require('flat');
var unflatten = require('flat').unflatten;

const root = '/home/farobot/far_app_data/';
const rlDataPath = root + 'app_roles/';
const btDataPath = root + 'app_roles/role_behavior_xml/';
const xsdDataPath = root + 'app_roles/xsd/';
const mapDataPath = root + 'app_map/';
const tritonMapPath = root + 'app_triton_map/';
const graphDataPath = root + 'app_map/';
const cellDataPath = root + 'app_map/';
const operationDataPath = root + 'app_operation/';
const paramsDataPath = root + 'app_params/';
const logDataPath = root + 'app_log/';
const debugLogPath = logDataPath + 'debug_log';
// const fleetDataPath = root;
const functionTypesPath = root + 'app_wms/';
const fleetDataPath = root + 'app_fleet/';
const artifactDataPath = '/home/farobot/far_fleet_data/template/{artifact_id}_data/artifacts_templates/';
const licensePath = '/home/farobot/far_app_data/app_license/'

// --- notification settings here ---
const Database = require('better-sqlite3');
const dbFile = './farobottech.db';
const notifyTable = new Database(dbFile, { verbose: console.log });

// swarm core api JUST TEST CODE
// let SwarmCoreAPIRequest = require('./api_gateway.js');
// const test = new SwarmCoreAPIRequest();

// test.test()
// console.log(test.getAccessToken());
// console.log('AAAAAAAAAAAAAAAAAAAAAAASDADASDASD');


notifyTable.prepare(`
  CREATE TABLE IF NOT EXISTS notification (
		line_token text NOT NULL,
		level_logicOp text,
		level_objVar text,
		content_logicOp text
		content_objVar text
)`).run();

const svrRoot = path.resolve(__dirname, '..') + '/';
const swarmUiRoot = root + 'app_params/ui/';
const servicePath = svrRoot + 'services/mock_cli/';

const excption_role = ['Correction'];

const assestList = ["agentAssets", "artifactAssets"];

checkSettingsSync(swarmUiRoot);

router.get('/', function (req, res) {
  res.render('index');
});

// ======== Miscellaneous =========================
const FALLBACK = {
  'en': 'en',
  'zh': 'zh',
  'zht': 'zh',
};

router.get("/agent_settings/lang/:lang", function (req, res) {
  console.log('--- line 68 ---');
  console.log(req.params.lang);
  let lng = req.params.lang;
  let lngToken = FALLBACK[req.params.lang];
  let responseResult = {};
  if (fs.existsSync(svrRoot + `/locales/${lng}.json`)) {
    let langObj = JSON.parse(fs.readFileSync(svrRoot + `/locales/${lng}.json`));
    // console.log(langObj.resources[lng].translation.api);
    langObj = langObj.resources[lngToken].translation.api.agent_settings
    console.log(langObj);
    responseResult = langObj;
  }

  res.json(responseResult);
});

router.get("/swarm_core/lang/:lang", function (req, res) {
  console.log(req.params.lang);
  let lng = req.params.lang;
  let lngToken = FALLBACK[req.params.lang];
  let responseResult = {};
  if (fs.existsSync(svrRoot + `/locales/${lng}.json`)) {
    let langObj = JSON.parse(fs.readFileSync(svrRoot + `/locales/${lng}.json`));
    langObj = langObj.resources[lngToken].translation.api.swarm_core_settings
    console.log(langObj);
    responseResult = langObj;
  }

  res.json(responseResult);
});

router.get("/swarm_core/lang/:lang/category/:category", function (req, res) {
  console.log(req.params.lang);
  console.log(req.params.category);
  let lng = req.params.lang;
  let lngToken = FALLBACK[req.params.lang];
  let responseResult = {};
  if (fs.existsSync(svrRoot + `/locales/${lng}.json`)) {
    let langObj = JSON.parse(fs.readFileSync(svrRoot + `/locales/${lng}.json`));
    langObj = langObj.resources[lngToken].translation[req.params.category].tmpl;
    console.log(langObj);
    responseResult = langObj;
  }

  res.json(responseResult);
});

router.get("/lang/:lang", function (req, res) {
  console.log(req.params.lang);
  let lng = req.params.lang;
  let responseResult = {};
  if (fs.existsSync(svrRoot + `/locales/${lng}.json`)) {
    let langObj = JSON.parse(fs.readFileSync(svrRoot + `/locales/${lng}.json`));
    console.log(langObj);
    responseResult = langObj;
  }

  res.json(responseResult);
});

/**
 * @swagger
 * /tooltips:
 *  get:
 *    tags: ["Misc."]
 *    produces:
 *      - application/json
 *    description: get available maps
 *    responses:
 *      '200':
 *        description: Tooltips got successfully
 *      '404':
 *        description: No tooltips are found
 *      '500':
 *        description: Server error
 */
router.get("/tooltips", function (req, res) {
  let responseResult = {};
  if (fs.existsSync(swarmUiRoot + 'settings.json')) {
    let settings = JSON.parse(fs.readFileSync(swarmUiRoot + 'settings.json'));
    console.log(settings);

    if (settings.hasOwnProperty('tooltips')) {
      responseResult = settings.tooltips;
    } else {
      // ------ default settings ------
      responseResult = { "role.roles.overview.title": "Role is a sequence of Behavior without any parameter.\nIn this page, user can define its sequence of behavior and required Capabilities for task execution. " };
    }
  }

  res.json(responseResult);
});

/**
 * @swagger
 * /assets:
 *  get:
 *    tags: ["Misc."]
 *    produces:
 *      - application/json
 *    description: get available maps
 *    responses:
 *      '200':
 *        description: Maps got successfully
 *      '404':
 *        description: No maps are found
 *      '500':
 *        description: Server error
 */
router.get("/assets", function (req, res) {
  let responseResult = {};
  if (fs.existsSync(swarmUiRoot + 'settings.json')) {
    let customAssetSettings = JSON.parse(fs.readFileSync(swarmUiRoot + 'settings.json'));

    assestList.forEach(function (asset_item, asset_index) {
      if (customAssetSettings.hasOwnProperty(asset_item)) {
        let customeSetting_dict = {};

        for (let [setting_key, setting_values] of Object.entries(customAssetSettings[asset_item])) {
          // update image path
          Object.keys(setting_values).forEach(function(key, index) {
            let default_image = "no_image-75x75.png";
            if (setting_values[key].length > 0){
              default_image = setting_values[key];
            }
            setting_values[key] = `/images/${default_image}`;
          });
          customeSetting_dict[setting_key.toLowerCase()] = setting_values;
        }

        Object.assign(responseResult, { [asset_item]: customeSetting_dict });
      }
    });
  }

  res.json(responseResult);
});

// ======== Maps ================================
/**
 * @swagger
 * /maps:
 *  get:
 *    tags: ["Maps"]
 *    produces:
 *      - application/json
 *    description: get available maps
 *    responses:
 *      '200':
 *        description: Maps got successfully
 *      '404':
 *        description: No maps are found
 *      '500':
 *        description: Server error
 */
router.get("/maps", function (req, res) {
  fs.readdir(mapDataPath, (err, files) => {
    var filtered = files.filter(el => path.extname(el) === '.png');
    res.json(filtered)
  });
});

// --- map image -----------
/**
 * @swagger
 * /maps/{map_name}/image:
 *  get:
 *    tags: ["Maps"]
 *    produces:
 *      - application/json
 *    description: get the map image
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Maps got successfully
 *      '404':
 *        description: No maps are found
 *      '500':
 *        description: Server error
 */
router.get('/maps/:filename/image', function (req, res) {
  const targetFile = mapDataPath + processFilename(req.params.filename) + '.png';
  var dimensions = sizeOf(targetFile);
  fs.readFile(targetFile, 'base64', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      var resData = {
        w: dimensions.width,
        h: dimensions.height,
        data: data
      };
      res.send(resData);
    }
  })
});

/**
 * @swagger
 * definitions:
 *   MapImage:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: string
 */

/**
 * @swagger
 * /maps/{map_name}/image:
 *   post:
 *     description: Create a map info
 *     tags: ["Maps"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: map_name
 *         in: path
 *         description: map meta-data
 *         required: true
 *         type: string
 *       - name: map_info
 *         description: map meta info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/MapInfo'
 *     responses:
 *       200:
 *         description: map meta info
 *         schema:
 *           $ref: '#/definitions/MapInfo'
 */
router.post('/maps/:filename/image', async function (req, res) {
  const targetFile = mapDataPath + req.params.filename + '.png';
  var base64Data = req.body.data.replace(/^data:image\/png;base64,/, "");
  console.log("--- get base 64 data ---");
  console.log(base64Data);
  fs.writeFile(targetFile, base64Data, 'base64', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  try {
    await axios.post('http://localhost:3000/testdb/addMapData', {
      mapImgNameArray: [req.params.filename]
    })
  } catch (error) {
    console.error(error);
  }
  res.sendStatus(200);
});

function checkThumbnailSync(_dir) {
  try {
    // --- directory is not created case ---
    if (!fs.existsSync(_dir)) {
      fs.mkdirSync(_dir);
      return;
    }
  } catch (err) {
    console.log(err)
  }
}

/**
 * @swagger
 * /maps/{map_name}/thumbnail:
 *  get:
 *    tags: ["Maps"]
 *    produces:
 *      - application/json
 *    description: get the map thumbnail
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Maps got successfully
 *      '404':
 *        description: No maps are found
 *      '500':
 *        description: Server error
 */
router.get('/map/:filename/thumbnail', async function (req, res) {
  const mapSource = mapDataPath + processFilename(req.params.filename) + '.png';
  checkThumbnailSync('./thumbnails');
  const mapThumbnail = path.join("./thumbnails/", req.params.filename + '.png');

  let isExist = fs.existsSync(mapSource);
  if (!isExist) {
    res.status(404).send({ errors: [{ "code": "no-file", "message": "the map is not found" }] });
  }

  isExist = fs.existsSync(mapThumbnail);
  if (!isExist) {
    try {
      const thumbnail = await resizeImg(fs.readFileSync(mapSource), {
        width: 128,
        height: 128
      });

      fs.writeFileSync(mapThumbnail, thumbnail);
    } catch (err) {
      console.error(err);
    }
  }

  fs.readFile(mapThumbnail, "base64", function (err, data) {
    if (err) {
      // console.log(err.message);
      res.status(500).send({ errors: [{ "code": "wrong-format", "message": "fail to read the map file" }] });
    } else {
      // console.log(data);
      res.status(200).send({ data: `data:image/png;base64,${data}` });
    }

  })

});

// --- map metadata --------
/**
 * @swagger
 * /maps/{map_name}/info:
 *  get:
 *    tags: ["Maps"]
 *    description: Get the map meta-data
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: The meta-data of the map is retrieved sucessfully
 */
router.get('/maps/:filename/info', function (req, res) {
  const targetFile = mapDataPath + processFilename(req.params.filename) + '.yaml';
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      res.send(data);
    }
  })
});

/**
 * @swagger
 * definitions:
 *   MapInfo:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: string
 */

/**
 * @swagger
 * /maps/{map_name}/info:
 *   post:
 *     description: Create a map info
 *     tags: ["Maps"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: map_name
 *         in: path
 *         description: map meta-data
 *         required: true
 *         type: string
 *       - name: map_info
 *         description: map meta info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/MapInfo'
 *     responses:
 *       200:
 *         description: map meta info
 *         schema:
 *           $ref: '#/definitions/MapInfo'
 */
router.post('/maps/:filename/info', function (req, res) {
  console.log('---map yaml post---');
  const targetFile = mapDataPath + req.params.filename + '.yaml';
  // console.log(targetFile);
  // console.log(`request body: ${req.body}`);
  // console.log(`request body: ${req.body.content}`);
  fs.writeFile(targetFile, req.body.content, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  res.sendStatus(200);
});

/**
 * @swagger
 * /maps/{map_name}/graph:
 *  get:
 *    tags: ["Maps"]
 *    description: Get the graph on map
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: The graph is retrieved sucessfully
 */
router.get('/maps/:filename/graph', function (req, res) {
  const targetFile = graphDataPath + processFilename(req.params.filename) + '.dot';
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    if (err) {
      res.send(err.message);
    } else {
      res.send(data);
    }
  });
});

/**
 * @swagger
 * definitions:
 *   MapGraph:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: string
 */

/**
 * @swagger
 * /maps/{map_name}/graph:
 *   post:
 *     description: Create a map graph
 *     tags: ["Maps"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: map_name
 *         in: path
 *         description: without extension
 *         required: true
 *         type: string
 *       - name: map_graph
 *         description: graph description(DOT format) on map
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/MapGraph'
 *     responses:
 *       200:
 *         description: map meta info
 *         schema:
 *           $ref: '#/definitions/MapGraph'
 */
router.post('/maps/:filename/graph', function (req, res) {
  console.log('--- graph post ---');
  const targetFile = graphDataPath + req.params.filename + '.dot';
  console.log(targetFile);
  console.log(`file content: ${req.body.content}`);
  fs.writeFile(targetFile, req.body.content, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  res.sendStatus(200);
});

// --- map cells -----------
/**
 * @swagger
 * /maps/{map_name}/cells:
 *  get:
 *    tags: ["Maps"]
 *    description: Get the cells on map
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: The cell is retrieved sucessfully
 */
router.get('/maps/:filename/cells', function (req, res) {
  // console.log('--- storage cells get ---');
  const targetFile = cellDataPath + processFilename(req.params.filename) + '.json';
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    if (err) {
      res.send(err.message);
    } else {
      res.send(data);
    }
  });
});

/**
 * @swagger
 * definitions:
 *   MapCells:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: string
 */

/**
 * @swagger
 * /maps/{map_name}/cells:
 *   post:
 *     description: Create a map cell layout
 *     tags: ["Maps"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: map_name
 *         in: path
 *         description: without extension
 *         required: true
 *         type: string
 *       - name: map_graph
 *         description: Cells description description on map
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/MapCells'
 *     responses:
 *       200:
 *         description: map meta info
 *         schema:
 *           $ref: '#/definitions/MapCells'
 */
router.post('/maps/:filename/cells', function (req, res) {
  console.log('--- storage cells post 2---');
  const targetFile = cellDataPath + req.params.filename + '.json';
  console.log(targetFile);
  console.log(`file content: ${req.body.content}`);
  console.log(`file type: ${typeof req.body.content}`);
  fs.writeFile(targetFile, req.body.content, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  res.sendStatus(200);
});

router.get('/swarm/functionTypes', function (req, res) {
  // console.log('--- storage cells get ---');
  const targetFile = functionTypesPath + 'function.json';
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    if (err) {
      res.send(err.message);
    } else {
      res.send(data);
    }
  });
});

router.post('/swarm/functionTypes', function (req, res) {
  // --- detect wether the directory exist ---
  try {
    if (!fs.existsSync(functionTypesPath)) {
      fs.mkdirSync(functionTypesPath);
    }
  } catch (err) {
    console.log(err);
  }
  const targetFile = functionTypesPath + 'function.json';
  // console.log(targetFile);
  // console.log(`file content: \n ${JSON.stringify(req.body)}`);
  fs.writeFileSync(targetFile, JSON.stringify(req.body), 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
  })
  res.sendStatus(200);
});

// --- map bundled files ---
/**
 * @swagger
 * /maps/{map_name}:
 *  delete:
 *    tags: ["Maps"]
 *    description: delete map
 *    parameters:
 *      - name: map_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: map deleted successfully
 *      '404':
 *        description: No maps are found
 *      '500':
 *        description: Server error
 */
router.delete('/maps/:filename', function (req, res) {
  const dir = mapDataPath + processFilename(req.params.filename);
  var targetFiles = ['.png', '.yaml', '.dot', '.json']
  targetFiles = targetFiles.map(fn => dir + fn);
  // console.log(targetFiles);

  targetFiles.forEach(function (filePath) {
    try {
      fs.unlinkSync(filePath);
    } catch (err) {
      console.log(err);
    }
  });
  res.sendStatus(200);
});

// --- Triton maps ---------
router.get("/triton/maps", function (req, res) {
  console.log('load triton map file on server side');
  fs.readdir(tritonMapPath, (err, files) => {
    // var filtered = files.filter(el => path.extname(el) === '.png');
    // res.json(filtered)
    res.json(files);
  });
});

// ======== Roles ==============================
/**
 * @swagger
 * /roles/behavior-tree/schema:
 *  get:
 *    tags: ["Roles"]
 *    description: Get the behavior tree schema
 *    responses:
 *      '200':
 *        description: The behavior-tree schema  is retrieved sucessfully
 */
router.get('/roles/behavior-tree/schema', function (req, res) {
  const targetFile = xsdDataPath + 'bt.xsd';
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      var jsonData = xmlConverter.xml2json(data, {
        compact: true
      });
      res.send(jsonData);
    }
  })
});

/**
 * @swagger
 * /roles/behaviors/schema:
 *  get:
 *    tags: ["Roles"]
 *    description: Get the behaviors schema
 *    responses:
 *      '200':
 *        description: The behaviors schema  is retrieved sucessfully
 */
router.get('/roles/behaviors/schema', function (req, res) {
  const targetFile = xsdDataPath + 'br.xsd';
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      var obj = xmlConverter.xml2json(data, {
        compact: true
      });
      // var jsonData = JSON.stringify(obj, null, 2);
      var jsonData = obj;
      res.send(jsonData);
      // res.send(data);
    }
  })
});

/**
 * @swagger
 * /roles/{role_name}/behaviors:
 *  get:
 *    tags: ["Roles"]
 *    description: Get the role behavior
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: The role behavior is retrieved sucessfully
 */
router.get('/roles/:filename/behaviors', function (req, res) {
  const targetFile = btDataPath + processFilename(req.params.filename) + '.xml';
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      var obj = xmlConverter.xml2json(data, {
        compact: false
      });
      var jsonData = obj;
      res.send(jsonData);
    }
  })
});

/**
 * @swagger
 * definitions:
 *   RoleBehaviors:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /roles/{role_name}/behaviors:
 *   post:
 *     description: Create role behaviors
 *     tags: ["Roles"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: role_name
 *         in: path
 *         description: without extension
 *         required: true
 *         type: string
 *       - name: role_behaviors
 *         description: the behaviors of the role
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/RoleBehaviors'
 *     responses:
 *       200:
 *         description: Role-Behehaviors got successfully
 *         schema:
 *           $ref: '#/definitions/RoleBehaviors'
 */
router.post('/roles/:filename/behaviors', function (req, res) {
  const targetFile = btDataPath + req.params.filename + '.xml';

  var xmlResult = xmlConverter.json2xml(req.body.content);
  xmlResult = formatter(xmlResult);
  console.log(xmlResult);

  fs.writeFile(targetFile, xmlResult, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  res.sendStatus(200);
});

/**
 * @swagger
 * /roles/{role_name}/behaviors:
 *  delete:
 *    tags: ["Roles"]
 *    description: delete role behaviors
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: intend to delete role behaviors (with .xml extension)
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Role-Behaviors deleted successfully
 *      '404':
 *        description: No Role-Behaviors are found
 *      '500':
 *        description: Server error
 */
router.delete('/roles/:filename/behaviors', function (req, res) {
  const targetFile = btDataPath + processFilename(req.params.filename) + '.xml';
  console.log(targetFile);
  fs.unlink(targetFile, function (error) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      res.sendStatus(200);
    }
  })
});

// --- Get Request Role File ---
/**
 * @swagger
 * /roles/{role_name}/capabilities:
 *  get:
 *    tags: ["Roles"]
 *    description: Get the capabilities of the role
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Role-Capabilities is retrieved sucessfully
 */
router.get('/roles/:filename/capabilities', function (req, res) {
  const targetFile = rlDataPath + processFilename(req.params.filename) + '.yaml';
  console.log(targetFile);
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    // console.log(data);
    if (err) {
      res.send(err.message);
    } else {
      var obj = yaml.load(data);
      var jsonData = JSON.stringify(obj, null, 2);
      res.send(jsonData);
    }
  });
});

/**
 * @swagger
 * definitions:
 *   RoleCapabilities:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /roles/{role_name}/capabilities:
 *   post:
 *     description: Create role capabilities
 *     tags: ["Roles"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: role_name
 *         in: path
 *         description: without extension
 *         required: true
 *         type: string
 *       - name: role_capabilities
 *         description: the capabilities of the role
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/RoleCapabilites'
 *     responses:
 *       200:
 *         description: Role-Capabilities is retrieved successfully
 *         schema:
 *           $ref: '#/definitions/RoleCapabilites'
 */
router.post('/roles/:filename/capabilities', function (req, res) {
  const targetFile = rlDataPath + req.params.filename;
  var yamlContent = req.body.content;
  console.log(yamlContent);
  fs.writeFile(targetFile, yamlContent, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
    }
    console.log('file saved');
  })
  res.sendStatus(200);
});

/**
 * @swagger
 * /roles/{role_name}/capabilities:
 *  delete:
 *    tags: ["Roles"]
 *    description: delete role capabilites
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Role-Capabilities is deleted successfully
 *      '404':
 *        description: No Role-Capabilities is found
 *      '500':
 *        description: Server error
 */
router.delete('/roles/:filename/capabilities', function (req, res) {
  const targetFile = rlDataPath + processFilename(req.params.filename);
  fs.unlink(targetFile, function (error) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      res.sendStatus(200);
    }
  })
});

/**
 * @swagger
 * /roles/mapping/{fleet_name}:
 *  get:
 *    tags: ["Roles"]
 *    description: Get roles mapping data by fleet
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Roles mapping data is retrieved successfully
 */
router.get('/roles/mapping/:filename', function (req, res) {
  const targetFile = fleetDataPath + processFilename(req.params.filename) + ".yaml";
  console.log(targetFile);
  var fleet = req.params.filename;
  var res_obj;
  var arr = [];

  try {
    var fleet_data = fs.readFileSync(targetFile, 'utf8');
    console.log(fleet_data)
    var obj = yaml.load(fleet_data);
    var roles = obj[fleet].roles;

    roles.forEach(function (roles_seq_item) {
      console.log(roles_seq_item)
      var path = btDataPath + roles_seq_item + ".xml";

      try {
        var fleet_xml_data = fs.readFileSync(path, 'utf8');
        var obj_str = xmlConverter.xml2json(fleet_xml_data, {
          compact: false
        });
        var obj_json = JSON.parse(obj_str);

        // if (roles_seq_item == 'test') {
        //   console.log('============= ==================')
        //   console.log(obj_json.elements[0].elements[0].elements[0].elements)
        //   console.log('--------------------------------')
        // }

        // 'role_name': roles_seq_item,
        //       'title_name': item_key,
        //       'title_content': title_content

        var check_val_dic = {};
        obj_json.elements[0].elements[0].elements[0].elements.forEach(function (seq_item, seq_index) {
          if (seq_item.name == "SetBlackboard") {
            check_val_dic[seq_item.attributes.output_key] = seq_item.attributes.value;
          }
        });

        obj_json.elements[0].elements[0].elements[0].elements.forEach(function (seq_item, seq_index) {
          var title_content = {};
          if (seq_item.name != "SetBlackboard") {
            if (seq_item.name == "Repeat") {
              seq_item.elements.forEach(function (sub_item, sub_index) {
                var title_sub_content = {};
                var need_add_to_arr = false;

                Object.values(sub_item.attributes).forEach(function (attr_k_item, attr_k_index) {
                  if (attr_k_item.includes('{')) {
                    var re_name = attr_k_item.replace('{', '').replace('}', '');
                    title_sub_content[re_name] = check_val_dic[re_name];
                    need_add_to_arr = true;
                  }
                });

                if (need_add_to_arr) {
                  if (seq_item.name == "Artifact") {
                    arr.push({
                      'role_name': roles_seq_item,
                      'title_name': seq_item.name,
                      'type': seq_item.attributes.type,
                      'service': seq_item.attributes.service,
                      'title_content': title_content
                    });
                  } else {
                    if (!excption_role.includes(seq_item.name)) {
                      let tiltle_name = seq_item.name;
                      if (tiltle_name == 'Docking') {
                        console.log(seq_item.attributes.dock)
                        if (seq_item.attributes.dock == 'false') {
                          tiltle_name = 'Undock';
                        } else if (seq_item.attributes.dock == 'true') {
                          tiltle_name = 'Dock';
                        }
                        console.log(tiltle_name)
                      }

                      arr.push({
                        'role_name': roles_seq_item,
                        'title_name': tiltle_name,
                        'title_content': title_content
                      });
                    }
                  };
                };
              });
            } else {
              var need_add_to_arr = false;

              if (typeof seq_item.attributes !== 'undefined') {
                Object.values(seq_item.attributes).forEach(function (attr_k_item, attr_k_index) {
                  if (attr_k_item.includes('{')) {
                    var re_name = attr_k_item.replace('{', '').replace('}', '');
                    title_content[re_name] = check_val_dic[re_name];
                    need_add_to_arr = true;
                  };
                });
              } else {
                if (!excption_role.includes(seq_item.name)) {
                  let tiltle_name = seq_item.name;
                  if (tiltle_name == 'Docking') {
                    console.log(seq_item.attributes.dock)
                    if (seq_item.attributes.dock == 'false') {
                      tiltle_name = 'Undock';
                    } else if (seq_item.attributes.dock == 'true') {
                      tiltle_name = 'Dock';
                    }
                    console.log(tiltle_name)
                  }

                  arr.push({
                    'role_name': roles_seq_item,
                    'title_name': tiltle_name,
                    'title_content': title_content
                  });
                }
              }

              if (need_add_to_arr) {
                if (seq_item.name == "Artifact") {
                  arr.push({
                    'role_name': roles_seq_item,
                    'title_name': seq_item.name,
                    'type': seq_item.attributes.type,
                    'service': seq_item.attributes.service,
                    'title_content': title_content
                  });
                } else {
                  if (!excption_role.includes(seq_item.name)) {
                    let tiltle_name = seq_item.name;
                    if (tiltle_name == 'Docking') {
                      console.log(seq_item.attributes.dock)
                      if (seq_item.attributes.dock == 'false') {
                        tiltle_name = 'Undock';
                      } else if (seq_item.attributes.dock == 'true') {
                        tiltle_name = 'Dock';
                      }
                      console.log(tiltle_name)
                    }

                    arr.push({
                      'role_name': roles_seq_item,
                      'title_name': tiltle_name,
                      'title_content': title_content
                    });
                  }
                };
              };
            };
          }
        });

        // var setvlackboard_dict = {};
        // for (let [item_key, item_val] of Object.entries(obj_json.root.BehaviorTree.Sequence)) {
        //   if (item_key == "SetBlackboard") {
        //     if (Array.isArray(item_val)) {
        //       item_val.forEach(function (SetBlackboard_item, SetBlackboard_index) {
        //         // console.log(item_key, SetBlackboard_item._attributes.output_key, SetBlackboard_item._attributes.value);
        //         setvlackboard_dict[item_key, SetBlackboard_item._attributes.output_key] = SetBlackboard_item._attributes.value;
        //       });
        //     } else {
        //       if (item_val._attributes != undefined) {
        //         var r = Object.values(item_val._attributes)
        //         var key = r[0];
        //         var val = r[1];
        //         setvlackboard_dict[key] = val;
        //       }
        //     };
        //   }
        // }

        // for (let [item_key, item_val] of Object.entries(obj_json.root.BehaviorTree.Sequence)) {
        //   if (item_key != "SetBlackboard") {
        //     // var order = 0;
        //     // var order_list = [];
        //     var title_content = {};
        //     var test_content = [];

        //     if (Array.isArray(item_val)) {
        //       item_val.forEach(function (t_item, t_index) {
        //         var a = Object.values(t_item._attributes)
        //         // console.log(a)
        //         a.forEach(function (item, t_index) {
        //           if (item[0] == '{') {
        //             var aa = item.replace('{', '').replace('}', '');
        //             var default_val = setvlackboard_dict[aa];
        //             if (!(aa in title_content)) {
        //               title_content[aa] = default_val;
        //             }
        //           }
        //         });
        //       });
        //     } else {
        //       if (item_val._attributes != undefined) {
        //         var r = Object.values(item_val._attributes)
        //         // console.log(r)
        //         r.forEach(function (item, t_index) {
        //           if (item[0] == '{') {
        //             var aa = item.replace('{', '').replace('}', '');
        //             var default_val = setvlackboard_dict[aa];
        //             if (!(aa in title_content)) {
        //               title_content[aa] = default_val;
        //             }
        //           }
        //         });
        //       }
        //     }
        //     arr.push({
        //       'role_name': roles_seq_item,
        //       'title_name': item_key,
        //       'title_content': title_content
        //     })
        //   }
        // };
      } catch (err) {
        console.log("Can not find target fleet xml" + err);
      };
      // console.log(arr)
    });
  } catch (err) {
    console.log("Can not find target fleet" + err);
  };
  res.status(200).send(arr);
});

/**
 * @swagger
 * /roles/roleParamList/{fleet_name}:
 *  get:
 *    tags: ["Roles"]
 *    description: Get roles mapping data by fleet
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Roles mapping data is retrieved successfully
 */
router.get('/roles/roleParamList/:filename', function (req, res) {
  const targetFile = fleetDataPath + processFilename(req.params.filename) + ".yaml";
  console.log(targetFile);
  var fleet = req.params.filename;
  var res_obj;
  var arr = [];
  var paramDict = {};

  try {
    var fleet_data = fs.readFileSync(targetFile, 'utf8');
    console.log(fleet_data)
    var obj = yaml.load(fleet_data);
    var roles = obj[fleet].roles;

    roles.forEach(function (roles_seq_item) {
      console.log(roles_seq_item)
      var path = btDataPath + roles_seq_item + ".xml";

      try {
        var fleet_xml_data = fs.readFileSync(path, 'utf8');
        var obj_str = xmlConverter.xml2json(fleet_xml_data, {
          compact: false
        });
        var obj_json = JSON.parse(obj_str);

        var check_val_dic = {};
        obj_json.elements[0].elements[0].elements[0].elements.forEach(function (seq_item, seq_index) {
          if (seq_item.name == "SetBlackboard") {
            check_val_dic[seq_item.attributes.output_key] = seq_item.attributes.value;
          }
        });

        obj_json.elements[0].elements[0].elements[0].elements.forEach(function (seq_item, seq_index) {
          var title_content = {};
          if (seq_item.name != "SetBlackboard") {
            if (seq_item.name == "Repeat") {
              seq_item.elements.forEach(function (sub_item, sub_index) {
                var title_sub_content = {};
                var need_add_to_arr = false;

                Object.values(sub_item.attributes).forEach(function (attr_k_item, attr_k_index) {
                  if (attr_k_item.includes('{')) {
                    var re_name = attr_k_item.replace('{', '').replace('}', '');
                    title_sub_content[re_name] = check_val_dic[re_name];
                    need_add_to_arr = true;
                  }
                });

                if (need_add_to_arr) {
                  if (seq_item.name == "Artifact" || seq_item.name == "DockingArtifact") {
                    arr.push({
                      'role_name': roles_seq_item,
                      'title_name': seq_item.name,
                      'type': seq_item.attributes.type,
                      'service': seq_item.attributes.service,
                      'title_content': title_content
                    });
                  } else {
                    if (!excption_role.includes(seq_item.name)) {
                      let tiltle_name = seq_item.name;
                      if (tiltle_name == 'Docking') {
                        console.log(seq_item.attributes.dock)
                        if (seq_item.attributes.dock == 'false') {
                          tiltle_name = 'Undock';
                        } else if (seq_item.attributes.dock == 'true') {
                          tiltle_name = 'Dock';
                        }
                        console.log(tiltle_name)
                      }

                      arr.push({
                        'role_name': roles_seq_item,
                        'title_name': tiltle_name,
                        'title_content': title_content
                      });
                    }
                  };
                };
              });
            } else {
              var need_add_to_arr = false;

              if (typeof seq_item.attributes !== 'undefined') {
                Object.values(seq_item.attributes).forEach(function (attr_k_item, attr_k_index) {
                  if (attr_k_item.includes('{')) {
                    var re_name = attr_k_item.replace('{', '').replace('}', '');
                    title_content[re_name] = check_val_dic[re_name];
                    need_add_to_arr = true;
                  };
                });
              } else {
                if (!excption_role.includes(seq_item.name)) {
                  let tiltle_name = seq_item.name;
                  if (tiltle_name == 'Docking') {
                    console.log(seq_item.attributes.dock)
                    if (seq_item.attributes.dock == 'false') {
                      tiltle_name = 'Undock';
                    } else if (seq_item.attributes.dock == 'true') {
                      tiltle_name = 'Dock';
                    }
                    console.log(tiltle_name)
                  }

                  arr.push({
                    'role_name': roles_seq_item,
                    'title_name': tiltle_name,
                    'title_content': title_content
                  });
                }
              }

              if (need_add_to_arr) {
                if (seq_item.name == "Artifact" || seq_item.name == "DockingArtifact") {
                  arr.push({
                    'role_name': roles_seq_item,
                    'title_name': seq_item.name,
                    'type': seq_item.attributes.type,
                    'service': seq_item.attributes.service,
                    'title_content': title_content
                  });
                } else {
                  if (!excption_role.includes(seq_item.name)) {
                    let tiltle_name = seq_item.name;
                    if (tiltle_name == 'Docking') {
                      console.log(seq_item.attributes.dock)
                      if (seq_item.attributes.dock == 'false') {
                        tiltle_name = 'Undock';
                      } else if (seq_item.attributes.dock == 'true') {
                        tiltle_name = 'Dock';
                      }
                      console.log(tiltle_name)
                    }

                    arr.push({
                      'role_name': roles_seq_item,
                      'title_name': tiltle_name,
                      'title_content': title_content
                    });
                  }
                };
              };
            };
          }
        });


      } catch (err) {
        console.log("Can not find target fleet xml" + err);
      };
      // console.log(arr)
    });

    arr.forEach((roleInfo) => {
      let roleName = roleInfo.role_name;
      let param_title_name = roleInfo.title_name;
      let paramData = {
        title_name: param_title_name,
        title_content: roleInfo.title_content
      }

      switch (param_title_name) {
        case 'Artifact':
          paramData.type = roleInfo.type;
          paramData.service = roleInfo.service;
          break;
        case 'Dock':
          paramData.title_content = {};
          break;
        case 'Undock':
          paramData.title_content = {};
          break;
        case 'DockingArtifact':
          paramData.type = roleInfo.type;
          paramData.service = roleInfo.service;
          break;

        default:
          break;
      }

      if (paramDict.hasOwnProperty(roleName)) {
        paramDict[roleName].push(paramData);
      } else {
        paramDict[roleName] = [paramData];
      }
    })
  } catch (err) {
    console.log("Can not find target fleet" + err);
  };
  res.status(200).send(paramDict);
});

/**
 * @swagger
 * /roles/{role_name}/chkFileNameExists:
 *  get:
 *    tags: ["Roles"]
 *    description: Check role file name is exists or not
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: role file name existence check successfully
 */
router.get('/roles/:filename/chkFileNameExists', function (req, res) {
  const check_filename = req.params.filename.toLowerCase();
  var fileNameDup = false;
  fs.readdirSync(rlDataPath).forEach(file => {
    if (path.extname(file) === '.yaml') {
      var fileName = file.replace('.yaml', '').toLowerCase();
      if (fileName === check_filename) {
        fileNameDup = true;
      }
    }
  });
  fs.readdirSync(btDataPath).forEach(file => {
    if (path.extname(file) === '.xml') {
      var fileName = file.replace('.xml', '').toLowerCase();
      if (fileName === check_filename) {
        fileNameDup = true;
      }
    }
  });
  res.send(fileNameDup);
});

/**
 * @swagger
 * /role_BT/{role_name}:
 *  get:
 *    tags: ["Roles"]
 *    description: Get the behavior of the role
 *    parameters:
 *      - name: role_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Role-Behaviors is retrieved sucessfully
 */
router.get('/role_BT/:filename', function (req, res) {
  const targetFile = btDataPath + processFilename(req.params.filename) + ".xml";
  var SetBlackboard_list = [];
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      var obj_str = xmlConverter.xml2json(data, {
        compact: true
      });
      var obj_json = JSON.parse(obj_str);
      console.log(obj_json.root.BehaviorTree.Sequence)
      for (let [item_key, item_val] of Object.entries(obj_json.root.BehaviorTree.Sequence)) {
        if (item_key == "SetBlackboard") {
          console.log(item_key, item_val);
          if (Array.isArray(item_val)) {
            item_val.forEach(function (SetBlackboard_item, SetBlackboard_index) {
              // console.log(item_key, SetBlackboard_item._attributes.output_key, SetBlackboard_item._attributes.value);
              SetBlackboard_list.push(SetBlackboard_item._attributes);
            });
          } else {
            SetBlackboard_list.push(item_val._attributes);
          }
        }
      }
      var jsonData = JSON.stringify(SetBlackboard_list, null, 2);
      res.send(jsonData);
    }
  })
});


// ======== Artifacts ===========================
router.get("/artifact/types", function (req, res) {
  var files = fs.readdirSync(artifactDataPath);
  // --- filter .json extension files ---
  files = files.filter(f => path.extname(f).toLowerCase() === '.json');
  // files = files.filter(f => f.charAt(0) !== '.'); // exclude hidden dot files
  // console.log(files);

  var targetResult = [];
  files.forEach((f) => {
    const filepath = artifactDataPath + f;

    var jsonData = fs.readFileSync(filepath, 'utf-8', (err, data) => {
      if (err) {
        console.error(err);
      }
    });

    console.log(jsonData);
    jsonData = JSON.parse(jsonData);
    // --- format validator. SHOULD contain services key ---
    if (!jsonData.hasOwnProperty("services")) { return; }
    targetResult.push(jsonData);
  });
  // console.log(targetResult);
  res.json(targetResult)
});


// ======== Fleets ==============================
/**
 * @swagger
 * /fleets:
 *  get:
 *    tags: ["Fleets"]
 *    description: get available fleets
 *    responses:
 *      '200':
 *        description: Fleets is retrieved successfully
 *      '404':
 *        description: No Fleets is found
 *      '500':
 *        description: Server error
 */
router.get("/fleets", function (req, res) {
  fs.readdir(fleetDataPath, (err, files) => {
    var filtered = files.filter(el => path.extname(el) === '.yaml');
    res.json(filtered)
  });
});

/**
 * @swagger
 * /fleets/{fleet_name}/settings:
 *  get:
 *    tags: ["Fleets"]
 *    description: Get the fleet settings
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Fleet-Settings is retrieved sucessfully
 */
router.get('/fleets/:filename/settings', function (req, res) {
  const targetFile = fleetDataPath + processFilename(req.params.filename) + '.yaml';
  console.log(targetFile);
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    // console.log(data);
    if (err) {
      res.send(err.message);
    } else {
      var obj = yaml.load(data);
      // var jsonData = JSON.stringify(obj, null, 2);
      // res.send(jsonData);
      res.send(obj);
    }
  });
});

/**
 * @swagger
 * definitions:
 *   FleetSettings:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /fleets/{fleet_name}/settings:
 *   post:
 *     description: Create fleet settings
 *     tags: ["Fleets"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: fleet_name
 *         in: path
 *         description: without extension
 *         required: true
 *         type: string
 *       - name: fleet_settings
 *         description: Cells description on map
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/FleetSettings'
 *     responses:
 *       200:
 *         description: Fleet-Settings is retrieved successfully
 *         schema:
 *           $ref: '#/definitions/FleetSettings'
 */
router.post('/fleets/:filename/settings', function (req, res) {
  var fleet_name = req.params.filename;
  var old_fleet_name = req.body.oldFilename;

  const targetFile = fleetDataPath + fleet_name + '.yaml';
  var oldTargetFile = "";
  if (old_fleet_name.length > 0) {
    oldTargetFile = fleetDataPath + old_fleet_name + '.yaml';
  }

  console.log(req.body.content)
  var obj = JSON.parse(req.body.content);
  var yamlContent = yaml.dump(obj);
  // console.log(yamlContent);
  fs.writeFile(targetFile, yamlContent, 'utf-8', function (err) {
    if (err) {
      res.send(err);
      console.log(err);
      return;
    }
    console.log(`${fleet_name}.yaml saved`);
  })

  if (oldTargetFile.length > 0 && fs.existsSync(oldTargetFile)) {
    fs.unlink(oldTargetFile, function (err) {
      if (err) {
        res.send(err);
        console.log(err);
        return;
      }
      console.log(`${old_fleet_name}.yaml deleted`);
    })
  }

  res.sendStatus(200);
});

/**
 * @swagger
 * /fleets/{fleet_name}/settings/artifacts:
 *  get:
 *    tags: ["Fleets"]
 *    description: Get the fleet settings artifacts
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Fleet settings artifacts is retrieved sucessfully
 */
router.get("/fleets/:fleet/settings/artifacts", function (req, res) {
  var fleet_name = req.params.fleet;
  const targetfleetconfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";

  try {
    var fleet_data = fs.readFileSync(targetfleetconfFile, 'utf8');
    console.log("=======test============")
    console.log(fleet_data)
    var obj = yaml.load(fleet_data);
    var artifacts = obj[fleet_name].artifacts;
    var art_dict_tmp = {};

    if (artifacts.length == 0) {
      art_dict_tmp["No Artifact"] = [];
      art_dict_tmp["No Artifact"].push("999");
    } else {
      if (artifacts.hasOwnProperty("agent")) {
        // agent
        artifacts.agent.forEach(function (agent_item, agent_index) {
          for (let [item_key, item_val] of Object.entries(agent_item)) {
            item_val.forEach(function (artifacts_item, artifacts_index) {
              var art_type = artifacts_item.split('@')[0];
              var art_id = artifacts_item.split('@')[1];

              if (art_dict_tmp[art_type] === undefined && typeof art_dict_tmp[art_type] === 'undefined') {
                art_dict_tmp[art_type] = [];
              }

              if (!art_dict_tmp[art_type].includes(art_id)) {
                art_dict_tmp[art_type].push(art_id);
              }
            });
          }
        });
      }

      if (artifacts.hasOwnProperty("external")) {
        // external
        artifacts.external.forEach(function (artifacts_item, artifacts_index) {
          var art_type = artifacts_item.split('@')[0];
          var art_id = artifacts_item.split('@')[1];

          if (art_dict_tmp[art_type] === undefined && typeof art_dict_tmp[art_type] === 'undefined') {
            art_dict_tmp[art_type] = [];
          }

          if (!art_dict_tmp[art_type].includes(art_id)) {
            art_dict_tmp[art_type].push(art_id);
          }
        });
      }
    }


    var jsonData = JSON.stringify(art_dict_tmp, null, 2);
    res.send(jsonData);
  } catch (err) {
    console.log("Can not find target fleet" + err);
  };
});

/**
 * @swagger
 * /fleets/{fleet_name}:
 *  delete:
 *    tags: ["Fleets"]
 *    description: delete fleet
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Fleet is deleted successfully
 *      '404':
 *        description: No Fleet is found
 *      '500':
 *        description: Server error
 */
router.delete('/fleets/:filename', function (req, res) {
  const targetFile = fleetDataPath + processFilename(req.params.filename);
  console.log(targetFile);
  fs.unlink(targetFile, function (error) {
    if (error) {
      console.error(error);
      res.send(error.message);
    } else {
      res.sendStatus(200);
    }
  })
});


// ======== Settings ===========================
function checkSettingsSync(_dir) {
  try {
    // --- directory is not created case ---
    if (!fs.existsSync(_dir)) {
      fs.mkdirSync(_dir);
      fs.copyFile('./settings.json', _dir + 'settings.json', (err) => {
        console.log(err);
      })
      return;
    }
    // --- settings file doesn't exit case ---
    if (!fs.existsSync(_dir + 'settings.json')) {
      fs.copyFile('./settings.json', _dir + 'settings.json', (err) => {
        console.log(err);
      })
      return;
    }
    // --- asset directory is not created case ---
    let assert_dir = `${_dir}/assets`;
    if (!fs.existsSync(assert_dir)) {
      fs.mkdirSync(assert_dir);

      // --- asset img check assect image ---
      checkDefaultAssetsImage(assert_dir);
      return;
    } else {
      // --- asset img check assect image ---
      checkDefaultAssetsImage(assert_dir);
    }

    // --- settings agentAssets not sync ---
    checkAssetsSetting(_dir, assestList);
  } catch (err) {
    console.log(err)
  }
}

function checkDefaultAssetsImage(custom_dir) {
  var files = fs.readdirSync(__dirname);
  // var files1 = fs.readdirSync('/snapshot/server/');
  let server_floder = path.join(__dirname, '../');
  console.log('===========================');
  console.log(files);
  console.log('============~~~~~~~~~~~~~~~~~~===============');
  // console.log(files1);
  console.log(server_floder);
  const defaut_assets_path = path.join(server_floder, 'public/dist/img/sprites');
  const regex = new RegExp('(.\.png)|(.\.jpeg)|(.\.jpg)');

  fs.readdirSync(defaut_assets_path).forEach(file => {
    if (!fs.existsSync(`${custom_dir}/${file}`) && regex.test(file)) {
      console.log(file);
      fs.copyFileSync(`${defaut_assets_path}/${file}`, `${custom_dir}/${file}`);
    } else {
      console.log('No need to add to asset: ' + file);
    }
  });
}

function checkAssetsSetting(_dir, assestList_) {
  if (fs.existsSync(_dir + 'settings.json')) {
    let customAssetSettings = JSON.parse(fs.readFileSync(_dir + 'settings.json'));
    let defaultAssetSettings = JSON.parse(fs.readFileSync('./settings.json'));
    let needUpdate = false;

    assestList_.forEach(function (asset_item, asset_index) {
      if (customAssetSettings.hasOwnProperty(asset_item)) {
        console.log(`custom_${asset_item}_Settings exist`);
        let defaultAssetSettingsList = Object.keys(defaultAssetSettings[asset_item]);
        let customAssetSettingsList = Object.keys(customAssetSettings[asset_item]);

        defaultAssetSettingsList.forEach(function (item, index) {
          if (!customAssetSettingsList.includes(item)) {
            let new_update_val = defaultAssetSettings[asset_item][item];
            Object.assign(customAssetSettings[asset_item], { [item]: new_update_val });
            needUpdate = true;
            console.log(`default_${asset_item}_Settings has new config`);
          }
        });
      } else {
        customAssetSettings[asset_item] = defaultAssetSettings[asset_item];
        needUpdate = true;
        console.log(`custom_${asset_item}_Settings not exist`);
      }
    });

    if (needUpdate) {
      console.log("Assets update");
      let data = JSON.stringify(customAssetSettings, null, 4);
      console.log(data);
      fs.writeFileSync(_dir + 'settings.json', data);
    } else {
      console.log("Assets no need update");
    }
  }
}


/**
 * @swagger
 * /settings/all:
 *  get:
 *    tags: ["Settings"]
 *    description: Get all settings
 *    responses:
 *      '200':
 *        description: Settings are retrieved sucessfully
 */
router.get("/settings/all", function (req, res) {
  checkSettingsSync(swarmUiRoot);

  const targetFile = swarmUiRoot + 'settings.json';
  console.log('--- settings ---');
  fs.readFile(targetFile, 'utf-8', (err, data) => {
    console.log(data);
    if (err) {
      res.send(err.message);
    } else {
      var jsonData = JSON.parse(data);
      res.json(jsonData);
    }
  });
});

/**
 * @swagger
 * definitions:
 *   CellTypeSettings:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /settings/cell-types:
 *   post:
 *     description: Create fleet settings
 *     tags: ["Settings"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: fleet_name
 *         in: path
 *         description: the fleet name
 *         required: true
 *         type: string
 *       - name: cell-types_settings
 *         description: Cells description on map
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/CellTypeSettings'
 *     responses:
 *       200:
 *         description: Cell-types is updated
 *         schema:
 *           $ref: '#/definitions/CellTypeSettings'
 */
router.post('/settings/cell-types', function (req, res) {
  const targetFile = swarmUiRoot + 'settings.json';
  // console.log(targetFile);
  // console.log(req.body.content);
  var obj = req.body.content;
  var jsonContent = JSON.stringify(obj, null, 4);
  console.log(jsonContent);

  fs.writeFile(targetFile, jsonContent, 'utf-8', function (err) {
    if (err) {
      res.send('cell types save fail');
    } else {
      res.send('cell types saved');
    }
  })
});

router.put('/settings/cell-types', function (req, res) {
  const targetFile = swarmUiRoot + 'settings.json';
  // console.log(targetFile);
  // console.log(req.body.content);
  // var obj = req.body.content;
  var obj = req.body;

  var settingsData = fs.readFileSync(targetFile, 'utf8');
  settingsData = JSON.parse(settingsData);
  var key = Object.keys(obj)[0];
  console.log(key);
  settingsData[key] = obj[key];

  var jsonContent = JSON.stringify(settingsData);

  fs.writeFile(targetFile, jsonContent, 'utf-8', function (err) {
    if (err) {
      res.send('cell types save fail');
    } else {
      res.send('cell types saved');
    }
  })
});

router.get('/settings/notify-groups', function (req, res) {
  var resVal = logManager.getNotifyGroups();

  // if (!bResVal) {
  //   res.send({ status_code: 400, message: "Failed to apply Filter Rules!" });
  //   return;
  // }
  res.send({ status_code: 200, message: "Fetch notify gropus successfully!", data: resVal });
});

router.put('/settings/notify-groups', function (req, res) {
  var obj = req.body;
  console.log(' --- notify groups is got ---');
  var strObj = JSON.stringify(obj);
  console.log(strObj);
  // obj = JSON.parse(obj);
  var resVal = logManager.setNotifyGroups(obj);


  // if (!bResVal) {
  //   res.send({ status_code: 400, message: "Failed to apply Filter Rules!" });
  //   return;
  // }
  res.send({ status_code: 200, message: "Set notify gropus successfully!" });
});

/**
 * @swagger
 * /settings/notifications:
 *  get:
 *    tags: ["Settings"]
 *    description: Get the notification settings
 *    parameters:
 *    responses:
 *      '200':
 *        description: Notification settings are retrieved successfully
 */
router.get('/settings/notifications', function (req, res) {
  // 1. read all the notification settings from database
  // 2. transform the data from sql to json
  // 3. send the response
  // var obj = req.body;
  // console.log(obj);

  // var bResVal = logManager.setRules(obj);

  // if (!bResVal) {
  //   res.send({ status_code: 400, message: "Failed to apply Filter Rules!" });
  //   return;
  // }
  // res.send({ status_code: 200, message: "Filter Rules are applied successfully!" });
});

/**
 * @swagger
 * definitions:
 *   UISettingsContent:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 *         required:
 *           - theme
 *           - fontSize
 *           - fontStyle
 *         properties:
 *           theme:
 *             type: string
 *           fontSize:
 *             type: string
 *           fontStyle:
 *             type: string
 */

/**
 * @swagger
 * /settings/settings-ui:
 *  post:
 *    description: Add UI settings to settings.json
 *    parameters:
 *      - name: settings
 *        description: UI settings content
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/UISettingsContent'
 *    tags: ["Settings"]
 *    responses:
 *      '200':
 *        description: UI settings saved
 */
router.post('/settings/settings-ui', function (req, res) {
  const targetFile = swarmUiRoot + 'settings.json';
  var obj = req.body.content;
  var data = fs.readFileSync(targetFile);
  var json = JSON.parse(data);
  json['settings.UI'] = obj;
  var jsonContent = JSON.stringify(json, null, 4);
  console.log(jsonContent)

  fs.writeFile(targetFile, jsonContent, 'utf-8', function (err) {
    if (err) {
      res.send('UI settings save fail');
    } else {
      res.send('UI settings saved');
    }
  })
});

/**
 * @swagger
 * definitions:
 *   GetUploadStatusContent:
 *     type: object
 *     required:
 *       - fileNameArray
 *     properties:
 *       fileNameArray:
 *         type: array
 *         items:
 *           type: string
 */

/**
 * @swagger
 * /sw-update/getUploadStatus:
 *  post:
 *    description: Check upload package status
 *    parameters:
 *      - name: fileNameArray
 *        description: files name list
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/GetUploadStatusContent'
 *    tags: ["Software update"]
 *    responses:
 *      '200':
 *        description: upload status got
 */
router.post('/sw-update/getUploadStatus', function (req, res) {
  const fileNameArray = req.body.fileNameArray;
  const uploadPath = process.cwd() + '/upload-zips/';
  var allFilesExist = false;
  fileNameArray.forEach(name => {
    const uploadFile = uploadPath + processFilename(name) + ".tar.gz";
    allFilesExist = fs.existsSync(uploadFile);
  });

  res.send({
    exists: allFilesExist,
    path: uploadPath
  });
});

/**
 * @swagger
 * definitions:
 *   removeUploadPkgContent:
 *     type: object
 *     required:
 *       - filename
 *     properties:
 *       filename:
 *         type: string
 */

/**
 * @swagger
 * /sw-update/removeUploadPkg:
 *  post:
 *    description: Remove upload package
 *    parameters:
 *      - name: filename
 *        description: package file name
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/removeUploadPkgContent'
 *    tags: ["Software update"]
 *    responses:
 *      '200':
 *        description: upload package removed
 */
router.post('/sw-update/removeUploadPkg', function (req, res) {
  const fileName = req.body.filename;
  const rmPath = process.cwd() + '/upload-zips/' + processFilename(fileName) + ".tar.gz";
  fs.unlink(rmPath, (err) => {
    if (err) {
      console.error(err);
      return;
    }
    console.log(`${fileName} removed.`);
  });
  res.sendStatus(200);
});

// ======== Wrapper SDK ==============================

var exec = require('child_process').exec;
const execSync = require('child_process').execSync;

function execute(command, callback) {
  exec(command, function (error, stdout, stderr) { callback(stdout); });
}

/**
 * @swagger
 * definitions:
 *   SdkCli:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /wrapper/sdk/cli:
 *   post:
 *     description: SDK Cli tool
 *     tags: ["Wrapper SDK"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: cell-types_settings
 *         description: Cells description on map
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/SdkCli'
 *     responses:
 *       200:
 *         description: SDK Cli is executed
 *         schema:
 *           $ref: '#/definitions/SdkCli'
 */
router.post('/wrapper/sdk/cli', function (req, res) {

  var cmd = req.body.cmd;
  var subcmd = req.body.subcmd;
  var args = req.body.args;

  exec(`python3 ${servicePath}artifact_sdk_cli.py ${cmd} ${subcmd} ${args}`, (error, stdout, stderr) => {
    if (error) {
      console.log(`error: ${error.message}`);
      res.send(error);
      return;
    }
    if (stderr) {
      console.log(`stderr: ${stderr}`);
      res.send(stderr);
      return;
    }
    console.log(`stdout: ${stdout}`);
    res.send(stdout);
  });

});


// ======== Operations ===========================
/**
 * @swagger
 * /operations/{fleet_name}:
 *  get:
 *    tags: ["Operations"]
 *    description: Get the operations of the fleet
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Fleet operations are retrieved successfully
 */
router.get("/operations/:fleetname", function (req, res) {
  const fleet_name = req.params.fleetname;
  fs.readdir(operationDataPath, (err, files) => {
    var filtered = files.filter(el => path.extname(el) === '.json' &&
      (path.basename(el, '.json').substring(0, path.basename(el, '.json').indexOf("-task-")) === fleet_name ||
        path.basename(el, '.json').substring(0, path.basename(el, '.json').indexOf("-flow-")) === fleet_name));
    res.json(filtered)
  });
});

/**
 * @swagger
 * /operations/{fleet_name}/tasks:
 *  get:
 *    tags: ["Operations"]
 *    description: Get all of the files content of the fleet tasks
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Files content of the fleet tasks are retrieved successfully
 */
router.get("/operations/:fleet/tasks", function (req, res) {
  var fleet_name = req.params.fleet;
  var fileArray = [];
  var jsonFileArray = [];
  fs.readdirSync(operationDataPath).forEach(file => {
    var fileNameArray = file.split("-");
    if (fileNameArray.length >= 3) {
      if (fileNameArray[0] === fleet_name && fileNameArray[1] === 'task' && path.extname(file) === '.json') {
        fileArray.push(file);
      }
    }
  });

  fileArray.forEach(fileName => {
    console.log(operationDataPath + fileName);
    var data = fs.readFileSync(operationDataPath + fileName, 'utf8');
    jsonFileArray.push(JSON.parse(data));
  });
  res.send(jsonFileArray);
});

/**
 * @swagger
 * /operations/task/{task_file_name}:
 *  get:
 *    tags: ["Operations"]
 *    description: Get the task file content
 *    parameters:
 *      - name: task_file_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Task file content is retrieved successfully
 */
router.get("/operations/task/:filename", function (req, res) {
  const targetFile = operationDataPath + processFilename(req.params.filename) + ".json";
  fs.readFile(targetFile, 'utf-8', function (error, data) {
    if (error) {
      console.error(error);
      res.status(404).send("No Data");
    } else {
      var jsonData = JSON.parse(data);
      res.send(jsonData);
    }
  })
});

/**
 * @swagger
 * definitions:
 *   TaskFileContent:
 *     type: object
 *     required:
 *       - fleetname
 *       - taskname
 *       - content
 *     properties:
 *       fleetname:
 *         type: string
 *       taskname:
 *         type: string
 *       content:
 *         type: object
 */

router.post('/operations/flattenPlanArtifactConf', function (req, res) {
  const artifact_conf = req.body.artifact_conf;
  var flatten_obj = flatten(artifact_conf);

  res.send(flatten_obj);
});

router.post('/operations/unflattenPlanArtifactConf', function (req, res) {
  const artifact_conf = req.body.artifact_conf;
  var unflatten_obj = unflatten(artifact_conf);

  res.send(unflatten_obj);
});

router.post('/fleet/getFlatten', function (req, res, next) {
  console.log(req.body.flatten_object)
  var flattenObject = JSON.parse(req.body.flatten_object);
  console.log(flattenObject)
  res.status(200).send(JSON.stringify(unflatten(flattenObject)));
});

/**
 * @swagger
 * /operations/task/{task_file_name}:
 *   post:
 *     description: Create task file
 *     tags: ["Operations"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: task_file_name
 *         in: path
 *         description: task file name
 *         required: true
 *         type: string
 *       - name: task content
 *         description: task file content
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/TaskFileContent'
 *     responses:
 *       200:
 *         description: Task file is created
 *         schema:
 *           $ref: '#/definitions/TaskFileContent'
 */
router.post('/operations/task/:filename', function (req, res) {
  const fleet_name = req.body.fleetname;
  const task_name = req.body.taskname;
  const targetFleetConfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";
  const targetFile = operationDataPath + processFilename(req.params.filename) + ".json";

  var confData = fs.readFileSync(targetFleetConfFile);
  var yamlData = yaml.load(confData);
  var fleet_conf = yamlData[fleet_name];
  if (fleet_conf.hasOwnProperty("tasks")) {
    if (!fleet_conf["tasks"].includes(task_name)) {
      fleet_conf.tasks.push(task_name);
    }
  } else {
    fleet_conf["tasks"] = [task_name];
  }
  var yamlstr = yaml.dump(yamlData);
  fs.writeFileSync(targetFleetConfFile, yamlstr, 'utf-8');

  var obj = req.body.content;
  var jsonContent = JSON.stringify(obj, null, 4);
  fs.writeFile(targetFile, jsonContent, 'utf-8', function (err) {
    if (err) {
      res.send('Task save fail');
    } else {
      res.send('Task saved');
    }
  })
});

/**
 * @swagger
 * /operations/task/{task_file_name}:
 *  delete:
 *    tags: ["Operations"]
 *    description: Delete task file
 *    parameters:
 *      - name: task_file_name
 *        in: path
 *        description: task file name
 *        required: true
 *        type: string
 *      - name: task_info
 *        in: body
 *        description: Task info
 *        schema:
 *          type: object
 *          required:
 *            - fleetname
 *            - taskname
 *          properties:
 *            fleetname:
 *              type: string
 *            taskname:
 *              type: string
 *    responses:
 *      '200':
 *        description: Task file deleted successfully
 */
router.delete('/operations/task/:filename', function (req, res) {
  var fleet_name = req.body.fleetname;
  var task_name = req.body.taskname;
  console.log("task: " + task_name);
  const targetFleetConfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";
  const targetFile = operationDataPath + processFilename(req.params.filename) + ".json";

  var confData = fs.readFileSync(targetFleetConfFile);
  var yamlData = yaml.load(confData);
  var fleet_conf = yamlData[fleet_name];
  if (fleet_conf.hasOwnProperty("tasks")) {
    fleet_conf.tasks = fleet_conf.tasks.filter(item => item !== task_name);
    if (fleet_conf.tasks.length == 0) {
      delete fleet_conf.tasks;
    }
    var yamlstr = yaml.dump(yamlData);
    fs.writeFileSync(targetFleetConfFile, yamlstr, 'utf-8');
  }
  fs.unlink(targetFile, function (error) {
    if (error) {
      console.error(error);
      res.send("Task deleted error");
    } else {
      res.send("Task deleted");
    }
  })
});

/**
 * @swagger
 * /operations/{fleet_name}/flows:
 *  get:
 *    tags: ["Operations"]
 *    description: Get all of the files content of the fleet flows
 *    parameters:
 *      - name: fleet_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Files content of the fleet flows are retrieved successfully
 */
router.get("/operations/:fleet/flows", function (req, res) {
  var fleet_name = req.params.fleet;

  var jsonFileArray = [];

  const targetFile_planner = operationDataPath;
  const targetfleetconfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";

  // var confData = fs.readFileSync(targetfleetconfFile);
  let confData;
  if (fs.existsSync(targetfleetconfFile)) {
    confData = fs.readFileSync(targetfleetconfFile);
  } else {
    console.log(3);
    res.status(404).send({ error: `${targetfleetconfFile} is not found` });
    return;
  }

  var yamlData = yaml.load(confData);
  var fleet_conf = yamlData[fleet_name];

  if (fleet_conf.hasOwnProperty("flows")) {
    fleet_conf.flows.forEach(fileName => {
      // let data = fs.readFileSync(targetFile_planner + `${fleet_name}-flow-${fileName}.json`, 'utf8');
      let flowFilename = targetFile_planner + `${fleet_name}-flow-${fileName}.json`;
      let data;
      if (fs.existsSync(flowFilename)) {
        data = fs.readFileSync(flowFilename, 'utf8');
      } else {
        console.log(3);
        res.status(404).send({ error: `${flowFilename} is not found` });
        return;
      }

      var jsonData = JSON.parse(data);
      jsonFileArray.push({
        "Event": jsonData.event_name === undefined ? jsonData.flow_type : jsonData.event_name,
        "flow_name": jsonData.flow_name
      });
    });
  } else {
    console.log(3)
    // No flow
  }

  res.send(jsonFileArray);
});

/**
 * @swagger
 * /operations/flow/{flow_file_name}:
 *  get:
 *    tags: ["Operations"]
 *    description: Get the flow file content
 *    parameters:
 *      - name: flow_file_name
 *        in: path
 *        description: without extension
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Flow file content is retrieved successfully
 */
router.get("/operations/flow/:filename", function (req, res) {
  const targetFile = operationDataPath + processFilename(req.params.filename) + ".json";
  try {
    if (fs.existsSync(targetFile)) {
      //file exists
      var data = fs.readFileSync(targetFile, 'utf8');
      res.status(200).send(data);
      // fs.readFile(targetFile, 'utf-8', function (error, data) {
      //   if (error) {
      //     res.status(500).send('File load fail');
      //   } else {

      //   }
      // })
    } else {
      res.status(404).send('File check done no file.');
    }
  } catch (err) {
    res.status(500).send('File check fail');
  }
});

/**
 * @swagger
 * /operations/flow/{flow_name}:
 *  delete:
 *    tags: ["Operations"]
 *    description: Delete flow and flow UI file
 *    parameters:
 *      - name: flow_name
 *        in: path
 *        description: flow name
 *        required: true
 *        type: string
 *      - name: fleet_name
 *        in: body
 *        description: fleet name
 *        schema:
 *          type: object
 *          required:
 *            - fleetname
 *          properties:
 *            fleetname:
 *              type: string
 *    responses:
 *      '200':
 *        description: Flow file and flow UI file deleted successfully
 */
router.delete('/operations/flow/:flowname', function (req, res) {
  var fleet_name = req.body.fleetname;
  var flow_name = req.params.flowname;
  var flowui_file_name = `${fleet_name}-flowui-${flow_name}`;
  var flow_file_name = `${fleet_name}-flow-${flow_name}`;

  const targetFile = operationDataPath + processFilename(flowui_file_name) + ".json";
  const targetFile_plan = operationDataPath + processFilename(flow_file_name) + ".json";
  const targetfleetconfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";

  var confData = fs.readFileSync(targetfleetconfFile);
  var yamlData = yaml.load(confData);
  var fleet_conf = yamlData[fleet_name];
  fleet_conf.flows = fleet_conf.flows.filter(item => item !== flow_name)

  var yamlstr = yaml.dump(yamlData);
  fs.writeFileSync(targetfleetconfFile, yamlstr, 'utf-8');

  fs.unlink(targetFile, function (error) {
    if (error) {
      console.error(error);
      // res.send("Flow deleted error " + error.message);
    } else {
      // res.send("Flow deleted");
    }
  })

  fs.unlink(targetFile_plan, function (error) {
    if (error) {
      console.error(error);
      res.send("Flow deleted error");
    } else {
      res.send("Flow deleted");
    }
  })
});

/**
 * @swagger
 * definitions:
 *   FlowUIFileContent:
 *     type: object
 *     required:
 *       - content
 *     properties:
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /operations/flowui/{flowui_file_name}:
 *   post:
 *     description: Create flow UI file
 *     tags: ["Operations"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: flowui_file_name
 *         in: path
 *         description: flow UI file name
 *         required: true
 *         type: string
 *       - name: flowui_file_content
 *         description: flow UI file content
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/FlowUIFileContent'
 *     responses:
 *       200:
 *         description: Flow UI file is created
 *         schema:
 *           $ref: '#/definitions/FlowUIFileContent'
 */
router.post('/operations/flowui/:filename', function (req, res) {
  const targetFile = operationDataPath + processFilename(req.params.filename) + ".json";
  console.log(targetFile);
  console.log(req.body.content);
  var obj = req.body.content;
  // var jsonContent = JSON.stringify(obj, null, 4);
  // console.log(jsonContent);
  fs.writeFile(targetFile, obj, 'utf-8', function (err) {
    if (err) {
      res.send('Flow save fail');
    } else {
      res.send('Flow saved');
    }
    // console.log(err);
  })
});

/**
 * @swagger
 * definitions:
 *   FlowFileContent:
 *     type: object
 *     required:
 *       - fleetname
 *       - flowname
 *       - content
 *     properties:
 *       fleetname:
 *         type: string
 *       flowname:
 *         type: string
 *       content:
 *         type: object
 */

/**
 * @swagger
 * /operations/flow/{flow_file_name}:
 *   post:
 *     description: Create flow file
 *     tags: ["Operations"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: flow_file_name
 *         in: path
 *         description: flow file name
 *         required: true
 *         type: string
 *       - name: flow_file_info
 *         description: flow file info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/FlowFileContent'
 *     responses:
 *       200:
 *         description: Flow file is created
 *         schema:
 *           $ref: '#/definitions/FlowFileContent'
 */
router.post('/operations/flow/:filename', function (req, res) {
  var fleet_name = req.body.fleetname;
  var file_name = req.params.filename;
  var flow_name = req.body.flowname;

  const targetFile = operationDataPath + processFilename(file_name) + ".json";
  const targetfleetconfFile = fleetDataPath + processFilename(fleet_name) + ".yaml";

  console.log(targetFile);
  console.log(req.body.content);
  var obj = req.body.content;
  // var jsonContent = JSON.stringify(obj, null, 4);

  var confData = fs.readFileSync(targetfleetconfFile);
  var yamlData = yaml.load(confData);
  var fleet_conf = yamlData[fleet_name];
  if (fleet_conf.hasOwnProperty("flows")) {
    if (fleet_conf["flows"].includes(flow_name)) {
      // Flow exist
    } else {
      fleet_conf.flows.push(flow_name);
    }
  } else {
    fleet_conf["flows"] = [flow_name];
  }

  var yamlstr = yaml.dump(yamlData);
  fs.writeFileSync(targetfleetconfFile, yamlstr, 'utf-8');
  fs.writeFile(targetFile, obj, 'utf-8', function (err) {
    if (err) {
      res.send('Flow planner save fail');
    } else {
      res.send('Flow planner saved');
    }
    // console.log(err);
  })
});

/**
 * @swagger
 * definitions:
 *   CheckOperationNameInfo:
 *     type: object
 *     required:
 *       - eventType
 *       - eventName
 *     properties:
 *       eventType:
 *         type: string
 *       eventName:
 *         type: string
 */

/**
 * @swagger
 * /operations/chkFileNameExists:
 *   post:
 *     description: Check operation file name is exists or not
 *     tags: ["Operations"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: event_info
 *         description: event info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/CheckOperationNameInfo'
 *     responses:
 *       200:
 *         description: Operation file name existence check successfully
 *         schema:
 *           $ref: '#/definitions/CheckOperationNameInfo'
 */
router.post("/operations/chkFileNameExists", function (req, res) {
  var event_type = req.body.eventType;
  var event_name = req.body.eventName;
  var fileNameDup = false;
  var dupName = "";
  var jsonObj = {};
  fs.readdirSync(operationDataPath).forEach(file => {
    if (path.extname(file) === '.json') {
      var fileName = file.replace('.json', '');
      if (fileName.split('-')[1] === event_type && fileName.split('-')[2] === event_name) {
        fileNameDup = true;
        dupName = fileName;
      }
    }
  });
  jsonObj.fileExists = fileNameDup;
  jsonObj.filename = dupName;
  res.json(jsonObj);
});

/**
 * @swagger
 * definitions:
 *   CheckRoleInUsedOperationInfo:
 *     type: object
 *     required:
 *       - fileNameArray
 *       - deleteRoleArray
 *     properties:
 *       fileNameArray:
 *         type: array
 *         items:
 *           type: string
 *       deleteRoleArray:
 *         type: array
 *         items:
 *           type: string
 */

/**
 * @swagger
 * /operations/chkRoleInUsed:
 *   post:
 *     description: Check operation files including roles that need to be deleted
 *     tags: ["Operations"]
 *     produces:
 *       - application/json
 *     parameters:
 *       - name: check_info
 *         description: check info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/CheckRoleInUsedOperationInfo'
 *     responses:
 *       200:
 *         description: Operation files including roles check successfully
 *         schema:
 *           $ref: '#/definitions/CheckRoleInUsedOperationInfo'
 */
router.post("/operations/chkRoleInUsed", function (req, res) {
  var file_name_arr = req.body.fileNameArray;
  var targetRoleName = req.body.deleteRoleArray;
  var deleteFileArray = [];

  // if fileNameArray is [] will not send the key
  if (typeof file_name_arr === 'undefined') {
    res.json(deleteFileArray);
    return;
  }
  // check xxx-flow-xxx.json file
  file_name_arr.forEach(fileName => {
    const targetOperationFile = operationDataPath + processFilename(fileName);
    var fileData = fs.readFileSync(targetOperationFile);
    var jsonData = JSON.parse(fileData);
    if (jsonData.hasOwnProperty("flow_name")) {
      jsonData.flow_data.forEach(dataObj => {
        if (deleteFileArray.includes(fileName)) return;
        if (dataObj.role_name === targetRoleName) {
          deleteFileArray.push(fileName);
          return;
        }
      });
    } else {
      if (targetRoleName.includes(jsonData.task_info.role.role_value)) {
        deleteFileArray.push(fileName);
      }
    }
  });
  // check xxx-flowui-xxx.json file
  file_name_arr.forEach(fileName => {
    const targetOperationUIFile = operationDataPath + processFilename(fileName).replace('-flow-', '-flowui-');
    const fileData = fs.readFileSync(targetOperationUIFile);
    const jsonData = JSON.parse(fileData);
    const flowUiFileStr = JSON.stringify(jsonData);

    const regexRoleName = new RegExp(`${targetRoleName}`, 'g');

    if (regexRoleName.test(flowUiFileStr)) {
      if (deleteFileArray.includes(fileName)){
        return;
      } else {
        deleteFileArray.push(fileName);
      }
    }
  });
  res.json(deleteFileArray);
});

// ------ logs ------------
/**
 * @swagger
 * /log/exportDebugLog:
 *  get:
 *    tags: ["Logs"]
 *    description: Export debug log
 *    responses:
 *      '200':
 *        description: Export debug log successfully
 */
router.get('/log/exportDebugLog', function (req, res) {
  cleanDebugLogs();
  var exists = fs.existsSync(debugLogPath);
  var stats = exists && fs.statSync(debugLogPath);
  var isDirectory = exists && stats.isDirectory();
  if (!isDirectory) {
    res.send({ errors: "log directory not found." });
    return;
  }

  const toPath = process.cwd() + '/tmp';
  execSync(`mkdir -p ${toPath}`);
  try {
    execSync(`sudo cp -r ${debugLogPath}/* ${toPath}`);
    console.log(`copy to path: ${toPath}`);
  } catch (error) {
    console.log(`copy to ${toPath} error: ${error.message}`);
    res.send({ errors: "copy log to temp folder error." });
  }

  const logTarFileName = `swr-debug-log-${Date.now()}.tar.gz`;
  // exec(`tar -C /var/log -czvf ${logTarFileName} farobot`, (error, stdout, stderr) => {
  exec(`sudo tar -C ${process.cwd()} -czvf ${logTarFileName} tmp`, (error, stdout, stderr) => {
    if (error) {
      console.log(`error: ${error.message}`);
      res.send({ errors: error });
      return;
    }
    if (stderr) {
      console.log(`stderr: ${stderr}`);
      res.send({ errors: stderr });
      return;
    }
    console.log(`stdout: ${stdout}`);
  });
  res.sendStatus(200);
});

/**
 * @swagger
 * /log/exportDebugLogProgress:
 *  get:
 *    tags: ["Logs"]
 *    description: Calculate debug log export progress
 *    responses:
 *      '200':
 *        description: Debug log progress retrieved successfully
 */
router.get('/log/exportDebugLogProgress', function (req, res) {
  const srcPath = debugLogPath;
  const destPath = process.cwd() + '/tmp';
  const tarFileName = getLogTarFileName();
  // console.log(tarFileName);
  const tarFilePath = process.cwd() + '/' + tarFileName;

  if (!fs.existsSync(destPath)) {
    console.log('temp log folder not generated yet.');
    res.send({ errorMsg: '', percent: 0 });
    return;
  }

  let logFileCount = execSync(`sudo find ${srcPath} | wc -l`).toString();
  console.log('source file count: ' + logFileCount);

  if (!fs.existsSync(tarFilePath)) {
    res.send({ errorMsg: 'tar file not exists.', percent: 0 });
    return;
  }
  let tarFileCount = execSync(`sudo tar -tzf ${tarFilePath} | wc -l`).toString();
  console.log('tar file count: ' + tarFileCount);

  let progress_per = Math.floor(Number(tarFileCount) / Number(logFileCount) * 100);
  res.send({ errorMsg: '', percent: progress_per });
});

/**
 * @swagger
 * /log/debugTarFileName:
 *  get:
 *    tags: ["Logs"]
 *    description: Get debug log tar file name
 *    responses:
 *      '200':
 *        description: Debug log tar file name is retrieved successfully
 */
router.get('/log/debugTarFileName', function (req, res) {
  let logTarFileName = getLogTarFileName();
  res.send(logTarFileName);
});

/**
 * @swagger
 * /log/downloadTarFile/{filename}:
 *  get:
 *    tags: ["Logs"]
 *    description: Download debug log tar file
 *    parameters:
 *      - name: filename
 *        in: path
 *        description: tar file name
 *        required: true
 *        type: string
 *    responses:
 *      '200':
 *        description: Debug log tar file downloaded successfully
 */
router.get('/log/downloadTarFile/:filename', function (req, res) {
  const fileName = req.params.filename;
  const logTarFile = process.cwd() + '/' + fileName;

  res.setHeader('Content-disposition', 'attachment; filename=' + fileName);
  const stream = fs.createReadStream(logTarFile);
  stream.pipe(res);

  cleanDebugLogs();
});

function getLogTarFileName() {
  const logTarFilePrefix = 'swr-debug-log';
  var logTarFileName = "";
  fs.readdirSync(process.cwd()).forEach(file => {
    if (file.indexOf(logTarFilePrefix) != -1) {
      logTarFileName = file;
      console.log(`find debug log tar file: ${logTarFileName}.`);
      return;
    }
  });
  return logTarFileName;
}

function cleanDebugLogs() {
  const tmpPath = process.cwd() + '/tmp';
  const logTarFilePrefix = 'swr-debug-log';

  try {
    // delete tmp folder
    if (fs.existsSync(tmpPath)) {
      execSync(`sudo rm -rf ${tmpPath}`);
      console.log(`${tmpPath} deleted.`);
    }

    // delete debug log tar file
    fs.readdirSync(process.cwd()).forEach(file => {
      if (file.indexOf(logTarFilePrefix) != -1) {
        var logTarFile = process.cwd() + '/' + file;
        fs.unlink(logTarFile, function (error) {
          if (error) {
            console.log(`${file} delete error.`);
          } else {
            console.log(`${file} deleted.`);
          }
        });
      }
    });
  } catch (err) {
    console.log('clear debug logs error: ' + err);
  }
}

// router.post("/log", function (req, res) {
//   var start = parseInt(req.body.start);
//   var end = start + parseInt(req.body.length);
//   var orderColNum = req.body.order[0].column;
//   var orderType = req.body.order[0].dir;
//   var orderCol = "";
//   var searchKeyword = req.body.search.value;
//   var colArray = req.body.columns;
//   var page = req.body.page;

//   const targetFile = paramsDataPath + processFilename('conf.yaml');
//   var logPath = logDataPath;
//   var fileList = [];
//   var dataList = [];
//   var idx = 0;

//   var confData = fs.readFileSync(targetFile);
//   var yamlData = yaml.load(confData);
//   var jsonData = JSON.stringify(yamlData, null, 2);
//   let parsed = JSON.parse(jsonData);
//   logPath += parsed.sim.logging_folder;

//   fs.readdirSync(logPath).forEach(file => {
//     fileList.push(file);
//   });

//   fileList.forEach(function (fileName, index, array) {
//     (async () => {
//       var newJsonObj = {};
//       var jsons = await csv2json({ delimiter: [";"] }).fromFile(logPath + '/' + fileName);
//       idx += 1;
//       jsons.forEach(function (obj) {
//         if (orderCol === "") {
//           orderCol = Object.keys(obj)[orderColNum];
//           // console.log(Object.keys(obj)[orderColNum]);
//         }
//       });

//       dataList = dataList.concat(jsons);
//       if (fileList.length === idx) {
//         // full-text searching
//         if (searchKeyword !== '') {
//           console.log(searchKeyword);
//           dataList = dataList.filter(jsonObj => Object.values(jsonObj).some((element) => element.includes(searchKeyword)));
//         }

//         // searching by columns
//         var criteria = {};
//         var dateRange = "";
//         colArray = colArray.filter(col => col.search.value !== '');
//         if (colArray.length > 0) {
//           colArray.forEach((col) => {
//             if (col.data !== 'TIME') {
//               criteria[col.data] = col.search.value;
//             } else {
//               dateRange = col.search.value;
//             }
//             // dataList = dataList.filter(jsonObj => jsonObj[col.data].includes(col.search.value));
//           });
//           console.log(criteria);
//           dataList = dataList.filter(jsonObj =>
//             Object.keys(criteria).every(key =>
//               // [protection] JSON data from csv file might lack the criteria key
//               jsonObj[key] !== undefined && jsonObj[key].includes(criteria[key])
//             )
//           );
//           if (dateRange !== '') {
//             // console.log(dateRange);
//             var startDate = dateRange.split(";")[0];
//             var endDate = dateRange.split(";")[1];
//             dataList = dataList.filter(jsonObj => new Date(startDate) <= new Date(jsonObj.TIME) && new Date(jsonObj.TIME) <= new Date(endDate));
//           }
//         }
//         // sorting
//         switch (orderColNum) {
//           case '0':
//             if (orderType === 'asc') {
//               dataList.sort((a, b) => {
//                 if (new Date(a.TIME).toString() === 'Invalid Date') {
//                   return 1;
//                 } else if (new Date(b.TIME).toString() === 'Invalid Date') {
//                   return -1;
//                 }
//                 return new Date(a.TIME) - new Date(b.TIME);
//               });
//             } else if (orderType === 'desc') {
//               dataList.sort((a, b) => {
//                 if (new Date(a.TIME).toString() === 'Invalid Date') {
//                   return 1;
//                 } else if (new Date(b.TIME).toString() === 'Invalid Date') {
//                   return -1;
//                 }
//                 return new Date(b.TIME) - new Date(a.TIME);
//               });
//             }
//             break;
//           default:
//             if (orderType === 'asc') {
//               dataList.sort((a, b) => (a[orderCol] > b[orderCol]) ? 1 : ((b[orderCol] > a[orderCol]) ? -1 : 0));
//             } else if (orderType === 'desc') {
//               dataList.sort((a, b) => (a[orderCol] > b[orderCol]) ? -1 : ((b[orderCol] > a[orderCol]) ? 1 : 0));
//             }
//             break;
//         }

//         // console.log("===============")
//         // console.log(dataList.length);

//         // newJsonObj.draw = 1;
//         if (page === "all") {
//           const csv = json2csv.parse(dataList);
//           res.send(csv);
//         } else {
//           newJsonObj.recordsTotal = dataList.length;
//           newJsonObj.recordsFiltered = dataList.length;
//           newJsonObj.data = dataList.slice(start, end);
//           res.send(newJsonObj);
//         }
//       }
//     })()
//       .catch((err) => {
//         console.log(err);
//       });
//   });
// });

/**
 * @swagger
 * /version:
 *  get:
 *    tags: ["Version"]
 *    description: Get UI version
 *    responses:
 *      '200':
 *        description: UI version is retrieved successfully
 */
router.get('/version', function (req, res) {
  var version = '0.0.0';
  if (packageJSON.hasOwnProperty('version')) {
    version = packageJSON.version;
  }
  res.send(version);
});


// ------ Swarm Flow State Statistics ------------
// --- flow file directory protection ---
const flowFileDir = '/home/farobot/far_app_data/app_log/test_log/';
if (!fs.existsSync(flowFileDir)) {
  fs.mkdirSync(flowFileDir);
}
const flowFile = flowFileDir + 'flowState.db';
// const flowDB = new Database(flowFile, { verbose: console.log });
const flowDB = new Database(flowFile);

router.post("/swarm/statistics", function (req, res) {
  let freq = 1;
  let duration = 400;
  let sqlCmd2 = `
    SELECT ROUND((JULIANDAY('now','+8 hours')- JULIANDAY(interval) )*86400) AS diff,
          number
    FROM (
      SELECT datetime((strftime('%s', ts) / ${freq}) * ${freq}, 'unixepoch') AS interval,
            count(*) number
      FROM FlowState
      GROUP BY interval)
    WHERE diff <= ${duration}
    ORDER BY diff
  `;

  let result = flowDB.prepare(sqlCmd2).all();
  // console.log(result);

  res.json(result);
});


router.post("/swarm/statistics/flows/pie-chart", function (req, res) {
  let sqlCmd = `
    SELECT state, count(*) number FROM FlowState
    GROUP BY state
    ORDER BY state
  `;
  let resPieChart = flowDB.prepare(sqlCmd).all();

  // console.log(resPieChart);
  res.json(resPieChart);
});

router.post("/swarm/statistics/flows/list-view", function (req, res) {
  let sqlCmd = `
    SELECT flow_id, flow_name, state, progress, fleet_name FROM FlowState
    WHERE ROUND((JULIANDAY('now','+8 hours')- JULIANDAY(last_updated) )*86400) < 3
  `;
  let resListView = flowDB.prepare(sqlCmd).all();

  // console.log(resListView);
  res.json(resListView);
});


// ------ Swarm Fleet State (Agents) Statistics ------------
// --- fleet file directory protection ---
const fleetFileDir = '/home/farobot/far_app_data/app_log/test_log/';
if (!fs.existsSync(fleetFileDir)) {
  fs.mkdirSync(fleetFileDir);
}
const fleetFile = fleetFileDir + 'fleetState.db';
// const fleetDB = new Database(fleetFile, { verbose: console.log });
const fleetDB = new Database(fleetFile);

router.post("/swarm/statistics/agents/pie-chart", function (req, res) {
  let sqlCmd = `
    SELECT mode, count(*) number FROM FleetState
    GROUP BY mode
    ORDER BY mode
  `;
  let resPieChart = fleetDB.prepare(sqlCmd).all();

  // console.log(resPieChart);
  res.json(resPieChart);
});

router.post("/swarm/statistics/agents/list-view", function (req, res) {
  let sqlCmd = `
    SELECT robot_id, robot_name, model, mode, role, pose_x, pose_y, pose_a, battery_percent, map, fleet_name FROM FleetState
    WHERE ROUND((JULIANDAY('now','+8 hours')- JULIANDAY(last_updated) )*86400) < 3
  `;
  let resListView = fleetDB.prepare(sqlCmd).all();

  // console.log(resListView);
  res.json(resListView);
});


// ==============================
//     Time Synchronization
// ==============================
router.post('/swarmsync', function (req, res) {
  var data = {
    id: (req.body && 'id' in req.body) ? req.body.id : null,
    result: Date.now()
  };
  console.log(data);
  res.json(data);
});


// ==============================
//     File Upload & Download
// ==============================
router.post('/upload-license', async (req, res) => {
  try {
    if (!req.files) {
      res.send({
        status: false,
        message: 'No file uploaded'
      });
    } else {
      let license = req.files.license;

      // --- move the uploaded file to designated directory ---
      var licDir = licensePath + license.name;
      await license.mv(licDir);

      // TODO: validate license
      license_state = 1;

      // --- send response ---
      res.send({
        status: true,
        message: 'File uploaded'
      });
    }
  } catch (err) {
    res.status(500).send(err);
  }
});

router.get('/license-expiry', function (req, res) {
  // --- move the uploaded file to designated directory ---
  var licDir = licensePath + 'license.lic';

  // --- [protection] file existence ---
  if (!fs.existsSync(licDir)) {
    res.status(404).send({ errors: "no-file", date: 'none' });
  }

  // --- move the uploaded file to designated directory ---
  var lic = fs.readFileSync(licDir, 'utf-8');

  var content = ini.parse(lic);

  // --- [protection] attributes existence ---
  if ((content['DEFAULT'] === undefined) || (!content.DEFAULT['valid-to'] === undefined)) {
    res.status(404).send({ errors: "no key to expiry date", date: 'none' });
    return;
  }

  var expiryDate = content.DEFAULT['valid-to'];
  var hwSig = content.DEFAULT['sig'];

  res.status(200).send({ date: expiryDate, hwSig: hwSig });
});

// --- license_state ---
// 0: invalid, 1: authorized
let license_state = 0;
router.get('/license-check', function (req, res) {
  res.send({ valid: license_state });
});

let licenseFilePath = licensePath + 'license.lic';
router.delete('/remove-license', function (req, res) {
  try {
    fs.unlinkSync(licenseFilePath);
  } catch (err) {
    console.log(err);
  }
  res.sendStatus(200);
});

const signatureFilename = 'hw_signature';
const hwSignature = licensePath + signatureFilename;
router.get('/signature', function (req, res) {
  if (!fs.existsSync(hwSignature)) {
    res.status(404).send({ errors: [{ "code": "no-file", "message": "the hardware signature is not found" }] });
  }

  res.setHeader('Content-disposition', 'attachment; filename=' + signatureFilename);
  const stream = fs.createReadStream(hwSignature);
  stream.pipe(res);
});

router.get('/hardware-signature', function (req, res) {
  // --- move the uploaded file to designated directory ---
  var hwsDir = licensePath + 'hw_signature';
  console.log(hwsDir);

  // --- [protection] file existence ---
  if (!fs.existsSync(hwsDir)) {
    res.status(404).send({ errors: "no-file", date: 'none' });
  }

  // --- move the uploaded file to designated directory ---
  let hwSig = fs.readFileSync(hwsDir, 'utf-8');

  res.status(200).send({ hwSig: hwSig });
});

// Plan Artifact
router.post('/fleets/:filename/addPlanArtifact', function (req, res) {
  let fleet_name = req.params.filename;
  const targetFile = fleetDataPath + fleet_name + '.yaml';

  let request_obj = JSON.parse(req.body.artifact_data);
  let artifct_type = request_obj['type'];
  let artifct_id = request_obj['artifact_id'];
  let plan_artifact = `${artifct_type}@${artifct_id}`;

  try {
    let res_str = '';
    let fileData = fs.readFileSync(targetFile, 'utf-8');
    let read_file_obj = yaml.load(fileData);
    console.log(read_file_obj)

    if (read_file_obj[fleet_name]['artifacts'].hasOwnProperty('plan')) {
      if (read_file_obj[fleet_name]['artifacts']['plan'].indexOf(plan_artifact) < 0) {
        read_file_obj[fleet_name]['artifacts']['plan'].push(plan_artifact)
        res_str = artifct_id;
      }
    } else {
      read_file_obj[fleet_name]['artifacts']['plan'] = [plan_artifact];
    }

    let yamlContent = yaml.dump(read_file_obj);

    try {
      fs.writeFileSync(targetFile, yamlContent, 'utf-8');

      res.status(200);
      res.send(`Add ${res_str} to fleet: ${fleet_name}`);
    } catch (error) {
      res.status(404);
      res.send(error.message);
    }
  } catch (error) {
    res.status(404);
    res.send(error.message);
  }
});

router.delete('/deletePlanArtifact/:fleetname', function (req, res) {
  var fleet_name = req.params.fleetname;
  const targetFile = fleetDataPath + fleet_name + '.yaml';
  let artifact_name = JSON.parse(req.body.artifact_name);

  try {
    let res_str = '';
    let fileData = fs.readFileSync(targetFile, 'utf-8');
    let read_file_obj = yaml.load(fileData);

    let replace_plan_art_list = [];
    if (read_file_obj[fleet_name]['artifacts'].hasOwnProperty('plan')) {

      read_file_obj[fleet_name]['artifacts']['plan'].forEach(function (plan_artifact_item) {
        let plan_artifact_id = plan_artifact_item.split('@')[1];
        if (artifact_name == plan_artifact_id) {
          res_str = `${artifact_name} remove! At ${fleet_name}`;
        } else {
          replace_plan_art_list.push(plan_artifact_item)
        }
      });
    } else {
      res_str = `Nothing Need To Be remove! At ${fleet_name}`;
    }

    read_file_obj[fleet_name]['artifacts']['plan'] = replace_plan_art_list;
    let yamlContent = yaml.dump(read_file_obj);

    try {
      fs.writeFileSync(targetFile, yamlContent, 'utf-8');

      res.status(200);
      res.send(res_str);
    } catch (error) {
      res.status(404);
      res.send(error.message);
    }
  } catch (error) {
    res.status(404);
    res.send(error.message);
  }
});

router.put('/flows/checkDuplicate', async function (req, res) {
  var obj = req.body;
  var fleetname = obj.fleet;

  var modiFilename = `${fleetname}-flow-${obj.modified}.json`;
  var modiUiFilename = `${fleetname}-flowui-${obj.modified}.json`;

  let filenames = fs.readdirSync(operationDataPath);

  console.log("\nCurrent directory filenames:");
  let check_list = [];
  filenames.forEach(file => {
    console.log(file);
    if (file === modiFilename || file === modiUiFilename) {
      check_list.push(true);
    } else {
      check_list.push(false);
    }
  });

  console.log(check_list);
  if (check_list.includes(true)) {
    res.status(200);
    res.send(`{"statusCode": 404, "message": "Duplicate flow name: ${obj.modified}"}`);
    return
  } else {
    res.status(200);
    res.send('{"statusCode": 200, "message": "No Duplicate"}');
  }
});

router.put('/flows/rename', async function (req, res) {
  var obj = req.body;
  var fleetname = obj.fleet;

  var modiFilename = `${fleetname}-flow-${obj.modified}.json`;
  var modiUiFilename = `${fleetname}-flowui-${obj.modified}.json`;

  var origFlowFilename = operationDataPath + `${fleetname}-flow-${obj.original}.json`;
  var modiFlowFilename = operationDataPath + modiFilename;

  var origFlowUiFilename = operationDataPath + `${fleetname}-flowui-${obj.original}.json`;
  var modiFlowUiFilename = operationDataPath + modiUiFilename;

  let filenames = fs.readdirSync(operationDataPath);

  console.log("\nCurrent directory filenames:");
  let check_list = [];
  filenames.forEach(file => {
    console.log(file);
    if (file === modiFilename || file === modiUiFilename) {
      check_list.push(true);
    } else {
      check_list.push(false);
    }
  });

  console.log(check_list);
  if (check_list.includes(true)) {
    res.status(200);
    res.send(`{"statusCode": 404, "message": "Duplicate flow name: ${obj.modified}"}`);
    return
  }

  try {
    var data = await fs.readFileSync(origFlowFilename, 'utf8');
    var parsedData = JSON.parse(data);
    parsedData['flow_name'] = obj.modified;
    await fs.writeFileSync(origFlowFilename, JSON.stringify(parsedData));
  } catch (e) {
    res.status(200);
    res.send(`{"statusCode": 500, "message": "Failed to R/W flow! ${e}"}`);
  }

  fs.renameSync(origFlowFilename, modiFlowFilename, function (err) {
    if (err) console.log('ERROR: ' + err);
  });

  fs.rename(origFlowUiFilename, modiFlowUiFilename, function (err) {
    if (err) console.log('ERROR: ' + err);
  });


  let fleetConfig = fleetDataPath + `${fleetname}.yaml`;
  let yamlData = yaml.load(fs.readFileSync(fleetConfig, 'utf8'));
  console.log(yamlData[fleetname]);
  if (yamlData[fleetname].hasOwnProperty('flows')) {
    let index = yamlData[fleetname].flows.indexOf(obj.original)
    if (index !== -1) {
      yamlData[fleetname].flows[index] = obj.modified;
    }
    fs.writeFile(fleetConfig, yaml.dump(yamlData), (err) => {
      if (err) {
        console.log(err);
      }
    });
  }

  res.status(200);
  res.send('{"statusCode": 200, "message": "Rename flow successfully!"}');
});

router.put('/roles/renameRoleInFlows', async function (req, res) {
  var obj = req.body;
  let fileNameArray = obj.fileNameArray;
  fileNameArray.forEach(fileName => {
    console.log(fileName)
  });

  res.sendStatus(200);
});

router.put('/roles/rename', async function (req, res) {
  var obj = req.body;
  console.log('========= roles rename callback ========')
  console.log(obj);

  var origCapabilityFilename = rlDataPath + `${obj.original}.yaml`;
  var modiCapabilityFilename = rlDataPath + `${obj.modified}.yaml`;
  console.log('--- original capability ---')
  console.log(origCapabilityFilename);
  console.log('--- modified capability ---')
  console.log(modiCapabilityFilename);

  var origBehaviorFilename = btDataPath + `${obj.original}.xml`;
  var modiBehaviorFilename = btDataPath + `${obj.modified}.xml`;
  console.log('--- original behavior ---')
  console.log(origBehaviorFilename);
  console.log('--- modified behavior ---')
  console.log(modiBehaviorFilename);

  let filenames = fs.readdirSync(btDataPath);

  console.log("\nCurrent directory filenames:");
  let check_list = [];
  filenames.forEach(file => {
    console.log(file);
    if (file === `${obj.modified}.xml`) {
      check_list.push(true);
    } else {
      check_list.push(false);
    }
  });

  console.log(check_list);
  if (check_list.includes(true)) {
    res.status(200);
    res.send(`{"statusCode": 404, "message": "Duplicate Role name: ${obj.modified}"}`);
    return
  }

  try {
    var fleet_data = fs.readFileSync(origCapabilityFilename, 'utf8');
    var fileContent = yaml.load(fleet_data);

    fileContent.Must['behavior'] = `${obj.modified}.xml`
    var yamlContent = yaml.dump(fileContent);
    fs.writeFileSync(origCapabilityFilename, yamlContent, 'utf-8', function (err) {
      if (err) {
        res.send(err);
        console.log(err);
        return;
      }
    })
  } catch (err) {
    console.error(err);
  }

  fs.renameSync(origCapabilityFilename, modiCapabilityFilename, function (err) {
    if (err) console.log('ERROR: ' + err);
  });

  fs.rename(origBehaviorFilename, modiBehaviorFilename, function (err) {
    if (err) console.log('ERROR: ' + err);
  });

  let fleetFolderFiles = fs.readdirSync(fleetDataPath);
  let fleetFileLsit = fleetFolderFiles.filter(el => path.extname(el) === '.yaml');

  console.log('--- modified fleet ---')
  console.log(fleetFileLsit)

  fleetFileLsit.forEach(function (fleet_name, fleet_name_index) {
    let fleetname = fleet_name.replace('.yaml', '');
    let fleetConfig = fleetDataPath + `${fleetname}.yaml`;
    let yamlData = yaml.load(fs.readFileSync(fleetConfig, 'utf8'));
    console.log(yamlData[fleetname]);
    if (yamlData[fleetname].hasOwnProperty('roles')) {
      let index = yamlData[fleetname].roles.indexOf(obj.original);
      if (index !== -1) {
        yamlData[fleetname].roles[index] = obj.modified;
      }
      fs.writeFile(fleetConfig, yaml.dump(yamlData), (err) => {
        if (err) {
          console.log(err);
        }
      });
    }
  });

  // rename relate flow
  let fileNameArray = obj.fileNameArray;
  console.log('Have any flow use this role?')
  console.log(fileNameArray)

  if (fileNameArray != undefined) {
    fileNameArray.forEach(fileName => {
      let flow_modify_list = [];
      let flow_name = fileName;
      let flow_ui_name = flow_name.replace('-flow-', '-flowui-');
      flow_modify_list.push(flow_name);
      flow_modify_list.push(flow_ui_name);

      console.log(flow_name, flow_ui_name);

      flow_modify_list.forEach(fileName => {
        if (fileName.includes("json")) {
          fs.readFile(`${operationDataPath}${fileName}`, 'utf8', function (err, data) {
            console.log(`${operationDataPath}${fileName}`);
            if (err) {
              res.status(200);
              res.send(`{"statusCode": 400, "message": "Read relate flow fail original: ${obj.original}, modify: ${obj.modified}."}`);
            }

            let write_file_content = '';
            let write_file_path = `${operationDataPath}${fileName}`;

            if (fileName.includes('-flowui-')) {
              const re_ui_node_class_ = new RegExp(`${obj.original}`, 'g');
              let modify_ui_node_class_flow = JSON.stringify(data).replace(re_ui_node_class_, `${obj.modified}`);
              // const re_ui = new RegExp(`${obj.original}\\\\n`, 'g');
              // let modify_ui_element_flow = String(modify_ui_node_class_flow).replace(re_ui, `${obj.modified}\\n`);
              // const re_ui_node_title_ = new RegExp(`title=\\"${obj.original}\\"`, 'g');
              // let modifyUiNodeTitleFlow = String(modify_ui_node_class_flow).replace(re_ui_node_title_, `title=\\"${obj.modified}\\"`);
              // const re_ui_node_rolename = new RegExp(`rolename=\\"${obj.original}\\">${obj.original}`, 'g');
              // let modifyUiNodeRolenameFlow = String(modifyUiNodeTitleFlow).replace(re_ui_node_rolename, `rolename=\\"${obj.modified}\\">${obj.modified}`);
              // console.log(`${operationDataPath}${fileName}`);
              // console.log(modifyUiNodeRolenameFlow)
              write_file_content = JSON.parse(modify_ui_node_class_flow);
            } else {
              const re = new RegExp(`"role_name":\\s*"${obj.original}"`, 'g');
              let modify_flow = String(data).replace(re, `"role_name": "${obj.modified}"`);

              const re2 = new RegExp(`"node_name":\\s*"${obj.original}"`, 'g');
              let modify_flow2 = modify_flow.replace(re2, `"node_name": "${obj.modified}"`);

              console.log(`${operationDataPath}${fileName}`);
              console.log(modify_flow2)
              console.log(re.test(data))
              write_file_content = modify_flow2;
            }

            fs.writeFile(write_file_path, write_file_content, function (err) {
              if (err) {
                res.status(200);
                res.send(`{"statusCode": 400, "message": "Rename relate flow fail original: ${obj.original}, modify: ${obj.modified}."}`);
                return;
              }
              console.log(`Flow data update: ${operationDataPath}${fileName}`);
            });
          });
        }
      })

    });
  }

  res.status(200);
  res.send('{"statusCode": 200, "message": "Rename role successfully!"}');
});

router.get('/getTime', async function (req, res) {
  let sys_date = new Date();
  let sys_mili = sys_date.getTime();

  res.status(200);
  res.send(`{"mili": ${sys_mili}}`);
});

module.exports = router;