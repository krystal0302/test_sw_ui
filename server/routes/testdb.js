'use strict';
const express = require('express');
const router = express.Router();
const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const bcrypt = require('bcrypt');
const Cryptr = require('cryptr');
const cryptr = new Cryptr('farobot-secret-key');
const nodemailer = require('nodemailer');
const randtoken = require('rand-token');
const hostIP = require('../util').getHostIP();
const logManager = require('../services/log-manager');
const saltRounds = 10;
const root = '/home/farobot/far_app_data/';
const swarmUiRoot = root + 'app_params/ui/';
const logDir = root + 'app_log/test_log/';
const mapDir = root + 'app_map/';
const dbFile = './farobottech.db';
const custDBFile = swarmUiRoot + 'farobottech.db';
const logDBFileName = 'systemlog.db';
const logDBFile = logDir + logDBFileName;

const sqlite3 = require('sqlite3').verbose();
const Database = require('better-sqlite3');
const confDB = new Database(dbFile);
var custDB = "";

async function db_init() {
  try {
    await confDB.prepare(`CREATE TABLE IF NOT EXISTS user (
                      user_id text NOT NULL PRIMARY KEY,
                      user_password text NOT NULL,
                      user_name text NOT NULL,
                      user_phoneNum text,
                      user_email text,
                      user_createdDate text,
                      user_role text NOT NULL,
                      user_password_change_mk BOOLEAN DEFAULT(FALSE),
                      user_int_mk BOOLEAN DEFAULT(FALSE),
                      reset_token text,
                      reset_token_exp text )`).run();

    await confDB.prepare(`CREATE TABLE IF NOT EXISTS password_history (
                      id text NOT NULL PRIMARY KEY,
                      user_id text NOT NULL,
                      hash_password	text NOT NULL,
                      created_time text)`).run();

    await confDB.prepare(`CREATE TABLE IF NOT EXISTS cell_type (
                      type_id text NOT NULL PRIMARY KEY,
                      type_name text NOT NULL,
                      detection_type_name text NOT NULL,
                      type_width REAL NOT NULL,
                      type_length REAL NOT NULL)`).run();

    await confDB.prepare(`CREATE TABLE IF NOT EXISTS map (
                      id text NOT NULL PRIMARY KEY,
                      name text NOT NULL,
                      alias_name text NOT NULL)`).run();

    await confDB.prepare(`CREATE TRIGGER IF NOT EXISTS add_user_password AFTER INSERT ON user 
                  BEGIN INSERT INTO password_history VALUES ('p'||lower(hex(randomblob(2))), new.user_id, new.user_password, DATETIME(REPLACE(new.user_createdDate,'/','-'))); END`).run();
    await confDB.prepare(`CREATE TRIGGER IF NOT EXISTS change_user_password AFTER UPDATE OF user_password ON user 
                  BEGIN INSERT INTO password_history VALUES ('p'||lower(hex(randomblob(2))), new.user_id, new.user_password, DATETIME('now','localtime')); END`).run();
    await confDB.prepare(`CREATE TRIGGER IF NOT EXISTS delete_user_password AFTER DELETE ON user
                  BEGIN DELETE FROM password_history WHERE user_id = old.user_id; END`).run();

    let ins_user_sql = `INSERT OR IGNORE INTO user 
                      VALUES ('admin', '$2b$10$HazVG2YLgRcoPzTps0l7zO8KRkatwUIxjqdRpr3vXCSQrOe8Ppu46', 'farobot_admin', '', '', '2021/10/13', 'admin', 0, 0, null, null),
                             ('root', '$2b$10$8ksL3Am/JLNa6xDo9JTeDO/vy49HCjX1nauLJGmiA.NUBRxOrgkFe', 'root', '', '', '2022/04/07', 'admin', 0, 1, null, null);`;
    await confDB.prepare(ins_user_sql).run();

    let ins_celltype_sql = `INSERT OR IGNORE INTO cell_type
                          VALUES ('celltype0001', 'rack', 'RackDetection', 3, 8),
                                 ('celltype0002', 'charger', 'ChargerDetection', 1, 1),
                                 ('celltype0003', 'qrcode', 'QRcode', 1, 1),
                                 ('celltype0004', 'position', 'Position', 8, 3)`;
    await confDB.prepare(ins_celltype_sql).run();
    checkConfDBSync();

  } catch (err) {
    console.error(err);
  }
}

db_init();

router.get('/testdb', function (req, res) {
  res.render('testdb');
});

// =================== Login/Logout ===================
/**
 * @swagger
 * /testdb/getLoginStatus:
 *  get: 
 *    tags: ["Login"]
 *    description: Get user login status
 *    responses:
 *      '200': 
 *        description: User login status retrieved sucessfully 
 */
router.get('/getLoginStatus', function (req, res) {
  if (req.session.userID) {
    var jsonData = {
      'greetingMsg': `Hi, ${req.session.userName}!`,
      'role': req.session.userRole,
      'userID': req.session.userID
    }
    res.json(jsonData);
  } else {
    res.redirect("/testdb/logout");
  }
});

/**
 * @swagger
 * /testdb/logout:
 *  get: 
 *    tags: ["Logout"]
 *    description: User logout
 *    responses:
 *      '200': 
 *        description: Logout sucessfully 
 */
router.get('/logout', function (req, res) {
  req.session.destroy(function (err) {
    if (err) {
      console.log("logout error: " + err);
      res.sendStatus(400);
    }

    res.clearCookie('user');
    res.status(200).send("logout");
  });
});


// =================== Maps ===================
/**
 * @swagger
 * /testdb/getMapData:
 *  get:
 *    tags: ["Maps"]
 *    description: Get all map data from database
 *    responses:
 *      '200':
 *        description: Map data is retrieved successfully
 */
router.get("/getMapData", function (req, res) {
  var files = fs.readdirSync(mapDir);
  files = files.filter(el => path.extname(el) === '.png');
  files = files.map(f => f.replace(".png", ""));
  // console.log(files);
  addMapData(files, true);

  var select_map_sql = `SELECT id, name, alias_name FROM map`;
  let rows = custDB.prepare(select_map_sql).all();
  if (rows.length === 0) {
    res.status(400).send('no map table data.');
  } else {
    var jsonContent = JSON.stringify(rows);
    // console.log(jsonContent);
    res.status(200).send(jsonContent);
  }
});

function addMapData(_mapImgNameArray, cleanData = false) {
  if (cleanData) {
    var del_map_sql = 'DELETE FROM map';
    if (_mapImgNameArray.length > 0) {
      del_map_sql = del_map_sql.concat(' WHERE id IN (SELECT id FROM map WHERE name NOT IN (');
      const paramArray = _mapImgNameArray.map(x => '?');
      del_map_sql = del_map_sql.concat(paramArray.join()).concat('))');
    }
    const info = custDB.prepare(del_map_sql).run(_mapImgNameArray);
    console.log(`Delete ${info.changes} map alias(es).`);
  }

  let insert_map_sql =
    `INSERT INTO map (id, name, alias_name)
   SELECT * FROM (SELECT 'm'||lower(hex(randomblob(2))), ?, ?) AS tmp
   WHERE NOT EXISTS (
     SELECT name FROM map WHERE name = ?
   ) LIMIT 1`;

  var mapImgNameArray = _mapImgNameArray;
  mapImgNameArray.forEach(function (mapName) {
    try {
      custDB.prepare(insert_map_sql).run(mapName, mapName, mapName);
      console.log(`Add map(${mapName}) table data success.`);
    } catch (err) {
      console.log(`Add map(${mapName}) table data failed: ${err}.`);
    }
  });
}

router.post("/addMapData", function (req, res) {
  addMapData(req.body.mapImgNameArray || []);
  res.sendStatus(200);
});

/**
 * @swagger
 * /testdb/deleteMapData:
 *  delete:
 *    tags: ["Maps"]
 *    description: Remove map data from table
 *    parameters:
 *      - name: map_name
 *        in: body
 *        description: map file name
 *        schema:
 *          type: object
 *          required:
 *            - mapname
 *          properties:
 *            mapname:
 *              type: string
 *    responses:
 *      '200':
 *        description: Map data deleted successfully
 */
router.delete("/deleteMapData", function (req, res) {
  var map_file_name = req.body.mapFileName;
  var delete_map_sql = `DELETE FROM map WHERE name = ?`;
  try {
    custDB.prepare(delete_map_sql).run(map_file_name);
    console.log(`Delete map(${map_file_name}) success.`);
    res.send({ status_code: 200, message: 'Map table data deleted.' });
  } catch (err) {
    console.log("Delete map table data failed: " + err);
    res.send({ status_code: 400, message: 'Delete map table data failed.' });
  }
});

router.post("/updateMapAlias", function (req, res) {
  var map_file_name = req.body.mapFileName;
  var new_map_name = req.body.newMapName;
  let update_map_sql = `UPDATE map SET alias_name = ? WHERE name = ?`;
  try {
    custDB.prepare(update_map_sql).run(new_map_name, map_file_name);
    res.send({ status_code: 200, message: 'Map alias updated.' });
  } catch (err) {
    res.send({ status_code: 400, message: 'Update map alias failed.' });
  }
});

router.get('/maps/:mapname/chkAlias', function (req, res) {
  var map_name = req.params.mapname;
  let chk_sql = 'SELECT COUNT(*) count FROM map WHERE alias_name = ?';
  try {
    const row = custDB.prepare(chk_sql).get(map_name);
    res.send(row.count > 0);
  } catch (err) {
    console.log("map alias count failed: " + err);
    res.send(false);
  }
});

router.get('/maps/:mapname/chkFileName', function (req, res) {
  var map_name = req.params.mapname;
  console.log(map_name);
  let chk_sql = 'SELECT COUNT(*) count FROM map WHERE name = ?';
  try {
    const row = custDB.prepare(chk_sql).get(map_name);
    res.send(row.count > 0);
  } catch (err) {
    console.log("map name count failed: " + err);
    res.send(false);
  }
});


// =================== Logs ===================

function transLogColumn(columnName) {
  return columnName === 'TIME' ?
    `(SUBSTR(TIME, -4) || '-' ||
      CASE SUBSTR(TIME, 5, 3) WHEN 'Jan' THEN '01'
                              WHEN 'Feb' THEN '02'
                              WHEN 'Mar' THEN '03'
                              WHEN 'Apr' THEN '04'
                              WHEN 'May' THEN '05'
                              WHEN 'Jun' THEN '06'
                              WHEN 'Jul' THEN '07'
                              WHEN 'Aug' THEN '08'
                              WHEN 'Sep' THEN '09'
                              WHEN 'Oct' THEN '10'
                              WHEN 'Nov' THEN '11'
                              WHEN 'Dec' THEN '12'
       END || '-' ||
      printf('%02d', SUBSTR(TIME, 9, 2)) || ' ' ||
      SUBSTR(TIME, 12, 8))`
    : columnName;
}

/**
 * @swagger
 * definitions:
 *   LogInfo:
 *     type: object
 *     required:
 *       - start
 *       - length
 *       - order
 *       - search
 *       - columns
 *       - page
 *     properties:
 *       start:
 *         type: string
 *       length:
 *         type: string
 *       order:
 *         type: array
 *         items:
 *           type: object
 *       search:
 *         type: object
 *       columns:
 *         type: array
 *         items:
 *           type: object
 *       page:
 *         type: string
 */

/**
 * @swagger
 * /testdb/logs:
 *   post:
 *     description: Get event log table data
 *     tags: ["Logs"]
 *     parameters:
 *       - name: log_info
 *         description: log info
 *         in: body
 *         required: true
 *         schema:
 *           $ref: '#/definitions/LogInfo'
 *     responses:
 *       200:
 *         description: event log info
 */
router.post("/logs", async function (req, res) {
  console.log('===================query event logs===================')
  // console.log(req.body);
  const log_max_cnt = 100000;
  var start = parseInt(req.body.start);
  var len = parseInt(req.body.length);
  var orderColNum = req.body.order[0].column;
  var orderType = req.body.order[0].dir; // desc or asc
  var orderCol = "";
  var searchKeyword = req.body.search.value;
  var colArray = req.body.columns;
  var page = req.body.page;

  var newJsonObj = { recordsTotal: 0, recordsFiltered: 0, data: [] };

  var select_syslog_sql = 'SELECT * FROM syslog';

  // --- generate criteria sql ---
  var criteriaArray = [];
  var criteriaSql = "";

  // full-text searching
  if (searchKeyword.length > 0) {
    colArray.forEach(function (item, idx) {
      var appendString = (idx === 0) ? 'WHERE' : 'OR';
      var sql = `${item.data} LIKE '%${searchKeyword}%'`;
      criteriaSql = criteriaSql.concat(` ${appendString} ${sql}`);
    });
    select_syslog_sql += criteriaSql;
  }

  // searching by columns
  var searchObjArray = colArray.filter(col => col.search.value !== '');
  if (searchKeyword.length === 0 && searchObjArray.length > 0) {
    searchObjArray.forEach((obj) => {
      var sql = "";
      if (obj.data === 'TIME') {
        var dateRange = obj.search.value;
        var startDate = dateRange.split(";")[0];
        var endDate = dateRange.split(";")[1];
        var startOffset_ms = (new Date(startDate)).getTimezoneOffset() * 60000;
        var endOffset_ms = (new Date(endDate)).getTimezoneOffset() * 60000;
        var startISODate = (new Date(new Date(startDate).getTime() - startOffset_ms)).toISOString().slice(0, -1);
        var endISODate = (new Date(new Date(endDate).getTime() - endOffset_ms)).toISOString().slice(0, -1);
        startDate = startISODate.replace(/T/, ' ').replace(/\..+/, '');
        endDate = endISODate.replace(/T/, ' ').replace(/\..+/, '')
        console.log('=======Log Date Range=======');
        console.log(startDate);
        console.log(endDate);
        sql = `${transLogColumn('TIME')} BETWEEN '${startDate}' AND '${endDate}'`;
      } else {
        if (obj.search.value == "swarm_core") {
          sql = `${obj.data} LIKE ''`;
        } else {
          sql = `${obj.data} LIKE '%${obj.search.value}%'`;
        }
      }
      criteriaArray.push(sql);
    });

    criteriaArray.forEach(function (item, idx) {
      var appendString = (idx === 0) ? 'WHERE' : 'AND';
      criteriaSql = criteriaSql.concat(` ${appendString} ${item}`);
    });
    select_syslog_sql += criteriaSql;
  }

  // --- generate order sql ---
  if (colArray.length > orderColNum) {
    let colObj = colArray[orderColNum];
    if (colObj.hasOwnProperty('data')) {
      orderCol = colObj.data;
      select_syslog_sql += ` ORDER BY ${transLogColumn(orderCol)} ${orderType}`;
    }
  }

  // --- limit records by page ---
  if (page === 'all') {
    var limit_max_records = ` LIMIT ${log_max_cnt}`;
    select_syslog_sql += limit_max_records;
    let exportSuccessed = logManager.exportAsCSV(select_syslog_sql);
    if (!exportSuccessed) {
      res.status(400).send('export csv file error.');
      return;
    }
    const filename = 'systemlog.tar.gz';
    const tarFile = logDir + filename;
    let fileExists = fs.existsSync(tarFile);
    if (fileExists) {
      res.sendStatus(200);
    } else {
      res.status(400).send('systemlog.tar.gz not exists.');
    }
    return;

  } else {
    var limit_records = ` LIMIT ${start}, ${len}`;
    select_syslog_sql += limit_records;
    // console.log(select_syslog_sql);
  }

  const logDB = new sqlite3.Database(logDBFile, (err) => {
    if (err) {
      console.log(`connected to ${logDBFileName} error: ${err.message}`);
    }
    console.log(`connected to ${logDBFileName}`);
  });
  // logDB.run('PRAGMA journal_mode = WAL;');
  logDB.all(select_syslog_sql, (err, rows) => {
    if (err) {
      console.log("get syslog table data error");
      res.send(newJsonObj);
      return;
    }
    logDB.get(`SELECT COUNT(*) cnt 
                 FROM syslog ${criteriaSql}`, function (err, data) {
      if (err) {
        console.log("get syslog table count error");
        res.send(newJsonObj);
        return;
      }

      rows.forEach(row => {
        if (row.robot_id == "") {
          // Use 'swarm_core' to replace null character for the robot_id
          row.robot_id = "swarm_core";
        }
      });

      let dataCnt = data['cnt'] > log_max_cnt ? log_max_cnt : data['cnt'];
      newJsonObj.recordsTotal = dataCnt;
      newJsonObj.recordsFiltered = dataCnt;
      newJsonObj.data = rows;
      // console.log(newJsonObj);
      res.send(newJsonObj);
    });
  });
  logDB.close();

});

/**
 * @swagger
 * /testdb/downloadEventLogs:
 *  get:
 *    tags: ["Logs"]
 *    description: Download event logs
 */
router.get('/downloadEventLogs', function (req, res) {
  const filename = 'systemlog.tar.gz';
  const tarFile = logDir + filename;
  let fileExists = fs.existsSync(tarFile);
  if (!fileExists) {
    res.redirect('back');
    return;
  }

  res.setHeader('Content-disposition', 'attachment; filename=' + filename);
  const stream = fs.createReadStream(tarFile);
  stream.pipe(res);
});


// =================== User ===================

function sendEmail(toMail, token, callback) {
  const emailConfFile = process.cwd() + '/email_conf.yaml';
  var emailConfData = fs.readFileSync(emailConfFile, 'utf8');
  var obj = yaml.load(emailConfData);
  var mailAddress = obj['sender'].mail_address;
  var authRToken = obj['sender'].refresh_token;
  var authAToken = obj['sender'].access_token;
  var encryptdClientID = obj['sender'].client_id;
  // encryptdClientID = cryptr.encrypt(encryptdClientID);
  // console.log('Encryptd client ID: ');
  // console.log(encryptdClientID);
  var encryptdClientSecret = obj['sender'].client_secret;
  // encryptdClientSecret = cryptr.encrypt(encryptdClientSecret);
  // console.log('Encryptd client secret: ');
  // console.log(encryptdClientSecret);
  try {
    var cilentID = cryptr.decrypt(encryptdClientID);
    var clientScrt = cryptr.decrypt(encryptdClientSecret);
    // console.log('Decrypted client ID = ', cilentID);
    // console.log('Decrypted client secret = ', clientScrt);
  } catch (err) {
    console.log('Get sender client ID or secret error:', err);
    return callback(false);
  }

  if (/@gmail\.com$/.test(mailAddress)) {
    var transporter = nodemailer.createTransport({
      host: 'smtp.gmail.com',
      port: 465,
      secure: true,
      auth: {
        type: "OAuth2",
        user: mailAddress,
        clientId: cilentID,
        clientSecret: clientScrt,
        refreshToken: authRToken,
        accessToken: authAToken
      }
    });

    var mailOptions = {
      from: {
        name: 'FARobot',
        address: mailAddress
      },
      to: toMail,
      subject: 'Reset Password',
      html: `<p>We received a request to reset the password for your account.<br> 
              Kindly use this <a href="http://${hostIP}:3000/testdb/resetPassword?token=${token}">link</a> to reset your password.</p>`
    };

    transporter.sendMail(mailOptions, function (error, info) {
      if (error) {
        console.log(error);
        return callback(false);
      } else {
        console.log('Email sent success: ' + info.response);
        return callback(true);
      }
    });
  } else {
    console.log('Sender email is not gmail.')
    return callback(false);
  }
}

function updateUserResetToken(userID, token = null, tokenExpireDate = null) {
  let update_token_sql = `UPDATE user SET reset_token = ?, reset_token_exp = ? WHERE user_id = ?`;
  try {
    custDB.prepare(update_token_sql).run(token, tokenExpireDate, userID);
    console.log(`Update ${userID} token.`);
  } catch (err) {
    console.log("Update user token failed.");
  }
}

router.post('/sendResetPasswordMail', function (req, res) {
  var token = randtoken.generate(20);
  var currentTime = new Date().getTime();
  var token_expire_date = new Date(currentTime + (1000 * 60 * 20));
  var user_id = req.body.userAccount;
  var user_mail = req.body.userMail;

  sendEmail(user_mail, token, function (status) {
    console.log(status);
    if (status) {
      let update_token_sql = `UPDATE user SET reset_token = ?, reset_token_exp = ? WHERE user_id = ?`;
      try {
        custDB.prepare(update_token_sql).run(token, token_expire_date.toString(), user_id);
        console.log(`Update ${user_id} token.`);
        res.status(200).send('Email sent! Please check your email.');
      } catch (err) {
        console.log(`Update user token failed: ${err}`);
        res.status(400).send('Sent email error! Please try again.');
      }
    } else {
      res.status(400).send('Sent email error! Please try again.');
    }
    return;
  });
});

router.get('/resetPassword', function (req, res) {
  var currentTime = new Date().getTime();
  var token = req.query.token;
  var title = '';
  var isExpired = true;

  let select_token_sql = 'SELECT reset_token_exp exp_date FROM user WHERE reset_token = ?';
  const row = custDB.prepare(select_token_sql).get(token);
  if (row) {
    const expTime = new Date(row.exp_date).getTime();
    isExpired = (currentTime > expTime);
    title = isExpired ? 'Reset Password Error' : 'Reset Password';
    res.render('reset-password', {
      title: title,
      isExpired: isExpired,
      token: token
    });
  } else {
    res.sendStatus(404);
  }
});

router.post('/updatePassword', function (req, res) {
  var token = req.body.token;
  var password = req.body.userPwd;

  let find_user_by_token_sql = 'SELECT user_id FROM user WHERE reset_token = ?';
  let row = custDB.prepare(find_user_by_token_sql).get(token);
  if (row) {
    let update_pwd_sql = `UPDATE user SET user_password = ?, user_password_change_mk = true
                             WHERE user_id = ?`;
    bcrypt.genSalt(saltRounds, function (err, salt) {
      bcrypt.hash(password, salt, function (err, hash) {
        try {
          custDB.prepare(update_pwd_sql).run(hash, row.user_id);
          updateUserResetToken(row.user_id);
          console.log(`Update ${row.user_id}'s password.`);
          res.status(200).send("Password reset.");
        } catch (err) {
          console.log("Update password failed: " + err);
          res.status(400).send("Reset password failed, please try again.");
        }
      });
    });
  } else {
    console.log("Update password failed: user not found.");
    res.status(400).send("Reset password failed, please try again.");
  }
});

/**
 * @swagger
 * /testdb/getUserInfo:
 *  get:
 *    tags: ["Users"]
 *    description: Get all users informaion from database
 *    responses:
 *      '200':
 *        description: Users information is retrieved successfully
 */
router.get("/getUserInfo", function (req, res) {
  var select_userinfo_sql = `SELECT user_id, user_name,
                                    CASE user_role WHEN 'admin' then 'Y'
                                                   WHEN 'general' then 'N'
                                                   ELSE 'N'
                                     END as isAdmin,
                                    user_phoneNum, user_email, user_createdDate 
                               FROM user
                              WHERE user_int_mk IS FALSE`;
  let rows = custDB.prepare(select_userinfo_sql).all();
  if (rows.length === 0) {
    res.status(400).send('no user information.');
  } else {
    var jsonContent = JSON.stringify(rows);
    // console.log(jsonContent);
    res.status(200).send(jsonContent);
  }
});

/**
 * @swagger
 * /testdb/getCurrentUserInfo:
 *  get:
 *    tags: ["Users"]
 *    description: Get current login user information from database
 *    responses:
 *      '200':
 *        description: Current user information is retrieved successfully
 */
router.get("/getCurrentUserInfo", function (req, res) {
  var select_curruser_sql = `SELECT user_id, user_name, 
                                    user_phoneNum, user_email 
                               FROM user
                              WHERE user_id = ?`;
  const rows = custDB.prepare(select_curruser_sql).all(req.session.userID);
  if (rows.length === 0) {
    res.status(400).send('no current user information.');
  } else {
    var jsonContent = JSON.stringify(rows);
    res.status(200).send(jsonContent);
  }
});

/**
 * @swagger
 * /testdb/getUserEmail:
 *  post:
 *    tags: ["Users"]
 *    description: Get user email from database
 *    produces:
 *      - application/json
 *    parameters:
 *      - name: user_info
 *        in: body
 *        required: true
 *        schema:
 *          type: object
 *          required:
 *            - userID
 *          properties:
 *            userID:
 *              type: string
 *    responses:
 *      '200':
 *        description: User email is retrieved successfully
 */
router.post("/getUserEmail", function (req, res) {
  var userID = req.body.userID;
  var select_usermail_sql = `SELECT user_email email
                               FROM user
                              WHERE user_id = ?`;
  const row = custDB.prepare(select_usermail_sql).get(userID);
  if (row) {
    const userEmail = row.email;
    res.status(200).send(userEmail);
  } else {
    res.status(400).send('Invalid account');
  }
});

/**
 * @swagger
 * definitions:
 *   UserInfo:
 *     type: object
 *     required:
 *       - accountID
 *       - accountPwd
 *       - userName
 *       - phoneNum
 *       - email
 *       - userAuth
 *     properties:
 *       accountID:
 *         type: string
 *       accountPwd:
 *         type: string
 *       userName:
 *         type: string 
 *       phoneNum:
 *         type: string
 *       email:
 *         type: string
 *       userAuth:
 *         type: string
 */

/**
 * @swagger
 * /testdb/addUserData:
 *  post:
 *    tags: ["Users"]
 *    produces:
 *      - application/json
 *    parameters:
 *      - name: user info
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/UserInfo'
 *    description: Insert user data into database
 *    responses:
 *      '200':
 *        description: User is added successfully
 */
router.post("/addUserData", function (req, res) {
  var userObj = req.body;
  var userID = userObj.accountID;
  let todayDate = new Date();
  let todayDateString = todayDate.getFullYear() + "/" + ((todayDate.getMonth() + 1).toString().padStart(2, '0')) + "/" + ((todayDate.getDate()).toString().padStart(2, '0'));
  // console.log(userObj);

  let select_user_sql = "SELECT COUNT(*) count FROM user WHERE user_id = ?";
  const row = custDB.prepare(select_user_sql).get(userID);
  if (row) {
    if (row.count > 0) {
      res.status(400).send(`Account ${userID} already exists!`);
    } else {
      bcrypt.genSalt(saltRounds, function (err, salt) {
        bcrypt.hash(userObj.accountPwd, salt, function (err, hash) {
          console.log(hash);
          let insert_user_sql = "INSERT INTO user (user_id, user_password, user_name, user_phoneNum, user_email, user_createdDate, user_role) VALUES (?, ?, ?, ?, ?, ?, ?)";
          try {
            custDB.prepare(insert_user_sql).run(userObj.accountID, hash, userObj.userName, userObj.phoneNum, userObj.email, todayDateString, userObj.userAuth);
            res.status(200).send("Add user success.");
          } catch (err) {
            console.log("Add user failed: " + err);
            res.status(400).send("Add user failed");
          }
        });
      });
    }
  } else {
    console.log("user count failed");
    res.status(400).send("Add user failed.");
  }
});

/**
 * @swagger
 * definitions:
 *   UsersRoleInfo:
 *     type: object
 *     required:
 *       - usersData
 *     properties:
 *       usersData:
 *         type: array
 *         items:
 *           type: object
 *           required:
 *             - user_role
 *             - user_id
 *           properties:
 *             user_role:
 *               type: string
 *             user_id:
 *               type: string
 */

/**
 * @swagger
 * /testdb/updateUsersRole:
 *  post:
 *    tags: ["Users"]
 *    parameters:
 *      - name: users role info
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/UsersRoleInfo'
 *    description: Update users role
 *    responses:
 *      '200':
 *        description: Users role updated successfully
 */
router.post("/updateUsersRole", function (req, res) {
  var usersDataArray = req.body.usersData;
  usersDataArray.forEach(function (userData) {
    // console.log(Object.values(userData));
    let data = Object.values(userData);
    let update_userRole_sql = `UPDATE user SET user_role = ? WHERE user_id = ?`;
    try {
      const info = custDB.prepare(update_userRole_sql).run(data);
      console.log(`Update ${info.changes} user(s).`);
    } catch (err) {
      console.log("Update user failed.");
    }
  });
  res.send("User updated.");
});

/**
 * @swagger
 * definitions:
 *   UpdateUserInfo:
 *     type: object
 *     required:
 *       - modifiedPwd
 *       - phoneNum
 *       - email
 *     properties:
 *       modifiedPwd:
 *         type: string
 *       phoneNum:
 *         type: string
 *       email:
 *         type: string
 */

/**
 * @swagger
 * /testdb/updateCurrentUserInfo:
 *  post:
 *    tags: ["Users"]
 *    parameters:
 *      - name: update_user_info
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/UpdateUserInfo'
 *    description: Update current user data
 *    responses:
 *      '200':
 *        description: Current user data updated successfully
 */
router.post("/updateCurrentUserInfo", function (req, res) {
  var userObj = req.body;
  var user_id = req.session.userID;
  var modified_pwd = userObj.modifiedPwd;
  var user_pwd_changed = false;
  // console.log(userObj);
  var update_user_sql = `UPDATE user 
                            SET user_phoneNum = ?, user_email = ?, user_password_change_mk = ?
                          WHERE user_id = ?`;
  let select_user_sql = "SELECT user_password pwd, user_password_change_mk change_mk FROM user WHERE user_id = ?";
  const row = custDB.prepare(select_user_sql).get(user_id);
  if (row && modified_pwd !== "") {
    bcrypt.compare(modified_pwd, row.pwd, function (err, result) {
      if (result) {
        user_pwd_changed = row.change_mk;
        console.log("changed password still the same.");
        try {
          const info = custDB.prepare(update_user_sql).run(userObj.phoneNum, userObj.email, user_pwd_changed, user_id);
          console.log(`Update ${info.changes} user.`);
          res.status(200).send("Update success.");
        } catch (err) {
          console.log("Update current user failed: " + err);
          res.status(400).send("Update failed");
        }
      } else {
        user_pwd_changed = Number(true);
        console.log("password changed.");
        update_user_sql = `UPDATE user 
                              SET user_password = ?, user_phoneNum = ?, user_email = ?, user_password_change_mk = ?
                            WHERE user_id = ?`;
        bcrypt.genSalt(saltRounds, function (err, salt) {
          bcrypt.hash(modified_pwd, salt, function (err, hash) {
            try {
              const info = custDB.prepare(update_user_sql).run(hash, userObj.phoneNum, userObj.email, user_pwd_changed, user_id);
              console.log(`Update ${info.changes} user.`);
              res.status(200).send("Update success.");
            } catch (err) {
              console.log("Update current user failed: " + err);
              res.status(400).send("Update failed");
            }
          });
        });
      }
    });
  } else if (row && modified_pwd === "") {
    user_pwd_changed = row.change_mk;
    console.log("password not changed.");
    try {
      const info = custDB.prepare(update_user_sql).run(userObj.phoneNum, userObj.email, user_pwd_changed, user_id);
      console.log(`Update ${info.changes} user.`);
      res.status(200).send("Update success.");
    } catch (err) {
      console.log("Update current user failed: " + err);
      res.status(400).send("Update failed");
    }
  } else {
    console.log("check user password failed: user not found.");
    res.status(400).send("Update failed.");
  }
});

/**
 * @swagger
 * definitions:
 *   LoginInfo:
 *     type: object
 *     required:
 *       - userID
 *       - pwd
 *     properties:
 *       userID:
 *         type: string
 *       pwd:
 *         type: string
 */

/**
 * @swagger
 * /testdb/login:
 *  post: 
 *    tags: ["Login"]
 *    description: User Login
 *    produces:
 *      - application/json
 *    parameters:
 *      - name: login_info
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/LoginInfo'
 *    responses:
 *      '200': 
 *        description: Login sucessfully
 *      '400':
 *        description: Login failed
 */
router.post("/login", function (req, res) {
  var select_user_sql = `SELECT user_id, user_name, user_role,
                                user_password pwd,
                                CASE user_password_change_mk WHEN 0 then 'N'
                                                             WHEN 1 then 'Y'
                                                             ELSE 'N'
                                 END as change_mk
                           FROM user WHERE user_id = ? Limit 1;`;
  const row = custDB.prepare(select_user_sql).get(req.body.userID);
  if (row) {
    console.log(`${row.user_name} exists.(password: ${req.body.pwd})`);
    bcrypt.compare(req.body.pwd, row.pwd, function (err, result) {
      if (result) {
        console.log("password matches!")
        req.session.userID = row.user_id;
        req.session.userName = row.user_name;
        req.session.userRole = row.user_role;
        res.status(200).send(row.change_mk);
      } else {
        res.status(400).send('Invalid password');
      }
    });
  } else {
    res.status(400).send('Invalid account');
  }
});

/**
 * @swagger
 * /testdb/deleteUsersData:
 *  delete:
 *    tags: ["Users"]
 *    description: Delete users
 *    parameters:
 *      - name: delete_users_info
 *        in: body
 *        description: delete user array
 *        schema:
 *          type: object
 *          required:
 *            - deleteUsers
 *          properties:
 *            deleteUsers:
 *              type: array
 *              items:
 *                type: string
 *    responses:
 *      '200':
 *        description: Users deleted successfully
 */
router.delete("/deleteUsersData", function (req, res) {
  var deleteUsersArray = req.body.deleteUsers;
  deleteUsersArray.forEach(function (userID) {
    // console.log(userID);
    var delete_user_sql = `DELETE FROM user WHERE user_id = ?`;
    try {
      const info = custDB.prepare(delete_user_sql).run(userID);
      console.log(`Delete ${info.changes} user(s).`);
    } catch (err) {
      console.log("Delete user failed: " + err);
    }
  });
  res.send("User deleted.");
});


// =================== History Password ===================
router.post('/chkIsRepeatedPassword', function (req, res) {
  var user_id = req.session.userID;
  var modified_pwd = req.body.modifiedPwd;
  var isRepeat = false;

  let get_all_pwd_sql = 'SELECT hash_password FROM password_history WHERE user_id = ?';
  let rows = custDB.prepare(get_all_pwd_sql).all(user_id);
  if (rows.length === 0) {
    res.send(false);
  } else {
    const pwdList = Object.values(rows).map(row => row.hash_password);
    for (const [index, val] of pwdList.entries()) {
      var result = bcrypt.compareSync(modified_pwd, val);
      if (result) {
        isRepeat = true;
        break;
      }
    }
    console.log('Password is repeated? ' + isRepeat);
    res.send(isRepeat);
  }
});


// =================== Settings: Cell Types ===================
/**
 * @swagger
 * /testdb/getCellTypes:
 *  get: 
 *    tags: ["Settings"]
 *    produces:
 *      - application/json
 *    description: Get cell types from database
 *    responses:
 *      '200': 
 *        description: Cell types got successfully
 */
router.get("/getCellTypes", function (req, res) {
  console.log('--- cell types ---');
  let sqlCmd = `SELECT type_name AS name,
                       type_id AS id,
                       detection_type_name AS detectionType,
                       type_width AS width,
                       type_length AS length
                  FROM cell_type;`;
  let rows = custDB.prepare(sqlCmd).all();
  if (rows.length === 0) {
    res.status(400).send('no cell types.');
  } else {
    rows.forEach((row) => {
      row.width = row.width.toFixed(2);
      row.length = row.length.toFixed(2);
    });
    console.log(rows);
    res.status(200).json(rows);
  }
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
 *         type: array
 *         items:
 *           type: object
 */

/**
 * @swagger
 * /testdb/postCellTypes:
 *  post: 
 *    tags: ["Settings"]
 *    produces:
 *      - application/json
 *    description: Insert or update cell types to database
 *    parameters:
 *      - name: cell-types_settings
 *        in: body
 *        required: true
 *        schema:
 *          $ref: '#/definitions/CellTypeSettings'
 *    responses:
 *      '200': 
 *        description: Cell types inserted/updated successfully
 */
router.post("/postCellTypes", function (req, res) {
  var obj = req.body.content;

  let check_count_sql = "SELECT COUNT(*) count FROM cell_type WHERE type_id = ?";
  Object.values(obj).forEach(function (cellTypeObj) {
    let type_id = cellTypeObj['id'] || 'NULL';
    let paramArray = [];
    paramArray.push(type_id);
    paramArray.push(cellTypeObj['name']);
    paramArray.push(cellTypeObj['detectionType']);
    paramArray.push(cellTypeObj['width']);
    paramArray.push(cellTypeObj['length']);
    const row = custDB.prepare(check_count_sql).get(type_id);
    if (row) {
      let sql_command = '';
      let editType = '';
      if (row.count > 0) {
        sql_command = `UPDATE cell_type
                          SET type_name = ?, detection_type_name = ?, 
                              type_width = ?, type_length = ?
                        WHERE type_id = ?;`
        paramArray.push(paramArray.shift())
        editType = 'Update';
      } else {
        sql_command = "INSERT INTO cell_type VALUES (?, ?, ?, ?, ?);"
        editType = 'Add';
      }
      if (sql_command.length === 0) return;
      try {
        custDB.prepare(sql_command).run(paramArray);
        console.log(`${editType} ${type_id} cell type success.`);
      } catch (err) {
        console.log(`${editType} ${type_id} cell type failed: ` + err);
      }
    } else {
      //
    }
  });

  res.sendStatus(200);
});

function checkConfDBSync() {
  try {
    // --- directory is not created case ---
    if (!fs.existsSync(swarmUiRoot)) {
      fs.mkdirSync(swarmUiRoot);
      fs.copyFile(dbFile, custDBFile, (err) => {
        if (err) {
          console.log(err);
        }
      })
      connectCustDB();
      return;
    }
    // --- settings database doesn't exist case ---
    if (!fs.existsSync(custDBFile)) {
      fs.copyFile(dbFile, custDBFile, (err) => {
        if (err) {
          console.log(err);
        }
      })
      connectCustDB();
      return;
    }
    connectCustDB();

  } catch (err) {
    console.log(err)
  }
}

function connectCustDB() {
  custDB = new Database(custDBFile);
  console.log(`connected to ${custDBFile}`);

  checkConfDBTableSync();
}

function checkConfDBTableSync() {
  let sqlCmd = `SELECT type, name, sql FROM sqlite_master
                 WHERE type = 'table' OR type = 'trigger' ORDER BY type`;
  let rows = confDB.prepare(sqlCmd).all();
  if (rows.length > 0) {
    // create table/triggers if not exists
    let chk_sql = `SELECT COUNT(*) count FROM sqlite_master
                    WHERE name = ? AND type = ?`;
    rows.forEach((row) => {
      let custDBRow = custDB.prepare(chk_sql).get(row.name, row.type);
      if (custDBRow.count > 0) return;
      try {
        custDB.prepare(row.sql).run();
        console.log(`Create ${row.name} ${row.type}.`);
      } catch (err) {
        console.log(`Create ${row.name} ${row.type} error: ${err}.`);
      }

      // insert default data to table
      if (row.type === 'trigger') return;
      let sel_sql = `SELECT * FROM ${row.name}`;
      let confDBRows = confDB.prepare(sel_sql).all();
      if (confDBRows.length > 0) {
        confDBRows.forEach((confDBRow) => {
          var insArray = Object.values(confDBRow).map(r => {
            if (r === null) {
              return 'null'
            }
            return typeof r === 'string' ? `'${r}'` : r
          });
          insArray = insArray.toString().replace(/'null'/g, null);
          let ins_sql = `INSERT OR IGNORE INTO ${row.name} VALUES (${insArray})`;
          // console.log(ins_sql);
          try {
            const info = custDB.prepare(ins_sql).run();
            console.log(`Insert ${info.changes} record(s) to ${row.name}.`);
          } catch (err) {
            console.log(`Insert data to ${row.name} table error: ${err}`)
          }
        });
      }

    });
  }
}

module.exports = router;