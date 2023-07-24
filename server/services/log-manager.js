const fs = require("fs");
const EventEmitter = require('events').EventEmitter;
const lineNotifier = require('./line-notifier');
const execSync = require('child_process').execSync;

const logDir = '/home/farobot/far_app_data/app_log/test_log/';

const limitFileSize_ = 1 * 1024 * 1024 * 1024; // unit: bytes. default: 1G limit. 
const limitRowNum_ = 3680000; // row count around 800Mb log file-size. 
const numOldestRows_ = 2000000; // row count around 400Mb log file-size. 
const Database = require('better-sqlite3');
const confFile = './farobottech.db';
const confDB = new Database(confFile);
const logFile = logDir + 'systemlog.db';
const logDB = new Database(logFile);


try {
	// --- create ui configuration db ---
	confDB.prepare(`
		CREATE TABLE IF NOT EXISTS notify_groups (
			line_token text NOT NULL PRIMARY KEY,
			level_op text,
			level_val text,
			content_op text,
			content_val text
		);
	`).run();

	// --- insert notify_groups default data ---
	let insCmd = "INSERT OR IGNORE INTO notify_groups VALUES ('IC2g1XwuQ4mH1q2YuYJooKfqzaX6a4UcLlf7mRhGR7l', 'eq', '2', 'none', '')";
	confDB.prepare(insCmd).run();

	// --- create system log db ---
	logDB.prepare(`
		CREATE TABLE IF NOT EXISTS syslog (
			TIME text NOT NULL PRIMARY KEY,
			event_type text NOT NULL,
			event_id text NOT NULL,
			module_pub text,
			module_sub text,
			level text,
			msg_cont text NOT NULL,
			robot_id text 
		);
	`).run();

	// --- access permission confirmation ---
	fs.chmod(logFile, 0o777, err => {
		if (err) throw err;
		console.log("System Log DB File permission changed!");
	});
} catch (err) {
	console.error(err);
}

let endMark_ = 0;

const levelMap_ = {
	ERROR: 2,
	WARN: 1,
	INFO: 0
}

const levelMap2_ = ['INFO', 'WARN', 'ERROR'];

let notifyRules_ = {};

function logNotifyFilter(_logMsg) {
	var logLevel = Number(levelMap_[_logMsg[5]]);
	var logCont = _logMsg[6];

	// --- log message level rules ---
	var lvOp = notifyRules_.logLevel.operator;
	var lvVal = Number(notifyRules_.logLevel.value);
	if (lvOp === "eq" && logLevel !== lvVal) { return false; }
	if (lvOp === "gt" && logLevel < lvVal) { return false; }
	if (lvOp === "lt" && logLevel > lvVal) { return false; }

	// --- message content rules ---
	var contOp = notifyRules_.logContent.operator;
	var contVal = notifyRules_.logContent.value;
	if (contOp === "includes" && !logCont.includes(contVal)) { return false; }
	if (contOp === "excludes" && logCont.includes(contVal)) { return false; }

	return true;
}

function genFilterSql() {
	// console.log('generate the literal transaction!');
	// --- log message level rules ---
	var lvCriteria = "";
	var lvOp = notifyRules_.logLevel.operator;
	var lvVal = Number(notifyRules_.logLevel.value);
	if (lvOp === "eq") { lvCriteria = `IN ('${levelMap2_[lvVal]}')` };

	var lvArr = [];
	if (lvOp === "gt") {
		console.log(lvVal);
		lvArr = levelMap2_.slice(lvVal);
	};
	if (lvOp === "lt") {
		lvArr = levelMap2_.slice(0, lvVal);
	};
	let lvGroup = "";
	lvArr.forEach((lv) => {
		lvGroup += `'${lv}',`
	})
	lvGroup = lvGroup.slice(0, -1);
	lvCriteria = `IN (${lvGroup})`

	// --- message content rules ---
	var cntCriteria = "LIKE '%%'";
	var contOp = notifyRules_.logContent.operator;
	var contVal = notifyRules_.logContent.value;
	if (contOp === "none") { cntCriteria = "LIKE '%%'"; }
	if (contOp === "includes") { cntCriteria = `LIKE '%${contVal}%'`; }
	if (contOp === "excludes") { cntCriteria = `NOT LIKE '%${contVal}%'`; }

	// --- return by conditions ---
	if (lvCriteria === "" && cntCriteria === "") {
		return "";
	}

	return `WHERE level ${lvCriteria} AND msg_cont ${cntCriteria}`;
}

class LogManager extends EventEmitter {

	constructor() {
		super();

		// --- LINE notifier initialization ---
		let sqlCmd = "SELECT * FROM notify_groups LIMIT 1;";
		let lineGroup = confDB.prepare(sqlCmd).all();
		lineNotifier.setLineGroupToken(lineGroup[0].line_token);

		// --- load notify rules ---
		notifyRules_ = {
			logLevel: {
				operator: lineGroup[0].level_op,
				value: lineGroup[0].level_val,
			},
			logContent: {
				operator: lineGroup[0].content_op,
				value: lineGroup[0].content_val
			}
		};

		// --- endMark Initialization ---
		sqlCmd = 'SELECT COUNT(*) FROM syslog;';
		let rowNum = Number(logDB.prepare(sqlCmd).all()[0]['COUNT(*)']);
		endMark_ = rowNum;
	}

	getNotifyGroups() {
		let sqlCmd = "SELECT * FROM notify_groups LIMIT 1;";
		let lineGroup = confDB.prepare(sqlCmd).all();
		// console.log(lineGroup);
		return lineGroup;
	}

	setNotifyGroups(_groups) {
		// --- cache the notify groups info ---
		let arrUpdated = [_groups.lineToken, _groups.logLevel.operator, _groups.logLevel.value, _groups.logContent.operator, _groups.logContent.value];
		// console.log(arrUpdated);
		let sqlCmd = "UPDATE notify_groups SET line_token = ?, level_op = ?, level_val = ?, content_op = ?, content_val = ? LIMIT 1";
		confDB.prepare(sqlCmd).run(arrUpdated);

		// --- update LINE token ---
		lineNotifier.setLineGroupToken(_groups.lineToken);

		// --- update notify rules ---
		notifyRules_ = {
			logLevel: {
				operator: _groups.logLevel.operator,
				value: _groups.logLevel.value
			},
			logContent: {
				operator: _groups.logContent.operator,
				value: _groups.logContent.value
			}
		};

		return true;
	}

	tickPeriod = (ms) => new Promise((resolve) => setTimeout(resolve, ms))

	async routineCheck(ms = 5000) {
		try {
			let sqlCmd = `SELECT COUNT(*) FROM syslog;`;
			let tableLen = Number(logDB.prepare(sqlCmd).all()[0]['COUNT(*)']);
			let rowNum = tableLen - endMark_;

			// --- messages notifier ---
			var critieria = genFilterSql(); // composing filter criteria
			sqlCmd = `SELECT * FROM (SELECT * FROM syslog LIMIT ${endMark_},${rowNum}) ${critieria};`;
			// console.log(sqlCmd);
			let latestRows = logDB.prepare(sqlCmd).all();

			// --- update the endMark_ ---
			endMark_ = tableLen;
			// console.log(`endMark: ${endMark_}`);

			// --- send LINE Notification ---
			if (latestRows.length) {
				console.log(`reading latest ${latestRows.length} rows logs`);
				latestRows.forEach((msg) => {
					// --- form notify messages ---
					let logMessage = msg['TIME'] + '#' + msg['event_type'] + '#' + msg['event_id'] + '#' + msg['module_pub'] + '#' + msg['module_sub'] + msg['level'] + '#' + msg['msg_cont'] + '#' + msg['robot_id'];
					// console.log(logMessage);
					lineNotifier.notify(logMessage);
				});
			}

		} catch (err) {
			console.error(err);
		}

		await this.tickPeriod(ms);
		// console.log('--- routine check ---');
		return await this.routineCheck(ms);
	}

	async dbFileSizeCheck(ms = 30000) {
		try {
			// --- file size control ---
			var dbStats = fs.statSync(logFile);
			if (dbStats.size > limitFileSize_) {
				console.log(`\x1b[36m [INFO]\x1b[0m Log File Size:\x1b[31m ${(dbStats.size / 1024 / 1024).toFixed(2)} MB \x1b[0mreaches limit \x1b[33m${(limitFileSize_ / 1024 / 1024).toFixed(2)} MB\x1b[0m! Size-Reduction is triggered!`);
				// let logSqlCmd = `DELETE FROM syslog WHERE ROWID < (SELECT COUNT(*) FROM syslog) - ${limitRowNum_};`;
				let logSqlCmd = `DELETE FROM syslog WHERE ROWID IN (SELECT ROWID FROM syslog ORDER BY ROWID ASC LIMIT ${numOldestRows_});`;
				await logDB.prepare(logSqlCmd).run();
				logDB.prepare('VACUUM;').run();
			}
		} catch (err) {
			console.error(err);
		}

		await this.tickPeriod(ms);
		console.log('--- log size check ---');
		return await this.dbFileSizeCheck(ms);
	}

	exportAsCSV(_sqlCommand, _filename = "systemlog", _compressed = true) {
		console.log("--- export as csv is triggered! ---");
		// --- running the export as csv command ---
		const csvFile = logDir + `${_filename}.csv`;
		try {
			execSync(`sqlite3 -header -csv ${logFile} "${_sqlCommand};" > ${csvFile}`);
		} catch (error) {
			console.log(error.message);
			return false;
		}
		if (!_compressed) { return true; }

		// --- compress the csv file to tar.gz fle ---
		const tarFile = logDir + `${_filename}.tar.gz`;
		try {
			execSync(`sudo tar -czvf ${tarFile} ${csvFile} --remove-files`);
		} catch (error) {
			console.log(error.message);
			return false;
		}
		return true;
	}
}
const logManager = new LogManager();

module.exports = logManager;