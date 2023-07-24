const fs = require("fs");

module.exports = ({ _logDB, _logFile, _limitFileSize, _limitRowNum }) => {
	try {
		console.log('--- start vacuuming ---');
		// --- file size control ---
		var dbStats = fs.statSync(_logFile);
		if (dbStats.size > _limitFileSize) {
			console.log(`\x1b[36m [INFO]\x1b[0m Log File Size:\x1b[31m ${(dbStats.size / 1024 / 1024).toFixed(2)} MB \x1b[0mreaches limit \x1b[33m${(_limitFileSize / 1024 / 1024).toFixed(2)} MB\x1b[0m! Size-Reduction is triggered!`);
			let logSqlCmd = `DELETE FROM syslog WHERE ROWID < (SELECT COUNT(*) FROM syslog) - ${_limitRowNum};`;
			_logDB.prepare(logSqlCmd).run();
			_logDB.prepare('VACUUM;').run();
		}
	} catch (err) {
		console.error(err);
	}

	console.log('vacuumed!!!');
};
