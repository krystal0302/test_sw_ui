'use strict';
const express = require('express');
const path = require('path');
const bodyParser = require('body-parser');
const routes = require('./routes/index');
// const rosRoutes = require('./routes/ros_routes');
const dbRoutes = require('./routes/testdb');
const compression = require('compression');
const hostIP = require('./util').getHostIP();
const printMessage = require('./services/print-message');
// const runWebSocketServer = require('./services/ws-comm');
const fileUpload = require('express-fileupload');
const cors = require('cors');
var MobileDetect = require('mobile-detect');

const app = express();
// require('./swagger.config')(app);

app.use(
	fileUpload({
		createParentPath: true,
	})
);
app.use(cors());

// ==============================
//     System Reporting Service
// ==============================
const logManager = require('./services/log-manager');
logManager.routineCheck(5000);
logManager.dbFileSizeCheck(5000);

// ==============================
//     Websocket Server
// ==============================
// runWebSocketServer();

// ==============================
//     HTTP Server Config.
// ==============================
var session = require('express-session');
var sessionParam = {
	secret: 'userSecret',
	name: 'user',
	saveUninitialized: false,
	cookie: {
		// maxAge: 10*60*1000, //ms
		httpOnly: false,
	},
};

const customMiddleware = function () {
	return function (req, res, next) {
		let userAgent = req.header('user-agent');
		let md = new MobileDetect(userAgent);
		// is iPad OS or Mac OS Safari
		let isIpadSafari = /Macintosh/i.test(userAgent);
		// console.log(md.mobile());
		if (md.mobile() || isIpadSafari) {
			app.set('views', path.join(__dirname, 'mobile_views'));
			return express.static(path.join(__dirname, 'public/mobile_html'), { index: 'login.html' })(req, res, next);
		} else {
			app.set('views', path.join(__dirname, 'views'));
			return express.static(path.join(__dirname, 'public'), { index: 'login.html' })(req, res, next);
		}
	}
}

const sharePublic = function () {
	return function (req, res, next) {
		let userAgent = req.header('user-agent');
		let md = new MobileDetect(userAgent);
		let isIpadSafari = /Macintosh/i.test(userAgent);
		if (md.mobile() || isIpadSafari) {
			return express.static(path.join(__dirname, 'public'))(req, res, next);
		}
		next();
	}
}

// view engine setup
// app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ limit: '50mb' }));
app.use(bodyParser.urlencoded({ extended: false }));
// app.use(
// 	express.static(path.join(__dirname, 'public'), { index: 'login.html' })
// );
app.use(customMiddleware())
app.use('/public', sharePublic())
app.use('/images', express.static('/home/farobot/far_app_data/app_params/ui/assets'));
app.use(compression());
app.use(session(sessionParam));
app.use('/', routes);
app.use('/testdb', dbRoutes);
// app.use('/ros2', rosRoutes);

// https://attacomsian.com/blog/uploading-files-nodejs-express
app.post('/upload-zips', async (req, res) => {
	console.log('upload-gz');
	let data = [];
	try {
		// to deal with different case of "1 file" or "multiple files"
		let files = [];
		let file = req.file;
		console.log('req.file: ', file);
		console.log('req.files: ', req.files);
		let zips = req.files.zips;
		let leng = req.files.zips.length;
		console.log('zips.length: ', zips, leng);
		if (undefined == leng) {
			files.push(zips);
		} else {
			files = zips;
		}
		console.log('files.length: ', files.length);

		if (!req.files) {
			res.send({
				status: false,
				message: 'No file uploaded'
			});
			return;
		}
		// https://stackoverflow.com/a/54367990/720276

		function processFile() {
			let file = files.shift();
			if (!file) {
				res.send({
					status: true,
					message: 'Files are uploaded',
					data: data
				});
				return; // stop sequence condition
			}
			file.mv('./upload-zips/' + file.name, function (err) {
				if (err) {
					console.log('err: ', err);
					res.send({
						status: false,
						message: `Move ${file.name} error`
					});
					return;
				} else {
					data.push({
						name: file.name,
						mimetype: file.mimetype,
						size: file.size,
					});
					processFile();
				}
			});
		}
		processFile();
		await new Promise((resolve) => setTimeout(resolve, 1000));
	} catch (err) {
		console.log('err: ', err);
		res.status(500).send(err);
	}
});


app.use(function (req, res, next) {
	const err = new Error('Not Found');
	err.status = 404;
	next(err);
});

app.use(function (err, req, res, next) {
	res.status(err.status || 500);
	console.error(err);
	res.json(err);
});

let port = process.env.PORT || 3000;

const printGreetings = () => {
	console.log(printMessage(hostIP, port))
};

app.listen(port, () => {
	printGreetings();
});

module.exports = app;
