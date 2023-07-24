const axios = require('axios')
var FormData = require('form-data');

let token_;

function setLineGroupToken(_token) {
	token_ = _token;
}

function lineNotify(_msg) {
	const form_data = new FormData();
	form_data.append("message", _msg);

	const headers = Object.assign({
		'Authorization': `Bearer ${token_}`
	}, form_data.getHeaders());

	axios({
		method: 'post',
		url: 'https://notify-api.line.me/api/notify',
		data: form_data,
		headers: headers
	}).then(function (response) {
		console.log("HTTP Status:" + response.status);
		console.log(response.data);
	}).catch(function (err) {
		console.error("LINE Notify Failed");
		if (err.response) {
			console.error("HTTP Status Code:" + err.response.status);
			console.error(err.response.data);
		} else {
			console.error(err);
		}
	});
}

module.exports.setLineGroupToken = setLineGroupToken;
module.exports.notify = lineNotify;
