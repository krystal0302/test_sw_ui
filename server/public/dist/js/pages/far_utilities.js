/*
 * Author: John Wu
 * Date: 06 Sep. 21,
 * Description:
 *   Expectation: variables independent, commonly-used
 **/

// ===========================
//   Radian/Angle Conversion     
// ===========================
/**
 * Convert number from radian to degree 
 * @param {number} _rad 
 * @returns {number} Range: -90 ~ 90 degree
 */
function cvtRad2Deg(_rad) {
	var deg = _rad * (180.0 / Math.PI);
	deg = Math.round(deg * 1000) / 1000; // CRITICAL! ensure the precision

	while (deg > 180) { deg -= 360; }
	while (deg < -180) { deg += 360; }

	return deg;
}

/**
 * Convert number from degree to number 
 * @param {number} _deg
 * @returns {number} Range: -PI/2 ~ PI/2 
 */
function cvtDeg2Rad(_deg) {
	while (_deg > 180) { _deg -= 360; }
	while (_deg < -180) { _deg += 360; }

	var rad = _deg * (Math.PI / 180.0);
	rad = Math.round(rad * 10000) / 10000; // CRITICAL! ensure the precision
	return rad;
}

// ==========================
//   File Format Conversion     
// ==========================
/**
 * Convert graph description from DOT to JSON 
 * @param {string} 
 * @returns {json}  
 */
function cvtDot2Json(_graphDot) {
	// --- return container ---
	var graphJson = {
		nodes: [],
		edges: []
	};

	// --- extract graph content ---
	var first = _graphDot.indexOf("{");
	var last = _graphDot.lastIndexOf("}");
	_graphDot = _graphDot.substring(first + 1, last);

	// --- split string by semicolon ---
	_graphDot = _graphDot.split(';');

	// --- split lines by semicolon ---
	for (l in _graphDot) {
		// -- ignore commented lines --
		if (_graphDot[l].search('//') == 0) {
			continue;
		}

		// --- check nodes or edges ---
		var line = _graphDot[l].trim();
		var lb = line.search(/\[/g);
		var rb = line.search(/\]/g);
		var arrow = line.search(/->/g);

		if (lb == -1 || rb == -1) continue;

		if (arrow == -1) {
			// --- node case ---
			var node = {};
			node["id"] = line.substring(0, lb).trim();

			var attr, anchor, e, c, v;

			attr = "label";
			anchor = line.search(/label/g);
			e = line.indexOf('=', anchor + attr.length);
			c = line.indexOf(',', e);
			c = (c == -1) ? rb : c;
			v = line.substring(e + 1, c);
			node[attr] = v.replace(/['"]+/g, '');

			attr = "x";
			anchor = line.search(/position_x/g);
			e = line.indexOf('=', anchor + attr.length);
			c = line.indexOf(',', e);
			v = line.substring(e + 1, c);
			node[attr] = v.replace(/['"]+/g, '');

			attr = "y";
			anchor = line.search(/position_y/g);
			e = line.indexOf('=', anchor + attr.length);
			c = line.indexOf(',', e);
			c = (c == -1) ? rb : c;
			v = line.substring(e + 1, c);
			//   console.log("find attr: " + attr + ", value: " + v); 
			node[attr] = v.replace(/['"]+/g, '');

			graphJson.nodes.push(node);
		} else {
			// --- edge case ---
			var edge = {};

			var attr, anchor, e, c, v;

			anchor = line.search(/->/g);
			var from = line.substring(0, anchor).trim();
			var to = line.substring(anchor + 2, lb).trim();
			edge["from"] = from;
			edge["to"] = to;

			attr = "weight";
			anchor = line.search(/weight/g);
			e = line.indexOf('=', anchor + attr.length);
			c = line.indexOf(',', e);
			c = (c == -1) ? rb : c;
			v = line.substring(e + 1, c);
			edge[attr] = v.replace(/['"]+/g, '');

			attr = "width";
			anchor = line.search(/width/g);
			e = line.indexOf('=', anchor + attr.length);
			c = line.indexOf(',', e);
			c = (c == -1) ? rb : c;
			v = line.substring(e + 1, c);
			edge[attr] = v.replace(/['"]+/g, '');

			graphJson.edges.push(edge);
		}
	}

	return graphJson;
}

/**
 * Convert graph description from JSON to DOT 
 * @param {json} 
 * @returns {string}  
 */
function cvtJson2Dot(_graphJson, _escapeType = 'farobot') {
	var nodesDot = '';
	var edgesDot = '';

	var nodeX;
	var nodeY;

	let escape = (_escapeType === 'farobot') ? '\\n' : '\n';

	for (key in _graphJson) {
		if (!_graphJson.hasOwnProperty(key)) {
			break;
		}

		nodeX = Number(_graphJson[key]["x"]);
		nodeY = Number(_graphJson[key]["y"]);

		nodesDot += _graphJson[key]["id"] + ' [label=' + _graphJson[key]["label"] + ', position_x=' + nodeX + ', position_y=' + nodeY + `, orientation=0.0];${escape}`;

		for (n in _graphJson[key]["connections"]) {
			edgesDot += _graphJson[key]["id"] + '->' + _graphJson[key]["connections"][n] + ` [weight=1,width=0.0];${escape}`;
		}
	}

	return `digraph G { ${escape}` + nodesDot + edgesDot + '}';
}

// ==========================
//   Date Format Conversion     
// ==========================
/**
 * Convert String with 'yyyy/mm/dd hh:mm' or 'yyyy/mm/dd' format to Date 
 * @param {string} 
 * @returns {date}  
 */
function cvtString2Date(_dateString) {
	const [dateComponents, timeComponents] = _dateString.split(' ');
	const [year, month, day] = dateComponents.split('/');
	if (timeComponents === undefined) {
		return new Date(+year, month - 1, +day);
	} else {
		const [hours, minutes] = timeComponents.split(':');
		return new Date(+year, month - 1, +day, +hours, +minutes);
	}
}

/**
 * Convert String with 'yyyy/mm/dd hh:mm' format to ISO8601 standard formatted string
 * @param {string} 
 * @returns {string}  
 */
function cvt2ISODateString(_dateString) {
	return moment(_dateString).toISOString(true);
}

/**
 * Convert String with 12 hours clock time format to 24 hours clock
 * @param {string} 
 * @returns {string}  
 */
function cvtTime12hTo24h(_timeString) {
	return moment(_timeString, 'hh:mm A').format('HH:mm');
}

// =================================
//     Coordinates Transfomation     
// =================================
function tfROS2Canvas(_mapDesc, _rosPos) {
	if (_mapDesc === undefined) { console.error('Map meta-data is not ready!'); return {}; }
	if (!(_mapDesc.hasOwnProperty('h'))) { console.error('Height of map is not defined!'); return {}; }
	if (!("origin" in _mapDesc)) { console.error('Origin of map is not loaded!'); return {}; }

	var xRosOffset = _mapDesc.origin.x / _mapDesc.resolution;
	var yRosOffset = _mapDesc.origin.y / _mapDesc.resolution;

	var xCanvas = (_rosPos.x / _mapDesc.resolution - xRosOffset).toFixed(4);
	var yCanvas = (_rosPos.y / _mapDesc.resolution - yRosOffset);
	yCanvas = (_mapDesc.h - yCanvas).toFixed(4);

	return {
		x: xCanvas,
		y: yCanvas
	};
}

function tfCanvas2ROS(_mapDesc, _cvsPos) {
	if (_mapDesc.h === undefined) { console.error('Height of map is not defined!'); }

	var xRosOffset = _mapDesc.origin.x / _mapDesc.resolution;
	var yRosOffset = _mapDesc.origin.y / _mapDesc.resolution;

	var xRos = ((Number(_cvsPos.x) + Number(xRosOffset)) * _mapDesc.resolution).toFixed(4);
	var yRos = ((Number(_mapDesc.h) - Number(_cvsPos.y) + Number(yRosOffset)) * _mapDesc.resolution).toFixed(4);

	return {
		x: xRos,
		y: yRos
	};
}

// =======================
//   Data Transformation 
// =======================
function isEmpty(_obj) {
	return Object.keys(_obj).length === 0;
}

function isEmptyString(str) {
	return $.trim(str) == "";
}

const flattenJSON = function (data) {
	var result = {};

	function recurse(cur, prop) {
		if (Object(cur) !== cur) {
			result[prop] = cur;
		} else if (Array.isArray(cur)) {
			for (var i = 0, l = cur.length; i < l; i++)
				recurse(cur[i], prop + "[" + i + "]");
			if (l == 0)
				result[prop] = [];
		} else {
			var isEmpty = true;
			for (var p in cur) {
				isEmpty = false;
				recurse(cur[p], prop ? prop + "." + p : p);
			}
			if (isEmpty && prop)
				result[prop] = {};
		}
	}
	recurse(data, "");
	return result;
};

const unflattenJSON = function (data) {
	"use strict";
	if (Object(data) !== data || Array.isArray(data))
		return data;
	var regex = /\.?([^.\[\]]+)|\[(\d+)\]/g,
		resultholder = {};
	for (var p in data) {
		var cur = resultholder,
			prop = "",
			m;
		while (m = regex.exec(p)) {
			cur = cur[prop] || (cur[prop] = (m[2] ? [] : {}));
			prop = m[2] || m[1];
		}
		cur[prop] = data[p];
	}
	return resultholder[""] || resultholder;
};

function removeAllChildNodes(parent) {
	while (parent.firstChild) {
		parent.removeChild(parent.firstChild);
	}
}

// ===================================
//   convert Quaternion to Euler 
// ===================================
function tfQuaternion2Euler(orientation) {
	var qw = parseFloat(orientation.w).toFixed(3);
	var qx = parseFloat(orientation.x).toFixed(3);
	var qy = parseFloat(orientation.y).toFixed(3);
	var qz = parseFloat(orientation.z).toFixed(3);
	// var siny_cosp = 2*(qw*qz + qx*qy);
	var siny_cosp = 2.0 * (qw * qz + qx * qy);
	var cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);

	var angle = Math.atan2(siny_cosp, cosy_cosp) * (180.0 / Math.PI);
	return angle;
};

// =======================
//   UUID generation 
// =======================
function genUuid() {
	var d = Date.now();
	if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
		d += performance.now(); //use high-precision timer if available
	}
	return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
		var r = (d + Math.random() * 16) % 16 | 0;
		d = Math.floor(d / 16);
		return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
	});
}

// ======================
//   Promise Functions
// ======================
function sleep(ms) {
	return new Promise(resolve => setTimeout(resolve, ms));
}

// ======================
//   Validation Functions
// ======================
if ($.validator != undefined) {
	$.validator.addMethod("chkInputVal", function (value, element) {
		let valid_res = inputCheck(value)

		if (valid_res) {
			element.style["border-color"] = "";
		} else {
			element.style["border-color"] = "red";
		}

		return valid_res;
	});
}

function inputCheck(input_str) {
	let test_str = String(input_str);
	const reg = new RegExp("^[a-zA-Z0-9_\*\+\=\-]*[a-zA-Z0-9]$", 'y');
	return reg.test(test_str);
}

function modalInputSetting(modal_input_id, setting) {
	let error_msg = `
		<div id="${modal_input_id}-error" class="error invalid-feedback" style="display: block;">Input include invalid characters</div>
	`;

	if (setting == 'warning') {
		$(`#${modal_input_id}`).css("border-color", 'red');
		$(`#${modal_input_id}-error`).remove();
		$(`#${modal_input_id}`).parent().append(error_msg);
	} else {
		$(`#${modal_input_id}-error`).remove();
		$(`#${modal_input_id}`).val('');
		$(`#${modal_input_id}`).css("border-color", '');
	}
}

function modalValidateEvent(modal_id) {
	$(`#${modal_id}`).on('hide.bs.modal', function (e) {
		let nameValid = false;
		let flow_name = this.getElementsByTagName('input')[0].value;
		let input_element_id = this.getElementsByTagName('input')[0].id;

		if (flow_name.length == 0) {
			modalInputSetting(input_element_id, 'default');
			nameValid = true;
		} else {
			if (!inputCheck(flow_name)) {
				modalInputSetting(input_element_id, 'warning');
			} else {
				modalInputSetting(input_element_id, 'default');
				nameValid = true;
			}
		}

		if (!nameValid) {
			e.preventDefault();
			e.stopPropagation();
			return;
		}
	});
}

// ==========================
//    User Agent Functions
// ==========================
function getMobileOS() {
	var userAgent = navigator.userAgent || navigator.vendor || window.opera;
	if (/windows phone/i.test(userAgent)) {
		return "windows phone";
	}
	if (/android/i.test(userAgent)) {
		return "android";
	}
	if (/iPad|iPhone|iPod/.test(userAgent) && !window.MSStream) {
		return "ios";
	}
	return "unknown";
}

function isTouchDevice() {
	return (('ontouchstart' in window) ||
		(navigator.maxTouchPoints > 0) ||
		(navigator.msMaxTouchPoints > 0));
}

function isMobile() {
	var isMobile = false;
	if (/(android|bb\d+|meego).+mobile|avantgo|bada\/|blackberry|blazer|compal|elaine|fennec|hiptop|iemobile|ip(hone|od)|ipad|iris|kindle|Android|Silk|lge |maemo|midp|mmp|netfront|opera m(ob|in)i|palm( os)?|phone|p(ixi|re)\/|plucker|pocket|psp|series(4|6)0|symbian|treo|up\.(browser|link)|vodafone|wap|windows (ce|phone)|xda|xiino|Macintosh/i.test(navigator.userAgent)
		|| /1207|6310|6590|3gso|4thp|50[1-6]i|770s|802s|a wa|abac|ac(er|oo|s\-)|ai(ko|rn)|al(av|ca|co)|amoi|an(ex|ny|yw)|aptu|ar(ch|go)|as(te|us)|attw|au(di|\-m|r |s )|avan|be(ck|ll|nq)|bi(lb|rd)|bl(ac|az)|br(e|v)w|bumb|bw\-(n|u)|c55\/|capi|ccwa|cdm\-|cell|chtm|cldc|cmd\-|co(mp|nd)|craw|da(it|ll|ng)|dbte|dc\-s|devi|dica|dmob|do(c|p)o|ds(12|\-d)|el(49|ai)|em(l2|ul)|er(ic|k0)|esl8|ez([4-7]0|os|wa|ze)|fetc|fly(\-|_)|g1 u|g560|gene|gf\-5|g\-mo|go(\.w|od)|gr(ad|un)|haie|hcit|hd\-(m|p|t)|hei\-|hi(pt|ta)|hp( i|ip)|hs\-c|ht(c(\-| |_|a|g|p|s|t)|tp)|hu(aw|tc)|i\-(20|go|ma)|i230|iac( |\-|\/)|ibro|idea|ig01|ikom|im1k|inno|ipaq|iris|ja(t|v)a|jbro|jemu|jigs|kddi|keji|kgt( |\/)|klon|kpt |kwc\-|kyo(c|k)|le(no|xi)|lg( g|\/(k|l|u)|50|54|\-[a-w])|libw|lynx|m1\-w|m3ga|m50\/|ma(te|ui|xo)|mc(01|21|ca)|m\-cr|me(rc|ri)|mi(o8|oa|ts)|mmef|mo(01|02|bi|de|do|t(\-| |o|v)|zz)|mt(50|p1|v )|mwbp|mywa|n10[0-2]|n20[2-3]|n30(0|2)|n50(0|2|5)|n7(0(0|1)|10)|ne((c|m)\-|on|tf|wf|wg|wt)|nok(6|i)|nzph|o2im|op(ti|wv)|oran|owg1|p800|pan(a|d|t)|pdxg|pg(13|\-([1-8]|c))|phil|pire|pl(ay|uc)|pn\-2|po(ck|rt|se)|prox|psio|pt\-g|qa\-a|qc(07|12|21|32|60|\-[2-7]|i\-)|qtek|r380|r600|raks|rim9|ro(ve|zo)|s55\/|sa(ge|ma|mm|ms|ny|va)|sc(01|h\-|oo|p\-)|sdk\/|se(c(\-|0|1)|47|mc|nd|ri)|sgh\-|shar|sie(\-|m)|sk\-0|sl(45|id)|sm(al|ar|b3|it|t5)|so(ft|ny)|sp(01|h\-|v\-|v )|sy(01|mb)|t2(18|50)|t6(00|10|18)|ta(gt|lk)|tcl\-|tdg\-|tel(i|m)|tim\-|t\-mo|to(pl|sh)|ts(70|m\-|m3|m5)|tx\-9|up(\.b|g1|si)|utst|v400|v750|veri|vi(rg|te)|vk(40|5[0-3]|\-v)|vm40|voda|vulc|vx(52|53|60|61|70|80|81|83|85|98)|w3c(\-| )|webc|whit|wi(g |nc|nw)|wmlb|wonu|x700|yas\-|your|zeto|zte\-/i.test(navigator.userAgent.substr(0, 4))) {
		isMobile = true;
	}
	return isMobile;
}

// ==========================
//    Polygon Intersection
// ==========================
class PolygonPoint {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}
}

class PolygonLine {
	constructor(p1, p2) {
		this.p1 = p1;
		this.p2 = p2;
	}
}

function onLine(l1, p) {
	// check whether point is on the line or not
	if (p.x <= Math.max(l1.p1.x, l1.p2.x)
		&& p.x <= Math.min(l1.p1.x, l1.p2.x)
		&& (p.y <= Math.max(l1.p1.y, l1.p2.y)
			&& p.y <= Math.min(l1.p1.y, l1.p2.y)))
		return true;

	return false;
}

function direction(a, b, c) {
	let val = (b.y - a.y) * (c.x - b.x)
		- (b.x - a.x) * (c.y - b.y);

	if (val == 0)
		// colinear
		return 0;

	else if (val < 0)
		// anti-clockwise
		return 2;

	// clockwise
	return 1;
}

function isIntersect(l1, l2) {
	// four direction for two lines and points of other line
	let dir1 = direction(l1.p1, l1.p2, l2.p1);
	let dir2 = direction(l1.p1, l1.p2, l2.p2);
	let dir3 = direction(l2.p1, l2.p2, l1.p1);
	let dir4 = direction(l2.p1, l2.p2, l1.p2);

	// intersecting
	if (dir1 != dir2 && dir3 != dir4)
		return true;

	// p2 of line2 are on the line1
	if (dir1 == 0 && onLine(l1, l2.p1))
		return true;

	// p1 of line2 are on the line1
	if (dir2 == 0 && onLine(l1, l2.p2))
		return true;

	// p2 of line1 are on the line2
	if (dir3 == 0 && onLine(l2, l1.p1))
		return true;

	// p1 of line1 are on the line2
	if (dir4 == 0 && onLine(l2, l1.p2))
		return true;

	return false;
}

/**
 * Check whether point is inside polygon
 * @param {json array, number, PolygonPoint}
 * @returns {boolean}
 */
function isPointInside(poly, n, p) {
	// create a point at infinity, y is same as point
	let tmp = new PolygonPoint(9999, p.y);
	let exline = new PolygonLine(p, tmp);
	let count = 0;
	let i = 0;
	do {
		// forming a line from two consecutive points of polygon
		let fromPoint = new PolygonPoint(poly[i].x, poly[i].y);
		let toPoint = new PolygonPoint(poly[(i + 1) % n].x, poly[(i + 1) % n].y);
		let side = new PolygonLine(fromPoint, toPoint);
		if (isIntersect(side, exline)) {
			// side is intersects exline
			if (direction(side.p1, p, side.p2) == 0)
				return onLine(side, p);
			count++;
		}
		i = (i + 1) % n;
	} while (i != 0);

	// count is odd
	return count & 1;
}

if (typeof module !== 'undefined') {
	module.exports = {
		cvtRad2Deg,
		cvtDeg2Rad,
		cvtDot2Json,
		cvtJson2Dot,
		tfROS2Canvas,
		tfCanvas2ROS,
		flattenJSON,
		unflattenJSON,
		inputCheck,
		modalValidateEvent
	}
}

// for test
// $('input').on('keyup', function (e) {

// 	console.log('asdasdasdasdadsasdasd')
// 	let param_val = $(this).val();

// 	if (!inputCheck(param_val)) {
// 		css_border_color = 'red';
// 	}else{
// 		css_border_color = '';
// 	}

// 	$(this).css('border-color', css_border_color)
// 	e.preventDefault();
// });
