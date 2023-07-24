// ======================
//       CONSTANTS 
// ======================
// --- default value update on FAR-2172 ---
const FUNCTION_TYPES = {
	"charger": [
		{
			"model": "far",
			"offset": [
				0.185,
				-0.271,
				0
			],
			"recognition": [
				"Charger"
			],
			"type": "charger_template"
		}
	],
	"rack": [
		{
			"offset": [
				0,
				0,
				0
			],
			"payload": 10,
			"recognition": [
				"Rack"
			],
			"size": [
				0.76,
				1.06,
				0.5
			],
			"type": "rack_template"
		}
	]
};

const wmsStyles_ = {
	'rack': { border: 'rgba(255,0,0,0.4)' },
	'empty': { border: 'rgba(0,255,0,0.4)' },
	'loaded': { border: 'rgba(98,91,87,0.4)' }
};


// ======================
//   Reflector Config. 
// ======================
const reflectors = {
	"reflectors": [
		{
			"coordinate": [
				1000.10653918236494064,
				100.40166059136390686
			],
			"id": 0,
			"type": "cylinder",
			"width": 0.1
		},
		{
			"coordinate": [
				1200.39493465423584,
				200.432021141052246
			],
			"id": 1,
			"type": "cylinder",
			"width": 0.1
		},
		{
			"coordinate": [
				1400.495660305023193,
				300.6764187812805176
			],
			"id": 2,
			"type": "board",
			"width": 0.1
		}
	]
};

// ======================
//     Cursor States
// ======================
const CURSOR = {
	ADD_FUNCTION_ZONE: 'ADD_FUNCTION_ZONE',
	EDIT_FUNCTION_ZONE: 'EDIT_FUNCTION_ZONE',
	DELETE_FUNCTION_ZONE: 'DELETE_FUNCTION_ZONE',
	VIEW_REFLECTOR: 'VIEW_REFLECTOR',
	ADD_CYLINDER_REFLECTOR: 'ADD_CYLINDER_REFLECTOR',
	ADD_BOARD_REFLECTOR: 'ADD_BOARD_REFLECTOR',
	DELETE_REFLECTOR: 'DELETE_REFLECTOR',
}

// const TOOLS = {
// 	'MAPIMG': {
// 		ADD_LINE: 'ADD_LINE',
// 		ERASER: 'ERASER'
// 	},
// 	'GRAPH': {
// 		ADD_VERTEX: 'ADD_VERTEX',
// 		DELETE_VERTEX: 'DELETE_VERTEX',
// 		ADD_EDGE: 'ADD_EDGE',
// 		DELELTE_EDGE: 'DELETE_EDGE',
// 	},
// 	'CELL': {
// 		ADD_CELL: 'ADD_CELL',
// 		DELETE_CELL: 'DELETE_CELL',
// 	},
// 	'CONNCELL': {

// 	},
// 	'FUNCZONE': {
// 		ADD_ZONE: 'ADD_ZONE',
// 		DELETE_ZONE: 'DELETE_ZONE',
// 	},
// 	'REFLECTOR': {
// 		ADD_CYLINDER: 'ADD_CYLINDER',
// 		ADD_BOARD: 'ADD_BOARD',
// 		DELETE_REFLECTOR: 'DELETE_REFLECTOR',
// 	},
// }

// ======================
//   Zone Type Stylings 
// ======================
const ZONE_TYPES = {
	1: {
		'imgName': 'config',
		'pointColor': '#5477e9',
		'fillColor': '#90a0c4'
	},
	2: {
		'imgName': 'bypass',
		'pointColor': '#ef9e4b',
		'fillColor': '#d3aa7e'
	}
};


// ===============================
//   Vis-Network Initial Options 
// ===============================
const VisOptions = {
	autoResize: true,
	layout: {
		hierarchical: {
			enabled: false
		}
	},
	physics: {
		enabled: false
	},
	nodes: {
		shape: 'dot',
		size: 5,
		font: {
			size: 12,
			color: 'rgba(255,80,0,0)' // hide therlabel of vertices
		},
		fixed: {
			x: true,
			y: true,
		}
	},
	edges: {
		color: "rgba(87, 87, 87, 0.9)",
		value: 1,
		scaling: {
			min: 1,
			max: 2
		},
		shadow: true,
		dashes: false, // smooth enabled false will affect dash lines
		smooth: {
			enabled: false
		},
		arrows: {
			to: {
				enabled: true,
				scaleFactor: 0.3,
				type: "arrow"
			}
		},
		background: {
			enabled: true,
			color: "white"
		}
	},
	// dragView: pan function
	interaction: {
		hover: false,
		multiselect: false,
		dragView: true
	},
	manipulation: {},
	groups: {
		navnode: {
			color: {
				background: "gray",
				border: "black"
			}
		},
		wms: {
			color: {
				background: "gray",
				border: "black"
			}
		}
	}
};