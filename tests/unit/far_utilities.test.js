const { cvtDeg2Rad, cvtRad2Deg, flattenJSON, unflattenJSON, cvtDot2Json, cvtJson2Dot } = require('../../server/public/dist/js/pages/far_utilities')
const { graphDOT, graphJSON, graphNetwork, graphUniDOT, graphUniJSON, graphUniNetwork } = require('../fixtures/mock_data_source')

describe('Radian <> Degree Conversion', () => {
	test(' [Rad2Deg] 0 to 0 ', () => {
		expect(cvtRad2Deg(0.0)).toEqual(0.0);
	})
	test(' [Rad2Deg] -PI/2 to -90.0 ', () => {
		expect(cvtRad2Deg(-Math.PI / 2)).toBeCloseTo(-90.0);
	})
	test(' [Rad2Deg] PI*3/2 to -90.0 ', () => {
		expect(cvtRad2Deg(Math.PI * 3 / 2)).toBeCloseTo(-90.0);
	})
	test(' [Rad2Deg] PI to 180.0 ', () => {
		expect(cvtRad2Deg(Math.PI)).toBeCloseTo(180.0);
	})

	test(' [Deg2Rad] 0 to 0', () => {
		expect(cvtDeg2Rad(0.0)).toEqual(0.0);
	})
	test(' [Deg2Rad] -90 to -PI/2', () => {
		expect(cvtDeg2Rad(-90.0)).toBeCloseTo(-Math.PI / 2);
	})
	test(' [Deg2Rad] 270 to -PI/2', () => {
		expect(cvtDeg2Rad(270.0)).toBeCloseTo(-Math.PI / 2);
	})
	test(' [Deg2Rad] 180 to PI', () => {
		expect(cvtDeg2Rad(180.0)).toBeCloseTo(Math.PI);
	})
})

describe('JSON object transformation', () => {
	const obj = {
		name: "farobot",
		address: {
			personal: "amr",
			office: {
				building: 'k-building',
				street: 'zhong-zheng road'
			}
		}
	};
	const flattenObj = {
		"name": "farobot",
		"address.personal": "amr",
		"address.office.building": "k-building",
		"address.office.street": "zhong-zheng road"
	};

	test('Nested json object should be flatten', () => {
		expect(flattenJSON(obj)).toEqual(flattenObj);
	})

	test('Flatten json object should be restore', () => {
		expect(unflattenJSON(flattenObj)).toEqual(obj);
	})

})

describe('Graph format transformation', () => {
	// Graph with bi-direction case
	test('[Dot2Json] Graph from DOT to JSON', () => {
		expect(cvtDot2Json(graphDOT)).toEqual(graphJSON);
	})

	test('[Json2Dot] Graph JSON to DOT', () => {
		expect(cvtJson2Dot(graphNetwork)).toEqual(graphDOT);
	})

	// Graph with uni-direction case
	test('[Dot2Json] Directed Graph from DOT to JSON', () => {
		expect(cvtDot2Json(graphUniDOT)).toEqual(graphUniJSON);
	})

	test('[Json2Dot] Directed Graph JSON to DOT', () => {
		expect(cvtJson2Dot(graphUniNetwork)).toEqual(graphUniDOT);
	})

})