import http from 'http';
import { Selector } from 'testcafe';

const getResponseData = (url) => new Promise((resolve, reject) => {
	http.get(url, res => {
		const { statusCode } = res;
		const contentType = res.headers['content-type'];

		res.setEncoding('utf8');
		let rawData = '';
		res.on('data', (chunk) => { rawData += chunk; });
		res.on('end', () => resolve({ statusCode, contentType, rawData }));
	}).on('error', e => reject(e));
});

// --- Run test here ---
const hostIP = 'localhost';
fixture`Map Editor Cell-Edit Feature`
	.page`http://${hostIP}:3000`;  // specify the entry page

const cellType = Selector('#cell-type');
const options = cellType.find('option').filter(opt => opt.value !== '_none');

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 200,
	})
		('connection status', async t => {
			await t
				.typeText('#user-account', 'admin')
				.typeText('#user-pwd', 'admin')
				.click('#user-login-btn')
				.navigateTo(`http://${hostIP}:3000/map.html`)
				.click('#customCheckbox')
				.click('#customRadio3')
			for (let j = 0; j < 10; ++j) {
				await t
					.click('#add-cell')
					// === create cells ===
					// --- create cell 1 ---
					.click('.vis-network', { offsetX: 500, offsetY: 500 })
					.typeText('#cell-label', '_test1')
					.click(cellType)
					.click(options.nth(0))
					.click('#cell-createButton')
					// --- create cell 2 ---
					.click('.vis-network', { offsetX: 550, offsetY: 500 })
					.typeText('#cell-label', '_test2')
					.click(cellType)
					.click(options.nth(1))
					.click('#cell-createButton')
					// --- create cell 3 ---
					.click('.vis-network', { offsetX: 550, offsetY: 550 })
					.typeText('#cell-label', '_test3')
					.click(cellType)
					.click(options.nth(2))
					.click('#cell-createButton')
					// --- create cell 4 ---
					.click('.vis-network', { offsetX: 500, offsetY: 550 })
					.typeText('#cell-label', '_test4')
					.click(cellType)
					.click(options.nth(3))
					.click('#cell-createButton')
					.click('#save-cells')
					.wait(1000)

				const selMap = await Selector('#map-select').value;
				let response = await getResponseData(`http://${hostIP}:3000/maps/${selMap}/cells`);
				let strResp = JSON.stringify(response);
				await t
					.expect(strResp).contains("new_test1")
					.expect(strResp).contains("new_test2")
					.expect(strResp).contains("new_test3")
					.expect(strResp).contains("new_test4")

				// === create cells ===
				// --- delete cell 1 ---
				await t
					.click('.vis-network', { offsetX: 500, offsetY: 500 })
					.click('#delete-cell')
					.click('.vis-network', { offsetX: 550, offsetY: 500 })
					.click('#delete-cell')
					.click('.vis-network', { offsetX: 550, offsetY: 550 })
					.click('#delete-cell')
					.click('.vis-network', { offsetX: 500, offsetY: 550 })
					.click('#delete-cell')
					.click('#cell-cancelButton')
					.click('#save-cells')
					.wait(1000)

				response = await getResponseData(`http://${hostIP}:3000/maps/${selMap}/cells`);
				strResp = JSON.stringify(response);
				await t
					.expect(strResp).notContains("new_test1")
					.expect(strResp).notContains("new_test2")
					.expect(strResp).notContains("new_test3")
					.expect(strResp).notContains("new_test4")
			}
		}
		);

}
