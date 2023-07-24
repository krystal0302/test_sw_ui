import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

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
const page = "map";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"]["cell"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const map_page_url = `${home_url}/${page}.html`;

fixture`Map Editor Cell-Edit Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const checkboxCustom = Selector('#customCheckbox', { timeout: element_wait_second });

const radioAddStorageCell = Selector('#customRadio3');
const btnAddCell = Selector('#add-cell');
const visnetwork = Selector('.vis-network');
const inputCellName = Selector('#cell-label');
const btnCreateCell = Selector('#cell-createButton');
const btnSaveCellChange = Selector('#save-changes');
const btnDeleteCell = Selector('#delete-cell');

const cellType = Selector('#cell-type');
const options = cellType.find('option').filter(opt => opt.value !== '_none');

const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

// const test_data = [
// 	{
// 		"text_val": "_testA",
// 		"moveTo": { offsetX: 500, offsetY: 100 }
// 	},
// 	{
// 		"text_val": "_testB",
// 		"moveTo": { offsetX: 700, offsetY: 100 }
// 	},
// 	{
// 		"text_val": "_testC",
// 		"moveTo": { offsetX: 700, offsetY: 200 }
// 	},
// 	{
// 		"text_val": "_testD",
// 		"moveTo": { offsetX: 500, offsetY: 200 }
// 	}	
// ];

// ====== cells edit test ======
console.log(test_data)

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 200,
	})
		('create, edit and delete `route` & `cell` testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(map_page_url)
				.wait(5000)
				.click(checkboxCustom);

			// ====== cells test ======
			// === create cells ===
			for (const [idx, test_obj] of test_data.entries()) {
				let total_cellType = 8;
				let selectIdx = Math.floor(Math.random() * total_cellType);
				await t
					.click(radioAddStorageCell)
					.click(btnAddCell)
					.click(visnetwork, test_obj["moveTo"])
					.typeText(inputCellName, test_obj["text_val"])
					.click(cellType)
					.click(options.nth(selectIdx))
					.click(btnCreateCell);
				
				if ((idx + 1) == test_data.length){
					await t
						.setNativeDialogHandler(() => true)
						.click(btnSaveCellChange)
						.wait(1000);
				}
			};

			// === delete cells ===
			for (const [idx, test_obj] of test_data.entries()) {				
				await t
					.rightClick(visnetwork, test_obj["moveTo"])
					.expect(btnDeleteCell.exists).ok({ timeout: 10000 });
					
				await t	
					.scrollIntoView(btnDeleteCell)
					.click(btnDeleteCell);
			
				if ((idx + 1) == test_data.length){
					await t
						.setNativeDialogHandler(() => true)
						.click(btnSaveCellChange)
						.wait(1000)
						.expect(notificationTitle.innerText).eql('Info');
				}
			};
		}
		);
}
