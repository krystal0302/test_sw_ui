import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "map";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"]["function_type"];
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

const funcTypeBtn = Selector('#function-type-btn');

const inputFunctionTypeName = Selector('#sbft-cell-name');
const ftDetect = Selector('#sbft-cell-detection-type');
const detectOptions = ftDetect.find('option').filter(opt => opt.value !== '_none');
const ftRecog = Selector('#sbft-recog');
const recogOptions = ftRecog.find('option').filter(opt => opt.value !== '_none');
const ftOffset = Selector('#sbft-offset', { timeout: element_wait_second });
const ftSize = Selector('#sbft-size');
const ftPayload = Selector('#sbft-payload');
const sbftAddBtn = Selector('#sbft-add-btn', { timeout: element_wait_second });
const sbftSaveBtn = Selector('#save-placeholder');

const labelFunctionTypeName = Selector(".cell-type-name");
const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

// let DETECTION = {
// 	Rack: 0,
// 	Charger: 1
// };

// let RECOG = {
// 	Rack: 0
// };

// const ftTestObjs = [
// 	{ name: 'testType_1', detectType: DETECTION.Rack, recogType: RECOG.Rack, offset: '1,1,1', size: '3,3,3', payload: '300' },
// 	{ name: 'testType_2', detectType: DETECTION.Rack, recogType: RECOG.Rack, offset: '2,2,2', size: '4,4,4', payload: '600' },
// ];

console.log(test_data)

test.timeouts({
	pageLoadTimeout: 200,
})
	('`Function Type` operation testing', async t => {		
		// --- POP-UP function type editing sidebar ---
		await t
			.typeText(inputLoginAccount, login_text)
			.typeText(inputLoginPassword, pwd_text)
			.click(btnLogin)
			.wait(5000)
			.navigateTo(map_page_url)
			.wait(5000)
			.click(checkboxCustom)
			.click(radioAddStorageCell)
			.click(funcTypeBtn)
			.wait(1000);
		
		// --- ADD function-type editing sidebar ---
		for (let ft of test_data) {
			await t
				.typeText(inputFunctionTypeName, ft.text_val)
				.click(ftDetect)
				.click(detectOptions.nth(ft.detectType)) // by far, 0: Rack, 1: Charger
				.click(ftRecog)
				.click(recogOptions.nth(ft.recogType))
				// .typeText(ftOffset, ft.offset)
				.typeText(ftSize, ft.size)
				.typeText(ftPayload, ft.payload)
				.click(sbftAddBtn)
				.wait(3000);
		}

		await t
			// TODO: add function-type test
			.scrollIntoView(sbftSaveBtn)
			.click(sbftSaveBtn)
			.wait(3000);

		// --- DELETE function-type editing sidebar ---
		for (let ft of test_data) {
			let btnDelete = Selector(`#ft-delete-${ft.text_val}`, { timeout: element_wait_second });			
			
			await t
				.scrollIntoView(btnDelete)
				.click(btnDelete, { offsetY: -30 })
				.wait(3000);
		}

		await t
			.scrollIntoView(sbftSaveBtn)
			.click(sbftSaveBtn)
			.expect(notificationTitle.innerText).eql('Info');

		for (let ft of test_data) {
			await t
			.expect(labelFunctionTypeName.withText(`${ft.text_val}`).exists).notOk();
		}

	})

