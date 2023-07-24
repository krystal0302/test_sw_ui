import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "dashboard";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const dashboard_page_url = `${home_url}/index.html`;

fixture`Dashboard Flow Pause-Resume Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const fleetSelect = Selector('#fleet-select');
const fleetOptions = fleetSelect.find('option');
const agentName = 'fb_0';
// const initAgentBtn = Selector('#init-pose-confirm-btn');
// const addAgentBtn = Selector('#add-agent');
// const editAgentBtn = Selector(`[data-btn-id="${agentName}-edit-btn"]`)
// const deleteAgentBtn = Selector(`[data-btn-id="${agentName}-delete-btn"]`)
// const agentCandidateRow = Selector(`#available-items-list > li > div > a`).withText(agentName);
const triggerFlowA = Selector(`#lsb-flowA`);
const btnConfirmFlow = Selector('#add-manual');


// const getStyleAttribute = ClientFunction((selector) => {
// 	const element = selector();
// 	return element.getAttribute('style');
// });

const ulAgentList = Selector("#fleet-agents-body").find('a').nth(0)
const p = Selector("#r-agent-ip > a")
const a = Selector('#far-editor').find('p').withText('Robot ID :')

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('`Flow` PAUSE-RESUME feature testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.navigateTo(dashboard_page_url)
				.wait(5000)

			await t
				.click(ulAgentList)
				.click(p)
				.wait(10000)
			
			await t
				.hover(a)
			
			let tt = await a.innerText
			console.log(tt)
			// for (let j = 0; j < 1; ++j) {
			// 	// === add flow ===
			// 	await t
			// 		.click(triggerFlowA)
			// 		.click(btnConfirmFlow)
			// 		.wait(2000)
			// 		.expect(Selector('#flows-table > tbody > tr > td > span').withText('flowA').count).eql(1)
			// 		.wait(1000);

			// 	// === pause flow ===
			// 	// console.log(Selector('#edit-flowA > i').classNames);
			// 	await t
			// 		.setNativeDialogHandler(() => true)
			// 		.click('#edit-flowA')
			// 		.wait(2000)
			// 		.expect(Selector('#edit-flowA > i').classNames).contains('fa-play')
			// 		.wait(1000);

			// 	// === resume flow ===
			// 	// console.log(Selector('#edit-flowA > i').classNames);
			// 	await t
			// 		.setNativeDialogHandler(() => true)
			// 		.click('#edit-flowA')
			// 		.wait(2000)
			// 		.expect(Selector('#edit-flowA > i').classNames).contains('fa-pause')
			// 		.wait(1000);

			// 	// // === remove flow ===
			// 	await t
			// 		.setNativeDialogHandler(() => true)
			// 		.click('#delete-flowA')
			// 		// .wait(5000)
			// 		// .expect(Selector('#flows-table > tbody > tr > td > span').withText('flowA').count).eql(1)
			// 		.wait(1000);
			// }
		});


}
