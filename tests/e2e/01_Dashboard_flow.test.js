const fs   = require('fs');
const yaml = require('js-yaml');

import { Selector } from 'testcafe';
import { getTestSetting, getRandomInt } from '../util';


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
const data_path = `${general_setting["data_path"]}/app_fleet`;

fixture`Dashboard Flow Pause-Resume Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

// const fleetSelect = Selector('#fleet-select');
// // const fleetOptions = fleetSelect.find('option');
// // const agentName = 'fb_0';
// const initAgentBtn = Selector('#init-pose-confirm-btn');
// const addAgentBtn = Selector('#add-agent');
// const editAgentBtn = Selector(`[data-btn-id="${agentName}-edit-btn"]`)
// const deleteAgentBtn = Selector(`[data-btn-id="${agentName}-delete-btn"]`)
// // const agentCandidateRow = Selector(`#available-items-list > li > div > a`).withText(agentName);
// const triggerFlowA = Selector(`#lsb-flowA`);
const btnConfirmFlow = Selector('#add-manual');
const spanTriggeredFlowTd = Selector('#flows-table > tbody > tr > td > span');


const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

// const getStyleAttribute = ClientFunction((selector) => {
// 	const element = selector();
// 	return element.getAttribute('style');
// });

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('`Flow` PAUSE-RESUME feature testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(dashboard_page_url)
				.wait(5000)

			// Get flow
			const selectedFleet = await Selector('#fleet-select', { timeout: element_wait_second }).value;
			const fleet_setting = yaml.load(fs.readFileSync(`${data_path}/${selectedFleet}.yaml`, 'utf8'));
			console.log(fleet_setting);

			let flow_list = fleet_setting[selectedFleet]["flows"];

			// for(let i = 0; i < total_roles; i++) {
			// 	const avaibleFleet = await elements.nth(i).innerText;
			// 	fleet_list.push(avaibleFleet);
			// }
			console.log(flow_list)
			
			for (let j = 0; j < test_repeat_times; ++j) {
				let random_pick_flow = flow_list[getRandomInt(flow_list.length)];
				random_pick_flow = 'only_move';

				const triggerFlow = Selector(`#lsb-${random_pick_flow}`);
				const btnPauseFlow = Selector(`#edit-${random_pick_flow}`);
				const iconPauseCheck = Selector(`#edit-${random_pick_flow} > i`);
				const btnDeleteFlow = Selector(`#delete-${random_pick_flow}`);
				let orignalFlowCount = await spanTriggeredFlowTd.withText(random_pick_flow).count;

				// === trigger Flow ===
				await t
					.click(triggerFlow)
					.click(btnConfirmFlow)
					.wait(2000)
					.expect(spanTriggeredFlowTd.withText(random_pick_flow).count).eql(orignalFlowCount+1)
					.wait(1000);
				
				// === pause flow ===
				await t
					.setNativeDialogHandler(() => true)
					.click(btnPauseFlow)
					.wait(2000)
					.expect(iconPauseCheck.classNames).contains('fa-play')
					.wait(1000);
				
				// === resume flow ===
				await t
					.setNativeDialogHandler(() => true)
					.click(btnPauseFlow)
					.wait(2000)
					.expect(iconPauseCheck.classNames).contains('fa-pause')
					.wait(1000);

				// === remove flow ===
				await t
					.setNativeDialogHandler(() => true)
					.click(btnDeleteFlow)
					.expect(notificationTitle.withText(/Success/).exists).ok()
					.wait(1000);
			}

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
