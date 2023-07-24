import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "fleet";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const fleet_page_url = `${home_url}/${page}.html`;

fixture`Fleet Configuration Agent ADD-DELETE Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const btnCreateFleet = Selector('[data-target="#new-fleet-modal"]');

const fleetSelect = Selector('#fleet-select');
const fleetOptions = fleetSelect.find('option');
const agentName = 'fb_1';
const initAgentBtn = Selector('#init-pose-confirm-btn');
const addAgentBtn = Selector('#add-agent');
const editAgentBtn = Selector(`[data-btn-id="${agentName}-edit-btn"]`)
const deleteAgentBtn = Selector(`[data-btn-id="${agentName}-delete-btn"]`)
const agentCandidateRow = Selector(`#available-items-list > li > div > a`).withText(agentName);


// const getStyleAttribute = ClientFunction((selector) => {
// 	const element = selector();
// 	return element.getAttribute('style');
// });

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('`Agent` REMOVE, ADD, INITIALIZE feature testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(fleet_page_url)
				.wait(5000)
				.expect(btnCreateFleet.exists).ok();

			console.log(fleetOptions.count);
			for (let j = 0; j < 1; ++j) {

				// === REMOVE the agent fb_0 ===
				/** 
				 * criterias: 
				 *   1. the agent fb_0 SHOULD NOT exist in the fleet row 
				 */
				await t
					.setNativeDialogHandler(() => true)
					.click(deleteAgentBtn)
					.wait(1000)
					.expect(Selector(`[data-robot-id="${agentName}"]`).count).eql(0);

				// === ADD the agent fb_0 ===
				/** 
				 * criterias: 
				 *   1. the agent fb_0 SHOULD exist in the fleet row 
				 */
				await t
					.click(addAgentBtn)
					.click(agentCandidateRow)
					.click("#close-agent-candidates")
					.expect(Selector(`[data-robot-id="${agentName}"]`).count).eql(1)
					.wait(1000);

				// === INITIALIZE the agent fb_0 ===
				/** 
				 * criterias: 
				 *   1. the initialized button should alter its styles `GREEN Ready`
				 */
				await t
					.click(editAgentBtn)
					.wait(8000)
					.click(initAgentBtn)
					// .expect(await getStyleAttribute(initAgentBtn)).contains('background-color: #28a745')
					.expect(Selector('#init-pose-confirm-btn').innerText).contains('Ready', { timeout: 10000 })
					.wait(1000);

			}
		});
}
