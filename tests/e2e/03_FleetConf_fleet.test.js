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

fixture`Fleet Configuration Fleet ADD-DELETE Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const selectFleet = Selector('#fleet-select');
const optionFleet = selectFleet.find('option');
const optionCheck = Selector('#fleet-select > option', { timeout: element_wait_second })


const btnCreateFleet = Selector('[data-target="#new-fleet-modal"]');
const inputFleetFileName = Selector('#fleet-filename');
const modalBtnSaveFleet = Selector('#create-fleet-form > div > button').withText('Create');
const btnDeleteFleet = Selector('[data-target="#delete-fleet-modal"]');
const modalBtnDeleteFleet = Selector('#delete-fleet-modal > .modal-dialog > .modal-content > .modal-footer > button').withText('Delete');

const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

console.log(test_data)

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('`Fleet` ADD and DELETE feature testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(fleet_page_url)
				.wait(5000)
				.expect(btnCreateFleet.exists).ok();
				

			for (const [idx, test_obj] of test_data.entries()) {
				// === create fleet ===
				/** 
				 * criterias: 
				 *   1. the fleet bar SHOULD exist in fleet select options 
				 */
				let fleet_name = test_obj["text_val"];

				await t
					.click(btnCreateFleet)
					.typeText(inputFleetFileName, fleet_name)
					.click(modalBtnSaveFleet)
					.wait(5000)
					.expect(optionCheck.withText(fleet_name).exists).ok();
				
				// === delete fleet ===
				/** 
				 * criterias: 
				 *   1. the fleet SHOULD NOT exist in fleet select options
				 */
				await t
					.click(selectFleet)
					.click(optionFleet.withText(fleet_name))
					.click(btnDeleteFleet)
					.click(modalBtnDeleteFleet)
					.wait(1000)
					.expect(notificationTitle.innerText).eql('Info')
					.expect(optionCheck.withText(fleet_name).exists).notOk();
			}
		});
}
