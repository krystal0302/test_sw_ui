import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "map";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"]["vertex"];
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
const btnDeleteVertex = Selector('#delete-node', { timeout: element_wait_second });
const radioAddRoute = Selector('#customRadio2');
const btnAddVertex = Selector('#add-node');
const visnetwork = Selector('.vis-network');
const inputVertexLable = Selector('#node-label');
const btnCreateVertex = Selector('#node-createButton');
const btnSave = Selector('#save-changes');
const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

// ====== routes, cells edit test ======
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

			// ====== routes test ======
			await t
				.click(radioAddRoute);

			// === create nodes ===
			for (const [idx, test_obj] of test_data.entries()) {
				await t
					.click(btnAddVertex)
					// --- node ---
					.click(visnetwork, test_obj["move_to"])
					.typeText(inputVertexLable, test_obj["text_val"])
					.click(btnCreateVertex)
			}

			await t
				.setNativeDialogHandler(() => true)
				.click(btnSave)
				.wait(1000);

			// === delete nodes ===
			for (const [idx, test_obj] of test_data.entries()) {
				await t
					.rightClick(visnetwork, test_obj["move_to"])
					.click(btnDeleteVertex);
			}

			await t
				.setNativeDialogHandler(() => true)
				.click(btnSave)
				.wait(1000)
				.expect(notificationTitle.innerText).eql('Info');
		}
		);
}
