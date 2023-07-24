import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "map";
const general_setting = getTestSetting(page);
// const test_data = general_setting["test_data"]["cell"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];

const map_page_url = `${home_url}/${page}.html`;

fixture`Map Editor Cell-Edit Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const checkboxCustom = Selector('#customCheckbox', { timeout: 10000 });

const radioAddStorageCell = Selector('#customRadio3');
const btnSetOnLive = Selector('#set-live-btn');
const spanLiveText = Selector('#live-btn-text');

const spanLiveTextExpectVal = "rgba(0, 255, 0, 0.6)";
const spanLiveTextExpectMsg = "assertion for color format should be aligned";

// ====== cells edit test ======
for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 200,
	})
		('cell live status testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(map_page_url)
				.wait(5000)
				.click(checkboxCustom);

			// ====== cells test ======
			for (let j = 0; j < test_repeat_times; ++j) {
				await t
					.click(radioAddStorageCell)
					.click(btnSetOnLive)
					.expect(spanLiveText.sibling('.fas').getStyleProperty('color')).eql(spanLiveTextExpectVal, spanLiveTextExpectMsg);
			}
		}
		);
}
