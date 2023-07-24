import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "map_live";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const livemap_page_url = `${home_url}/${page}.html`;

fixture`Map Editor Cell-Edit Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const iForCheckLiveMap = Selector('#liveview > i', { timeout: element_wait_second });

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
				.navigateTo(livemap_page_url)
				.wait(5000)

			// ====== cells status test ======
			for (let j = 0; j < test_repeat_times; ++j) {
				await t
					.expect(iForCheckLiveMap.getStyleProperty('color')).eql("rgba(0, 255, 0, 0.6)", "assertion for color format should be aligned")
					.wait(1000);
			}
		}
		);
}
