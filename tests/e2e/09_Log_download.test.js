import { Selector, RequestLogger } from 'testcafe';
import fs from 'fs';
import downloadFolder from 'downloads-folder';
import { getTestSetting } from '../util';

// --- Run test here ---
const general_setting = getTestSetting();
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const log_page_url = `${home_url}/log.html`;

console.log(log_page_url)

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');
const anchorDownloadBatteryLog = Selector('#export-battery-log-card > a', { timeout: element_wait_second });
const anchorDownloadDebugLog = Selector('#export-debug-log-card > a', { timeout: element_wait_second });

const btnExportBatteryLog = Selector('#export-battery-log');
const btnExportDebugyLog = Selector('#export-debug-log');

const fileBatteryLog = downloadFolder() + '/battery_data.zip';
const logger = RequestLogger({ log_page_url, method: 'post' }, {
	logRequestHeaders: true,
	logRequestBody: true,
})

fixture`System Log Export Feature`
	.page(home_url)  // specify the entry page
	.requestHooks(logger)


test.timeouts({
	pageLoadTimeout: 1000,
})
	('export system log testing', async t => {
		await t
			.typeText(inputLoginAccount, login_text)
			.typeText(inputLoginPassword, pwd_text)
			.click(btnLogin)
			.wait(5000)
			.navigateTo(log_page_url)
			.wait(5000);

		for (let j = 0; j < test_repeat_times; ++j) {
			// === update swarm core settings ===
			/** 
			 * criterias: 
			 *   1. the download file exist in the Downloads folder 
			 */

			await t
				.click(btnExportBatteryLog)
				.click(btnExportDebugyLog)
				.wait(10000)
				.click(anchorDownloadBatteryLog)
				.click(anchorDownloadDebugLog)
				.wait(10000)

			if (j > 2) {
				await t
					.expect(fs.existsSync(fileBatteryLog)).ok();
			}

		}
	});
