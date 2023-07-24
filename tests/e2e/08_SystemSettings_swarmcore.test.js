import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "settings2";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"]["swarm_core"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const setting2_page_url = `${home_url}/${page}.html`;

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const navTabSwarmCore = Selector('#tab-swarm-core', { timeout: element_wait_second });
const lowBatteryThresholdInput = Selector('input', { timeout: element_wait_second }).withAttribute('name', 'low_battery_threshold');
const fullBatteryThresholdInput = Selector('input', { timeout: element_wait_second }).withAttribute('name', 'full_battery_threshold');
const chargingDueTimeInput = Selector('input', { timeout: element_wait_second }).withAttribute('name', 'charging_due_time');
const batteryLogTimeInput = Selector('input', { timeout: element_wait_second }).withAttribute('name', 'battery_log_time');
const batteryLogThresholdInput = Selector('input', { timeout: element_wait_second }).withAttribute('name', 'battery_log_threshold');
const sysSettingsSaveButton = Selector('.system-settings-save-btn', { timeout: element_wait_second });

// param
const default_low_battery_threshold = "40";
const default_full_battery_threshold = "80";
const default_charging_due_time = "4500";
const default_battery_log_time = "60";
const default_battery_log_threshold = "46080";

fixture`System Settings Settings-Edit Feature`
	.page(home_url);  // specify the entry page
	
for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('edit system settings testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(setting2_page_url)
				.wait(5000);
			
			for (const [idx, test_obj] of test_data.entries()) {
				let low_battery_threshold = test_obj["low_battery_threshold"];
				let full_battery_threshold = test_obj["full_battery_threshold"];
				let charging_due_time = test_obj["charging_due_time"];
				let battery_log_time = test_obj["battery_log_time"];
				let battery_log_threshold = test_obj["battery_log_threshold"];

				// === update swarm core settings ===
				/** 
				 * criterias: 
				 *   1. the value is CORRECTLY saved to the system-side 
				 */

				 await t
					.click(navTabSwarmCore)
					.typeText(lowBatteryThresholdInput, low_battery_threshold, { replace: true })
					.typeText(fullBatteryThresholdInput, full_battery_threshold, { replace: true })
					.typeText(chargingDueTimeInput, charging_due_time, { replace: true })
					.typeText(batteryLogTimeInput, battery_log_time, { replace: true })
					.typeText(batteryLogThresholdInput, battery_log_threshold, { replace: true })
					.click(sysSettingsSaveButton)
					.wait(1000);

			 	await t.eval(() => location.reload(true));

				await t
					.expect(lowBatteryThresholdInput.value).eql(low_battery_threshold)
					.expect(fullBatteryThresholdInput.value).eql(full_battery_threshold)
					.expect(chargingDueTimeInput.value).eql(charging_due_time)
					.expect(batteryLogTimeInput.value).eql(battery_log_time)
					.expect(batteryLogThresholdInput.value).eql(battery_log_threshold)
				
				// === set-back swarm core settings ===
				await t
					.click(navTabSwarmCore)
					.typeText(lowBatteryThresholdInput, default_low_battery_threshold, { replace: true })
					.typeText(fullBatteryThresholdInput, default_full_battery_threshold, { replace: true })
					.typeText(chargingDueTimeInput, default_charging_due_time, { replace: true })
					.typeText(batteryLogTimeInput, default_battery_log_time, { replace: true })
					.typeText(batteryLogThresholdInput, default_battery_log_threshold, { replace: true })
					.click(sysSettingsSaveButton)
			}
		});
}
