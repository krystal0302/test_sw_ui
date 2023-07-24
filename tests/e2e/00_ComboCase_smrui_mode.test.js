import { Selector } from 'testcafe';
import { getHostIP } from '../util';
import { faker } from '@faker-js/faker';

const hostIP = getHostIP();

// ======== Testing Configuration ========
const urlDashboard = `http://${hostIP}:3000/index.html`

// ======== Swarm UI Testing Selectors ========
// --- [Login] ---
const inputUserAccount = Selector('#user-account');
const inputUserPassword = Selector('#user-pwd');
const btnUserLogin = Selector("#user-login-btn");

// --- [Dashboard] ---
const liFleetAgent = Selector('#fleet-agents-body > li');
const btnFirstAgentDetail = liFleetAgent.nth(0).find('.agent-more');
const btnAgentLink = Selector('#r-agent-ip > a');

// ======== SMR UI Testing Selectors ========
// --- [Common] left sidebar ---
const lsbMap = Selector('li.has-treeview > a > p').withText('Map');
const lsbLiveMap = Selector('li.has-treeview > .nav-treeview > li > a > p').withText('Live View');
const lsbCreateMap = Selector('li.has-treeview > .nav-treeview > li > a > p').withText('Create Map');
const lsbSettings = Selector('li.nav-item > a > p').withText('Settings');

// --- [Dashboard] ---
const rowMode = Selector('#r_md');

// --- [Map] ---
const swJoystick = Selector('#joystickSwitch .toggle');

// --- [Settings] ---
const tabSystem = Selector('#ul_li > div > li > a').withText('system');
const inputMap = Selector('[id="input-system.map"]');
const divEditMap = inputMap.nextSibling(0);
const btnSave = Selector('#save_Settings');

// ======== Testing Cases ========
fixture`SMR UI SMR Mode Feature`
	.page`http://${hostIP}:3000`;

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('combo testing - smr mode', async t => {
			await t
				.typeText(inputUserAccount, 'admin')
				.typeText(inputUserPassword, 'admin')
				.click(btnUserLogin)
				.navigateTo(urlDashboard)
				.wait(5000);

			if (await liFleetAgent.count > 0) {
				// -- link to AMR UI --
				await t
					.click(btnFirstAgentDetail)
					.click(btnAgentLink)
					.wait(10000);

				let mode = await rowMode.innerText;
				// -- check SMR can be controlled by joystick only on manual mode --
				await t
					.click(lsbMap)
					.click(lsbLiveMap)
					.wait(1000);

				if (mode.toLowerCase() === 'manual') {
					await t
						.expect(swJoystick.visible).eql(true);
				} else {
					await t
						.expect(swJoystick.visible).eql(false);
				}

				await t
					.click(lsbCreateMap)
					.wait(1000);

				if (mode.toLowerCase() === 'manual') {
					await t
						.expect(swJoystick.visible).eql(true);
				} else {
					await t
						.expect(swJoystick.visible).eql(false);
				}

				// -- check SMR configuration can't be changed only on auto mode --
				await t
					.click(lsbSettings)
					.wait(3000)
					.click(tabSystem)

				if (mode.toLowerCase() === 'auto') {
					await t
						.expect(divEditMap.exists).notOk();
				} else {
					const randomName = faker.system.fileName().replace(/\.[^\/.]+$/, '').substr(0, 20);
					await t
						.click(divEditMap)
						.typeText(inputMap, randomName, { replace: true })
						.click(btnSave)
						.wait(3000)
						.expect(inputMap.value).eql(randomName);
				}
			}
		});
}
