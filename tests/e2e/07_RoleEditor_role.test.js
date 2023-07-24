import { Selector } from 'testcafe';
import { getTestSetting } from '../util';

// --- Run test here ---
const page = "role";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const role_page_url = `${home_url}/${page}.html`;

fixture`Role Editor Role-Edit Feature`
	.page(home_url);  // specify the entry page

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

const moveMenu = Selector('#bt-toolbar').find('li').nth(0);
const move = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(0);
const charge = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(1);
const logicMenu = Selector('#bt-toolbar').find('li').nth(1);
const wait = Selector('#bt-toolbar').find('li').nth(1).find('div > a').nth(0);
const othersMenu = Selector('#bt-toolbar').find('li').nth(2);
const artifact = Selector('#bt-toolbar').find('li').nth(2).find('div > a').nth(0);

const divMoveVerify = Selector('[data-title="Nav2Client"]');
const divChargeVerify = Selector('[data-title="Charge"]');
const divWaitVerify = Selector('[data-title="WaitAction"]');
const divArtifactVerify = Selector('[data-title="Artifact"]');

const inputRole = Selector('#role-filename');

const btnCreateNewRole = Selector('[data-target="#new-role-modal"]', { timeout: element_wait_second });
const btnConfirmCreate = Selector('#confirm-create-role');
const btnSave = Selector('#save-role-btn');
const btnBack = Selector('#back-role-btn');

// Test Flow
for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('create, edit and delete `role` testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(role_page_url)
				.wait(5000);

			for (const [idx, test_obj] of test_data.entries()) {
				let role_name = test_obj["text_val"];
				let role_elemet = Selector(`#rolebar-${role_name}`);
				let role_edit_btn = Selector(`#${role_name}-edit-btn`);
				let role_remove_btn = Selector(`#${role_name}-remove-btn`);
				let setting_copy = Object.assign({}, test_obj);
				delete setting_copy["text_val"];

				await t
					.click(btnCreateNewRole)
					.typeText(inputRole, role_name)
					.click(btnConfirmCreate)
					.wait(1000)
					.expect(role_elemet.exists).ok();
				
				await t
					.click(role_edit_btn)
					.wait(200)
				
				// === create role ===
				for (const [menu, behavior_list] of Object.entries(setting_copy)) {
					let menu_selector = undefined;

					switch (menu) {
						case "move_menu":
							menu_selector = moveMenu;
							break;
						case "logic_menu":
							menu_selector = logicMenu;
							break;
						case "others_menu":
							menu_selector = othersMenu;
							break;
					
						default:
							menu_selector = moveMenu;
							break;
					}

					for (const [idx, behavior] of behavior_list.entries()) {						
						let behavior_selector = undefined;
						switch (behavior) {
							case "move":
								behavior_selector = move;
								break;
							case "charge":
								behavior_selector = charge;
								break;
							case "wait":
								behavior_selector = wait;
								break;
							case "artifact":
								behavior_selector = artifact;
								break;
						
							default:
								behavior_selector = wait;
								break;
						}
						await t
							.click(menu_selector)
							.click(behavior_selector)
					}
				}

				await t
					.setNativeDialogHandler(() => true)
					.click(btnSave)
					.wait(2000)
					.click(btnBack)
					.wait(1000)
				
				// check role
				await t
					.click(role_edit_btn)
					
				for (const [menu, behavior_list] of Object.entries(setting_copy)) {
					let menu_selector = undefined;

					switch (menu) {
						case "move_menu":
							menu_selector = moveMenu;
							break;
						case "logic_menu":
							menu_selector = logicMenu;
							break;
						case "others_menu":
							menu_selector = othersMenu;
							break;
					
						default:
							menu_selector = moveMenu;
							break;
					}

					for (const [idx, behavior] of behavior_list.entries()) {						
						let check_behavior_selector = undefined;
						switch (behavior) {
							case "move":
								check_behavior_selector = divMoveVerify;
								break;
							case "charge":
								check_behavior_selector = divChargeVerify;
								break;
							case "wait":
								check_behavior_selector = divWaitVerify;
								break;
							case "artifact":
								check_behavior_selector = divArtifactVerify;
								break;
						
							default:
								check_behavior_selector = divMoveVerify;
								break;
						}
						await t
							.expect(check_behavior_selector.exists).ok()
					}
				}
				
				await t
					.click(btnBack)
					.wait(1000);
				
				// delete role
				await t
					.setNativeDialogHandler(() => true)
					.click(role_remove_btn)
					.wait(1000)
					.expect(role_elemet.exists).notOk();
			}
		});
}
