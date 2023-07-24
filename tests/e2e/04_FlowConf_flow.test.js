import { Selector } from 'testcafe';
import { getTestSetting, getRandomInt } from '../util';

// --- Run test here ---
const page = "operation";
const general_setting = getTestSetting(page);
const test_data = general_setting["test_data"];
const home_url = general_setting["homeUrl"];
const login_text = general_setting["login_account"];
const pwd_text = general_setting["login_pwd"];
const test_repeat_times = general_setting["repeat_times"];
const element_wait_second = general_setting["element_wait_second"]*1000;

const operation_page_url = `${home_url}/${page}.html`;

fixture`Flow Configuration Flow-Edit Feature`
	.page(home_url);  // specify the entry page


const test_count = 4;

// Selector
// Login
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');


const inputFlowName = Selector('#flow-name');
const btnAddFlow = Selector('#flow-item', { timeout: element_wait_second });
const btnCreateFlow = Selector('#add-flow');
const drawflow = Selector('#drawflow');
const btnSaveFlow = Selector('#save-flow-div');
const btnBack = Selector('#flow-back');

const startOutput = Selector('#node-2 > .outputs > .output_1'); 
const FinishInput = Selector('#node-3 > .inputs > .input_1');

const taskTriggerSelect = Selector('#flow_select');
const triggerOption = taskTriggerSelect.find('option');

const notificationTitle = Selector('.notification-title', { timeout: element_wait_second });

// const test_data = [
// 	{
// 		"text_val": "_a",
// 		"moveTo": { offsetX: 100, offsetY: 100 }
// 	},
// 	{
// 		"text_val": "_b",
// 		"moveTo": { offsetX: 200, offsetY: 200 }
// 	},
// 	{
// 		"text_val": "_c",
// 		"moveTo": { offsetX: 300, offsetY: 300 }
// 	}	
// ];

console.log(test_data)

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('create, edit and delete `flow` testing', async t => {
			await t
				.typeText(inputLoginAccount, login_text)
				.typeText(inputLoginPassword, pwd_text)
				.click(btnLogin)
				.wait(5000)
				.navigateTo(operation_page_url)
				.wait(5000);

			for (const [idx, test_obj] of test_data.entries()) {
				const randomName = test_obj["text_val"];
				const createdFlowItem = Selector(`#flowbar-${randomName}`);
				const btnEditFlow = Selector(`#edit-${randomName}`);
				const btnDeleteFlow = Selector(`#delete-flow-${randomName}`);
				const flowType = "Manual";

				// === create flow ===
				/** 
				 * criterias: 
				 *   1. the flow bar SHOULD exist in flows list 
				 */
				await t
					.scrollIntoView(btnAddFlow)
					.click(btnAddFlow)
					.typeText(inputFlowName, randomName)
					.click(btnCreateFlow)
					.wait(1000)
					.expect(createdFlowItem.exists).ok();

				// === edit flow ===
				// --- composing a flow ---
				await t
					.click(btnEditFlow)
					.wait(1000)
					.click(taskTriggerSelect)
					.click(triggerOption.withText(flowType));

				// Gen role list
				let role_list = [];
				if (test_obj["roles"].length == 0){
					const elements = Selector('.drag-drawflow > span');
					let total_roles = await elements.count;
					let avaible_roles = [];

					for(let i = 0; i < total_roles; i++) {
						const avaibleRole = await elements.nth(i).innerText;
						avaible_roles.push(avaibleRole);
					}

					for (var i = 0; i < test_count; i++) {
						role_list.push(avaible_roles[getRandomInt(avaible_roles.length)]);
					}
				}else{
					role_list = test_obj["roles"];
				}

				// Drag role to canvas
				role_list.forEach(async function (role, role_index) {
					let roleSelector = Selector(`[data-node=${role}]`);
					let base_idx = 3 + 1;
					const nodeInput = Selector(`#node-${base_idx+role_index} > .inputs > .input_1`);
					const nodeOutput = Selector(`#node-${base_idx+role_index} > .outputs > .output_1`); 
					let connection_count = Selector('.connection').count

					await t
						.dragToElement(roleSelector, drawflow)
						.dragToElement(startOutput, nodeInput)
						.expect(connection_count).eql(2*role_index + 1)
						.dragToElement(nodeOutput, FinishInput)
						.expect(connection_count).eql(2*role_index + 2)
						.wait(1000);
				});

				await t
					.click(btnSaveFlow)
					.expect(notificationTitle.withText(/Success/).exists).ok();

				// === delete flow ===
				/** 
				 * criterias: 
				 *   1. the flow bar SHOULD NOT exist in flows list 
				 */
				await t
					.click(btnBack)
					.setNativeDialogHandler(() => true)
					.dispatchEvent(btnDeleteFlow, 'click')
					.wait(1000)
					.expect(createdFlowItem.exists).notOk();
			}
		});
}
