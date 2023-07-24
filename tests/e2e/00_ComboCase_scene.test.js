import { Selector } from 'testcafe';
import { getHostIP } from '../util';
import { faker } from '@faker-js/faker';

const hostIP = getHostIP();

// ======== Testing Configuration ========
// const urlEntryPoint = `http://${hostIP}:3000`
const urlDashboard = `http://${hostIP}:3000/index.html`
const urlFleetConfiguration = `http://${hostIP}:3000/fleet.html`
const urlFlowConfiguration = `http://${hostIP}:3000/operation.html`
const fakeFlowName = faker.system.fileName().replace(/\.[^\/.]+$/, '');

// ======== Testing Selectors ========
// --- [Common] navigation sidebar (left-hand side) ---
const lsbFlowTrigger = Selector(`#lsb-${fakeFlowName}`); // Left-hand Side-Bar flow trigger

// --- [Login] ---
const inputUserAccount = Selector('#user-account');
const inputUserPassword = Selector('#user-pwd');
const btnUserLogin = Selector("#user-login-btn");

// --- [Dashboard]  ---
const spanFlowsListFlowName = Selector('#flows-table > tbody > tr > td > span');
const labelAgentMode = Selector('#fb_0-mode-label');

// --- [Flow configuration] ---
const btnCreateFlow = Selector('#flow-item');
const inputFlowName = Selector('#flow-name');
const btnConfirmAddFlow = Selector('#add-flow');
const btnEditFlow = Selector(`#edit-${fakeFlowName}`);
const btnSaveFlow = Selector('#save-flow-div');
const divRole_rack_to_rack = Selector('[data-node="rack_to_rack"]');
const divDrawFlow = Selector('#drawflow');

const selectTaskTrigger = Selector('#flow_select');
const optionTaskTrigger = selectTaskTrigger.find('option');
const btnConfirmFlow = Selector('#add-manual');

// --- [Flow configuration] build flow connection ---
const labelAgentFleetStatus = Selector('.agent-fleet-status');

// --- [Flow configuration] build flow connection ---
const divStartNodeOutput = Selector('#node-2 > .outputs > .output_1');
const divFinishNodeInput = Selector('#node-3 > .inputs > .input_1');
const div_rack_to_rack_NodeInput = Selector('#node-4 > .inputs > .input_1');
const div_rack_to_rack_NodeOutput = Selector('#node-4 > .outputs > .output_1');
const btn_rack_to_rack_NodeConfig = Selector('#node-4 > .drawflow_content_node > div > div.box > button');

// --- [Flow configuration] configure flow parameters ---
const select_rack_to_rack_1MoveArea = Selector('#goal_ja7tD-area');
const option_rack_to_rack_1MoveArea = select_rack_to_rack_1MoveArea.find('option');
// TODO: refactor the identifiers
// const select_rack_to_rack_1MoveCell = Selector('#goal_ja7tD-cell');
const select_rack_to_rack_1MoveCell = select_rack_to_rack_1MoveArea.nextSibling();
const option_rack_to_rack_1MoveCell = select_rack_to_rack_1MoveCell.find('option');

const select_rack_to_rack_2DockArea = Selector('#goal_Jhb6L-area');
const option_rack_to_rack_2DockArea = select_rack_to_rack_2DockArea.find('option');
// TODO: refactor the identifiers
const select_rack_to_rack_2DockCell = select_rack_to_rack_2DockArea.nextSibling();
const option_rack_to_rack_2DockCell = select_rack_to_rack_2DockCell.find('option');

const select_rack_to_rack_3MoveArea = Selector('#goal_KMrDs-area');
const option_rack_to_rack_3MoveArea = select_rack_to_rack_3MoveArea.find('option');
// TODO: refactor the identifiers
const select_rack_to_rack_3MoveCell = select_rack_to_rack_3MoveArea.nextSibling();
const option_rack_to_rack_3MoveCell = select_rack_to_rack_3MoveCell.find('option');

const select_rack_to_rack_4MoveArea = Selector('#goal_3bgqA-area');
const option_rack_to_rack_4MoveArea = select_rack_to_rack_4MoveArea.find('option');
// TODO: refactor the identifiers
const select_rack_to_rack_4MoveCell = select_rack_to_rack_4MoveArea.nextSibling();
const option_rack_to_rack_4MoveCell = select_rack_to_rack_4MoveCell.find('option');

// ======== Testing Cases ========
fixture`Dashboard Flow Pause-Resume Feature`
	.page`http://${hostIP}:3000`;

for (let i = 0; i < 1; ++i) {
	test.timeouts({
		pageLoadTimeout: 1000,
	})
		('Usecase combo testing - flow dispatchment', async t => {
			await t
				.typeText(inputUserAccount, 'admin')
				.typeText(inputUserPassword, 'admin')
				.click(btnUserLogin)
				.navigateTo(urlDashboard);

			for (let j = 0; j < 1; ++j) {
				// --- [pre-requisites] no duplicated flow-name ---
				try {
					await t
						.expect(Selector(lsbFlowTrigger).exists).ok();
				} catch (e) {
					await t
						// --- redirect to flow creation page ---
						.setNativeDialogHandler(() => true)
						.navigateTo(urlFlowConfiguration)
						.wait(1000)
						// --- create a flow ---
						.click(btnCreateFlow)
						.typeText(inputFlowName, fakeFlowName)
						.click(btnConfirmAddFlow)
						.wait(1000)
						// --- compose the flow ---
						.click(btnEditFlow)
						.wait(1000)
						.click(selectTaskTrigger)
						.click(optionTaskTrigger.withText('Manual'))
						.dragToElement(divRole_rack_to_rack, divDrawFlow)
						.dragToElement(divStartNodeOutput, div_rack_to_rack_NodeInput)
						.dragToElement(div_rack_to_rack_NodeOutput, divFinishNodeInput)
						// TODO: fill out the valid parameters
						// --- config the flow ---
						.click(btn_rack_to_rack_NodeConfig)
						.click(select_rack_to_rack_1MoveArea)
						.click(option_rack_to_rack_1MoveArea.withText('Rack_one'))
						.click(select_rack_to_rack_1MoveCell)
						.click(option_rack_to_rack_1MoveCell.withText('11701'))
						.click(select_rack_to_rack_2DockArea)
						.click(option_rack_to_rack_2DockArea.withText('Rack_one'))
						.click(select_rack_to_rack_2DockCell)
						.click(option_rack_to_rack_2DockCell.withText('11701'))
						.click(select_rack_to_rack_3MoveArea)
						.click(option_rack_to_rack_3MoveArea.withText('A_temp_area'))
						.click(select_rack_to_rack_3MoveCell)
						.click(option_rack_to_rack_3MoveCell.withText('a_temp_cell'))
						.click(select_rack_to_rack_4MoveArea)
						.click(option_rack_to_rack_4MoveArea.withText('A_temp_area'))
						.click(select_rack_to_rack_4MoveCell)
						.click(option_rack_to_rack_4MoveCell.withText('a_temp_cell'))
						.wait(1000)
						.click(divDrawFlow)
						.setNativeDialogHandler(() => true)
						.click(btnSaveFlow)
						// --- redirect to dashboard page ---
						.setNativeDialogHandler(() => true)
						.expect(Selector(lsbFlowTrigger).exists).ok()
						.navigateTo(urlDashboard);
				}

				// TODO: [pre-requisites] available agents exist ---
				try {
					console.log(`!!! avalaible agents: ${labelAgentFleetStatus.withText('auto').count}`)
					await t
						.expect(labelAgentFleetStatus.withText('auto').count).eql(1);
				} catch (e) {
					console.error(e);
					// await t
					// 	// --- redirect to flow creation page ---
					// 	.setNativeDialogHandler(() => true)
					// 	.navigateTo(urlFleetConfiguration)
					// 	.wait(1000)
				}

				// TODO: [pre-requisites] existing agents initialized ---

				// --- [main] verify flow dispatchment testing  ---
				await t
					// --- expect dispatched flow will be shown in 'Flows List' ---
					.click(lsbFlowTrigger)
					.click(btnConfirmFlow)
					.wait(2000)
					.expect(spanFlowsListFlowName.withText(fakeFlowName).count).eql(1)
					.wait(200)
					// --- expect the designated agent status is active ---
					.expect(labelAgentMode.withText('ACTIVE').count).eql(1)
					.wait(1000);
			}
		});


}
