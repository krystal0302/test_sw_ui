import { Selector } from 'testcafe';
import { getHostIP } from '../util';

const { getGeneralUserFunctionNames, getAdminUserFunctionNames } = require('../../server/public/dist/js/pages/far_comm_data')
const { adminUserAccount } = require('../fixtures/mock_user_data.js');

// --- Run test here ---
const hostIP = getHostIP();

const urlUserManagement = `http://${hostIP}:3000/manage_user.html`;
const urlSystemSettings = `http://${hostIP}:3000/settings2.html`;

const btnLogout = Selector('#user-logout-btn');
const lsbFunctions = Selector('.function-link > a > p');

// --- [Login]  ---
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');

// --- [User Management]  ---
const selectUserAuth = Selector('#user-auth');
const selectUserAuthOptions = selectUserAuth.find('option');
const inputUserId = Selector('#account-id');
const inputUserPwd = Selector('#account-pwd');
const inputUserName = Selector('#user-name');
const inputUserPhone = Selector('#phoneNum');
const inputUserMail = Selector('#mail-address');
const userTableRows = Selector('#user-jsGrid > .jsgrid-grid-body > table > tbody > tr');
const btnOpenUserModal = Selector('#open-user-modal-btn');
const btnCreateUser = Selector('#create-user-btn');
const btnSaveUserChanges = Selector('#save-users-btn');

// --- [System Settings]  ---
const navTabUser = Selector('#tab-user-settings');
const divEditPhone = Selector('#user-phone-edit');
const divEditMail = Selector('#user-email-edit');
const inputAccount = Selector('#user-account');
const inputPhone = Selector('#user-phone');
const inputMail = Selector('#user-email');
const btnUpdateUser = Selector('#update-user-btn');

fixture`System Settings-User Settings & Permissions Modification`
	.page`http://${hostIP}:3000`;  // specify the entry page

test.timeouts({
	pageLoadTimeout: 1000,
})
	('modify user settings & permissions testing', async t => {
		await t
			.typeText(inputLoginAccount, 'admin')
			.typeText(inputLoginPassword, 'admin')
			.click(btnLogin)
			.navigateTo(urlUserManagement)
			.wait(2000);

		for (let i = 0; i < adminUserAccount.length; i++) {
			let accountObj = adminUserAccount[i];

			// -- create a new admin test user --
			await t
				.click(btnOpenUserModal)
				.click(selectUserAuth)
				.click(selectUserAuthOptions.withText('Admin'))
				.typeText(inputUserId, accountObj.id)
				.typeText(inputUserPwd, accountObj.password)
				.typeText(inputUserName, accountObj.name)
				.typeText(inputUserPhone, accountObj.phone)
				.typeText(inputUserMail, accountObj.email)
				.setNativeDialogHandler(() => true)
				.click(btnCreateUser)
				.click(btnLogout);

			// -- verify account id can't be updated --
			await t
				.typeText(inputLoginAccount, accountObj.id)
				.typeText(inputLoginPassword, accountObj.password)
				.click(btnLogin)
				.navigateTo(urlSystemSettings)
				.wait(1000)
				.click(navTabUser)
				.expect(inputAccount.getAttribute('readonly')).eql('true')
				.click(divEditPhone)
				.typeText(inputPhone, accountObj.phone2, { replace: true })
				.click(divEditMail)
				.typeText(inputMail, accountObj.email2, { replace: true })
				.click(btnUpdateUser);

			await t.eval(() => location.reload(true));

			// -- verify settings are updated --
			await t
				.click(navTabUser)
				.expect(inputPhone.value).eql(accountObj.phone2)
				.expect(inputMail.value).eql(accountObj.email2)
				.click(btnLogout);

			// -- change user permission from admin to non-admin --
			await t
				.typeText(inputLoginAccount, 'admin')
				.typeText(inputLoginPassword, 'admin')
				.click(btnLogin)
				.navigateTo(urlUserManagement)
				.wait(1000)
				.click(userTableRows.find('td:nth-child(3)').withText(accountObj.id).sibling("td").find('.row_check'))
				.click(userTableRows.find('td:nth-child(3)').withText(accountObj.id).sibling("td").find('.switch'))
				.click(btnSaveUserChanges)
				.wait(1000)
				.click(btnLogout);

			// -- verify the sidebar function list content --
			await t
				.typeText(inputLoginAccount, accountObj.id)
				.typeText(inputLoginPassword, accountObj.password)
				.click(btnLogin)
				.wait(1000)

			var functionCnt = await lsbFunctions.count;
			var functionNameArr = [];
			for (let j = 0; j < functionCnt; j++) {
				let functionName = await lsbFunctions.nth(j).innerText;
				functionNameArr.push(functionName);
			}
			await t
				.expect(functionNameArr).eql(getGeneralUserFunctionNames());

			// -- change user permission from non-admin to admin --
			await t
				.click(btnLogout)
				.typeText(inputLoginAccount, 'admin')
				.typeText(inputLoginPassword, 'admin')
				.click(btnLogin)
				.navigateTo(urlUserManagement)
				.wait(1000)
				.click(userTableRows.find('td:nth-child(3)').withText(accountObj.id).sibling("td").find('.row_check'))
				.click(userTableRows.find('td:nth-child(3)').withText(accountObj.id).sibling("td").find('.switch'))
				.click(btnSaveUserChanges)
				.wait(1000)
				.click(btnLogout);

			// -- verify the sidebar function list content --
			await t
				.typeText(inputLoginAccount, accountObj.id)
				.typeText(inputLoginPassword, accountObj.password)
				.click(btnLogin)
				.wait(1000)

			functionCnt = await lsbFunctions.count;
			functionNameArr = [];
			for (let j = 0; j < functionCnt; j++) {
				let functionName = await lsbFunctions.nth(j).innerText;
				functionNameArr.push(functionName);
			}
			await t
				.expect(functionNameArr).eql(getAdminUserFunctionNames());

			// -- recovery: delete test user --
			await t
				.click(btnLogout)
				.typeText(inputLoginAccount, 'admin')
				.typeText(inputLoginPassword, 'admin')
				.click(btnLogin)
				.navigateTo(urlUserManagement)
				.click(userTableRows.find('td:nth-child(3)').withText(accountObj.id).sibling("td").find('.jsgrid-delete-button'));
		}
	});

