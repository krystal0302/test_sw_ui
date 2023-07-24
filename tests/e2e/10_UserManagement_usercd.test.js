import { Selector } from 'testcafe';
import { getHostIP } from '../util';
import { faker } from '@faker-js/faker';

const { adminUserAccount, invalidUserAccount } = require('../fixtures/mock_user_data.js');

// --- Run test here ---
const hostIP = getHostIP();

var oriUserCnt = 0;
const loginAccountId = 'admin';
const newAccountId = adminUserAccount.length > 0 ? adminUserAccount[0].id : faker.datatype.uuid().replaceAll('-', '');
const newAccountPwd = adminUserAccount.length > 0 ? adminUserAccount[0].password : '@0' + faker.internet.password(8);
const newAccountName = adminUserAccount.length > 0 ? adminUserAccount[0].name : faker.internet.userName();
const newAccountPhone = adminUserAccount.length > 0 ? adminUserAccount[0].phone : faker.phone.number();
const newAccountMail = adminUserAccount.length > 0 ? adminUserAccount[0].email : faker.internet.email();
const inputLoginAccount = Selector('#user-account');
const inputLoginPassword = Selector('#user-pwd');
const btnLogin = Selector('#user-login-btn');
const btnOpenUserModal = Selector('#open-user-modal-btn');
const selectUserAuth = Selector('#user-auth');
const selectUserAuthOptions = selectUserAuth.find('option');
const inputUserId = Selector('#account-id');
const inputUserPwd = Selector('#account-pwd');
const inputUserName = Selector('#user-name');
const inputUserPhone = Selector('#phoneNum');
const inputUserMail = Selector('#mail-address');
const btnCreateUser = Selector('#create-user-btn');
const userTableRows = Selector('#user-jsGrid > .jsgrid-grid-body > table > tbody > tr');
const chkbxloginUser = userTableRows.find('td:nth-child(3)').withText(loginAccountId).sibling("td").find('.row_check');
const btnDeleteUsers = Selector('#delete-users-btn');

fixture`User Management-Admin User Account Creation & Deletion Feature`
	.page`http://${hostIP}:3000`;  // specify the entry page

test.timeouts({
	pageLoadTimeout: 1000,
})
	('create and delete admin user testing', async t => {
		await t
			.typeText(inputLoginAccount, loginAccountId)
			.typeText(inputLoginPassword, 'admin')
			.click(btnLogin)
			.navigateTo(`http://${hostIP}:3000/manage_user.html`)
			.wait(2000)
			.click(btnOpenUserModal);

		oriUserCnt = await userTableRows.count;

		for (let i = 0; i < invalidUserAccount.length; i++) {
			let invalidAccount = invalidUserAccount[i];

			// -- verify if any of the fields is not filled or invalid --
			await t
				.click(selectUserAuth)
				.click(selectUserAuthOptions.withText('Admin'))
				.typeText(inputUserId, loginAccountId, { replace: true })
				.typeText(inputUserPwd, newAccountPwd, { replace: true })
				.typeText(inputUserPhone, invalidAccount.phone, { replace: true })
				.typeText(inputUserMail, invalidAccount.email, { replace: true })
				.selectText(inputUserName)
				.pressKey('delete')
				.click(btnCreateUser)
				.expect(inputUserName.hasClass('is-invalid')).ok()
				.expect(inputUserPhone.hasClass('is-invalid')).ok()
				.expect(inputUserMail.hasClass('is-invalid')).ok()
				.wait(1000);

			// -- same account testing --
			await t
				.typeText(inputUserName, newAccountName, { replace: true })
				.typeText(inputUserPhone, newAccountPhone, { replace: true })
				.typeText(inputUserMail, newAccountMail, { replace: true })
				.setNativeDialogHandler(() => true)
				.click(btnCreateUser)
				.expect(userTableRows.count).eql(oriUserCnt)
				.wait(1000);

			// -- account special characters testing --
			await t
				.typeText(inputUserId, invalidAccount.special_symbol_id, { replace: true })
				.click(btnCreateUser)
				.expect(inputUserId.hasClass('is-invalid')).ok()
				.typeText(inputUserId, invalidAccount.space_id, { replace: true })
				.click(btnCreateUser)
				.expect(inputUserId.hasClass('is-invalid')).ok()
				.typeText(inputUserId, invalidAccount.chinese_id, { replace: true })
				.click(btnCreateUser)
				.expect(inputUserId.hasClass('is-invalid')).ok()
				.typeText(inputUserId, invalidAccount.fullwidth_id, { replace: true })
				.click(btnCreateUser)
				.expect(inputUserId.hasClass('is-invalid')).ok()
		}

		// -- create account testing --
		await t
			.typeText(inputUserId, newAccountId, { replace: true })
			.wait(200);
		
		const tmp_user_id = await inputUserId.value;
		await t
			.click(btnCreateUser)
			.expect(userTableRows.count).eql(oriUserCnt + 1)
			.wait(1000);

		// -- delete account testing --
		await t
			.click(userTableRows.find('td:nth-child(3)').withText(tmp_user_id).sibling("td").find('.row_check'))
			.click(btnDeleteUsers)
			.expect(userTableRows.count).eql(oriUserCnt)
			.expect(chkbxloginUser.getAttribute('disabled')).eql('disabled');
	});