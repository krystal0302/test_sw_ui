import { Selector, t } from 'testcafe';
import { getTestSetting } from '../util';

export class Login {
    constructor (setting_path='') {
        this.settings = getTestSetting('', setting_path);
        this.login_text = this.settings["login_account"];
        this.pwd_text = this.settings["login_pwd"];
        this.element_timeout = this.settings["element_wait_second"] * 1000;

        this.account_element  = Selector("#user-account");
        this.password_element = Selector("#user-pwd");
        this.btnlogin_element = Selector("#user-login-btn");
        this.btnsignout_element = Selector("button",  { timeout: this.element_timeout }).withText("Sign out");
    }

    async login() {
        await t
            .maximizeWindow()
            .typeText(this.account_element, this.login_text)
            .typeText(this.password_element, this.pwd_text)
            .click(this.btnlogin_element)
            .wait(5000)
            .expect(this.btnsignout_element.exists).ok();
    }
}