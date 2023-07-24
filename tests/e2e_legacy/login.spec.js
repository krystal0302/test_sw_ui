const { browserContext } = require("../../jest-puppeteer.config")

const hostURL = 'http://localhost:3000'

describe('Login page tests', () => {
	beforeAll(async () => {
		await page.goto(hostURL)
	})

	it('should display "Sign in" text on page', async () => {
		await expect(page).toMatch('Sign in')
	})

})

describe('Login behavior tests', () => {
	beforeAll(async () => {
		await page.goto(hostURL)
	})

	it('should login successfully', async () => {
		await page.waitForSelector('#user-account', { timeout: 5000 }); // 超過5秒就timeout
		await page.type('#user-account', 'admin')
		await page.type('#user-pwd', 'admin')
		await page.click('#user-login-btn')
		page.on('dialog', async dialog => {
			console.log(dialog.message());
			await dialog.accept();
		});
		await page.waitForNavigation()

		let text = await page.evaluate(() => document.body.textContent)
		expect(text).toContain("Sign out")
	})

});