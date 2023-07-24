const options = {
	path: 'crop.png',
	fullPage: false,
	clip: {
		x: 0,
		y: 0,
		width: 1920,
		height: 1080
	}
}

const hostURL = 'http://localhost:3000';

describe('Map Editor testing', () => {

	beforeAll(async () => {
		// await browser.newPage();
		await page.setViewport({ width: 1920, height: 1080 })
		await page.goto(hostURL, { waitUntil: 'load' })
		await page.waitForSelector('#user-account');
		await page.type('#user-account', 'admin')
		await page.type('#user-pwd', 'admin')
		await page.click('#user-login-btn')

		await page.waitFor(3000)
		page.on('dialog', async dialog => {
			console.log(dialog.message());
			await dialog.accept();
		});
	})

	it('should make a map editor snapshot', async () => {
		await page.goto(`${hostURL}/map.html`, { waitUntil: 'load' })

		await page.waitForSelector('#far-network2 > div > canvas', { waitUntil: 'load' })
		await page.waitForTimeout(8000);
		await page.screenshot(options)
	});
});