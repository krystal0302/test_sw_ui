const hostURL = 'http://localhost:3000'

describe('Swarm System connection check', () => {
	beforeAll(async () => {
		await page.goto(hostURL, { waitUntil: 'load' })
		await page.waitForSelector('#user-account'); // 超過5秒就timeout
		await page.type('#user-account', 'admin')
		await page.type('#user-pwd', 'admin')
		await page.click('#user-login-btn')

		await page.waitForTimeout(1000)
		page.on('dialog', async dialog => {
			console.log(dialog.message());
			await dialog.accept();
		});
	})

	test('User Settings page should be connection: ON', async () => {
		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Dashboard page should be connection: ON', async () => {
		await page.goto(`${hostURL}/index.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Fleet page should be connection: ON', async () => {
		await page.goto(`${hostURL}/fleet.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Log page should be connection: ON', async () => {
		await page.goto(`${hostURL}/log.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Manage User page should be connection: ON', async () => {
		await page.goto(`${hostURL}/manage_user.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Live Map page should be connection: ON', async () => {
		await page.goto(`${hostURL}/map_live.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Map page should be connection: ON', async () => {
		await page.goto(`${hostURL}/map.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Operation page should be connection: ON', async () => {
		await page.goto(`${hostURL}/operation.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Role page should be connection: ON', async () => {
		await page.goto(`${hostURL}/role.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('Settings page should be connection: ON', async () => {
		await page.goto(`${hostURL}/settings.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('SLAM page should be connection: ON', async () => {
		await page.goto(`${hostURL}/slam.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})

	test('User Settings page should be connection: ON', async () => {
		await page.goto(`${hostURL}/user_settings.html`, { waitUntil: 'load' })

		await page.waitForSelector('#statusIndicator', { visible: true });
		await page.waitForTimeout(1000)
		const statusText = await page.$eval('#statusIndicator', (el) => el.textContent);
		console.log(statusText)
		expect(statusText).toBe('connection: ON')
	})
});

