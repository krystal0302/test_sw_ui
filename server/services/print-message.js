/**
 * @param String ip: The users local IP address or hostname
 * @param Integer port: the port number 
 * @returns A string formatted for the terminal
 */
module.exports = (ip, port) => {
	// --- color codes used in the message ---
	const colors = {
		RESET: '\x1b[0m',
		CYAN: '\x1b[36m',
		GREEN: '\x1b[32m',
		BLUE: '\x1b[34m',
		BRIGHT: '\x1b[1m',
		BR: '\n',
	};

	// --- functions to insert string of set length of characters --
	const printChars = (count, char) => new Array(count).fill(char).join('');
	const line = (count) => printChars(count, '━');
	const blanks = (count) => printChars(count, ' ');

	// --- FARobot logo  ---
	const title = `${colors.BLUE}\n\n`
		+ ' ███████╗ █████╗ ██████╗  ██████╗ ██████╗  ██████╗ ████████╗\n'
		+ ' ██╔════╝██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝\n'
		+ ' █████╗  ███████║██████╔╝██║   ██║██████╔╝██║   ██║   ██║   \n'
		+ ' ██╔══╝  ██╔══██║██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   \n'
		+ ' ██║     ██║  ██║██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   \n'
		+ ` ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝   \n${colors.RESET}`;

	// --- Messages for users of running app ---
	let msg = `${colors.GREEN}┏${line(60)}┓${colors.BR}`
		+ `┃ ${colors.CYAN}Welcome to FARobot! 🚀${blanks(37)}${colors.GREEN}┃${colors.BR}`
		+ `┃ ${colors.CYAN}Swarm UI is running at ${colors.BRIGHT}`
		+ `http://${ip}:${port}${colors.RESET}${blanks(24 - ip.length)}${colors.GREEN}┃${colors.BR}`
		+ `┗${line(60)}┛${colors.BR}${colors.BR}${colors.RESET}`;

	return title + msg;
};