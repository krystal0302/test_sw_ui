/*
 * Author: John Wu
 * Date: 01 Oct. 21,
 * Description:
 *   Documenting Swarm Core Resources APIs  
 **/
const getHostIP = require('./util').getHostIP;
const swaggerDefinition = {
	info: {
		title: "Swarm Core Resources APIs",
		description: "RESTful APIs for developers to get resources on Swarm Core host",
		version: "0.1.0",
	},
	servers: [{
		url: `http://${getHostIP}:3000`,
		description: "Swarm-Core Local Server"
	}]
}

const swaggerJsDoc = require('swagger-jsdoc');
const swaggerUi = require('swagger-ui-express');

const swaggerOptions = {
	swaggerDefinition,
	apis: ["./routes/*.js"]
};

/**
 * Configure Swagger UI.
 * @param {express} app Application express
 */
const setup = app => app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(swaggerJsDoc(swaggerOptions)));

module.exports = setup;