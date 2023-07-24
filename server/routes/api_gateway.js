const request = require('request');
const hostIP = require('../util').getHostIP();

module.exports = class SwarmCoreAPIRequest {
    constructor () {
        this.home_url = hostIP;
        this.port = 5000;
        this.api_version = "/v2";
        this.username = "root";
        this.password = "root@farobot";
        this.access_token = "",
        this.token_type = ""
    }

    test(){
        console.log('AAAAAAAAAA!!!!!!!!!!!!!!!!!!!!!')
        console.log(this.home_url)
    }

    getAccessTokenPromis(){
        let self = this;
        return new Promise(function (resolve, reject) {
            request.post({
                async: true,
                followAllRedirects: true,
                headers: {
                    accept: "application/json",
                    "Content-Type": "application/x-www-form-urlencoded",
                },
                url: `http://${self.home_url}:${self.port}/login/access-token/`,
                body: `username=${self.username}&password=${self.password}`
            }, function(error, response, body){
                if (!error && response.statusCode == 200) {
                    console.log('A!')
                    console.log('body:', typeof body);
                    let tk_obj = JSON.parse(body);
                    self.access_token = tk_obj["access_token"];
                    self.token_type = tk_obj["token_type"];
                    token = tk_obj;
                    resolve(body)
                }else{
                    console.log('B!')
                    console.log('error', error, response && response.statusCode);
                    reject(error);
                }
            });
        })
    }

    getAccessToken(){    
        console.log('AAAAAAAAAA!!!!!!!!!!!!!!!!!!!!VVVVVVVVVVVVVVVVVVVVVVV!')
        let token = undefined;
        let self = this;
        token = self.getAccessTokenPromis();

        return token;
    }

    async getFleetStatus() {
        // v2/fleets/status
    // let a = await t.request({
    //             url: `${api_url}/v2/fleets/status`,
    //             headers: {
    //                 accept: "application/json",
    //                 Authorization: `${token.token_type} ${token.access_token}`,
    //                 'Content-Type': 'application/json'
    //             }, 
    //             auth: {
    //                 username: 'admin', 
    //                 password: 'admin'
    //             },
    //             params: {
    //                 fleet_name: 'AUO', 
    //                 mode: 'agent_only'
    //             },
    //             timeout: 40000
    //         });
    }

    async getRobotsScan() {
        let token = await this.getAccessToken();

        let res = await t.request({
            url: `${this.api_url}${this.api_version}/robots/scan`,
            headers: {
                accept: "application/json",
                Authorization: `${token.token_type} ${token.access_token}`,
                'Content-Type': 'application/json'
            },
            timeout: 40000
        });

        return res.body;
    }

    async getArtifactsScan() {
        let token = await this.getAccessToken();

        let res = await t.request({
            url: `${this.api_url}${this.api_version}/artifacts/scan`,
            headers: {
                accept: "application/json",
                Authorization: `${token.token_type} ${token.access_token}`,
                'Content-Type': 'application/json'
            },
            timeout: 40000
        });

        return res.body;
    }

    async getFlowsStatus(fleet_name='') {
        let token = await this.getAccessToken();

        let res = await t.request({
            url: `${this.api_url}${this.api_version}/flows/status`,
            headers: {
                accept: "application/json",
                Authorization: `${token.token_type} ${token.access_token}`,
                'Content-Type': 'application/json'
            },
            params: {
                fleet_name: fleet_name, 
            },
            timeout: 40000
        });

        return res.body;
    }
}