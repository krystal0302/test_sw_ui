let ws;
let reconnectServer;
let broadcastChannel = new BroadcastChannel("WebSocketChannel");

function connectServer() {
  // --- open a WebSocket connection. should be opened only once ---
  ws = new WebSocket(`ws://${self.location.hostname}:8081`);

  ws.onopen = function () {
    console.log(' --- web socket connection opened --- ');
    // heartbeat();
    broadcastChannel.postMessage({ type: "WSState", state: ws.readyState });
  }

  // ws.on('ping', heartbeat);
  ws.onclose = function () {
    console.log(' --- web socket connection closed --- ');
    // clearTimeout(this.pingTimeout);
    setTimeout(function () {
      connectServer();
    }, 600);
    broadcastChannel.postMessage({ type: "WSState", state: ws.readyState });
  }

  ws.onerror = function (e) {
    console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
    ws.close();
  }

  // --- receive data from the server ---
  ws.onmessage = ({ data }) => {
    // console.log(data);
    msgObj = JSON.parse(data);
    // console.log(msgObj);
    const parsedData = { type: 'message', data: msgObj };
    // --- broadcast the recieving messages ---
    broadcastChannel.postMessage(parsedData);
  };
}

connectServer();

// ---  on a tab (context) tries to connect to this worker ---
onconnect = e => {
  const port = e.ports[0];
  console.log(e.ports);
  // console.log(ws);
  if (ws.readyState === WebSocket.CLOSED) {
    connectServer();
  }
  port.onmessage = msg => {
    console.log(msg);
   // --- forward messages to the ws connection ---
    ws.send(JSON.stringify(msg.data));
  };

  // --- notify the current state of WS connection --- 
  port.postMessage({ type: "WSState", state: ws.readyState });
};

// ======== Dashboard Info Requests =============
function fetchAgentsList() {
  return fetch(`http://${self.location.hostname}:3000/swarm/statistics/agents/list-view`, {
    method: "POST",
    headers: {
      accept: "application/json",
    },
  })
    .then((response) => response.json())
    .then((data) => {
      return data;
    });
}

function fetchFlowsList() {
  return fetch(`http://${self.location.hostname}:3000/swarm/statistics/flows/list-view`, {
    method: "POST",
    headers: {
      accept: "application/json",
    },
  })
    .then((response) => response.json())
    .then((data) => {
      return data;
    });
}

function fetchFlowsStatistics() {
  return fetch(`http://${self.location.hostname}:3000/swarm/statistics/`, {
    method: "POST",
    headers: {
      accept: "application/json",
    },
  })
    .then((response) => response.json())
    .then((data) => {
      return data;
    });
}

async function runApiRequest() {
  var agentList = await fetchAgentsList();
  var flowList = await fetchFlowsList();
  var flowFigures = await fetchFlowsStatistics();
  broadcastChannel.postMessage({
    agentListView: agentList,
    flowListView: flowList,
    flowFigures: flowFigures,
    type: "apiMessages"
  });
}

setInterval(runApiRequest, 800);
