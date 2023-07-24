import React from "react";

/*
0: INFO
1: INFO
2: SUCCESS
3: WARN
4: ERROR
*/
const colors = ["white", "white", "lime", "orange", "red"];

const PrintOutWindow = ({ outputs, category }) => {
  // --- protection ---
  // category = category || 'wrapper';
  outputs = (outputs) ? outputs[category] : '';

  outputs = outputs?.split('\n');
  outputs = outputs?.map(o => {
    let status = 0;
    if (o.includes('INFO')) status = 1;
    if (o.includes('SUCCESS')) status = 2;
    if (o.includes('WARN')) status = 3;
    if (o.includes('ERROR')) status = 4;
    return { status: status, message: o }
  });
  const messages = outputs?.map(om => ({ ...om, "color": colors[om.status] }));
  // console.log(messages)

  return (
    <div
      className="rounded-md text-white"
      style={{ width: "100%", height: "100%", backgroundColor: "black", borderRadius: "10px", textAlign: "left", "overflowY": "auto", "overflowX": "auto" }}>
      <div style={{ paddingTop: "1rem" }}>
        {(messages) ? messages.map((msg, index) => <pre key={index} className="px-4 py-0 font-normal" style={{ color: msg.color, display: "block", whiteSpace: "pre-wrap" }}>{msg.message}</pre>) : null}
      </div>
    </div>
  );
};

export default PrintOutWindow;
