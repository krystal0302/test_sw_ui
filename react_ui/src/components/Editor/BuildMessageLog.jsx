import React, { useState } from 'react';
import { useQuery } from 'react-query';
import { Button } from "react-bootstrap";
import PrintOutWindow from '../PrintOutWindow';
import { fetchGetArtifactLogs } from '../../api/FetchApi';

function BuildMessageLog({ targetId, dbgMsg }) {

	const outputMsg = { debug_msg: dbgMsg['debug_msg'] || '' };


	return (
		<div className="card card-dark">
			<div className="card-header border-transparent ui-sortable-handle" style={{ cursor: "move" }}>
				<h2 className="card-title">
					<i className="fas fa-list mr-1"></i>
					<span style={{ fontFamily: "Arial", fontSize: "100%" }}>build message</span>
				</h2>
				<div className="card-tools">
					<button type="button" className="btn btn-tool" data-card-widget="collapse">
						<i className="fas fa-minus"></i>
					</button>
				</div>
			</div>
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "300px", display: "block", overflowY: "auto" }}>
				<div style={{ padding: "0 0 .0rem 0", display: "flex" }}>
				</div>
				<div className="" style={{ height: "100%" }}>
					<PrintOutWindow outputs={outputMsg} category="debug_msg" />
				</div>
			</div>
		</div >

	)
}

export default BuildMessageLog;