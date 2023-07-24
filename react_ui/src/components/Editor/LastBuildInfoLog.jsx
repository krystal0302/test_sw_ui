import React, { useState, useRef } from 'react';
import { useQuery } from 'react-query';
import { Button } from "react-bootstrap";
import PrintOutWindow from '../PrintOutWindow';
import { fetchGetArtifactLogs } from '../../api/FetchApi';

function LastBuildInfoLog({ targetId, buildMsg }) {

	// console.log(buildMsg);
	let buildMsg2 = `env_version: ${buildMsg['env_version']}\n` +
		`build_time: ${buildMsg['build_time']}\n` +
		`build_result: ${buildMsg['build_result']}`;
	const outputMsg2 = { "last_build": buildMsg2 || '' };

	return (
		<div className="card card-dark">
			<div className="card-header border-transparent ui-sortable-handle" style={{ cursor: "move" }}>
				<h2 className="card-title">
					<i className="fas fa-list mr-1"></i>
					<span style={{ fontFamily: "Arial", fontSize: "100%" }}>Last build info</span>
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
					<PrintOutWindow outputs={outputMsg2} category="last_build" />
				</div>
			</div>
		</div >

	)
}

export default LastBuildInfoLog;