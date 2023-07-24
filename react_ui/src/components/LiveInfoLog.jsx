import React, { useState, useContext } from 'react';
import { useQuery } from 'react-query';
import { Button } from "react-bootstrap";
import PrintOutWindow from './PrintOutWindow';
import { fetchGetArtifactLogs } from '../api/FetchApi';
import { TokenContext } from '../utils/TokenContextProvider';

function LiveInfoLog({ message }) {
	let msgObj = message?.state?.live_info;

	// --- concatenate the artifact profiles into one sigle string --- 
	let targetMsg = (msgObj) ? Object.keys(msgObj).reduce((acc, curr) => acc + `${curr}:${msgObj[curr]}\n`, '') : '';

	let outputMsg = { 'live_info': targetMsg }

	return (
		<div className="card card-dark">
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "250px", display: "block", overflowY: "auto" }}>
				<div style={{ padding: "0 0 .8rem 0", display: "flex" }}>
				</div>
				<div className="" style={{ height: "80%" }}>
					<div style={{ padding: ".0rem .5rem .5rem .5rem" }}>Live Info</div>
					<PrintOutWindow outputs={outputMsg} category="live_info" />
				</div>
			</div>
		</div >
	)
}

export default LiveInfoLog;