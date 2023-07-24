import React, { useState, useRef, useContext } from 'react';
import { useQuery } from 'react-query';
import { Button } from "react-bootstrap";
import PrintOutWindow from './PrintOutWindow';
import { fetchGetArtifactLogs } from '../api/FetchApi';
import { TokenContext } from '../utils/TokenContextProvider';
import { useGetTargetLogsPackage } from '../hooks/FetchHooks';

function LogCollapsibleCard2({ targetId }) {
	const logCategories = [
		{
			value: "wrapper",
			text: "wrapper"
		},
		{
			value: "low_level",
			text: "low level"
		},
		{
			value: "sys_manager",
			text: "system"
		},
		{
			value: "comms(dds)",
			text: "comms"
		},
	];
	const logCatRef = useRef(logCategories[0].value);

	const [printPause, setPrintPause] = useState(false);
	const [refetchPeriod, setRefetchPeriod] = useState(800);

	const { token } = useContext(TokenContext);

	const { data: dataLogs } = useQuery(
		['printout-log', token, targetId],
		fetchGetArtifactLogs,
		{
			refetchInterval: refetchPeriod,
			enabled: !!token,
			onSuccess: (data) => {
				// console.log(data);
			},
			onError: (err) => {
				// console.log(err)
			}
		}
	);

	// ------ artifact log package export ------
	// artifact id: targetId 
	const dWrapperLogPackage = useGetTargetLogsPackage(token, targetId, { enabled: true })


	return (
		<div className="card card-dark">
			<div className="card-header border-transparent ui-sortable-handle" style={{ cursor: "move" }}>
				<h2 className="card-title">
					<i className="fas fa-list mr-1"></i>
					<span style={{ fontFamily: "Arial", fontSize: "100%" }}>Log</span>
				</h2>
				<div className="card-tools">
					<button type="button" className="btn btn-tool" data-card-widget="collapse">
						<i className="fas fa-minus"></i>
					</button>
				</div>
			</div>
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "580px", display: "block", overflowY: "auto" }}>
				<div style={{ padding: "0 0 .8rem 0", display: "flex" }}>
					<div className="col-5"></div>
					<div className="col-7 col align-self-end" style={{ display: "flex" }}>
						<Button
							style={{ margin: "0 0.5rem" }}
							className="col align-self-end"
							variant="secondary"
							onClick={() => {
								setRefetchPeriod((printPause) ? 800 : Infinity);
								setPrintPause(!printPause);
							}}
						>
							{(printPause ? 'Resume' : 'Pause')}
						</Button>
						<Button
							style={{ margin: "0 0.5rem" }}
							className="col align-self-end"
							variant="secondary"
							onClick={async () => {
								// console.log('export artifact log');
								// --- create a download link ---
								const url = await window.URL.createObjectURL(new Blob([dWrapperLogPackage.data]));
								const link = document.createElement('a');
								link.href = url;
								link.download = 'artifact-log-' + targetId + '.tar.gz';
								document.body.appendChild(link);
								link.click();
								link.remove();                   // removecomponent after download
								window.URL.revokeObjectURL(url); // release blob object
							}}>Export</Button>
					</div>
				</div>
				<div className="" style={{ height: "40%" }}>
					{/* <PrintOutWindow outputs={printOut} /> */}
					<div style={{ padding: ".0rem .5rem .5rem .5rem" }}>Wrapper Log</div>
					<PrintOutWindow outputs={dataLogs} category="wrapper" />
					<div style={{ padding: "10px" }}>Low Level Log</div>
					<PrintOutWindow outputs={dataLogs} category="low_level" />
				</div>
				{/* <div className="" style={{height: "40%"}}>
					<div style={{padding: "10px"}}>Low Level Log</div>
					<PrintOutWindow outputs={dataLogs} category={logCatRef.current.value} />
				</div> */}

			</div>
			<div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}></div>
		</div >

	)
}

export default LogCollapsibleCard2;