import React, { useState, useRef, useContext } from 'react';
import { Button } from "react-bootstrap";
import PrintOutWindow from '../PrintOutWindow';
import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetTargetLogs } from '../../hooks/FetchHooks';

function LogCollapsibleCard({ targetId }) {
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

	const outputMsg = (logCatRef.current?.value === "system") ? ([
		{
			status: 1,
			message: "Hello Debugging log Terminal!"
		},
		{
			status: 1,
			message: "Here is some debugging messages,... i am so verbose, so it need to wrap up all the messages in a row",
		},
		{
			status: 2,
			message: "[Success] the service works like a charm!",
		},
		{
			status: 3,
			message: "[Warn] some code snippet may cause potential issues...",
		},
		{
			status: 4,
			message: "[Error] some damages happened!!!",
		}
	]) :
		([
			{
				status: 1,
				message: "Hello System log Terminal!"
			},
			{
				status: 1,
				message: "Here is some debugging messages,... i am so verbose, so it need to wrap up all the messages in a row",
			},
			{
				status: 2,
				message: "[Success] the service works like a charm!",
			},
			{
				status: 3,
				message: "[Warn] some code snippet may cause potential issues...",
			},
			{
				status: 4,
				message: "[Error] some damages happened!!!",
			}
		]);

	const [printOut, setPrintOut] = useState(outputMsg);
	// console.log(printOut)

	const { token } = useContext(TokenContext);

	const dTargetLogs = useGetTargetLogs(token, targetId);

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
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "280px", display: "block", overflowY: "auto" }}>
				<div style={{ padding: "0 0 .8rem 0", display: "flex" }}>
					<div className="col-4 col align-self-start">
						<select
							className="form-control"
							style={{ fontSize: "100%" }}
							ref={logCatRef}
						>
							{(logCategories) ? logCategories
								.map((opt) => <option key={opt.value} value={opt.value}>{opt.text}</option>)
								: null
							}
						</select>
					</div>
					<div className="col-6"></div>
					<div className="col-2 col align-self-end">
						<Button className="col align-self-end" variant="secondary" onClick={() => { setPrintOut([]) }}>Clear</Button>
					</div>
				</div>
				<div className="" style={{ height: "80%" }}>
					{/* <PrintOutWindow outputs={printOut} /> */}
					{/* <PrintOutWindow outputs={dataLogs} category={logCatRef.current.value} /> */}
					<PrintOutWindow outputs={dTargetLogs.data} category={logCatRef.current.value} />
				</div>

			</div>
			<div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}></div>
		</div >

	)
}

export default LogCollapsibleCard;