import React, { useState, useContext } from 'react'
import { TokenContext } from '../utils/TokenContextProvider';
import {
	useGetWrapperServices, usePostRunArtifactService, usePutResumeArtifactService,
	useDeleteCancelArtifactService
} from '../hooks/FetchHooks';

import { Button } from "react-bootstrap";
import { FaServer } from 'react-icons/fa';
import { propTypes } from 'react-bootstrap/esm/Image';
import { notificationMsg } from '../components/Notifications';

import { Backdrop } from '@mui/material';
import CircularProgress from '@mui/material/CircularProgress';


const ParamRow = (props) => {
	console.log(props)
	return (
		<div className="row col-12" style={{ padding: "0.5rem 0" }}>
			<div className="col-4">
				<label style={{ fontSize: "100%" }}>{props.name} : </label>
			</div>
			<div className="col-3" />
			<div className="col-5">
				<input
					id={`${props.name}`}
					type="text"
					className={`form-control ${props.serviceName}`}
					placeholder="param. 1"
					style={{ fontSize: "100%" }}
					defaultValue={props.defaultValue}
					onChange={() => {
						console.log('current value')
					}}
				/>
			</div>
		</div>
	);
};

const ServiceCollapsibleBar = (props) => {
	console.log(props)
	// console.log(props.status.service[props.name]?.response)
	let serviceStatus = (props.status.service) ? props.status.service[props.name]?.response.status : "";
	// console.log(serviceStatus)
	let serviceMsg = (props.status.service) ? props.status.service[props.name]?.response.response_msg : "";
	// console.log(serviceMsg)
	const [open, setOpen] = useState(false);

	const { token } = useContext(TokenContext);

	// ====== With Swarm APIs ======
	const hRunArtifactService = {
		onSuccess: (data) => { notificationMsg('success', `The Wrapper is launched Successfully`); },
		onError: (err) => { notificationMsg('error', `Failed to launch the wrapper!!`); }
	}
	const dPostRunArtifactService = usePostRunArtifactService(hRunArtifactService);

	const hResumeArtifactService = {
		onSuccess: (data) => { notificationMsg('success', `The Wrapper is resumed Successfully`); },
		onError: (err) => { notificationMsg('error', `Failed to resume the wrapper!!`); }
	}
	const dPutResumeArtifactService = usePutResumeArtifactService(hResumeArtifactService);

	const hCancelArtifactService = {
		onSuccess: (data) => { notificationMsg('success', `The Wrapper is deleted Successfully`); },
		onError: (err) => { notificationMsg('error', `Failed to delete the wrapper!!`); }
	}
	const dDeleteCancelArtifactService = useDeleteCancelArtifactService(hCancelArtifactService);


	async function handleRunService(paramsData) {
		let artifactId = props.status.id;
		let serviceName = props.name;
		await dPostRunArtifactService.mutateAsync({ token: token, artifactId: artifactId, serviceName: serviceName, params: paramsData })
	}

	async function handlePauseResumeService(paramsData, type = 'pause') {
		let artifactId = props.status.id;
		let serviceName = props.name;
		await dPutResumeArtifactService.mutateAsync({ token: token, artifactId: artifactId, serviceName: serviceName, actionType: type })
	}

	async function handleCancelService() {
		let serviceName = props.name || '';
		let artifactId = props.status.id;
		console.log(`CANCEL: artifact id: ${artifactId}, service: ${serviceName}`);
		await dDeleteCancelArtifactService.mutateAsync({ token: token, artifactId: artifactId, serviceName: serviceName })
	}

	const states = {
		'idle': (
			<>
				<Button
					type="button"
					className="btn btn-tool"
					variant="primary"
					style={{ margin: "0 .5rem" }}
					onClick={async () => {
						setOpen(!open);
						// --- fetch latest param values ---
						let paramsData = {};
						let params = document.getElementsByClassName(`${props.name}`);
						for (let p of params) {
							// let q = document.querySelector(`#${p.id}`);
							paramsData[p.id] = p.value;
						}
						// --- post the result through api ---
						await handleRunService(paramsData);
						setOpen(false);
					}}
				>
					<span>Run</span>
				</Button>
			</>
		),
		'running': (
			<>
				<Button
					type="button"
					className="btn btn-tool"
					variant="primary"
					style={{ margin: "0 .5rem" }}
					onClick={async () => {
						setOpen(!open);
						// --- fetch latest param values ---
						let paramsData = {};
						let params = document.getElementsByClassName(`${props.name}`);
						for (let p of params) {
							let q = document.querySelector(`#${p.id}`);
							paramsData[p.id] = q.value;
						}

						// --- post the result through api ---
						await handlePauseResumeService(paramsData, 'pause');
						setOpen(false);
					}}
				>
					<span>Pause</span>
				</Button>
				<Button
					type="button"
					className="btn btn-tool"
					variant="primary"
					style={{ margin: "0 .5rem" }}
					onClick={async () => { await handleCancelService() }}
				>
					<span>Cancel</span>
				</Button>
			</>
		),
		'paused': (
			<>
				<Button
					type="button"
					className="btn btn-tool"
					variant="primary"
					style={{ margin: "0 .5rem" }}
					onClick={async () => {
						setOpen(!open);
						// --- fetch latest param values ---
						let paramsData = {};
						let params = document.getElementsByClassName(`${props.name}`);
						for (let p of params) {
							let q = document.querySelector(`#${p.id}`);
							paramsData[p.id] = q.value;
						}

						// --- post the result through api ---
						await handlePauseResumeService(paramsData, 'resume');
						setOpen(false);
					}}
				>
					<span>Resume</span>
				</Button>
				<Button
					type="button"
					className="btn btn-tool"
					variant="primary"
					style={{ margin: "0 .5rem" }}
					onClick={async () => { await handleCancelService() }}
				>
					<span>Cancel</span>
				</Button>
			</>
		),
	}

	return (
		<div className="card card-dark collapsed-card">
			<div className="card-header border-transparent ui-sortable-handle" style={{ cursor: "move" }}>
				<h2 className="card-title">
					<FaServer size={14} />
					<span style={{ fontFamily: "Arial", fontSize: "100%", paddingLeft: ".5rem" }}>{props.name}</span>
					<span style={{ fontFamily: "Arial", fontSize: "100%", paddingLeft: ".5rem" }}>{serviceStatus}</span>
				</h2>
				<div className="card-tools">
					{states[serviceStatus]}
					<button type="button" className="btn btn-tool" data-card-widget="collapse">
						<i className="fas fa-plus"></i>
					</button>
				</div>
			</div>
			<div className="card-header border-transparent ui-sortable-handle" style={{ cursor: "move" }}>
				<span style={{ padding: '0 1rem' }}>msg: </span>
				<span>{serviceMsg}</span>
			</div>
			<div className="card-body" style={{ fontSize: "16px", height: "150px", overflowY: "auto" }}>
				{
					(props.params) ?
						props.params.map(
							(param, index) =>
								<ParamRow
									key={index}
									name={param.param_name}
									defaultValue={param.default}
									serviceName={props.name}
								/>) :
						null
				}
			</div>
			<Backdrop
				sx={{ color: '#fff', zIndex: (theme) => theme.zIndex.drawer + 1 }}
				open={open}
			>
				<CircularProgress color="inherit" />
			</Backdrop>
		</div>
	);
};

const ServiceCardFrame = ({ children }) => {
	return (
		<div className="card card-dark">
			<div style={{ textAlign: 'center', fontSize: '20px', padding: '1rem 0 0.5rem 0' }}>Service in current wrapper</div>
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "34vh", overflowY: "auto" }}>
				{children}
			</div>
		</div >
	);
};

function WrapperServicesBox({ location, serviceStatus = {}, services }) {
	const wrapperName = location.state.wrapper_name;
	// console.log(serviceStatus)
	// console.log(services)

	const { token } = useContext(TokenContext);
	// const dServicesInWrapper = useGetWrapperServices(token, wrapperName);
	// console.log(dServicesInWrapper.data)

	return (
		<ServiceCardFrame>
			{/* {(dServicesInWrapper?.data) ? dServicesInWrapper?.data
				.map((service, index) => <ServiceCollapsibleBar key={service.name} name={service.name} params={service.parameter} status={serviceStatus} />)
				: null
			} */}
			{(services) ? services
				.map((service, index) => <ServiceCollapsibleBar key={service.name} name={service.name} params={service.parameter} status={serviceStatus} />)
				: null
			}
		</ServiceCardFrame>
	)
}

export default WrapperServicesBox