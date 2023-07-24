import React, { useState, useEffect, useContext } from 'react'
import { nanoid } from 'nanoid';
import { Button } from "react-bootstrap";

import { usePutWrapperConfigParam } from '../../hooks/FetchHooks';

import ConfigParamRow from './ConfigParamRow';
import { TokenContext } from '../../utils/TokenContextProvider';
import { notificationMsg } from '../Notifications';

function ConfigParamPane(props) {
	const [configParams, setConfigParams] = useState(props.param.map(p => ({ ...p, id: nanoid(5) })));
	const [pageTitle, setPageTitle] = useState(null);

	useEffect(() => {
		function route() {
			if (props.title === pageTitle) return () => { };

			setPageTitle(props.title);
			setConfigParams(props.param.map(p => ({ ...p, id: nanoid(5) })));
		}

		route();
	});

	const createParam = () => {
		setConfigParams([...configParams, { id: nanoid(5), name: '', value: '' }]);
	};

	const deleteParam = (id) => {
		const updatedParams = configParams.filter(cp => cp.id !== id);
		setConfigParams(updatedParams);
	};

	const { token } = useContext(TokenContext);

	// ------ PUT save config param ------
	const handleSaveConfigParam = {
		onSuccess: (data) => {
			setConfigParams(configParams);
			notificationMsg('success', `Config Params are set Successfully`);
		}
	}
	const dPutSaveConfigParam = usePutWrapperConfigParam(handleSaveConfigParam);

	return (
		<div className="card card-dark">
			<div className="row" style={{ textAlign: 'center', fontSize: '20px', paddingTop: '1rem' }}>
				<div className="col-md-6 offset-md-3">
					<span>{props.title}</span>
				</div>
				<Button style={{margin:'0 1rem'}} onClick={() => { createParam(); }}>Add</Button>
				<Button style={{margin:'0 1rem'}}
					onClick={
						async () => {
							let results = configParams.map(cp => {
								const paramName = document.getElementById(`${cp.id}_name`).value;
								const paramValue = document.getElementById(`${cp.id}_value`).value;
								return { id: cp.id, name: paramName, value: paramValue }
							});

							// --- validation ---
							let emptyValidate = results.filter(r => r.name === '');
							console.log(emptyValidate.length);
							if (emptyValidate.length) {
								notificationMsg('warn', 'Some param name not be filled!');
								return;
							}

							await dPutSaveConfigParam.mutateAsync({
								uri: props.uri,
								token: token,
								wrapperName: props.location.state.wrapper_name,
								params: results
							});
						}}
				>Save</Button>
				<div className="col-md-1">
					<i className="fa fa-info-circle" title=""></i>
				</div>
			</div>
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "45vh", display: "block", overflowY: "auto" }}>
				<div className="table-responsive" style={{ display: "block" }}>
					<table className="table table-hover">
						<thead>
							<tr>
								<th className="col-5" style={{ textalign: "left" }} data-field="name">
									<div className="th-inner " style={{ fontsize: "100%" }}>Key</div>
								</th>
								<th className="col-5" style={{}} data-field="status">
									<div className="th-inner sortable both" style={{ fontSize: "100%" }}>Default value</div>
								</th>
								<th className="col-2" style={{}} data-field="icon">
									<div className="th-inner " style={{ fontSize: "100%" }} />
								</th>
							</tr>
						</thead>
						<tbody>
							{(configParams) ? configParams
								.map((param) => <ConfigParamRow
									key={param.id}
									param={{ id: param.id, key: param.name, value: param.value }}
									cbs={{ delete: deleteParam }} />)
								: null
							}
						</tbody>
					</table>
				</div>
				{/* <Button onClick={() => { createParam(); }}>Add</Button> */}
			</div>
			{/* <div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}> */}
				{/* <Button
					onClick={
						async () => {
							let results = configParams.map(cp => {
								const paramName = document.getElementById(`${cp.id}_name`).value;
								const paramValue = document.getElementById(`${cp.id}_value`).value;
								return { id: cp.id, name: paramName, value: paramValue }
							});

							// --- validation ---
							let emptyValidate = results.filter(r => r.name === '');
							console.log(emptyValidate.length);
							if (emptyValidate.length) {
								notificationMsg('warn', 'Some param name not be filled!');
								return;
							}

							await dPutSaveConfigParam.mutateAsync({
								uri: props.uri,
								token: token,
								wrapperName: props.location.state.wrapper_name,
								params: results
							});
						}}
				>Save</Button> */}
			{/* </div> */}
		</div >
	);
}

export default ConfigParamPane