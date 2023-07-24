import React, { useState, useEffect, useContext } from 'react'
import { nanoid } from 'nanoid';
import { Button } from "react-bootstrap";

import { usePutWrapperServiceParam } from '../../hooks/FetchHooks';

import ServiceParamRow from './ServiceParamRow';
import { TokenContext } from '../../utils/TokenContextProvider';
import { notificationMsg } from '../Notifications';

function ServiceParamPane(props) {
	const [serviceParams, setServiceParams] = useState(props.param.map(p => ({ ...p, id: nanoid(5) })));
	const [pageTitle, setPageTitle] = useState('');

	useEffect(() => {
		function route() {
			if (props.title === pageTitle) return () => { };

			setPageTitle(props.title);
			setServiceParams(props.param.map(p => ({ ...p, id: nanoid(5) })));
		}

		route();
	});

	const createParam = () => {
		setServiceParams([...serviceParams, { id: nanoid(5), param_name: '', data_type: 'int', data_range: '[1:100]', default: '50' }]);
	};

	const deleteParam = (id) => {
		const updatedParams = serviceParams.filter(sp => sp.id !== id);
		setServiceParams(updatedParams);
	};

	const { token } = useContext(TokenContext);

	const dPutSaveServiceParam = usePutWrapperServiceParam();

	const paramCols = [
		{ name: "Parameter name", classTag: "col-2" },
		{ name: "Data type", classTag: "col-2" },
		{ name: "Input range", classTag: "col-3" },
		{ name: "Default value", classTag: "col-2" },
		{ name: "", classTag: "col-1" },
	];
	return (
		<div className="card card-dark">
			<div className="row" style={{ textAlign: 'center', fontSize: '20px', paddingTop: '1rem' }}>
				<div className="col-md-6 offset-md-3">
					<span>{props.title}</span>
				</div>
				<Button style={{ margin: '0 1rem' }} onClick={() => { createParam(); }}>Add</Button>
				<Button style={{ margin: '0 1rem' }}
					onClick={
						async () => {
							let results = serviceParams.map(cp => {
								const paramName = document.getElementById(`${cp.id}_name`).value;
								const dataType = document.getElementById(`${cp.id}_type`).value;
								let dataRange = '';
								if (dataType !== 'bool') {
									const minVal = document.getElementById(`${cp.id}_min`).value;
									const maxVal = document.getElementById(`${cp.id}_max`).value;
									dataRange = `[${minVal}:${maxVal}]`;
								}
								const defaultValue = document.getElementById(`${cp.id}_default`).value;
								return { param_name: paramName, data_type: dataType, data_range: dataRange, default: defaultValue }
							});
							// console.log(results);

							// --- validation ---
							let emptyValidate = results.filter(r => r.param_name === '');
							console.log(emptyValidate.length);
							if (emptyValidate.length) {
								notificationMsg('warn', 'Some param name not be filled!');
								return;
							}

							await dPutSaveServiceParam.mutateAsync({
								token: token,
								wrapperName: props.location.state.wrapper_name,
								serviceName: props.title,
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
					<table id="flows-table" className="table table-hover">
						<thead>
							<tr>
								{(paramCols) ? paramCols
									.map((col) => (
										<th className={col.classTag}>
											<div className="th-inner" style={{ fontsize: "100%" }}>{col.name}</div>
										</th>
									)) : null
								}
							</tr>
						</thead>
						<tbody>
							{(serviceParams) ? serviceParams
								.map((service) =>
									<ServiceParamRow
										key={service.id}
										param={service}
										cbs={{ delete: deleteParam }} />)
								: null
							}
						</tbody>
					</table>
				</div>
				{/* <Button onClick={() => { createParam(); }}>Add</Button> */}
			</div>
			{/* <div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}>
				<Button
					onClick={
						async () => {
							let results = serviceParams.map(cp => {
								const paramName = document.getElementById(`${cp.id}_name`).value;
								const dataType = document.getElementById(`${cp.id}_type`).value;
								const minVal = document.getElementById(`${cp.id}_min`).value;;
								const maxVal = document.getElementById(`${cp.id}_max`).value;;
								const dataRange = `[${minVal}:${maxVal}]`;
								const defaultValue = document.getElementById(`${cp.id}_default`).value;
								return { param_name: paramName, data_type: dataType, data_range: dataRange, default: defaultValue }
							});
							// console.log(results);

							// --- validation ---
							let emptyValidate = results.filter(r => r.param_name === '');
							console.log(emptyValidate.length);
							if (emptyValidate.length) {
								notificationMsg('warn', 'Some param name not be filled!');
								return;
							}


							await dPutSaveServiceParam.mutateAsync({
								token: token,
								wrapperName: props.location.state.wrapper_name,
								serviceName: props.title,
								params: results
							});
						}}
				>Save</Button>
			</div> */}
		</div >
	);
}

export default ServiceParamPane