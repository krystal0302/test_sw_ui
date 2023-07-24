import React, { useState } from 'react'

function ServiceParamRow({ param, cbs }) {
	// console.log(param?.data_type)
	const [nowSelect, setNowSelect] = useState(param.data_type);

	const renderInputRange = () => {
		switch (nowSelect) {
			case "int":
			case "double":
			case "string":
				const [min, max] = param.data_range.replace(/[\[\]]/g, '').split(':');
				return (
					<>
						<input id={`${param.id}_min`} className="col-4 form-control" type="text" defaultValue={min} />
						<i className="col-1 fa fa-info-circle" title={`Min${nowSelect === 'string' ? '_char' : ''}`} />
						<label className="col-1">~</label>
						<input id={`${param.id}_max`} className="col-4 form-control" type="text" defaultValue={max} />
						<i className="col-1 fa fa-info-circle" title={`Max${nowSelect === 'string' ? '_char' : ''}`} />
					</>
				);
			default:
				return null;
		}
	};

	const renderDefaultValue = () => {
		switch (nowSelect) {
			case "int":
			case "double":
			case "string":
				return <input type="text" id={`${param.id}_default`} className="form-control" defaultValue={param.default} />;
			case "bool":
				return (
					<>
						<select
							id={`${param.id}_default`}
							className="form-control"
							defaultValue={param.default}
						>
							<option value={"true"}>true</option>
							<option value={"false"}>false</option>
						</select>
					</>
				);
			default:
				return null;
		}
	};

	return (
		<tr>
			<td className="col-2">
				<input type="text" id={`${param.id}_name`} className="form-control" defaultValue={param.param_name} />
			</td>
			<td className="col-2">
				<select
					id={`${param.id}_type`}
					className="form-control"
					// value={param.data_type}
					value={nowSelect}
					defaultValue={param.data_type}
					onChange={(e) => {
						// console.log(e.target.value)
						setNowSelect(e.target.value);
					}}
				>
					<option value={"int"}>int</option>
					<option value={"double"}>double</option>
					<option value={"bool"}>bool</option>
					<option value={"string"}>string</option>
				</select>
			</td>
			<td className="col-3">
				<div className="row justify-content-center align-items-center">
					{renderInputRange()}
				</div>
			</td>
			<td className="col-2">
				{renderDefaultValue()}
			</td>
			<td className="col-1">
				<button
					type="button"
					className="btn btn-tool"
					style={{ paddingTop: '15px' }}
					onClick={() => { cbs.delete(param.id) }} >
					<i className="fas fa-trash fa-xl"></i>
				</button>
			</td>
		</tr>
	)
}

export default ServiceParamRow