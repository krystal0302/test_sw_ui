import React from 'react'

function ConfigParamRow({ param, cbs }) {
	return (
		<tr>
			<td className="col-5">
				<input
					id={`${param.id}_name`}
					type="text"
					className="form-control col-md-4 offset-md-4"
					defaultValue={param.key}
				/>
			</td>
			<td className="col-5">
				<input
					id={`${param.id}_value`}
					type="text"
					className="form-control col-md-4 offset-md-4"
					defaultValue={param.value}
				/>
			</td>
			<td className="col-md-2">
				<button
					type="button"
					className="btn btn-tool"
					style={{ paddingTop: '15px' }}
					onClick={() => { console.log(`deleting param ${param.id}`); cbs.delete(param.id) }}>
					<i className="fas fa-trash fa-xl"></i>
				</button>
			</td>
		</tr>
	)
}

export default ConfigParamRow