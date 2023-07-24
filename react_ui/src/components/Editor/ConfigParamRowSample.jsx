// import React from 'react'
import propTypes from "prop-types";

function ConfigParamRow() {
	return (
		// <div>ConfigParamRow</div>
		<tr>
			<td className="col-5">
				<input
					// id={`${param.id}_name`}
					type="text"
					className="form-control col-md-4 offset-md-4"
				// defaultValue={param.key}
				/>
			</td>
			<td className="col-5">
				<input
					// id={`${param.id}_value`}
					type="text"
					className="form-control col-md-4 offset-md-4"
				// defaultValue={param.value}
				/>
			</td>
			<td className="col-md-2">
				<button
					type="button"
					className="btn btn-tool"
					style={{ paddingTop: '15px' }}
				// onClick={() => { console.log(`deleting param ${param.id}`); cbs.delete(param.id) }}
				>
					<i className="fas fa-trash fa-xl"></i>
				</button>
			</td>
		</tr>
	)
}

ConfigParamRow.propTypes = {
	// label: PropTypes.string,
	// backgroundColor: PropTypes.string,
	// size: PropTypes.oneOf(["sm", "md", "lg"]),
	// color: PropTypes.oneOf(["blue", "white", "black"]),
	// handleClick: PropTypes.func,
};

export default ConfigParamRow