import React, { useState, useEffect } from 'react'
import Select from 'react-select';

function MappingParamRow(props) {
	const uuid = props.param.id;
	const rowPerm = props.rowPerm;
	const schema = props.schema;
	const changeRowId = Object.keys(rowPerm)[0];
	const changeRowPerm = (Object.values(rowPerm)[0]) ? Object.values(rowPerm)[0] : props.param.readwrite;
	let mappingSchema = (schema[changeRowPerm].length > 0) ? schema[changeRowPerm][0] : {};
	mappingSchema = mappingSchema?.map_to;
	const mappingColKeys = Object.keys(mappingSchema).filter(key => key !== 'server');

	const [options, setOptions] = useState({});
	const [selectedOptions, setSelectedOptions] = useState({});

	useEffect(() => {
		if (changeRowId !== 'all' && uuid !== changeRowId) { return; }
		let newOptions = {};
		let newDefaultOption = {};
		mappingColKeys.forEach((key) => {
			const obj = mappingSchema[key];
			if (typeof obj !== 'object') return;
			const opts = obj.map(op => { return { value: op, label: op }; });
			const mapToVal = props.param.map_to[key];
			const mapToOpt = (obj.indexOf(mapToVal) !== -1) ? { value: mapToVal, label: mapToVal } : opts[0];
			newOptions[key] = opts;
			newDefaultOption[key] = mapToOpt;
		});
		setOptions(newOptions);
		setSelectedOptions(newDefaultOption);
	}, [rowPerm]);

	useEffect(() => {
		const isEmpty = Object.keys(selectedOptions).length === 0;
		if (isEmpty) return;
		props.cbs.selectChange(uuid, selectedOptions);
	}, [selectedOptions]);

	const handleSelectChange = (value, key) => {
		setSelectedOptions({
			[key]: value
		});
	};

	const selectStyles = {
		option: (provided, state) => ({
			...provided,
			color: state.isSelected ? '#ffffff' : '#000000',
		}),
	};

	const renderMappingCols = (key, col) => {
		switch (typeof col) {
			case "string":
				if (col.startsWith('string')) {
					return <input type="text"
						id={`${uuid}_${key}`}
						className="form-control"
						defaultValue={props.param.map_to[key]} />
				} else if (col.startsWith('integer')) {
					// console.log(col);
					const regex = /(?<=range: )(.*?)(?=\|)/i;
					const range = col.match(regex) ? col.match(regex)[0] : "";
					const minVal = Number(range.split('-')[0]);
					const maxVal = Number(range.split('-')[1]);

					function handleRange(e, min, max) {
						if (e.target.value === '') return;
						const inputNum = Number(e.target.value);
						if (inputNum < min) {
							e.target.value = min;
						} else if (inputNum > max) {
							e.target.value = max;
						}
					}

					return <input type="number"
						id={`${uuid}_${key}`}
						className="form-control"
						defaultValue={props.param.map_to[key]}
						min={minVal}
						max={maxVal}
						step="1"
						pattern="[0-9]*"
						onChange={(evt) => handleRange(evt, minVal, maxVal)}
						onKeyDown={(evt) => (evt.key === 'e' || evt.key === '.') && evt.preventDefault()} />
				}
				return null;
			case "object":
				return <Select
					inputId={`${uuid}_${key}_input`}
					options={(options[key]) ? options[key] : null}
					value={(selectedOptions[key]) ? selectedOptions[key] : null}
					styles={selectStyles}
					onChange={(value) => handleSelectChange(value, key)} />
			default:
				return null;
		}
	};

	return (
		<tr>
			{(mappingColKeys) ? mappingColKeys
				.map((key) => (
					<td style={{ verticalAlign: "middle" }}>
						{renderMappingCols(key, mappingSchema[key])}
					</td>
				))
				: null
			}
			<td className="col-1">
				<button
					type="button"
					className="btn btn-tool"
					style={{ paddingTop: '15px' }}
					onClick={() => {
						props.cbs.delete(uuid);
					}}>
					<i className="fas fa-trash fa-xl"></i>
				</button>
			</td>
		</tr>
	)
}

export default MappingParamRow