import React, { useEffect } from 'react'
import Select from 'react-select';

function MappingTagParamRow(props) {
	const uuid = props.param.id;
	const schema = props.schema;
	const rwVal = props.param.readwrite;

	const rwOptions = Object.keys(schema)?.map(s => { return { value: s, label: s }; });
	const dfRw = (rwVal) ? { value: rwVal, label: rwVal } : rwOptions[0];

	const rwSchema = (schema?.[dfRw.value].length > 0) ? schema[dfRw.value][0] : {};
	let tagSchema = { ...rwSchema };
	['description', 'map_to'].forEach(e => delete tagSchema[e]);

	useEffect(() => {
		Object.entries(tagSchema).map(([key, value]) => {
			if (typeof value !== 'object') return;
			const paramVal = props.param[key];
			const options = rwSchema[key]?.map(opt => { return opt });
			const dfOption = (paramVal) ? paramVal : options[0];
			props.cbs.selectChange(uuid, { [key]: dfOption });
		})
	}, [rwVal]);

	const selectStyles = {
		option: (provided, state) => ({
			...provided,
			color: state.isSelected ? '#ffffff' : '#000000',
		}),
	};

	const renderTagCols = (key, col) => {
		switch (typeof col) {
			case "string":
				if (col.startsWith('string')) {
					return <input type="text"
						id={`${uuid}_${key}`}
						className="form-control"
						defaultValue={props.param[key]} />
				} else if (col.startsWith('integer')) {
					// console.log(col);
					const regex = /(?<=range: )(.*?)(?=\))/i;
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
						defaultValue={props.param[key]}
						min={minVal}
						max={maxVal}
						step="1"
						pattern="[0-9]*"
						onChange={(evt) => handleRange(evt, minVal, maxVal)}
						onKeyDown={(evt) => (evt.key === 'e' || evt.key === '.') && evt.preventDefault()} />
				}
				return null;
			case "object":
				const paramVal = props.param[key];
				const options = rwSchema[key]?.map(dt => { return { value: dt, label: dt }; });
				const dfOption = (paramVal) ? { value: paramVal, label: paramVal } : options[0];
				return <Select
					inputId={`${uuid}_${key}_input`}
					defaultValue={dfOption}
					options={options}
					styles={selectStyles}
					onChange={(choice) => {
						props.cbs.selectChange(uuid, { [key]: choice.value })
					}}
				/>;
			default:
				return null;
		}
	};

	return (
		<tr>
			<td className="col-1"></td>
			{Object.entries(tagSchema).map(([key, value]) => {
				return (
					<td className="col-3">
						{renderTagCols(key, value)}
					</td>
				);
			})}
			<td className="col-2">
				<Select
					inputId={`${uuid}_rw_input`}
					defaultValue={dfRw}
					options={rwOptions}
					styles={selectStyles}
					onChange={(choice) => {
						// --- re-render mapping table row ---
						props.cbs.rwChange(uuid, choice.value);
					}}
				/>
			</td>
		</tr>
	)
}

export default MappingTagParamRow