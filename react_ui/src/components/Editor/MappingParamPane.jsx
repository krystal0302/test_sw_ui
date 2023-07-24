import React, { useState, useContext, useEffect } from 'react'
import { Button } from "react-bootstrap";
import Select from 'react-select';
import { nanoid } from 'nanoid';

import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetLowLevel, usePutLowLevel, usePostLowLevel } from '../../hooks/FetchHooks';
import { notificationMsg } from '../Notifications';

import MappingTagParamRow from './MappingTagParamRow';
import MappingParamRow from './MappingParamRow';
import CommunicationSettingModal from './CommunicationSettingModal';

function MappingParamPane(props) {
	// --- API data ---
	const wrapperName = props.location.state.wrapper_name;
	const lowLevelSchema = props.schema;
	const lowLevels = props.param?.low_level;
	const enabledProtocol = (props.param?.enabled_low_level.length > 0) ? props.param.enabled_low_level[0] : null;
	const protocolOptions = lowLevelSchema.support?.map(prot => { return { value: prot, label: prot, isDisabled: prot === enabledProtocol }; });
	const { token } = useContext(TokenContext);
	const hPutLowLevel = {
		onSuccess: (data) => { notificationMsg('success', `Low level params are set successfully`); },
		onError: (err) => { notificationMsg('error', `Failed to set low level params!!`); }
	};
	const hPostLowLevel = {
		onSuccess: (data) => { notificationMsg('success', `Low level is created successfully`); }
	};
	const dLowLevel = useGetLowLevel(token, wrapperName);
	const dPutSaveLowLevel = usePutLowLevel(hPutLowLevel);
	const dPostCreateLowLevel = usePostLowLevel(hPostLowLevel);

	const [rowPerm, setRowPerm] = useState(null);
	const [params, setParams] = useState([]);
	const [paramSchema, setParamSchema] = useState(null);
	const [tagHeaderCols, setTagHeaderCols] = useState([]);
	const [mappingHeaderCols, setMappingHeaderCols] = useState([]);
	const [setting, setSettings] = useState(null);
	const [settingSchema, setSettingSchema] = useState({});
	const [selectedProtocol, setSelectedProtocol] = useState(enabledProtocol);
	const [protocols, setProtocols] = useState(protocolOptions);

	useEffect(() => {
		if (selectedProtocol === null) return;

		setRowPerm({ 'all': '' });

		const lowLevel = lowLevelSchema.low_level?.find(ll => ll.protocol.type === selectedProtocol);
		const tags = lowLevel?.tags;
		setParamSchema(tags);
		const setting = lowLevel?.protocol?.setting;
		let serverSetting = (setting?.servers) ? setting.servers[0] : {};
		delete serverSetting['server_name'];
		setSettingSchema(serverSetting);

		// --- set tag table columns data ---
		const readSchema = (tags?.read.length > 0) ? tags.read[0] : {};
		let tagCols = { ...readSchema };
		['description', 'map_to'].forEach(e => delete tagCols[e]);
		tagCols = Object.keys(tagCols).map(name => {
			return { name: name, headerType: "text", class: "col-3" }
		});
		const addCol = { name: "", headerType: "button", class: "col-1" };
		const rwCol = { name: "readwrite", headerType: "text", class: "col-2" };
		tagCols = [addCol, ...tagCols, rwCol];
		setTagHeaderCols(tagCols);

		// --- set mapping table columns data ---
		const mapToSchema = readSchema?.map_to;
		let mappingCols = Object.keys(mapToSchema).filter(key => key !== 'server').map(name => {
			return { name: name, class: "" }
		});
		mappingCols = [...mappingCols, { name: "", class: "col-1" }];
		setMappingHeaderCols(mappingCols);

		// --- set low level tags param data ---
		const data = lowLevels?.find(ll => ll.protocol.type === selectedProtocol);
		let readArr = (data?.tags?.read) ? data.tags.read : [];
		readArr = readArr.map(o => {
			return { id: nanoid(5), readwrite: 'read', ...o }
		});
		let writeArr = (data?.tags?.write) ? data.tags.write : [];
		writeArr = writeArr.map(o => {
			return { id: nanoid(5), readwrite: 'write', ...o }
		});
		setParams([...readArr, ...writeArr]);

		// --- set modal data ---
		let settingData = data?.protocol?.setting;
		settingData = (settingData?.servers) ? settingData.servers[0] : {};
		setSettings(settingData);
	}, [selectedProtocol]);

	// --- button events ---
	const createMappingParam = () => {
		const newId = nanoid(5);
		setRowPerm({ [newId]: 'read' });

		const readSchema = (paramSchema?.read.length > 0) ? paramSchema.read[0] : {};
		const mapToSchema = readSchema?.map_to;

		let newSchema = { ...readSchema };
		Object.keys(newSchema).forEach(key => {
			newSchema[key] = "";
		});
		let mappingSchema = { ...mapToSchema };
		Object.keys(mappingSchema).forEach(key => {
			mappingSchema[key] = "";
		});
		newSchema.map_to = mappingSchema;
		setParams([...params, { id: newId, readwrite: 'read', ...newSchema }]);
	};

	const deleteMappingParam = (id) => {
		const updatedParams = params.filter(p => p.id !== id);
		setParams(updatedParams);
	};

	async function handelCreateLowLevelOnChanged(protocolName) {
		try {
			await dPostCreateLowLevel.mutateAsync({
				token: token,
				wrapperName: wrapperName,
				protocolName: protocolName
			});
		} catch (error) {
			console.log(error);
		}
	}

	const handleSaveLowLevelOnClicked = async () => {
		let isDupTagName = false;
		let isEmptyTagName = false;
		let tagNames = [];
		let readArr = [];
		let writeArr = [];
		params.map(p => {
			// --- tag columns ---
			Object.keys(p).forEach(key => {
				const tagCols = tagHeaderCols.map(col => col.name);
				if (!tagCols.includes(key)) return;
				const tagEl = document.getElementById(`${p.id}_${key}`);
				const tagType = (tagEl) ? tagEl.type : 'select';
				if (tagType === 'number') {
					p[key] = Number(tagEl.value);
				} else if (tagType === 'text') {
					p[key] = tagEl.value;
				}
			});

			// --- mapping columns ---
			Object.keys(p.map_to).forEach(key => {
				const el = document.getElementById(`${p.id}_${key}`);
				const type = (el) ? el.type : 'select';
				let mapVal = "";
				if (type === 'select') {
					if (key === 'server') {
						mapVal = (p.map_to[key]) ? p.map_to[key] : setting.server_name;
					} else {
						mapVal = p.map_to[key];
					}
				} else if (type === 'number') {
					mapVal = Number(el.value);
				} else {
					mapVal = el.value;
				}
				p.map_to[key] = mapVal;
			});

			if (p.tag_name.trim() == "") {
				isEmptyTagName = true;
			}
			if (tagNames.includes(p.tag_name)) {
				isDupTagName = true;
			}
			tagNames.push(p.tag_name);

			if (p.readwrite === 'read') {
				readArr.push(p);
			} else {
				writeArr.push(p);
			}
		});

		// --- validation ---
		if (isEmptyTagName) {
			alert('TAG NAME should NOT be empty!');
			return;
		}
		if (isDupTagName) {
			alert('DUPLICATED TAG NAME!');
			return;
		}

		const tags = { "read": readArr, "write": writeArr };
		const result = {
			"tags": tags
		}

		// console.log(params);
		// console.log(JSON.stringify(result));

		await dPutSaveLowLevel.mutateAsync({
			token: token,
			wrapperName: wrapperName,
			protocolName: selectedProtocol,
			params: result
		});

		await dLowLevel.refetch();
	};

	// --- select events ---
	const changeRwOption = (id, option) => {
		setRowPerm({ [id]: option });
		let selParam = params.find(p => p.id === id);
		selParam.readwrite = option;
	};

	const changeTagOption = (id, obj) => {
		let selParam = params.find(p => p.id === id);
		Object.keys(obj).forEach(key => {
			selParam[key] = obj[key];
		});
	};

	const changeMapToOptions = (id, obj) => {
		let selParam = params.find(p => p.id === id);
		Object.keys(obj).forEach(key => {
			selParam.map_to[key] = obj[key].value;
		});
	};

	const selectStyles = {
		option: (provided, state) => ({
			...provided,
			color: state.isSelected ? '#ffffff' : '#000000',
		}),
	};

	const renderTagHeaderCols = (col) => {
		switch (col.headerType) {
			case "text":
				return <div style={{ fontsize: "100%" }}>{col.name}</div>
			case "button":
				return <div><Button onClick={() => { createMappingParam(); }}><i className="fas fa-plus fa-1x" /></Button></div>
			default:
				return null;
		}
	};

	return (
		<div className="card card-dark">
			<div className="row" style={{ textAlign: 'center', paddingTop: '1rem' }}>
				<div className="col-md-6 offset-md-3" style={{ fontSize: "20px" }}>
					<span>{props.title}</span>
				</div>
				<Select
					placeholder={'Protocol selection'}
					options={protocols}
					defaultValue={selectedProtocol ? { value: selectedProtocol, label: selectedProtocol } : null}
					styles={selectStyles}
					onChange={async (choice) => {
						setRowPerm({ 'all': '' });
						const selProtocol = choice.value;
						protocolOptions.forEach((option) => {
							if (option.value === selProtocol) {
								option.isDisabled = true;
							} else {
								option.isDisabled = false;
							}
						});
						setSelectedProtocol(selProtocol);
						setProtocols(protocolOptions);

						// --- create protocol low level ---
						await handelCreateLowLevelOnChanged(selProtocol);
					}}
				/>
			</div>
			<div className="card-body" style={{ fontSize: "16px", height: "60vh", display: "block", overflowY: "auto" }}>
				<div className="row">
					<div className="col-5">
						<div className="col-md-6 offset-md-3" style={{ fontSize: "20px", padding: "1rem 0" }}>
							<span style={{ fontWeight: 'bold' }}>Tag</span>
						</div>
						{/* <div className="table-responsive" style={{ display: "block" }}> */}
						<table className="table table-hover">
							<thead>
								<tr>
									{(tagHeaderCols) ? tagHeaderCols
										.map((col) => (
											<th className={col.class} style={{ textAlign: "center", verticalAlign: "middle", height: 55 }}>
												{renderTagHeaderCols(col)}
											</th>
										)) : null
									}
								</tr>
							</thead>
							<tbody>
								{(params.length > 0) ? params
									.map((param) =>
										<MappingTagParamRow
											key={param.id}
											param={param}
											schema={paramSchema}
											cbs={{
												selectChange: changeTagOption,
												rwChange: changeRwOption
											}}
										/>
									) : null
								}
							</tbody>
						</table>
						{/* </div> */}
					</div>
					<div className="col-7">
						<div className="row align-items-center">
							<div className="col-md-6 offset-md-3" style={{ fontSize: "20px", padding: "1rem 0" }}>
								<span style={{ fontWeight: 'bold' }}>Address mapping table</span>
							</div>
							<div className="col-md-3" style={{ textAlign: "right" }}>
								<CommunicationSettingModal
									param={setting}
									schema={settingSchema}
									wrapper={wrapperName}
									protocol={selectedProtocol}
									cbs={{
										updateServerSetting: (obj) => {
											setSettings(obj);
										}
									}}
								/>
							</div>
						</div>
						<table className="table table-hover">
							<thead>
								<tr>
									{(mappingHeaderCols) ? mappingHeaderCols
										.map((col) => (
											<th className={col.class} style={{ textAlign: "center", verticalAlign: "middle", height: 55 }}>
												{col.name}
											</th>
										)) : null
									}
								</tr>
							</thead>
							<tbody>
								{(params.length > 0) ? params
									.map((param) =>
										<MappingParamRow
											key={param.id}
											rowPerm={rowPerm}
											param={param}
											schema={paramSchema}
											cbs={{
												selectChange: changeMapToOptions,
												delete: deleteMappingParam
											}}
										/>
									) : null
								}
							</tbody>
						</table>
					</div>
				</div>
			</div>
			<div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}>
				<Button disabled={selectedProtocol === null || (setting !== null && Object.keys(setting).length === 0)} onClick={async () => { await handleSaveLowLevelOnClicked(); }}>Save</Button>
			</div>
		</div >
	);
}

export default MappingParamPane