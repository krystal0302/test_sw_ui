import React, { useState, useContext } from "react";
import { Modal, Button } from 'react-bootstrap';

import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetLowLevel, usePutLowLevel } from '../../hooks/FetchHooks';
import { notificationMsg } from '../Notifications';
import { nanoid } from "nanoid";

export default function CommunicationSettingModal(props) {
	const param = props.param;
	const schema = props.schema;
	const settingColKeys = Object.keys(schema);

	const [show, setShow] = useState(false);
	const handleClose = () => setShow(false);
	const handleShow = () => setShow(true);

	// --- API data ---
	const { token } = useContext(TokenContext);
	const hPutLowLevel = {
		onSuccess: (data) => {
			props.cbs.updateServerSetting(serverObj);
			handleClose();
			notificationMsg('success', `Low level settings are set successfully`);
		},
		onError: (err) => {
			notificationMsg('error', `Failed to set low level settings!!`);
		}
	};
	const dLowLevel = useGetLowLevel(token, props.wrapper);
	const dPutSaveLowLevel = usePutLowLevel(hPutLowLevel);

	let serverObj = {};
	const saveSetting = async () => {
		if (settingColKeys.length === 0) { return; }
		settingColKeys.map(key => {
			const el = document.getElementById(key);
			const type = (el) ? el.type : 'select';
			let settingVal = "";
			if (type === 'number') {
				settingVal = Number(el.value);
			} else {
				// --- text ---
				settingVal = el.value;
				// --- bool ---
				if (settingVal === 'true' || settingVal === 'false') {
					settingVal = (settingVal === 'true');
				}
			}
			serverObj = { ...serverObj, [key]: settingVal };
		});
		const dfServerName = (Object.keys(param).length > 0) ? param.server_name : 'server_' + nanoid(5);
		serverObj.server_name = dfServerName;

		const result = {
			"protocol": {
				"type": props.protocol,
				"setting": {
					"servers": [serverObj]
				}
			}
		};

		// console.log(JSON.stringify(result));

		await dPutSaveLowLevel.mutateAsync({
			token: token,
			wrapperName: props.wrapper,
			protocolName: props.protocol,
			params: result
		});

		await dLowLevel.refetch();
	}

	const renderSettingsCol = (key, col) => {
		switch (typeof col) {
			case "string":
				if (col.startsWith('string')) {
					return <input type="text"
						id={key}
						className="form-control"
						defaultValue={(param) ? param[key] : null} />
				} else if (col.startsWith('integer')) {
					return <input type="number"
						id={key}
						className="form-control"
						defaultValue={(param) ? param[key] : null}
						min="0"
						step="1"
						pattern="[0-9]*"
						onKeyDown={(evt) => (evt.key === 'e' || evt.key === '.') && evt.preventDefault()} />
				} else if (col.startsWith('float')) {
					return <input type="number"
						id={key}
						className="form-control"
						defaultValue={(param) ? param[key] : null}
						min="0.0"
						step="0.1"
						pattern="[0-9]*"
						onKeyDown={(evt) => evt.key === 'e' && evt.preventDefault()} />
				} else if (col.startsWith('bool')) {
					return <select
						id={key}
						className="form-control"
						defaultValue={param[key]} >
						<option value={"true"}>Enable</option>
						<option value={"false"}>Disable</option>
					</select>
				}
				return null;
			case "object":
				// TODO: for select case
				return null;
			default:
				return null;
		}
	};

	return (
		<>
			<Button className="align-middle" variant="secondary" disabled={settingColKeys.length === 0} onClick={handleShow}>
				<i className="nav-icon fas fa-cog" />
			</Button>

			<Modal show={show} onHide={handleClose} animation={false}>
				<Modal.Header>
					<Modal.Title>Communication setting</Modal.Title>
				</Modal.Header>
				<Modal.Body>
					{(settingColKeys.length > 0) ?
						settingColKeys.map((key) => (
							<div className="row">
								<div className="col-5"> <span style={{ fontSize: '100%' }}>{key}</span> </div>
								<div className="col-7">{renderSettingsCol(key, schema[key])}</div>
							</div>
						)) : null}
				</Modal.Body>
				<Modal.Footer>
					<Button variant="secondary" onClick={handleClose}>
						Cancel
					</Button>
					<Button variant="primary" onClick={() => saveSetting()}>
						Save
					</Button>
				</Modal.Footer>
			</Modal>
		</>
	);
}