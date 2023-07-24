import React, { useState, useRef } from "react";
import { Modal, Button } from 'react-bootstrap';
import { notificationMsg } from '../Notifications';

export default function ImportWrapperModal({ token }) {
	const [show, setShow] = useState(false);
	const wrapperNameRef = useRef('');
	const pkgNameRef = useRef('');
	const [file, setFile] = useState(null);

	const handleClose = () => setShow(false);
	const handleShow = () => setShow(true);

	async function importWrapperPackage(_token, _wrapperName) {
		console.log(file);

		var formData = new FormData();
		formData.append('wrapper_pkg', file);

		let data = await fetch(`http://${window.location.hostname}:5000/v2/sdk/artifact-wrapper/package?wrapper_name=${_wrapperName}`, {
			method: "PUT",
			headers: {
				accept: "application/json",
				Authorization: `${_token.token_type} ${_token.access_token}`,
				// "Content-Type": "application/json"
			},
			body: formData,
		})
			.then((response) => { return response; });

		console.log(data);
		if (data.status === 200) {
			notificationMsg('success', 'The wrapper is imported Successfully');
		} else {
			notificationMsg('error', 'Failed to import the wrapper');
		}

		handleClose();
	}

	// --- api data ---
	return (
		<>
			<Button className="col-2" variant="primary" onClick={handleShow}>
				<i className="fas fa-file-import fa-1x" />
				<span> Import from file</span>
			</Button>

			<Modal show={show} onHide={handleClose} animation={false}>
				<Modal.Header>
					<Modal.Title>Import a wrapper</Modal.Title>
				</Modal.Header>
				<Modal.Body>
					<div className="row">
						<div className="col-6"> <span style={{ fontSize: '100%' }}>wrapper name: </span> </div>
						<div className="col-6">
							<input type="text" className="form-control" id="core-firmware-cur-ver" style={{ fontSize: '100%' }} ref={wrapperNameRef} />
						</div>
					</div>
					<div className="row" style={{ paddingTop: '1rem' }}>
						<input
							accept=".tar.gz"
							type="file"
							style={{ margin: '0 auto' }}
							ref={pkgNameRef}
							encType="multipart/form-data"
							onChange={e => { setFile(e.target.files[0]); }}
						/>
					</div>
				</Modal.Body>
				<Modal.Footer>
					<Button variant="secondary" onClick={handleClose}>
						Cancel
					</Button>
					<Button variant="primary" onClick={async () => { await importWrapperPackage(token, wrapperNameRef.current.value); }}>
						Upload
					</Button>
				</Modal.Footer>
			</Modal>
		</>
	);
}