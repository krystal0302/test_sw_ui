import React from "react";
import { Modal, Button } from 'react-bootstrap';

export default function FlowTriggerModal(props) {

	const handleClose = () => props.callbacks.closeModal();

	const triggerFlow = () => {
		console.log('flow trigger confirmed!');
		handleClose();
	}

	return (
		<>
			<Modal show={props.show} onHide={handleClose} animation={false}>
				<Modal.Body>
					<div className="row">
						<div className="col-12"> Are you sure to trigger {props.flowName}?</div>
					</div>
				</Modal.Body>
				<Modal.Footer>
					<Button variant="secondary" onClick={handleClose}>
						Cancel
					</Button>
					<Button variant="primary" onClick={() => triggerFlow()}>
						Confirm
					</Button>
				</Modal.Footer>
			</Modal>
		</>
	);
}