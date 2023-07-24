import { SelectAllTwoTone } from "@mui/icons-material";
import React, { useState, useRef } from "react";
import { useEffect } from "react";
import { Modal, Button } from 'react-bootstrap';
import Select from 'react-select';

export default function CreateWrapperModal({ item, callbacks, types, categories }) {
	// console.log(types)
	// console.log(categories)

	const [show, setShow] = useState(false);
	const [typeChoice, setTypeChoice] = useState("");
	const [selectedOption, setSelectedOption] = useState(null);
	const [IsCustomeType, setIsCustomeType] = useState(false);
	const [enableSelect, setEnableSelect] = useState(true);

	const nameRef = useRef('');
	const newTypeRef = useRef('');

	const handleClose = () => setShow(false);
	const handleShow = () => setShow(true);

	// --- API data transformation ---
	const typeOptions = types?.map(t => { return { value: t.type, label: t.type }; });
	const catOptions = categories?.map(c => { return { value: c, label: c }; })

	// --- [init] gear to the related category ---
	let defaultCat = types?.find(t => t.type === types[0].type);
	defaultCat = defaultCat?.category;

	const createWrapper = () => {
		// --- wrapper type ---
		const defaultType = (types.length) ? types[0].type : '';
		const selType = (typeChoice) ? typeChoice.value : defaultType;
		const wrapperType = (IsCustomeType) ? newTypeRef.current.value : selType;

		// --- wrapper category ---
		const wrapperCategory = selectedOption ? selectedOption.value : defaultCat;

		let profile = {
			wrapper_name: nameRef.current.value,
			wrapper_type: wrapperType,
			category: wrapperCategory,
		};

		callbacks.create(profile);

		// --- reset states ---
		setTypeChoice(defaultType)
		setSelectedOption({value: defaultCat, label: defaultCat});
		setIsCustomeType(false);
		setEnableSelect(true);

		handleClose();
	}

	return (
		<>
			<Button className="col-2" variant="primary" onClick={handleShow}>
				<i className="fas fa-plus fa-1x" />
				<span> Create new Wrapper</span>
			</Button>

			<Modal show={show} onHide={handleClose} animation={false}>
				<Modal.Header>
					<Modal.Title>Create New {item}</Modal.Title>
				</Modal.Header>
				<Modal.Body>
					<div className="row">
						<div className="col-6"> <span style={{ fontSize: '100%' }}>wrapper name: </span> </div>
						<div className="col-6">
							<input type="text" className="form-control" style={{ fontSize: '100%' }} ref={nameRef} />
						</div>
					</div>
					<div className="row">
						<div className="col-6"> <span style={{ fontSize: '100%' }}>wrapper type: </span> </div>
						<div className="col-6">
							<Select
								defaultValue={typeOptions ? typeOptions[0] : {}}
								options={typeOptions}
								onChange={(choice) => {
									setIsCustomeType((choice.value === 'user-define'));
									setEnableSelect(!(choice.value === 'user-define'));
									setTypeChoice(choice)
									console.log(choice)

									// --- gear to the related category ---
									let cat = types.find(t => t.type === choice.value);
									cat = cat?.category;
									// console.log(cat)
									setSelectedOption((cat) ? { value: cat, label: cat } : {});
								}}
							/>
						</div>
					</div>
					{(IsCustomeType) ? (<div className="row" style={{ display: IsCustomeType }}>
						<div className="col-6"> <span style={{ fontSize: '100%' }}>New type name: </span> </div>
						<div className="col-6">
							<input type="text" className="form-control" style={{ fontSize: '100%' }} ref={newTypeRef} />
						</div>
					</div>) : null}
					<div className="row">
						<div className="col-6"> <span style={{ fontSize: '100%' }}>wrapper cat.: </span> </div>
						<div className="col-6">
							<Select
								isDisabled={enableSelect}
								value={selectedOption ? selectedOption : { value: defaultCat, label: defaultCat }}
								options={catOptions}
								onChange={(choice) => {
									setSelectedOption(choice);
								}}
							/>
						</div>
					</div>
				</Modal.Body>
				<Modal.Footer>
					<Button variant="secondary" onClick={handleClose}>
						Cancel
					</Button>
					<Button variant="primary" onClick={() => createWrapper()}>
						Save
					</Button>
				</Modal.Footer>
			</Modal>
		</>
	);
}