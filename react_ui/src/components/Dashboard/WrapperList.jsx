import React, { useState, useContext } from 'react'
import { Modal, Button } from 'react-bootstrap';

import SearchBox from './SearchBox'
import WrapperItem from './WrapperItem';
import CreateWrapperModal from "./CreateWrapperModal";
import ImportWrapperModal from "../Editor/ImportWrapperModal";
import { TokenContext } from '../../utils/TokenContextProvider';
import { notificationMsg } from '../Notifications';
import {
	useGetWrapperCategories, useGetWrappers, useGetWrapperTypes,
	usePostCreateWrapper, useDeleteWrapper
} from '../../hooks/FetchHooks';


const WrapperList = ({ farOverlay }) => {
	const [show, setShow] = useState(false);
	const [keyword, setKeyword] = useState('');

	const handleClose = () => setShow(false);
	const handleShow = () => setShow(true);

	const createWrapperCb = async (wp) => {
		await dPostCreateWrapper.mutateAsync({ token: token, wrapper: { name: wp.wrapper_name, type: wp.wrapper_type, category: wp.category } });
	};

	const deleteWrapperCb = async (name) => {
		await dDeleteWrapper.mutateAsync({ token: token, wrapperName: name });
	};

	const searchWrapper = (key) => { setKeyword(key); };


	const { token } = useContext(TokenContext);

	// ====== With Swarm APIs ======
	// ------ GET ------
	const dWrappers = useGetWrappers(token);
	const dWrapperTypes = useGetWrapperTypes(token);
	const dWrapperCategories = useGetWrapperCategories(token);

	// ------ POST create wrapper ------
	const hCreateWrapper = {
		onSuccess: (data) => { notificationMsg('success', `The Wrapper is created Successfully`); },
		onError: (err) => {
			let errMsg = "";
			if (err.response.status === 422) {
				errMsg = err.response.data.detail[0].msg;
			} else {
				errMsg = err.response.data;
			}
			notificationMsg('error', errMsg || `Failed to create the wrapper!!`);
		}
	}
	const dPostCreateWrapper = usePostCreateWrapper(hCreateWrapper);

	// ------ DELETET wrapper ------
	const hDeleteWrapper = {
		onSuccess: (data) => { notificationMsg('success', `The Wrapper is deleted Successfully`); },
		onError: (err) => { notificationMsg('error', `Failed to delete the wrapper!!`); }
	}
	const dDeleteWrapper = useDeleteWrapper(hDeleteWrapper);


	return (
		<>
			<SearchBox item="wrapper" callbacks={{ search: searchWrapper }} />
			<div className="card-body" id="roles-deck" style={{ overflowY: 'scroll', height: '200px', fontSize: '100%' }}>
				{/* {(apiWrappers) ? apiWrappers */}
				{(dWrappers.data) ? dWrappers.data
					.filter(w => w?.wrapper_name.includes(keyword))
					.map((wrapper, index) => <WrapperItem key={index} content={wrapper} callbacks={{ show: handleShow, delete: deleteWrapperCb, farOverlay: farOverlay }} token={token} />)
					: null
				}
			</div >
			<div className="row" style={{ margin: '1.0rem' }}>
				<CreateWrapperModal style={{ float: 'left', width: '50%' }} item='wrapper' types={dWrapperTypes.data} categories={dWrapperCategories.data} callbacks={{ create: createWrapperCb }} />
				<div className="col-8"></div>
				<ImportWrapperModal style={{ float: 'right', width: '50%' }} token={token} />

			</div>

			<Modal show={show} onHide={handleClose} animation={false}>
				<Modal.Header>
					<Modal.Title>Wrapper Header</Modal.Title>
				</Modal.Header>
				<Modal.Body>Wrapper Content</Modal.Body>
				<Modal.Footer>
					<Button variant="secondary" onClick={handleClose}>
						Cancel
					</Button>
					<Button variant="primary" onClick={() => createWrapperCb()}>
						Save
					</Button>
				</Modal.Footer>
			</Modal>
		</>
	)
}

export default WrapperList;
