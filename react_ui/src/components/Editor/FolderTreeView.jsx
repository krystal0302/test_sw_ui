import 'react-folder-tree/dist/style.css';

import { useContext } from 'react'
import { useNavigate } from 'react-router-dom';
import FolderTree from 'react-folder-tree';
import { FaFile, FaTrash } from 'react-icons/fa';

import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetWrapperCode, useGetWrapperConfigParam, useGetWrapperData, useGetWrapperLiveInfo, useGetWrapperServices, useGerLowLevelSchema, useGetLowLevel } from '../../hooks/FetchHooks';


const FolderTreeView = ({ configData, location, cbs }) => {
	const wrapperName = location.state.wrapper_name;
	const navigate = useNavigate();

	const { token } = useContext(TokenContext);
	const dLiveInfo = useGetWrapperLiveInfo(token, wrapperName);
	const dConfigParam = useGetWrapperConfigParam(token, wrapperName);
	const dWrapperData = useGetWrapperData(token, wrapperName);
	const dServices = useGetWrapperServices(token, wrapperName);
	const dWrapperCode = useGetWrapperCode(token, wrapperName);
	const dLowLevelSchema = useGerLowLevelSchema(token);
	const dLowLevel = useGetLowLevel(token, wrapperName);


	const onClick = async ({ defaultOnClick, nodeData }) => {
		console.log(nodeData);

		if (nodeData.name === "Live info") {
			await dLiveInfo.refetch();
			navigate("/wrapper_sdk_editor/live_info", { state: { ...location.state, content: dLiveInfo.data } });
			return;
		}
		if (nodeData.name === "Configuration") {
			await dConfigParam.refetch();
			navigate("/wrapper_sdk_editor/config", { state: { ...location.state, content: dConfigParam.data } });
			return;
		}
		if (nodeData.name === "Wrapper data") {
			await dWrapperData.refetch();
			navigate("/wrapper_sdk_editor/data", { state: { ...location.state, content: dWrapperData.data } });
			return;
		}
		if (nodeData.name === "Picture") {
			navigate("/wrapper_sdk_editor/picture", location);
			return;
		}
		if (nodeData.category === "root") {
			console.log('service list root');
			console.log(defaultOnClick);
			defaultOnClick();
			return;
		}
		if (nodeData.category === "service") {
			await dServices.refetch();
			let targetParams = dServices.data.find(s => s.name === nodeData.name);
			navigate("/wrapper_sdk_editor/service", { state: { ...location.state, content: targetParams.parameter, serviceNode: nodeData.name } });
			defaultOnClick();
			return;
		}
		if (nodeData.category === "mapping") {
			await dLowLevelSchema.refetch();
			await dLowLevel.refetch();
			navigate("/wrapper_sdk_editor/mapping", { state: { ...location.state, schema: dLowLevelSchema.data, content: dLowLevel.data } });
			return;
		}
		if (nodeData.category === "code") {
			await dWrapperCode.refetch();
			navigate("/wrapper_sdk_editor/code", { state: { ...location.state, content: dWrapperCode.data } });
			return;
		}
	}

	const AddFolderIcon = (...args) => null;
	const AddFileIcon = ({ ...args }) => {
		console.log(args);
		return <FaFile onClick={() => {
			$("#new-service-modal").modal("show");
		}} />
	};
	const EditIcon = (...args) => null;
	const DeleteIcon = ({ ...args }) => {
		// --- root SHOULD NOT be removed ---
		if (args.nodeData._id === 0) {
			return null;
		}
		return <FaTrash onClick={async () => {
			await cbs.delete(wrapperName, args.nodeData.name);
		}} />
	}
	const OKIcon = (...args) => null;

	return (
		<FolderTree
			data={configData}
			showCheckbox={false}
			onNameClick={onClick}
			iconComponents={{
				AddFolderIcon,
				AddFileIcon,
				EditIcon,
				DeleteIcon,
				OKIcon
			}}
		/>
	);
}

export default FolderTreeView;