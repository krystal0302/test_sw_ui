import 'react-reflex/styles.css'

import React, { useState, useEffect, useContext, useRef } from 'react'
import { nanoid } from 'nanoid';
import { Button } from "react-bootstrap";
import { useQuery, useMutation } from 'react-query';
import { useNavigate, useLocation, useParams } from 'react-router-dom';
import { ReflexContainer, ReflexSplitter, ReflexElement } from 'react-reflex'

import FolderTreeView from '../components/Editor/FolderTreeView';
import ConfigParamPane from '../components/Editor/ConfigParamPane';
import ImageUploadPane from '../components/Editor/ImageUploadPane';
import MappingParamPane from '../components/Editor/MappingParamPane';
import ServiceParamPane from '../components/Editor/ServiceParamPane';
import CodeEditorPane from '../components/Editor/CodeEditorPane';

import WrapperServicesBox from '../components/WrapperServicesBox';
// import LogCollapsibleCard from "../components/Editor/LogCollapsibleCard";

import LoadingOverlay from 'react-loading-overlay';
import { notificationMsg } from '../components/Notifications';
import LastBuildInfoLog from '../components/Editor/LastBuildInfoLog';
import BuildMessageLog from '../components/Editor/BuildMessageLog';

import {
	useDeleteWrapperService, useGetWrapperBuild, useGetWrapperTreeview,
	usePutCreateWrapperService, usePutWrapperBuild
} from '../hooks/FetchHooks';

import { TokenContext } from '../utils/TokenContextProvider';


const SdkEditor = (props) => {
	useEffect(() => {
		window.onbeforeunload = function () {
			console.log('ready to reload the page')
			return true;
		};

		return () => {
			console.log('reload the page now')
			window.onbeforeunload = null;
		};
	}, []);

	const navigate = useNavigate();
	const location = useLocation();

	const [isActive, setActive] = useState(false);
	let wrapperName = location.state?.wrapper_name || location.state?.id;
	let wrapperId = location.state?.wrapper_id;

	const { token } = useContext(TokenContext);

	function adaptor(data) {
		// --- data adaption ---
		let adaptee = data.find(d => d.name === "Wrapper configuration");
		adaptee.children = adaptee.children.map(c => { return { ...c, category: "config" } })

		adaptee = data.find(d => d.name === "Protocol mapping table");
		adaptee.children = adaptee.children.map(c => { return { ...c, category: "mapping" } })

		adaptee = data.find(d => d.name === "Service list");
		adaptee['category'] = "root";
		adaptee.children = adaptee.children.map(c => { return { ...c, category: "service" } })

		adaptee = data.find(d => d.name === "Operation logic");
		adaptee.children = adaptee.children.map(c => { return { ...c, category: "code" } })

		return data;
	}

	const dTreeView = useGetWrapperTreeview(token, wrapperName, { select: adaptor })

	// ------ API Queries ------
	const dCreateService = usePutCreateWrapperService();
	const dDeleteService = useDeleteWrapperService();

	const hPutWrapperBuild = {
		onSuccess: (data) => {
			notificationMsg('success', `The Wrapper is built Successfully`);
			setActive(false);
		},
		onError: (err) => {
			notificationMsg('error', `Failed to build the wrapper!!`);
			setActive(false);
		},
	};
	const dPutWrapperBuild = usePutWrapperBuild(hPutWrapperBuild);
	// const dWrapperBuildMsg = useGetWrapperBuild(token, wrapperName, { refetchInterval: 1000 });
	const dWrapperBuildMsg = useGetWrapperBuild(token, wrapperName); // only need to fetch once after build
	// console.log(dWrapperBuildMsg.data)

	const handleCreateService = async (wrapperName, serviceName) => {
		await dCreateService.mutateAsync({ token: token, wrapperName: wrapperName, serviceName: serviceName });
		dTreeView.refetch(token, wrapperName);
	}

	const handleDeleteService = async (wrapperName, serviceName) => {
		await dDeleteService.mutateAsync({ token: token, wrapperName: wrapperName, serviceName: serviceName });
		dTreeView.refetch(token, wrapperName);
	}

	const serviceNameRef = useRef(null);

	const { pane } = useParams()

	const componentSwitch = {
		'live_info': <ConfigParamPane title="Live info" uri="live-info" param={location.state.content} location={location} />,
		'config': < ConfigParamPane title="Configuration" uri="config" param={location.state.content} location={location} />,
		'data': <ConfigParamPane title="Wrapper data" uri="data" param={location.state.content} location={location} />,
		'picture': <ImageUploadPane title="Picture" location={location} />,
		'mapping': <MappingParamPane title="Protocol Mapping Table" schema={location.state.schema} param={location.state.content} location={location} />,
		'service': <ServiceParamPane title={location.state.serviceNode} param={location.state.content} location={location} />,
		'code': <CodeEditorPane param={location.state.content} location={location} />,
	}

	return (
		<>
			<div className="modal fade" id="new-service-modal" tabIndex="-1" role="dialog" aria-hidden="true">
				<div className="modal-dialog">
					<div className="modal-content">
						<div className="modal-header">
							<h4 className="modal-title">Create a New Service</h4>
							<button type="button" className="close" data-dismiss="modal"><span aria-hidden="true">&times;</span><span
								className="sr-only">Close</span></button>
						</div>
						<div className="modal-body">
							<input type="text" className="form-control role-input" ref={serviceNameRef} />
						</div>
						<div className="modal-footer">
							<button type="button" className="btn btn-default" data-dismiss="modal" >Cancel</button>
							<button type="button" className="btn btn-primary" data-dismiss="modal"
								onClick={async () => {
									let serviceName = serviceNameRef.current.value;
									await handleCreateService(wrapperName, serviceName);
								}}>Create</button>
						</div>
					</div>
				</div>
			</div>

			<LoadingOverlay
				active={isActive}
				spinner
				text='Building the wrapper...'
			>
				{/* EXTEND ver. */}
				{/* <div className="content-wrapper farobot-dark-mode" style={{ minHeight: "calc(100vh - 5.5rem)" }}> */}
				{/* COLLAPSE ver. */}
				<div className="farobot-dark-mode" style={{ minHeight: "calc(100vh - 5.5rem)" }}>
					<div className="content" style={{ height: "95.4vh" }}>

						<ReflexContainer orientation="horizontal">
							<ReflexElement className="header" minSize={50} maxSize={50}>
								<div className="pane-content">
									<div style={{ width: "100%", height: "5%", textAlign: "center", fontSize: "20px", verticalAlign: "middle", padding: "10px" }}>
										<Button variant="secondary" style={{ float: "left" }} onClick={() => navigate("/wrapper.html")}>Back</Button>
										<span>Wrapper - {location.state.wrapper_name}</span>
									</div>
								</div>
							</ReflexElement>
							<ReflexSplitter />
							<ReflexElement>
								<ReflexContainer orientation="vertical">
									<ReflexElement className="left-pane" minSize={200} maxSize={200}>
										{/* {(dataTreeView ? dataTreeView.map((dtv, index) => */}
										{(dTreeView.data ? dTreeView.data.map((dtv, index) =>
											<FolderTreeView
												key={index}
												configData={dtv}
												location={location}
												cbs={{ create: handleCreateService, delete: handleDeleteService }} />
										) : null
										)}
									</ReflexElement>
									<ReflexSplitter />
									<ReflexElement className="right-pane">
										<div className="pane-content" style={{ textAlign: "center", padding: "10px" }}>
											{componentSwitch[pane] || null}
										</div>
									</ReflexElement>
								</ReflexContainer>
							</ReflexElement>
							<ReflexSplitter />
							{/* <ReflexElement className="left-pane" minSize={250} maxSize={250}> */}
							<ReflexElement className="left-pane">
								<div className="pane-content">
									<div style={{ verticalAlign: "center", textAlign: "center", display: "flex" }}>
										<div className="col-2">
											<Button
												variant="secondary"
												style={{ height: "30%", width: "60%", margin: "40% 25%", float: "center", fontSize: "50px" }}
												onClick={async () => {
													setActive(true);
													// setTimeout(() => { setActive(false); }, 1000)
													await dPutWrapperBuild.mutateAsync({ token: token, wrapperName: wrapperName });
													// setActive(false);
													// --- refetch the build result message after build ---
													await dWrapperBuildMsg.refetch(token, wrapperName);
												}}>Build</Button>
										</div>
										<div className="col-5" style={{ padding: "10px" }}>
											{/* <WrapperServicesBox location={location} /> */}
											<BuildMessageLog location={location} dbgMsg={dWrapperBuildMsg?.data || ""} />
										</div>
										<div className="col-5" style={{ padding: "10px" }}>
											{/* <LogCollapsibleCard targetId={location.state.wrapper_id} /> */}
											<LastBuildInfoLog location={location} buildMsg={dWrapperBuildMsg?.data || ""} />
										</div>
									</div>
								</div>
							</ReflexElement>
						</ReflexContainer>
					</div>
				</div>
			</LoadingOverlay>
		</>
	)
}


export default SdkEditor;