import 'react-reflex/styles.css'

import React, { useState, useEffect, useRef, useContext, useMemo } from 'react'
import { Button } from "react-bootstrap";
import { useNavigate, useLocation } from 'react-router-dom';
import { ReflexContainer, ReflexSplitter, ReflexElement } from 'react-reflex'

import WrapperServicesBox from '../components/WrapperServicesBox';
import LiveInfoLog from '../components/LiveInfoLog';
// import LogCollapsibleCard from "../components/LogCollapsibleCard";
import LogCollapsibleCard2 from "../components/LogCollapsibleCard2";
import { TokenContextProvider } from '../utils/TokenContextProvider';
import { FaInfoCircle, FaCog, FaIcons } from 'react-icons/fa';
import {
	useGetArtifactStatus, useGetArtifactProperty, useGetArtifacts,
	usePostArtifactConfigs, usePutArtifactConfigs
} from '../hooks/FetchHooks';
import { TokenContext } from '../utils/TokenContextProvider';

import {
	Drawer, Skeleton, Backdrop, CircularProgress,
	ListItem, ListItemIcon, ListItemText, TextField
} from '@mui/material';

import SettingsOutlinedIcon from '@mui/icons-material/SettingsOutlined';
import { AccessAlarm, ThreeDRotation, Settings } from '@mui/icons-material';

import { notificationMsg } from '../components/Notifications';


function genTooltip(msgObj) {
	const targetMsg = (msgObj) ? Object.keys(msgObj).reduce((acc, curr) => acc + `${curr}:${msgObj[curr]}\n`, '') : '';
	return targetMsg;
}

function SdkTesting() {
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

	const [open, setOpen] = useState(false);
	const [sidebarOpen, setSidebarOpen] = useState(false);

	const navigate = useNavigate();
	const location = useLocation();
	const { token } = useContext(TokenContext);
	const artifactId = location.state?.artifact_id;
	const dArtifactStatus = useGetArtifactStatus(token, artifactId, { refetchInterval: 1000 });
	const dArtifacts = useGetArtifacts(token, { refetchInterval: 1000 });
	const targetArtifact = dArtifacts?.data?.artifacts?.find(a => a.artifact_id == location.state.artifact_id);
	console.log(targetArtifact);
	const dArtifactProperty = useGetArtifactProperty(token, artifactId);
	const artfifactProperty = (dArtifactProperty.data) ? dArtifactProperty.data[0] : {};
	const target = (dArtifactProperty.data) ? dArtifactProperty?.data[0].description : {};
	let tooltip = genTooltip(target);
	tooltip = `Wrapper data: \n` + tooltip;
	// console.log(tooltip);
	// let thumbnail = (dWrapper.data) ? `data:image/png;base64,${dWrapper?.data[0].thumbnail}` : null;
	let thumbnail = (dArtifactProperty.data) ? `data:image/png;base64,${dArtifactProperty?.data[0].thumbnail}` : null;
	// console.log(thumbnail);

	const artifactServices = (dArtifactProperty.data) ? dArtifactProperty.data[0].service : [];
	console.log(artifactServices)

	const [artifactConf, setArtifactConf] = useState(null);
	let refs = artifactConf?.map(() => React.createRef());

	// ------ POST artifact configuration ------
	const hPostArtifactConfig = {
		onSuccess: (data) => {
			const conf = JSON.parse(data?.artifact_settings[0].artifact_conf.replace(/'/g, '"'));
			console.log(conf)

			let result = []
			// --- data conversion ---
			for (const [k, v] of Object.entries(conf)) {
				result.push({ name: k, value: v })
			}
			// console.log(result);

			// --- update the state ---
			setArtifactConf(result);
		},
	}
	const dPostArtifactConfig = usePostArtifactConfigs(hPostArtifactConfig);

	const hSaveArtifactConfig = {
		onSuccess: () => { notificationMsg('success', "Artifact Config is saved successfully!!!") },
		onError: () => { notificationMsg('error', "Failed to  save Artifact Config!!!") }
	};
	const dPutArtifactConfig = usePutArtifactConfigs(hSaveArtifactConfig);

	const getArtifactParamList = () => (
		<>
			<div style={{ width: 450 }} >
				{artifactConf?.map((item, index) => (
					<ListItem button key={index}>
						<ListItemIcon><SettingsOutlinedIcon style={{ color: 'white' }} /></ListItemIcon>
						<ListItemText style={{ color: 'white' }} primary={item.name} />
						{/* <TextField ref={refs[index]} variant="outlined" defaultValue={item.value} /> */}
						{/* <TextField
							// ref={refs[index]}
							inputRef={(ref) => (refs[index] = ref)}
							variant="outlined"
							defaultValue={item.value}
						/> */}
						<input id={item.name} ref={refs[index]} variant="outlined" defaultValue={item.value} />
					</ListItem>
				))}
			</div>
			<Button
				style={{ textAlign: 'center', margin: '1rem 8rem' }}
				onClick={async () => {
					console.log(refs.length)
					setSidebarOpen(!sidebarOpen);
					let artifactConfig = {};
					for (const ref of refs) {
						// console.log(ref.current.value)
						// console.log(ref.current.id)
						artifactConfig[ref.current.id] = ref.current.value;
					}
					artifactConfig = JSON.stringify(artifactConfig).replace(/"/g, "'");
					console.log(artifactConfig)
					await dPutArtifactConfig.mutateAsync({ token: token, artifactId: artifactId, artifactConfig: artifactConfig })
					setSidebarOpen(false);
				}}>
				Save
			</Button>
			<Backdrop
				sx={{ color: '#fff', zIndex: (theme) => theme.zIndex.drawer + 1 }}
				open={sidebarOpen}
			>
				<CircularProgress color="inherit" />
			</Backdrop>
		</>
	);

	return (
		<>
			{/* EXTEND ver. */}
			{/* <div className="content-wrapper farobot-dark-mode" style={{ minHeight: "calc(100vh - 5.5rem)" }}></div> */}
			{/* COLLAPSE ver. */}
			<div className="farobot-dark-mode" style={{ minHeight: "calc(100vh - 5.5rem)" }}>
				<div className="content" style={{ height: "95.4vh" }}>

					<TokenContextProvider>
						<Drawer
							open={open}
							anchor={"right"}
							onClose={() => setOpen(false)}
							PaperProps={{
								sx: {
									backgroundColor: '#373e44',
									height: '100%'
								}
							}}
						>
							{
								dPostArtifactConfig.isLoading ?
									(<p style={{ width: 450 }}><div style={{ fontSize: '18px', color: 'white', textAlign: 'center' }}>Loading data...</div></p>) :
									getArtifactParamList()
							}
						</Drawer>
						<ReflexContainer orientation="horizontal">
							<ReflexElement className="header" minSize={50} maxSize={50}>
								<div className="pane-content">
									<div style={{ width: "100%", height: "5%", textAlign: "center", fontSize: "20px", verticalAlign: "middle", padding: "10px" }}>
										<Button variant="secondary" style={{ float: "left" }} onClick={() => navigate("/wrapper.html")}>Close</Button>
										<span>Testing</span>
									</div>
								</div>
							</ReflexElement>
							<ReflexSplitter />
							<ReflexElement className="left-pane" minSize={60} maxSize={60}>
								<div className="pane-content">
									<div className="row" style={{ width: "100%", height: "5%", textAlign: "center", fontSize: "20px" }}>
										<div className="col-2" style={{ height: "50px", paddingTop: "5px" }} >
											{(thumbnail) ? (
												<img style={{ height: "100%" }} src={thumbnail} alt="Card image cap" />
											) : (
												<Skeleton
													sx={{ bgcolor: 'grey.600' }}
													variant="rounded"
													// style={{ height: "100%", width:"50%" }} />
													style={{ height: "100%", aspectRatio: "1/1", margin: '0 auto' }} />
											)}
										</div>
										<div className="col-3" style={{ color: "gray", paddingTop: "18px" }}>Artifact name: <span style={{ color: "white" }}>{targetArtifact?.artifact_name}</span></div>
										<div className="col-3" style={{ color: "gray", paddingTop: "18px" }}>Artifact ID: <span style={{ color: "white" }}>{targetArtifact?.artifact_id}</span></div>
										<div className="col-4" style={{ color: "gray", paddingTop: "18px" }}>Artifact State:
											<span style={{ color: "white", paddingRight: "1rem" }}>{dArtifactStatus?.data?.state?.state}</span>
											<FaCog onClick={async () => {
												setOpen(true)
												await dPostArtifactConfig.mutateAsync({ token: token, artifactId: artifactId });
											}}
											/>
										</div>
									</div>
								</div>
							</ReflexElement>
							<ReflexSplitter />
							<ReflexElement className="left-pane" minSize={60} maxSize={60}>
								<div className="pane-content">
									<div className="row" style={{ width: "100%", height: "5%", textAlign: "center", fontSize: "16px", verticalAlign: "middle", padding: "18px 10px 10px 10px" }}>
										<div className="col-3" style={{ color: "gray" }}>Wrapper name: <span style={{ color: "white" }}>{targetArtifact?.wrapper_name}</span> </div>
										<div className="col-2" style={{ color: "gray" }}>Wrapper type: <span style={{ color: "white" }}>{targetArtifact?.type}</span> </div>
										<div className="col-2" style={{ color: "gray" }}>Cat.: <span style={{ color: "white" }}>{location.state.artifact_category}</span> </div>
										<div className="col-2" style={{ color: "gray" }}>build_env: <span style={{ color: "white" }}>{targetArtifact?.build_env}</span> </div>
										<div className="col-3" style={{ color: "gray" }}>build_time:
											<span style={{ color: "white", paddingRight: ".5rem" }}>{targetArtifact?.build_time}</span>
											<FaInfoCircle title={tooltip} />
										</div>
									</div>
								</div>
							</ReflexElement>
							<ReflexSplitter />
							<ReflexElement>
								<ReflexContainer orientation="vertical">
									<ReflexElement className="left-pane">
										<div style={{ padding: "15px" }}>
											<WrapperServicesBox location={location} serviceStatus={dArtifactStatus?.data} services={artifactServices} />
											<LiveInfoLog message={dArtifactStatus.data} />
										</div>
									</ReflexElement>
									<ReflexSplitter />
									<ReflexElement className="right-pane">
										<div style={{ padding: "15px 15px 0 15px" }}>
											<LogCollapsibleCard2 targetId={location.state.artifact_id} />
										</div>
									</ReflexElement>
								</ReflexContainer>
							</ReflexElement>
							<ReflexSplitter />

						</ReflexContainer>
					</TokenContextProvider>
				</div>
			</div>

		</>
	)
}

export default SdkTesting