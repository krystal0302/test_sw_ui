import React, { useState, useContext } from 'react'
import { Modal, Button, Form } from 'react-bootstrap';
import SearchBox from './SearchBox';
import { notificationMsg } from '../Notifications';
import { TokenContext } from '../../utils/TokenContextProvider';
import { usePutDeployWrapper } from '../../hooks/FetchHooks';

import Box from '@mui/material/Box';
import Card from '@mui/material/Card';
import CardActions from '@mui/material/CardActions';
import CardContent from '@mui/material/CardContent';
import Typography from '@mui/material/Typography';
import Divider from '@mui/material/Divider';
import { Category } from '@mui/icons-material';
import { Backdrop } from '@mui/material';
import CircularProgress from '@mui/material/CircularProgress';

import ArtifactCard from './ArtifactCard';

const ArtifactContent = ({ ar }) => {
	// console.log(ar)
	return (
		<>
			<Typography variant="body1">
				<span>{ar.deploy_artifact_id}</span>
			</Typography>
			<Typography variant="body1" style={{ paddingLeft: '1rem' }}>
				<span>deploy_result</span>:	<span>{ar.deploy_result}</span>
			</Typography>
			<Typography variant="body1" style={{ paddingLeft: '1rem' }}>
				<span>deploy_msg</span>:	<span>{ar.deploy_msg}</span>
			</Typography>
			<Typography variant="body1" style={{ paddingLeft: '1rem' }}>
				<span>bind_robot_id</span>:	<span>{ar.bind_robot_id}</span>
			</Typography>
			<Typography variant="body1" style={{ paddingLeft: '1rem' }}>
				<span>bind_result</span>:	<span>{ar.bind_result}</span>
			</Typography>
			<Typography variant="body1" style={{ paddingLeft: '1rem' }}>
				<span>bind_msg</span>:	<span>{ar.bind_msg}</span>
			</Typography>
		</>
	);
}

const DeployInfoCard = (props) => {
	// console.log(props.deployResult)
	const wrapperInfo = props.deployResult?.wrapper_info;
	// console.log(wrapperInfo)
	const artifactResult = props.deployResult?.result;
	// console.log(artifactResult)
	return (
		<div style={{ margin: '1%' }}>
			<Card>
				<CardContent>
					<Typography variant="body1" style={{ textAlign: 'center', fontWeight: 'bold' }}>
						Deploy Result:
					</Typography>
					{(wrapperInfo) ? (
						<>
							<Typography variant="body1">
								<span>wrapper_name</span>:	<span>{wrapperInfo['wrapper_name']}</span>
							</Typography>
							<Typography variant="body1">
								<span>wrapper_id</span>:	<span>{wrapperInfo['wrapper_id']}</span>
							</Typography>
							<Typography variant="body1">
								<span>wrapper_type</span>:	<span>{wrapperInfo['wrapper_type']}</span>
							</Typography>
							<Typography variant="body1">
								<span>category</span>:	<span>{wrapperInfo['category']}</span>
							</Typography>
							<Typography variant="body1">
								<span>env_version</span>:	<span>{wrapperInfo['env_version']}</span>
							</Typography>
							<Typography variant="body1">
								<span>build_time</span>:	<span>{wrapperInfo['build_time']}</span>
							</Typography>
						</>
					) : null}
					<Divider />
					{(artifactResult) ? (artifactResult.map(ar => <ArtifactContent ar={ar} />)) : null}
				</CardContent>
			</Card>
		</div>
	);
}

let selArtifactRecord = [];
export default function ArtifactsOptionModal({ show, callbacks, data, wrapperName }) {
	const [keyword, setKeyword] = useState('');
	const [returnData, setReturnData] = useState(null);
	const [selArtifactRecord, setSelArtifactRecord] = useState([]);
	const [open, setOpen] = React.useState(false);

	// ------ artifact deployment ------
	const { token } = useContext(TokenContext);

	const handles = {
		onSuccess: (data) => {
			setReturnData(data.data);
			notificationMsg('success', "The Wrapper is deployed Successfully");
			// callbacks.close();
			setOpen(false);
		},
		onError: (err) => {
			console.log(err.response.data);
			notificationMsg('error', err.response.data || "Failed to Deploy the wrapper!!");
			setOpen(false);
		}
	};
	const dPutDeployWrapper = usePutDeployWrapper(handles);

	const searchArtifact = (key) => { setKeyword(key); };

	return (
		<Modal show={show} animation={false}>
			<Modal.Header>
				<Modal.Title>Deploy Artifacts</Modal.Title>
			</Modal.Header>
			<Modal.Body>
				<SearchBox item="artifact" callbacks={{ search: searchArtifact }} />
				<Form>
					{data
						.filter(d => d.includes(keyword))
						.map((d, index) => (
							<div key={`${index}`} className="mb-3">
								<Form.Check
									type="checkbox"
									key={d}
									id={d}
									label={d}
									onChange={(e) => {
										const isChecked = e.target.checked;
										const isIncluded = selArtifactRecord.includes(e.target.id);
										// console.log(`state: checked: ${isChecked}, included: ${isIncluded}`);

										// --- updated the mis-aligned cases ---
										if (!isChecked && isIncluded) {
											const updated = selArtifactRecord.filter((t) => t !== e.target.id);
											setSelArtifactRecord([...updated])
										}

										if (isChecked && !isIncluded) {
											setSelArtifactRecord([...selArtifactRecord, e.target.id])
										}
										console.log(selArtifactRecord);
									}}
								/>
							</div>
						))}
				</Form>
				{(returnData) ? (
					<>
						<Divider />
						<div style={{ paddingTop: '1.5rem' }}>
							{/* <h4>Deploy Result</h4> */}
							<DeployInfoCard deployResult={returnData} />
						</div>
					</>
				) : null}
			</Modal.Body>
			<Modal.Footer>
				{(returnData) ? (
					<>
						<Button
							variant="primary"
							onClick={() => {
								setReturnData(null);
								callbacks.close();
							}}>
							OK
						</Button>
					</>
				) : (
					<>
						<Button
							variant="secondary"
							onClick={() => {
								setReturnData(null);
								callbacks.close();
							}}>
							Cancel
						</Button>
						<Button
							variant="primary"
							onClick={async () => {
								setOpen(!open);
								await dPutDeployWrapper.mutateAsync({ token: token, wrapperName: wrapperName, artifacts: selArtifactRecord })
								setSelArtifactRecord([...updated])
							}}>
							Deploy
						</Button>
					</>
				)
				}

				<Backdrop
					sx={{ color: '#fff', zIndex: (theme) => theme.zIndex.drawer + 1 }}
					open={open}
				// onClick={() => setOpen(false)}
				>
					<CircularProgress color="inherit" />
				</Backdrop>
			</Modal.Footer >
		</Modal >
	)
}

