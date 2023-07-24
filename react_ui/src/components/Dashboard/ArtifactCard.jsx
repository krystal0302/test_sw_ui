import React from 'react'
import lift from '../../assets/img/sprites/lift_module-440x220.png'
import { useNavigate } from 'react-router-dom';
import { useGetArtifactAvatarProperty } from '../../hooks/FetchHooks';
import { Skeleton } from '@mui/material';

export default function ArtifactCard({ content, token }) {
	const navigate = useNavigate();

	const onClick = (e) => {
		navigate("/wrapper_sdk_testing", { state: content });
	}

	const dArtifactAvatar = useGetArtifactAvatarProperty(token, content.artifact_id);
	const avatar = (dArtifactAvatar.data) ? `data:image/png;base64, ${dArtifactAvatar.data[0].avatar}` : null;

	return (
		// <div className="card col-lg-3 card-dark" style={{ margin: '1rem', minWidth: '250px' }}>
		<div className="card card-dark image-container increase-size" >
			{/* <img className="card-img-top" src={lift} alt="Card image cap" /> */}
			{(avatar) ? (
				<img className="card-img-top" src={avatar} alt="Card image cap" />
			) : (
				<Skeleton
					sx={{ bgcolor: 'grey.600' }}
					variant="rounded"
					style={{ height: "100%", aspectRatio: "1/2", margin: '0 auto' }} />
			)}
			<div className="card-body">
				<div className="info-box-content">
					<div style={{ textAlign: 'center' }}>
						<div className="row">
							<div className="col-9" style={{ fontSize: '18px' }}>
								<span className="info-box-number ml-2" style={{ fontSize: '100%' }}>{content.artifact_name}</span>
							</div>
							<div className="col-9" style={{ fontSize: '18px' }}>
								<span className="info-box-number ml-2" style={{ fontSize: '90%' }}>{content.artifact_id}</span>
							</div>
							<div className="col-9" style={{ fontSize: '18px' }}>
								<span className="info-box-number ml-2" style={{ fontSize: '90%' }}>{content.wrapper_name}</span>
							</div>
							<div className="col-9" style={{ fontSize: '18px' }}>
								<span className="info-box-number ml-2" style={{ fontSize: '90%' }}>{content.type}</span>
							</div>
							<div className="col-9" style={{ fontSize: '18px' }}>
								<span className="info-box-number ml-2" style={{ fontSize: '90%' }}>{content.build_time}</span>
							</div>
							<div className="col-3" style={{ fontSize: '100%' }}>
							</div>
						</div>
						<div className="row">
							<div className="col-9">
								<label className="agent-mode ml-2 agent-fleet-status" style={{ color: 'red', fontSize: '100%' }}>{content.artifact_category}</label>
							</div>
							<div className="col-3">
								<button className="btn agent-settings" style={{ float: 'right' }} onClick={onClick}>
									<i className="nav-icon fas fa-cog" />
								</button>
							</div>
						</div>
					</div>
				</div>
			</div>
		</div>
	)
}
