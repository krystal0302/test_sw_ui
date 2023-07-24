import React, { useState } from 'react'
import { useNavigate } from 'react-router-dom';
import { useGetWrapperPackage, useGetWrapperDeployCandidates } from '../../hooks/FetchHooks';

import ArtifactsOptionModal from './ArtifactsOptionModal';
import {
	FaInfo, FaPen, FaDownload, FaSignOutAlt, FaTrash, FaBatteryThreeQuarters
} from 'react-icons/fa';


const genTooltip = (content) => {
	let services = content.services.map(s => `  - ${s.service}\n`).join('');
	// console.log(services);
	return `id: ${content.wrapper_id}\n` +
		`name: ${content.wrapper_name}\n` +
		`type: ${content.wrapper_type}\n` +
		`cat.: ${content.category}\n` +
		`desc: ${JSON.stringify(content.description)}\n` +
		`services:\n${services}\n`;
};

export default function WrapperItem({ content, callbacks, token }) {
	const wrapperName = content.wrapper_name;
	console.log(content)

	const navigate = useNavigate();

	const [show, setShow] = useState(false);
	const [artifactOptions, setArtifactOptions] = useState([]);

	// ------ artifact deployment candidate list ------
	const [filter1, setFilter1] = useState(false);
	const customizedOptions = {
		enabled: filter1,
		onSuccess: (data) => { setArtifactOptions(data?.candidate); },
		onError: (err) => { console.error(err) },
		refetchInterval: 1000
	}
	const dWrapperDeployCandidates = useGetWrapperDeployCandidates(token, wrapperName, customizedOptions);

	// ------ artifact package export ------
	const dWrapperPackage = useGetWrapperPackage(token, wrapperName, { enabled: true })

	const onClose = () => setShow(false);

	return (
		<>
			<div className="card card-default" id="rolebar-charging_undock_task" style={{ backgroundColor: '#343a40' }}>
				<div className="card-body p-0">
					<table className="table">
						<tbody>
							<tr>
								<td style={{ width: '30px' }}>
									<div className="product-img">
										<img src={`data:image/png;base64, ${content.thumbnail}`} className="map-thumbnail img-size-50" />
									</div>
								</td>
								<td className="align-middle" style={{ fontSize: '1.4rem', width: '30%' }}>
									<div style={{ fontSize: '100%' }}>{content.wrapper_name}</div>
								</td>
								<td className="text-right align-middle" style={{ fontSize: '1.2rem', width: '25%' }}>
									<div style={{ fontSize: '100%', textAlign: 'left' }}>build time: <span>{content.build_time}</span> </div>
								</td>
								<td className="text-right py-0 align-middle">
									<div className="btn-group btn-group-sm">
										<a className="btn" title={genTooltip(content)}>
											<FaInfo size={14} />
										</a>
										<a className="btn" title="Edit Wrapper"
											onClick={() => navigate(`/wrapper_sdk_editor`, { state: { wrapper_name: content.wrapper_name, wrapper_id: content.wrapper_id } })}>
											<FaPen size={14} />
										</a>
										<a className="btn" title="Deploy the Wrapper"
											onClick={() => {
												// --- toggle the deployment candidate query ---
												setFilter1(true);
												setShow(!show);
											}}>
											<FaDownload size={14} />
										</a>
										<a className="btn" title="Export Wrapper"
											onClick={async () => {
												// --- create a download link ---
												// console.log(dWrapperPackage)
												const url = await window.URL.createObjectURL(new Blob([dWrapperPackage.data]));
												const link = document.createElement('a');
												link.href = url;
												link.download = 'wrapper-sdk-' + wrapperName + '.tar.gz';
												document.body.appendChild(link);
												link.click();
												link.remove();                   // remove component after download
												window.URL.revokeObjectURL(url); // release blob object
											}}
										>
											<FaSignOutAlt size={14} />
										</a>
										<a className="btn remove-role" title="Remove Wrapper"
											onClick={() => callbacks.delete(content.wrapper_name)}>
											<FaTrash size={14} />
										</a>
									</div>
								</td>
							</tr>
						</tbody>
					</table>
				</div>
			</div>
			<ArtifactsOptionModal show={show} callbacks={{ close: onClose, farOverlay: callbacks.farOverlay }} data={artifactOptions} wrapperName={content.wrapper_name} />
		</>
	)
}
