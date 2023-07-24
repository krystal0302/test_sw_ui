import React, { useState, useContext } from 'react'
import { nanoid } from 'nanoid';
import { Button } from "react-bootstrap";

import { useGetWrapperPicture } from '../../hooks/FetchHooks';

import UploadImageHolder from './UploadImageHolder';
import ImageUploadBox from './ImageUploadBox';
import ImageCrop from './ImageCrop';

import { TokenContext } from '../../utils/TokenContextProvider';

function ImageUploadPane(props) {
	const [files, setFiles] = useState({});
	const { token } = useContext(TokenContext);

	let wrapperName = props.location.state.wrapper_name;

	const handlePicture = { onSuccess: (data) => { setFiles(data); } }
	const dWrapperPicture = useGetWrapperPicture(token, wrapperName, handlePicture);

	function clearUploadImage(imgType) {
		(imgType === 'avatar') ? setFiles({ ...files, avatar: '' }) : setFiles({ ...files, thumbnail: '' });
	}

	function handleImgFile(srcObj, imgType) {
		(imgType === 'avatar') ? setFiles({ ...files, avatar: srcObj }) : setFiles({ ...files, thumbnail: srcObj });
	}

	return (
		<ImageUploadBox>
			<div className="row">
				<div className="col-6">
					{(files?.avatar == '') ?
						<UploadImageHolder handleFile={handleImgFile} imgType="avatar" /> :
						<ImageCrop
							image={(files.avatar?.includes('blob')) ? `${files?.avatar}` : `data:image/png;base64, ${files?.avatar}`}
							imgType="avatar"
							wrapperName={wrapperName}
							size={{ w: 440, h: 220 }}
							cbs={{ clear: clearUploadImage }} />
					}
				</div>
				<div className="col-6">
					{(files?.thumbnail == '') ?
						<UploadImageHolder handleFile={handleImgFile} imgType="thumbnail" /> :
						<ImageCrop
							image={(files.thumbnail?.includes('blob')) ? `${files?.thumbnail}` : `data:image/png;base64, ${files?.thumbnail}`}
							imgType="thumbnail"
							wrapperName={wrapperName}
							size={{ w: 150, h: 150 }}
							cbs={{ clear: clearUploadImage }} />
					}
				</div>
			</div>
		</ImageUploadBox>
	);
}

export default ImageUploadPane