import React, { useRef } from 'react'
import './UploadImageHolder.css'

function UploadImageHolder(props) {
	const hiddenImgInput = useRef(null);

	const handleClick = (e) => {
		hiddenImgInput.current.click();
	};

	return (
		<label for="images" className="drop-container" onClick={handleClick}>
			<span className="drop-title">Click to Upload Image</span>
			<input
				type="file"
				accept="image/*"
				ref={hiddenImgInput}
				onChange={async (e) => {
					let srcObj = URL.createObjectURL(e.target.files[0]);
					props.handleFile(srcObj, props.imgType);
				}}
				style={{ display: 'none' }} />
		</label>
	)
}

export default UploadImageHolder