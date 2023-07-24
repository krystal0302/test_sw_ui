import "./ImageCrop.css";

import { useCallback, useState, useRef, useContext } from "react";
import { Slider } from '@mui/material';
import Cropper from "react-easy-crop";

import getCroppedImg from "../../utils/Crop";
import { TokenContext } from '../../utils/TokenContextProvider';
import { FaCrop, FaSave, FaTrash, FaEdit } from "react-icons/fa";
import { notificationMsg } from "../Notifications";
import { usePutWrapperPicture } from "../../hooks/FetchHooks";


function getBase64Image(img) {
	var canvas = document.createElement("canvas");
	canvas.width = img.width;
	canvas.height = img.height;
	var ctx = canvas.getContext("2d");
	ctx.drawImage(img, 0, 0);
	var dataURL = canvas.toDataURL("image/png");
	return dataURL;
}

const ImageCrop = ({ image, size, cbs, imgType, wrapperName }) => {
	const [crop, setCrop] = useState({ x: 0, y: 0 });
	const [zoom, setZoom] = useState(1);
	// const [rotation, setRotation] = useState(0);
	const [croppedAreaPixels, setCroppedAreaPixels] = useState(null);
	const [croppedImage, setCroppedImage] = useState(null);
	const croppedImgRef = useRef(null);

	console.log(size);

	const onCropComplete = useCallback((croppedArea, croppedAreaPixels) => {
		setCroppedAreaPixels(croppedAreaPixels);
	}, []);

	const { token } = useContext(TokenContext);

	const handleWrapperPicture = {
		onSuccess: async (data) => { notificationMsg('success', `Picture is saved Successfully`); },
		onError: async (err) => { notificationMsg('error', `Failed to save Picture!!`); }
	}
	const dPutWrapperPicture = usePutWrapperPicture(handleWrapperPicture);


	const saveCroppedImage = useCallback(async () => {
		try {
			let data = croppedImage.split(',');
			dPutWrapperPicture.mutateAsync({ token: token, wrapperName: wrapperName, imgBase64: data[1], imgType: imgType })
		} catch (e) {
			console.error(e);
		}
	});

	const showCroppedImage = useCallback(async () => {
		try {
			const croppedImage = await getCroppedImg(
				image,
				croppedAreaPixels,
				size
				// { w: 440, h: 220 }
				// rotation
			);
			// console.log("done", { croppedImage });
			console.log(croppedImage);
			setCroppedImage(croppedImage);
		} catch (e) {
			console.error(e);
		}
	}, [croppedAreaPixels, image]);
	// }, [croppedAreaPixels, rotation, image]);

	const onClose = useCallback(() => {
		setCroppedImage(null);
	}, []);

	return (
		<div>
			<div
				className="container"
				style={{
					display: image === null || croppedImage !== null ? "none" : "block",
				}}
			>
				<div className="crop-container">
					<Cropper
						image={image}
						crop={crop}
						zoom={zoom}
						// rotation={rotation}
						showGrid={true}
						cropSize={{ width: size.w, height: size.h }}
						onCropChange={setCrop}
						onCropComplete={onCropComplete}
						onZoomChange={setZoom}
					// onRotationChange={setRotation}
					/>
				</div>
				{/* <div>
					<div className="row">
						<div className="col-3">
							<label>Rotate</label>
						</div>
						<div className="col-9">
							<Slider
								value={rotation}
								min={0}
								max={360}
								step={1}
								aria-labelledby="rotate"
								onChange={(e, rotation) => setRotation(rotation)}
								className="range col-9"
							/>
						</div>
					</div>
				</div> */}
				{/* <label>
					Zoom
					<Slider
						value={zoom}
						min={1}
						max={3}
						step={0.1}
						aria-labelledby="zoom"
						onChange={(e, zoom) => setZoom(zoom)}
						className="range"
					/>
				</label> */}
			</div>
			<div
				className="cropped-image-container"
				style={{
					display: image === null || croppedImage !== null ? "block" : "none",
				}}
			>
				<img
					src={croppedImage}
					alt="cropped image"
					className="cropped-image"
					style={{ width: size.w, height: size.h }}
					ref={croppedImgRef}
				/>
			</div>
			<div style={{ textAlign: "center", paddingTop: "1.5rem" }}>
				<button
					className="btn btn-secondary"
					style={{ display: croppedImage ? 'inline' : 'none', marginRight: "0.5rem" }}
					onClick={onClose}>
					<FaEdit />
					<span style={{ paddingLeft: ".5rem" }}>Edit</span>
				</button>
				<button
					className="btn btn-secondary"
					style={{ display: (!croppedImage) ? 'inline' : 'none', marginRight: "0.5rem" }}
					onClick={showCroppedImage}>
					<FaCrop />
					<span style={{ paddingLeft: ".5rem" }}>Crop</span>
				</button>
				<button
					className="btn btn-secondary"
					style={{ display: croppedImage ? 'inline' : 'none', marginRight: "0.5rem" }}
					onClick={saveCroppedImage}>
					<FaSave />
					<span style={{ paddingLeft: ".5rem" }}>Save</span>
				</button>
				<button
					className="btn btn-secondary"
					style={{ display: 'inline' }}
					onClick={() => { cbs.clear(imgType); }}>
					<FaTrash />
					<span style={{ paddingLeft: ".5rem" }}>Clear</span>
				</button>
			</div>

		</div>
	);
};

export default ImageCrop;