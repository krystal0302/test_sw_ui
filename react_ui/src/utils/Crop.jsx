export const createImage = (url) => {
	return new Promise((resolve, reject) => {
		const image = new Image();
		image.addEventListener("load", () => resolve(image));
		image.addEventListener("error", (error) => reject(error));
		image.setAttribute("crossOrigin", "anonymous"); // to avoid cross-origin issues 
		image.src = url;
	});
}

export function getRadian(degreeValue) {
	return (degreeValue * Math.PI) / 180;
}

// --- returns the new bounding area of a rotated rectangle. ---
export function rotateSize(width, height, rotation) {
	const rotRad = getRadian(rotation);

	return {
		width: Math.abs(Math.cos(rotRad) * width) + Math.abs(Math.sin(rotRad) * height),
		height: Math.abs(Math.sin(rotRad) * width) + Math.abs(Math.cos(rotRad) * height),
	};
}

export default async function getCroppedImg(
	imageSrc,
	pixelCrop,
	size,
	rotation = 0,
	flip = { horizontal: false, vertical: false }
) {
	const image = await createImage(imageSrc);
	const canvas = document.createElement("canvas");
	const ctx = canvas.getContext("2d");

	if (!ctx) {
		return null;
	}

	const rotRad = getRadian(rotation);

	// --- calculate bounding box of the rotated image ---
	const { width: bBoxWidth, height: bBoxHeight } = rotateSize(
		image.width,
		image.height,
		rotation
	);

	// -- fit canvas to the bounding box ---
	// canvas.width = bBoxWidth;
	// canvas.height = bBoxHeight;
	canvas.width = size.w;  // 440
	canvas.height = size.h; // 220


	// transform canvas context into a central location to allow rotating and flipping around the center
	ctx.translate(bBoxWidth / 2, bBoxHeight / 2);
	ctx.rotate(rotRad);
	ctx.scale(flip.horizontal ? -1 : 1, flip.vertical ? -1 : 1);
	ctx.translate(-image.width / 2, -image.height / 2);

	// --- draw rotated image ---
	// ctx.drawImage(image, 0, 0);

	// --- croppedAreaPixels are bounding box relative extract the cropped image using these values ---
	const data = ctx.getImageData(
		pixelCrop.x,
		pixelCrop.y,
		pixelCrop.width,
		pixelCrop.height
	);
	ctx.drawImage(image, pixelCrop.x, pixelCrop.y, pixelCrop.width, pixelCrop.height, 0, 0, size.w, size.h); // w: 440, h: 220

	// --- convert to base64 string format ---
	let res = canvas.toDataURL();
	return res;

}
