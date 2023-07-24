import PropTypes from "prop-types";

function FarButton({
	label,
	backgroundColor = "red",
	color = "black",
	size = "md",
	handleClick,
}) {
	let scale = 1;
	if (size === "sm") scale = 0.5;
	if (size === "lg") scale = 2;

	const style = {
		backgroundColor,
		padding: `${scale * 0.5}rem ${scale * 1}rem`,
		border: "2px solid black",
		borderRadius: "25px",
		color,
	};

	return (
		<button onClick={handleClick} style={style}>
			{label}
		</button>
	);
}

FarButton.propTypes = {
	label: PropTypes.string,
	backgroundColor: PropTypes.string,
	size: PropTypes.oneOf(["sm", "md", "lg"]),
	color: PropTypes.oneOf(["blue", "white", "black"]),
	handleClick: PropTypes.func,
};

export default FarButton;