import React from 'react'
import Form from 'react-bootstrap/Form';
import Button from "react-bootstrap/Button";
import InputGroup from 'react-bootstrap/InputGroup';
import { FaSearch } from 'react-icons/fa';


const capitalize = (str) => str.charAt(0).toUpperCase() + str.slice(1);

const SearchBox = ({ item, callbacks }) => {
	return (
		<React.Fragment>
			<InputGroup className="mb-3">
				<Form.Control
					type="text"
					placeholder={`Search for ${capitalize(item)}s...`}
					onKeyUp={(e) => { callbacks.search(e.target.value) }}
				/>
				<Button
					variant="outline-secondary"
				>
					<FaSearch />
				</Button>
			</InputGroup>
		</React.Fragment>
	);
}

export default SearchBox;
