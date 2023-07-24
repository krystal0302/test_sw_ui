import React, { useState, useRef, useContext } from "react";
import { Button } from "react-bootstrap";

import * as monaco from 'monaco-editor'
import Editor, { loader } from "@monaco-editor/react";
loader.config({ monaco });

import { notificationMsg } from "../Notifications";
import { TokenContext } from '../../utils/TokenContextProvider';
import { useGetWrapperCode, usePutSaveCode } from "../../hooks/FetchHooks";


const CodeEditorPane = (props) => {
	const [value, setValue] = useState("");

	const editorRef = useRef(null);

	const handleEditorChange = (value) => {
		setValue(value);
	};

	const handleEditorDidMount = (editor) => {
		editorRef.current = editor;
	}

	const { token } = useContext(TokenContext);

	const handles = {
		onSuccess: (data) => { notificationMsg('success', `Code is saved Successfully`); },
		onError: (err) => {
			notificationMsg('error', err.response.data);
		}
	};

	const wrapperName = props.location.state.wrapper_name;
	const dWrapperCode = useGetWrapperCode(token, wrapperName);
	const dPutSaveCode = usePutSaveCode(handles);

	const handleCodeOnSave = async (token, wrapperName) => {
		const codeSnippet = editorRef.current.getValue();
		const jsonCodeSnippet = JSON.stringify(codeSnippet);

		await dPutSaveCode.mutateAsync({ token: token, wrapperName: wrapperName, codeSnippet: jsonCodeSnippet });
		await dWrapperCode.refetch();
	};

	return (
		<div className="card card-dark">
			<div style={{ textAlign: 'center', fontSize: '20px', padding: '1rem 0 0.5rem 0' }}>Operational logic</div>
			<Editor
				onMount={handleEditorDidMount}
				height="36vh"
				width={"100%"}
				language={"python"}
				value={value}
				theme={"vs-dark"}
				defaultValue={props.param.content}
				onChange={handleEditorChange}
			/>
			<div className="card-footer text-center" style={{ fontSize: "100%", display: "block" }}>
				<Button onClick={async () => { await handleCodeOnSave(token, wrapperName); }}>Save</Button>
			</div>
		</div >
	)
};

export default CodeEditorPane;