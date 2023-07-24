import React from 'react'

function ImageUploadBox(props) {
	return (
		<div className="card card-dark">
			<div className="row" style={{ textAlign: 'center', fontSize: '20px', paddingTop: '1rem' }}>
				<div className="col-md-6 offset-md-3">
					<span>Picture</span>
				</div>
				<div className="col-md-1 offset-md-1">
					<i class="fa fa-info-circle" title=""></i>
				</div>
			</div>
			<div className="card-body flow-list-panel" style={{ fontSize: "16px", height: "45vh", overflowY: "auto" }}>
				{props.children}
			</div>
		</div >
	)
}

export default ImageUploadBox