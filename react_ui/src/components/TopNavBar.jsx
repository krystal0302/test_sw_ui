import { FaThLarge } from 'react-icons/fa';

const TopNavBar = () => {
	return (
		<>
			{/* EXTEND ver. */}
			{/* <nav className="main-header navbar navbar-expand navbar-dark farobot-dark-mode"> */}
			{/* COLLAPSE ver. */}
			<nav className="navbar navbar-expand navbar-dark farobot-dark-mode">
				{/* <!-- Left navbar links --> */}
				<ul className="navbar-nav">
					<li className="nav-item">
						{/* EXTEND ver. */}
						{/* <a className="nav-link" data-widget="pushmenu" href="#" role="button"><i className="fas fa-bars"></i></a> */}
						{/* COLLAPSE ver. */}
						<a className="nav-link" data-widget="pushmenu" href="#" role="button"><i className="fas fa-bars" style={{ color: 'gray' }}></i></a>
					</li>
					<li className="nav-item d-none d-sm-inline-block ml-auto">
						<a href="#" className="nav-link" id="statusIndicator"></a>
					</li>
				</ul>

				{/* <!-- Right navbar links --> */}
				{/* <ul className="navbar-nav ml-auto">
				<li className="nav-item align-self-center font-weight-bold" id="user-login-status">Hi, Farobot_admin!</li>
				<li className="nav-item align-self-center font-weight-bold">&emsp;|</li>
				<li className="nav-item align-self-center">
					<button type="button" data-i18n="nav.sign_out" className="btn btn-link">Sign out</button>
				</li>
				<li className="nav-item">
					<a className="nav-link" data-slide="true" href="#" role="button">
						<FaThLarge size={14} />
					</a>
				</li>
			</ul> */}
			</nav>
		</>
	);
};

export default TopNavBar;