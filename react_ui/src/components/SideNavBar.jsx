import React, { useState, useContext } from "react";
import { Link } from "react-router-dom";
import { useGetFleets, useGetFlows, usePostFlow } from "../hooks/FetchHooks";
import { TokenContext } from '../utils/TokenContextProvider';
import FlowTriggerModal from "./FlowTriggerModal";

const fleetNavData = [
	{
		"label": "Dashboard",
		"url": "/dashboard",
		"i18n": 'nav.dashboard',
		"icon": 'fas fa-tachometer-alt'
	},
	{
		"label": "Live Map",
		"url": "/map_live",
		"i18n": 'nav.dashboard',
		"icon": 'fas fa-map-marked-alt'
	},
	{
		"label": "Fleet configuration",
		"url": "/fleet_config",
		"i18n": 'nav.fleet_configuration',
		"icon": 'fas fa-th',
	},
	{
		"label": "Flow configuration",
		"url": "/flow_config",
		"i18n": 'nav.flow_configuration',
		"icon": 'fas fa-edit'
	}
];

const genericNavData = [
	{
		"label": "Map",
		"url": "#",
		"i18n": 'nav.map',
		"icon": 'fas fa-map',
		children: [
			{
				"label": "Create Map",
				"url": "/slam",
				"i18n": 'nav.create_map',
				"icon": 'far fa-circle nav-icon'
			},
			{
				"label": "Edit Map",
				"url": "/map",
				"i18n": 'nav.edit_map',
				"icon": 'far fa-circle nav-icon'
			},
		]
	},
	{
		"label": "Role",
		"url": "/role",
		"i18n": 'nav.role',
		"icon": 'fas fa-users'
	},
	{
		"label": "System settings",
		"url": "/sys_settings",
		"i18n": 'nav.role',
		"icon": 'fas fa-cog'
	},
	{
		"label": "Wrapper SDK",
		"url": "/wrapper_sdk",
		"i18n": 'nav.wrapper_sdk',
		"icon": 'fas fa-gift'
	},
	{
		"label": "Log",
		"url": "/log",
		"i18n": 'nav.log',
		"icon": 'fas fa-clipboard-list'
	},
	{
		"label": "User management",
		"url": "/usr_management",
		"i18n": 'nav.user_management',
		"icon": 'fas fa-user'
	},
	{
		"label": "API info",
		"url": "/ap_info",
		"i18n": 'nav.api_info',
		"icon": 'fas fa-info-circle'
	}
];

function makeNavItems(_navItems) {
	return React.Children.toArray(_navItems.map(nav => {
		let dom;
		if (Object.keys(nav).includes('children') && nav.children.length) {
			let children = React.Children.toArray(nav.children.map(c => (
				<li className="nav-item">
					<Link href={c.url} className="nav-link">
						<i className={c.icon} />
						<p style={{ 'fontSize': '100%' }}>{c.label}</p>
					</Link>
				</li>
			)));

			dom = (
				<li className="nav-item has-treeview menu-close">
					<Link href="#" className={`nav-link ${(nav.url === window.location.pathname) ? "active" : ""}`} >
						<i className={`nav-icon ${nav.icon}`} />
						<p data-i18n={nav.i18n} style={{ fontFamily: 'Arial', fontSize: '100%' }}>{nav.label}</p>
					</Link>
					<ul className="nav nav-treeview">
						{children}
					</ul>
				</li>
			);
		} else {
			dom = (
				<li className="nav-item fleet-group">
					<Link className={`nav-link ${(nav.url === window.location.pathname) ? "active" : ""}`} to={nav.url}>
						<i className={`nav-icon ${nav.icon}`} />
						<p data-i18n={nav.i18n} style={{ 'fontFamily': 'Arial', 'fontSize': '100%' }}>{nav.label}</p>
					</Link>
				</li>
			);
		}
		return dom;
	}
	));
}

const FlowTrigger = ({ flow, callbacks }) => {
	console.log(flow)
	return (
		<li
			className="nav-item trigger-item"
			id={`lsb-${flow}`}
			onClick={() => {
				callbacks.open();
				callbacks.setFlowName(flow);
			}}
		>
			<a className="nav-link">
				<p style={{ 'fontSize': '100%' }}>{flow}</p>
			</a>
		</li >
	);
};

const SideNavBar = (sideNavExpanded, setSideNavExpanded) => {
	const fleetNavItems = makeNavItems(fleetNavData)
	const genericNavItems = makeNavItems(genericNavData)
	const [selFleet, setSelFleet] = useState('');
	const [flows, setFlows] = useState([]);

	const { token } = useContext(TokenContext);

	const handleFleet = {
		onSuccess: (data) => { setSelFleet(data[0]) }
	}
	const dFleets = useGetFleets(token, handleFleet);

	const handleFlow = {
		onSuccess: (data) => {
			let flowArr = data.swarm_data.find(sd => sd.fleet_name === selFleet);
			flowArr = flowArr?.flows || [];
			setFlows(flowArr);
		}
	}
	const dFlows = useGetFlows(token, selFleet, handleFlow);

	const handleOpenModal = () => {
		console.log('open modal')
		setShowTriggerModal(true);
	};

	const handleCloseModal = () => {
		console.log('close modal')
		setShowTriggerModal(false);
	};

	const handleSetTrigFlowName = (name) => {
		setTrigFlowName(name);
	};

	const [showTriggerModal, setShowTriggerModal] = useState(false);
	const [trigFlowName, setTrigFlowName] = useState('');


	return (
		<aside className="main-sidebar sidebar-dark-primary elevation-4" onToggle={() => { setSideNavExpanded(!sideNavExpanded); }}>
			<a href="index.html" className="brand-link logo-switch">
				<img src="dist/img/FARobotLogo.png" alt="FARobot Logo" className="brand-image img-circle elevation-3 logo-xs" />
				<img src="dist/img/FARobotLogo_lg.png" alt="FARobot Logo" className="brand-image logo-xl" />
			</a>
			<div className="sidebar os-host os-theme-light os-host-resize-disabled os-host-scrollbar-horizontal-hidden os-host-scrollbar-vertical-hidden os-host-transition">
				<div className="os-resize-observer-host observed">
					<div className="os-resize-observer" style={{ left: '0px', right: 'auto', 'fontSize': '100%' }} />
				</div>
				<div className="os-size-auto-observer observed" style={{ 'height': 'calc(90% + 1px)', float: 'left' }}>
					<div className="os-resize-observer" style={{ 'fontSize': '100%' }} />
				</div>
				<div className="os-content-glue" style={{ margin: '0px -8px', 'fontSize': '100%', width: '249px', height: '773px' }} />
				<div className="os-viewport os-viewport-native-scrollbars-invisible">
					<div className="os-content" style={{ 'padding': '0px 8px', height: '100%', width: '100%' }}>
						<nav className="mt-2">
							<ul className="nav nav-pills nav-sidebar flex-column" data-widget="treeview" role="menu" data-accordion="false">
								<div className="user-panel" style={{ width: '100%' }}>
									<li className="nav-header">
										<select
											id="fleet-select"
											className="form-control select-bold-text"
											onChange={(e) => {
												console.log(e.target.value);
												setSelFleet(e.target.value);
												dFlows.refetch(token, e.target.value)
											}}
											style={{ 'fontSize': '100%' }}>
											{(dFleets.data) ? dFleets.data
												.map((fleet, index) => <option key={index} value={fleet}>{fleet}</option>) : null
											}
										</select>
									</li>
									{fleetNavItems}
									<li className="nav-item has-treeview menu-open fleet-group" id="trigger-list">
										<a href="#" className="nav-link">
											<p data-i18n="nav.flow_trigger" style={{ 'fontFamily': 'Arial', 'fontSize': '100%' }}>Flow trigger</p>
										</a>
										<ul className="nav nav-treeview">
											{(flows) ? flows
												.map((flow, index) => <FlowTrigger key={index} flow={flow} callbacks={{ open: handleOpenModal, setFlowName: handleSetTrigFlowName }} />)
												: null
											}
										</ul>
									</li>
								</div>
								{genericNavItems}
							</ul>
						</nav>
					</div>
				</div>
				<div className="os-scrollbar os-scrollbar-horizontal os-scrollbar-unusable os-scrollbar-auto-hidden">
					<div className="os-scrollbar-track">
						<div className="os-scrollbar-handle" style={{ width: '100%', transform: 'translate(0px, 0px)', 'fontSize': '100%' }} />
					</div>
				</div>
				<div className="os-scrollbar os-scrollbar-vertical os-scrollbar-unusable os-scrollbar-auto-hidden">
					<div className="os-scrollbar-track">
						<div className="os-scrollbar-handle" style={{ height: '100%', transform: 'translate(0px, 0px)', 'fontSize': '100%' }} />
					</div>
				</div>
				<div className="os-scrollbar-corner" style={{ 'fontSize': '100%' }} />
			</div>
			<div className="p-3 sidebar-footer" style={{ fontFamily: 'Arial', color: 'white', textAlign: 'center', borderTop: "solid 1px lightgray" }}>
				<span style={{ fontSize: '100%' }}>Version</span> 1.0.223
			</div>
			<FlowTriggerModal
				show={showTriggerModal}
				callbacks={{ closeModal: handleCloseModal }}
				flowName={trigFlowName}
			/>

		</aside >
	);
};


export default SideNavBar;