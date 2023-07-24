import React from "react";
import { BrowserRouter as Router, Routes, Route, Navigate } from "react-router-dom";
import TopNavBar from "./components/TopNavBar";
import SideNavBar from "./components/SideNavBar";
import SdkDashboard from "./pages/SdkDashboard";
import { QueryClient, QueryClientProvider } from "react-query";
import { ReactQueryDevtools } from 'react-query/devtools';
import SdkEditor from "./pages/SdkEditor";
import SdkTesting from "./pages/SdkTesting";
import { NotificationContainer } from "./components/Notifications";
import { TokenContextProvider } from './utils/TokenContextProvider';

const queryClient = new QueryClient();

const Index = () => window.location.href = `http://${document.location.host}/index.html`;
const LiveMap = () => window.location.href = `http://${document.location.host}/map_live.html`;
const FleetConfig = () => window.location.href = `http://${document.location.host}/fleet.html`;
const FlowConfig = () => window.location.href = `http://${document.location.host}/operation.html`;
const CreateMap = () => window.location.href = `http://${document.location.host}/slam.html`;
const EditMap = () => window.location.href = `http://${document.location.host}/map.html`;
const Role = () => window.location.href = `http://${document.location.host}/role.html`;
const System = () => window.location.href = `http://${document.location.host}/settings2.html`;
const Log = () => window.location.href = `http://${document.location.host}/log.html`;
const UserManagement = () => window.location.href = `http://${document.location.host}/usr_settings.html`;
const ApiInfo = () => window.location.href = `http://${document.location.hostname}:5000/docs`;


export default function App() {
	return (
		<>
			<QueryClientProvider client={queryClient}>
				<TokenContextProvider>
					<TopNavBar />
					<Router>
						{/* <SideNavBar /> */}
						<Routes>
							<Route index element={<Index />} />
							<Route path="dashboard" element={<Index />} />
							<Route path="map_live" element={<LiveMap />} />
							<Route path='fleet_config' element={<FleetConfig />} />
							<Route path='flow_config' element={<FlowConfig />} />

							<Route path='slam' element={<CreateMap />} />
							<Route path='map' element={<EditMap />} />
							<Route path='role' element={<Role />} />
							<Route path='sys_settings' element={<System />} />
							<Route path='log' element={<Log />} />
							<Route path='usr_management' element={<UserManagement />} />
							<Route path='api_info' element={<ApiInfo />} />

							<Route path='wrapper.html' element={<SdkDashboard />} />
							{/* <Route path='wrapper_sdk' element={<SdkDashboard />} /> */}
							<Route path='wrapper_sdk_editor' element={<SdkEditor />} >
								<Route path=':pane' element={<SdkEditor />} />
							</Route>
							<Route path='wrapper_sdk_testing' element={<SdkTesting />} />

							{/* <Route path="*" element={<Navigate to="/wrapper_sdk" replace />} /> */}
						</Routes>
					</Router>
					<ReactQueryDevtools initialIsOpen={false} />
					<NotificationContainer />
				</TokenContextProvider>
			</QueryClientProvider>
		</>
	);
}