import { useQuery, useMutation } from "react-query";
import * as api from "../api/FetchApi"
import { notificationMsg } from '../components/Notifications';

// ====== configurations ======
const defaultOptions = {
	refetchOnWindowFocus: false,
	refetchOnmount: false,
	refetchOnReconnect: false,
	retry: false,
}

// ====== fleets ======
export const useGetFleets = (token, configOptions = {}) => {
	return useQuery(
		['fleets', token],
		api.fetchGetFleets,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ====== flows ======
export const useGetFlows = (token, selFleet, configOptions = {}) => {
	return useQuery(
		['flow-profiles', token],
		api.fetchGetFlows,
		{
			...defaultOptions,
			enabled: !!token && !!selFleet,
			...configOptions,
		}
	);
}

export const usePostFlow = (configOptions = {}) => {
	return useMutation(
		api.fetchPostFlow,
		{
			...configOptions,
		}
	);
}

// ====== v1 artifacts ======
export const usePostArtifactConfigs = (configOptions = {}) => {
	return useMutation(
		api.fetchPostArtifactConfigs,
		{
			...configOptions,
		}
	);
}

export const usePutArtifactConfigs = (configOptions = {}) => {
	return useMutation(
		api.fetchPutArtifactConfigs,
		{
			...configOptions,
		}
	);
}

// ====== artifacts ======
export const useGetArtifacts = (token, configOptions = {}) => {
	return useQuery(
		['artifact-profiles', token],
		api.fetchGetArtifacts,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const useGetArtifactStatus = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['artifact-status', token, artifactId],
		api.fetchGetArtifactStatus,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const useGetArtifactAvatarProperty = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['artifact-avatar', token, artifactId],
		api.fetchGetArtifactAvatarProperty,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const useGetArtifactProperty = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['artifact-property', token, artifactId],
		api.fetchGetArtifactProperty,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ====== SDK ======

// ------ artifact-request (artifact services) ------
export const useGetArtifactService = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['artifact-services', token, artifactId],
		api.fetchGetArtifactService,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePostRunArtifactService = (configOptions = {}) => {
	return useMutation(
		api.fetchPostRunArtifactService,
		{
			...configOptions,
		}
	);
}

export const usePutResumeArtifactService = (configOptions = {}) => {
	return useMutation(
		api.fetchPutResumeArtifactService,
		{
			...configOptions,
		}
	);
}

export const useDeleteCancelArtifactService = (configOptions = {}) => {
	return useMutation(
		api.fetchDeleteCancelArtifactService,
		{
			...configOptions,
		}
	);
}


// ------ runtime logs ------
export const useGetTargetLogs = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['log-printout', token, artifactId],
		api.fetchGetArtifactLogs,
		{
			// refetchInterval: 800,
			enabled: !!token,
		}
	);
}

// ------ runtime logs package ------
export const useGetTargetLogsPackage = (token, artifactId, configOptions = {}) => {
	console.log('export artifact: ' + artifactId);
	return useQuery(
		['export-artifact-log', token, artifactId],
		api.fetchGetArtifactLogsPackage,
		{
			defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}


// ------ wrappers ------
export const useGetWrappers = (token, wrapperName = '', configOptions = {}) => {
	return useQuery(
		['wrapper-profiles', token, wrapperName],
		api.fetchGetWrappers,
		{
			...defaultOptions,
			enabled: !!token,
			refetchInterval: 1000,
			...configOptions,
		}
	);
}

export const usePostCreateWrapper = (configOptions = {}) => {
	return useMutation(
		api.fetchPostCreateWrapper,
		{
			...configOptions,
		}
	);
}

export const useDeleteWrapper = (configOptions = {}) => {
	return useMutation(
		api.fetchDeleteWrapper,
		{
			...configOptions,
		}
	);
}

// ------ wrapper types ------
export const useGetWrapperTypes = (token, configOptions = {}) => {
	return useQuery(
		['wrapper-types', token],
		api.fetchGetWrapperTypes,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ------ wrapper categories ------
export const useGetWrapperCategories = (token, configOptions = {}) => {
	return useQuery(
		['wrapper-categories', token],
		api.fetchGetWrapperCategories,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ------ wrapper package ------
export const useGetWrapperPackage = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-package', token, wrapperName],
		api.fetchGetWrapperPackage,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	)
}

// ------ build ------
export const useGetWrapperBuild = (token, artifactId, configOptions = {}) => {
	return useQuery(
		['wrapper-build', token, artifactId],
		api.fetchGetWrapperBuild,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutWrapperBuild = (configOptions = {}) => {
	return useMutation(
		api.fetchPutWrapperBuild,
		{
			onSuccess: (data) => { notificationMsg('success', `The Wrapper is built Successfully`); },
			onError: (err) => { notificationMsg('error', `Failed to build the wrapper!!`); },
			...configOptions,
		}
	);
}

// ------ deploy ------
export const useGetWrapperDeployCandidates = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-deploy-candidates', token, wrapperName],
		api.fetchGetWrapperDeployCandidates,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	)
}

export const usePutDeployWrapper = (configOptions = {}) => {
	return useMutation(
		api.fetchPutDeployWrapper,
		{
			...configOptions,
		}
	);
}

// ------ edit (triee-view) ------
export const useGetWrapperTreeview = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['sdk-treeview', token, wrapperName],
		api.fetchGetSdkEditTreeView,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ------ live info ------
export const useGetWrapperLiveInfo = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-live-info', token, wrapperName],
		api.fetchGetSdkEditLiveInfo,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ------ configuration ------
export const useGetWrapperConfigParam = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-config-param', token, wrapperName],
		api.fetchGetWrapperConfigParam,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutWrapperConfigParam = (configOptions = {}) => {
	return useMutation(
		api.fetchPutWrapperConfigParam,
		{
			onSuccess: async (data) => { notificationMsg('success', `Config Params are set Successfully`); },
			onError: async (err) => { notificationMsg('error', `Failed to set Config Params!!`); },
			...configOptions,
		}
	);
}

// ------ wrapper data ------
export const useGetWrapperData = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-data', token, wrapperName],
		api.fetchGetWrapperData,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

// ------ picture ------
export const useGetWrapperPicture = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-picture', token, wrapperName],
		api.fetchGetWrapperPicture,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutWrapperPicture = (configOptions = {}) => {
	return useMutation(
		api.fetchPutWrapperPicture,
		{
			...configOptions,
		}
	);
}

// ------ service (services in a wrapper) ------
export const useGetWrapperServices = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-services', token, wrapperName],
		api.fetchGetWrapperServices,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutCreateWrapperService = (configOptions = {}) => {
	return useMutation(
		api.fetchPutCreateService,
		{
			onSuccess: (data) => { notificationMsg('success', `Service is created Successfully`); },
			onError: (err) => { notificationMsg('error', `Failed to create the Service!!`); },
			...configOptions,
		}
	);
}

export const useDeleteWrapperService = (configOptions = {}) => {
	return useMutation(
		api.fetchDeleteService,
		{
			onSuccess: (data) => { notificationMsg('success', `Service is deleted Successfully`); },
			onError: (err) => { notificationMsg('error', `Failed to delete the Service!!`); },
			...configOptions,
		}
	);
}

export const usePutWrapperServiceParam = (configOptions = {}) => {
	return useMutation(
		api.fetchPutWrapperServiceParam,
		{
			onSuccess: async (data) => { notificationMsg('success', `Service Parameters saved Successfully`); },
			onError: async (err) => { notificationMsg('error', (err.response) ? err.response.data : 'Failed to save Service Parameters!!'); },
			...configOptions,
		}
	);
}
// ------ code ------
export const useGetWrapperCode = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['wrapper-op-logic', token, wrapperName],
		api.fetchGetSdkEditCode,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutSaveCode = (configOptions = {}) => {
	return useMutation(
		api.fetchPutSdkEditCode,
		{
			...configOptions,
		}
	);
}

// ------ low level ------
export const useGerLowLevelSchema = (token, configOptions = {}) => {
	return useQuery(
		['low-level-schema', token],
		api.fetchGetLowLevelSchema,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePostLowLevel = (configOptions = {}) => {
	return useMutation(
		api.fetchPostLowLevel,
		{
			...configOptions,
		}
	);
}

export const useGetLowLevel = (token, wrapperName, configOptions = {}) => {
	return useQuery(
		['low-level', token, wrapperName],
		api.fetchGetLowLevel,
		{
			...defaultOptions,
			enabled: !!token,
			...configOptions,
		}
	);
}

export const usePutLowLevel = (configOptions = {}) => {
	return useMutation(
		api.fetchPutLowLevel,
		{
			...configOptions,
		}
	);
}