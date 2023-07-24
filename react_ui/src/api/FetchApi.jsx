import axios from "axios";

// ====== configurations ======
const swarmHostName = `${window.location.hostname}`;
const swarmPort = '5000';
const swarmHost = `http://${swarmHostName}:${swarmPort}`;

// ====== authentication ======
export const fetchGetToken = async () => {
  let { data } = await axios({
    method: "POST",
    url: `${swarmHost}/login/access-token`,
    headers: {
      accept: "application/json",
      "Content-Type": "application/x-www-form-urlencoded",
    },
    data: "username=root&password=root@farobot",
  })
  return data;
}

// ====== fleets ======
export const fetchGetFleets = async ({ queryKey }) => {
  const [_, token] = queryKey;
  const { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/fleets`,
    headers: {
      accept: "application/json",
      'Authorization': `${token.token_type} ${token.access_token}`
    }
  })
  let transformedData = [];
  for (let d of Object.values(data)) {
    transformedData.push(d);
  }
  return transformedData;
}

// ====== flows ======
export const fetchGetFlows = async ({ queryKey }) => {
  const [_, token, selFleet] = queryKey;
  const { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/flows`,
    headers: {
      accept: "application/json",
      'Authorization': `${token.token_type} ${token.access_token}`
    }
  })
  console.log(data);
  return data;
}

export const fetchPostFlow = async ({ token, flowName, options }) => {
  let { data } = await axios({
    method: 'POST',
    url: `${swarmHost}/v2/flows/${flowName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{ "args": ${JSON.stringify(options.args)}}`
  })
  return data;
}

// ====== v1 artifacts ======
export const fetchPostArtifactConfigs = async ({ token, artifactId }) => {
  let { data } = await axios({
    method: 'POST',
    url: `${swarmHost}/settings/get_artifact_settings/`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{ "artifact_list": [ "${artifactId}" ] }`
  })
  return data;
}

export const fetchPutArtifactConfigs = async ({ token, artifactId, artifactConfig }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/settings/set_artifact_settings/`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{ "artifact_list": ["${artifactId}"], "artifact_settings": {"artifact_conf": "${artifactConfig}"}  }`
  })
  return data;
}

// ====== artifacts ======
export const fetchGetArtifacts = async ({ queryKey }) => {
  const [_, token] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/artifacts/scan?support_sdk=true&include_test=true`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}

export const fetchGetArtifactStatus = async ({ queryKey }) => {
  const [_, token, artifactId] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/artifacts/status/${artifactId}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}

export const fetchGetArtifactAvatarProperty = async ({ queryKey }) => {
  const [_, token, artifactId] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/artifacts/property?artifact_id=${artifactId}&mode=avatar_only`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}

export const fetchGetArtifactProperty = async ({ queryKey }) => {
  const [_, token, artifactId] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/artifacts/property?artifact_id=${artifactId}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}


// ====== SDK ======

// --- artifact-request (artifact services) ---
export const fetchGetArtifactService = async ({ queryKey }) => {
  const [_, token, artifactId] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-request/artifact_id=${artifactId}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}

export const fetchPostRunArtifactService = async ({ token, artifactId, serviceName, params = {} }) => {
  let { data } = await axios({
    method: 'POST',
    url: `${swarmHost}/v2/sdk/artifact-request/${artifactId}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{ 
      "service": "${serviceName}",
      "parameters": ${JSON.stringify(params)} 
    }`
  })
  return data;
}

export const fetchPutResumeArtifactService = async ({ token, artifactId, serviceName, actionType }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-request/${artifactId}?service=${serviceName}&action=${actionType}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
  })
  return data;
}

export const fetchDeleteCancelArtifactService = async ({ token, artifactId, serviceName }) => {
  let { data } = await axios({
    method: 'DELETE',
    url: `${swarmHost}/v2/sdk/artifact-request/${artifactId}?service=${serviceName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
  })
  return data;
}

// ------ runtime logs ------
export const fetchGetArtifactLogs = async ({ queryKey }) => {
  const [_, token, targetId] = queryKey;
  // console.log(targetId)
  const { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-runtime-log?artifact_id=${targetId}&line_amount=10&log_category=all`,
    headers: {
      accept: "application/json",
      'Authorization': `${token.token_type} ${token.access_token}`
    }
  })
  return data.runtime_log;
}

// ------ runtime logs package ------
export const fetchGetArtifactLogsPackage = async ({ queryKey }) => {
  const [_, token, targetId] = queryKey;
  const { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-runtime-log/package?artifact_id=${targetId}`,
    headers: {
      accept: "application/json",
      'Authorization': `${token.token_type} ${token.access_token}`
    },
    responseType: 'blob'
  })
  return data;
}

// ------ wrappers ------
export const fetchGetWrappers = async ({ queryKey }) => {
  const [_, token] = queryKey;
  const { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper`,
    headers: {
      accept: "application/json",
      'Authorization': `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

export const fetchPostCreateWrapper = async ({ token, wrapper }) => {
  const { data } = await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper?wrapper_name=${wrapper.name}&wrapper_type=${wrapper.type}&category=${wrapper.category}`,
    method: "POST",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  return data;
}

export const fetchDeleteWrapper = async ({ token, wrapperName }) => {
  const { data } = await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper?wrapper_name=${wrapperName}`,
    method: "DELETE",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  return data;
}

// ------ wrapper types ------
export const fetchGetWrapperTypes = async ({ queryKey }) => {
  const [_, token] = queryKey;
  const { data } = await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper/type-templates`,
    method: "GET",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  // return data.map(d => d.type)
  return data.map(d => {
    return { type: d.type, category: d.category }
  })
}

// ------ wrapper categories ------
export const fetchGetWrapperCategories = async ({ queryKey }) => {
  const [_, token] = queryKey;
  const { data } = await axios({
    url: `http://${window.location.hostname}:5000/v2/sdk/artifact-wrapper/category`,
    method: "GET",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  return data.map(d => d.category);
}

// ------ wrapper package ------
export const fetchGetWrapperPackage = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `http://${window.location.hostname}:5000/v2/sdk/artifact-wrapper/package?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    },
    responseType: 'blob'
  })
  // console.log(res);
  return data;
}

// ------ build ------
export const fetchGetWrapperBuild = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/build?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    },
  })
  // console.log(res);
  return data;
}

export const fetchPutWrapperBuild = async ({ token, wrapperName }) => {
  console.log(token);
  console.log(wrapperName);
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/build?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  console.log(data);
  return data;
}

// ------ deploy ------
export const fetchGetWrapperDeployCandidates = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  const { data } = await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper/deploy-candidate?wrapper_name=${wrapperName}`,
    method: "GET",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  // console.log(data)
  return data;
}

export const fetchPutDeployWrapper = async ({ token, wrapperName, artifacts }) => {
  return await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper/deployment?wrapper_name=${wrapperName}`,
    method: "PUT",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: JSON.stringify({ "artifact_list": artifacts })
  })
}

// ------ edit (triee-view) ------
export const fetchGetSdkEditTreeView = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

// ------ live info ------
export const fetchGetSdkEditLiveInfo = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/live-info?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

// ------ configuration ------
export const fetchGetWrapperConfigParam = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/config?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

export const fetchPutWrapperConfigParam = async ({ uri, token, wrapperName, params }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/${uri}?wrapper_name=${wrapperName}&action=update`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: JSON.stringify(params)
  })
  return data;
}

// ------ wrapper data ------
export const fetchGetWrapperData = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/data?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

// ------ picture ------
export const fetchGetWrapperPicture = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/picture?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
  })
  return data;
}

export const fetchPutWrapperPicture = async ({ token, wrapperName, imgBase64, imgType }) => {
  console.log(imgType);
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/picture?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{"${imgType}": "${imgBase64}"}`
  })
  return data;
}

// ------ service (services in a wrapper) ------
export const fetchGetWrapperServices = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/service?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

export const fetchPutCreateService = async ({ token, wrapperName, serviceName }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/service?wrapper_name=${wrapperName}&action=add&service_name=${serviceName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);	
  return data;
}

export const fetchDeleteService = async ({ token, wrapperName, serviceName }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/service?wrapper_name=${wrapperName}&action=remove&service_name=${serviceName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);	
  return data;
}

export const fetchPutWrapperServiceParam = async ({ token, wrapperName, serviceName, params }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/service/param?wrapper_name=${wrapperName}&service_name=${serviceName}&action=update`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: JSON.stringify(params)
  })
  return data;
}

// ------ code ------
export const fetchGetSdkEditCode = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/code?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  // console.log(res);
  return data;
}

export const fetchPutSdkEditCode = async ({ token, wrapperName, codeSnippet }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/code?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: `{ "content": ${codeSnippet} }`
  })
  return data;
}

// ====== low level ======
export const fetchGetLowLevelSchema = async ({ queryKey }) => {
  const [_, token] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/low-level/schema`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

export const fetchPostLowLevel = async ({ token, wrapperName, protocolName }) => {
  const { data } = await axios({
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/low-level?wrapper_name=${wrapperName}&low_level_protocol=${protocolName}`,
    method: "POST",
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
    },
  })
  return data;
}

export const fetchGetLowLevel = async ({ queryKey }) => {
  const [_, token, wrapperName] = queryKey;
  let { data } = await axios({
    method: 'GET',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/low-level?wrapper_name=${wrapperName}`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`
    }
  })
  return data;
}

export const fetchPutLowLevel = async ({ token, wrapperName, protocolName, params }) => {
  let { data } = await axios({
    method: 'PUT',
    url: `${swarmHost}/v2/sdk/artifact-wrapper/edit/low-level?wrapper_name=${wrapperName}&low_level_protocol=${protocolName}&enable=true`,
    headers: {
      accept: "application/json",
      Authorization: `${token.token_type} ${token.access_token}`,
      "Content-Type": "application/json"
    },
    data: JSON.stringify(params)
  })
  return data;
}