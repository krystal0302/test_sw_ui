const handleShowSideBarCard = (nodeID, behaviorType, collapesBtnText, behaviorIndex) => {
    const SideBarCardTmplate = document.getElementById('SidebarCard');
    let node = document.importNode(SideBarCardTmplate.content, true);
    let span = node.querySelector('.behaviorIndex');
    let behaviorVlidateSpan = node.querySelector('.behaviorDataValidate');
    let collapesBtn = node.querySelector('#behaviorCollapseBtn');
    let collapesAnchor = node.querySelector('#behaviorCollapseAnchor');
    let collapesContent = node.querySelector('#behaviorCollapseContent');

    span.textContent = adjustNumberDisplayText(behaviorIndex);

    collapesBtn.id = `${behaviorType}Btn_${behaviorIndex}`;
    collapesBtn.textContent = `${collapesBtnText}`;
    collapesBtn.dataset.target = `#${behaviorType}_${behaviorIndex}`;

    collapesAnchor.id = `${behaviorType}Anchor_${behaviorIndex}`;
    collapesAnchor.href = `#${behaviorType}_${behaviorIndex}`;
    collapesAnchor.dataset.target = `#${behaviorType}_${behaviorIndex}`;

    collapesContent.id = `${behaviorType}_${behaviorIndex}`;
    // accordion type collapse
    // collapesContent.dataset.parent = `#${sidebarID}`;

    let contentElement = undefined;
    const dataIdx = behaviorIndex - 1;
    const haveSavedData = haveSaveCachePageData(nodeID);
    const behaviorSavedVal = haveSavedData? flowPageData.currentSavedData[nodeID].behavior[dataIdx].title_content: undefined;
    let saveData = undefined;

    if (haveSavedData) {
        setBehaviorDataVlidate(collapesContent, 'hide');
    }

    if (behaviorType === "DockingArtifact") {
        behaviorType = "Artifact"
    }

    switch (behaviorType) {
        case "Nav2Client":
            saveData = haveSavedData? Object.values(behaviorSavedVal)[0]: undefined;
            contentElement = handleGoalTypeParam(saveData);

            if (haveSavedData && isNotCompleteSaveData(saveData)) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;
        case "Dock":
            behaviorVlidateSpan.style.display = 'none';
            break;
        case "Undock":
            behaviorVlidateSpan.style.display = 'none'
            break;
        case "Rotate":
            saveData = haveSavedData? rotateDataProcess(behaviorSavedVal): undefined;
            contentElement = handleRotateTypeParam(saveData, dataIdx);

            if (haveSavedData && isNotCompleteSaveData(saveData.data)) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;
        case "Charge":
            saveData = haveSavedData? chargeDataProcess(behaviorSavedVal): undefined;
            contentElement = handleChargeTypeParam(saveData);

            if (haveSavedData && (isNotCompleteSaveData(saveData.goal) || isNotCompleteSaveData(saveData.percentage))) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;
        case "Artifact":
            const templateData = getArtifactTemplateData(nodeID, dataIdx);
            saveData = haveSavedData? behaviorSavedVal: undefined;
            contentElement = handleArtifactTypeParam(saveData, templateData, dataIdx);

            if (haveSavedData && isNotCompleteSaveData(Object.values(saveData)[0]) && isNotCompleteSaveData(Object.values(saveData)[1])) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;
        case "WaitAction":
            saveData = haveSavedData? Object.values(behaviorSavedVal)[0]: undefined;
            contentElement = handleWaitActionTypeParam(saveData);

            if (haveSavedData && isNotCompleteSaveData(saveData)) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;

        case "Report":
            saveData = haveSavedData? Object.values(behaviorSavedVal)[0]: undefined;
            contentElement = handleReportTypeParam(saveData);

            if (haveSavedData && isNotCompleteSaveData(saveData)) {
                setBehaviorDataVlidate(collapesContent, 'show');
            }
            break;

        default:
            break;
    }

    if (contentElement !== undefined) {
        let cardBody = document.createElement('div');
        cardBody.classList = 'card-body';
        cardBody.appendChild(contentElement);
        const replaceTarget = node.querySelector('.card-body');
        collapesContent.replaceChild(cardBody, replaceTarget);
    }

    return node;
}

const getArtifactTemplateData = (nodeID, behaviorIdx) => {
    // Get relate sc artifact template
    let selectedRole = document.getElementById(nodeID).querySelector('.normal-node-text').dataset.rolename;
    let behaviorsInRole = flowPageData.currentFleetSetting.roles[selectedRole];
    const artifactBehaviorData = behaviorsInRole[behaviorIdx];
    const artifactType = artifactBehaviorData.type;
    const artifactService = artifactBehaviorData.service;
    let targetData = _.cloneDeep(flowPageData.currentFleetSetting.artifacts[artifactType]);
    let templateData = {};

    let artifact_info = {
        artifactType: artifactType,
        artifactService: artifactService
    }

    templateData.artifactInfo = artifact_info;

    if (targetData !== undefined) {
        // have artifact in fleet.yaml
        templateData.data = targetData;
    } else {
        // no artifact in fleet.yaml
        const targetArtifactTemplate = _.cloneDeep(flowPageData.currentFleetSetting.artifactsTemplates[artifactType]);
        const artifactData = {
            availableIDs: [],
            params: targetArtifactTemplate
        }

        templateData.data = artifactData;
    }

    return templateData
}