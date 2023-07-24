import { Selector, t, ClientFunction } from 'testcafe';
import { getTestSetting } from '../util';

export class Role {
    constructor (setting_path='') {
        this.defaut_role = 'test';
        this.defaut_role_rename = 'test_rename';
        this.settings = getTestSetting('role', setting_path);
        this.test_data = this.settings["test_data"];
        this.special_character_data = this.settings["special_character"];
        this.element_timeout = this.settings["element_wait_second"] * 1000;

        this.moveMenu = Selector('#bt-toolbar').find('li').nth(0);
        this.logicMenu = Selector('#bt-toolbar').find('li').nth(1);
        this.othersMenu = Selector('#bt-toolbar').find('li').nth(2);

        this.move = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(0);
        this.charge = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(1);
        this.docking = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(2);
        this.rotate = Selector('#bt-toolbar').find('li').nth(0).find('div > a').nth(3);        
        this.wait = Selector('#bt-toolbar').find('li').nth(1).find('div > a').nth(0);        
        this.artifact = Selector('#bt-toolbar').find('li').nth(2).find('div > a').nth(0);

        // this.movelistItem = Selector('[data-title="Nav2Client"]');
        // this.chargelistItem = Selector('[data-title="Charge"]');
        // this.dockinglistItem = Selector('[data-title="Docking"]');
        // this.rotatelistItem = Selector('[data-title="Rotate"]');
        // this.waitlistItem = Selector('[data-title="WaitAction"]');
        // this.artifactlistItem = Selector('[data-title="Artifact"]');

        this.behaviorSpan = Selector('.dd3-content > span');
        this.divTrashBehavior = Selector('.item-trash');

        this.optionDockingTypes = Selector('[data-type="docking_types"]');
        this.optionArtifacTypes = Selector('[data-type="artifact_types"]');
        this.optionArtifactServices = Selector('[data-type="artifact_services"]');

        this.inputRole = Selector('#role-filename');
        this.btnCreateNewRole = Selector('[data-target="#new-role-modal"]', { timeout: this.element_timeout });
        this.btnConfirmCreate = Selector('#confirm-create-role');
        this.btnCancelCreate = Selector('button').withText('Cancel');
        this.btnSave = Selector('#save-role-btn');
        this.btnBack = Selector('#back-role-btn');

        this.btnEditRoleName = Selector('#sb-role-edit');
        this.inputEditingRoleName = Selector('input#editing-role-name');
        this.spanEditingRoleName = Selector('span#editing-role-name');

        this.notificationTitle_Warning = Selector('.notification-title', { timeout: this.element_timeout }).withText(/Warning/);
        this.notificationTitle_Error = Selector('.notification-title', { timeout: this.element_timeout }).withText(/Error/);
        this.notificationTitle_Success = Selector('.notification-title', { timeout: this.element_timeout }).withText(/Success/);
        this.notificationTitle_Info = Selector('.notification-title', { timeout: this.element_timeout }).withText(/Info/);

        this.docking_element_index = 0;
        this.artifact_element_index = 0;
    }

    set setDefaultRoleElement(role_name){
        this.defaut_role = role_name;
    }

    arraymove(arr, fromIndex, toIndex) {
        var element = arr[fromIndex];
        arr.splice(fromIndex, 1);
        arr.splice(toIndex, 0, element);
        return arr
    }

    divRolebar(role_name) {
        return Selector(`#rolebar-${role_name}`);
    }

    btnEditRole(role_name) {
        return Selector(`#${role_name}-edit-btn`);
    }

    btnRemoveRole(role_name) {
        return Selector(`#${role_name}-remove-btn`);
    }

    divBehaviorDrag(idx) {
        return Selector(`.dd3-handle`).nth(idx);
    }

    async addRoleWithDuplicateName(role_name="test") {
        console.log(`Check Duplicate Role Name. : ${role_name}`)
        await t
            .click(this.btnCreateNewRole)
            .typeText(this.inputRole, role_name)
            .click(this.btnConfirmCreate)
            .wait(1000)
            .expect(this.notificationTitle_Error.exists).ok()
            .wait(1000);
    }

    async addRoleWithSpecialCharacter(role_name="test") {
        console.log(`Check Special Character Role Name. : ${role_name}`)
        await t
            .click(this.btnCreateNewRole)
            .typeText(this.inputRole, role_name)
            .click(this.btnConfirmCreate)
            .wait(1000)
            .expect(this.notificationTitle_Warning.exists).ok()
            .wait(1000)
            .click(this.btnCancelCreate);
    }

    async addRole(role_name="", role_setting_obj={"move_menu": ["move"]}) {
        console.log(`Add Role Name. : ${role_name}`)
        
        await t
            .click(this.btnCreateNewRole)
            .typeText(this.inputRole, role_name)
            .click(this.btnConfirmCreate)
            .wait(1000)
            .scrollIntoView(this.divRolebar(role_name))
            .expect(this.divRolebar(role_name).exists).ok();
				
        await t
            .click(this.btnEditRole(role_name))
            .wait(1000)
        
        // === create role ===
        await this.chooseBehaviorSelector(role_setting_obj, "add");

        await t
            .setNativeDialogHandler(() => true)
            .click(this.btnSave)
            .wait(2000)
            .click(this.btnBack)
            .wait(1000);
        
        this.docking_element_index = 0;
        this.artifact_element_index = 0;

        // check role
        await t
            .scrollIntoView(this.btnEditRole(role_name))
            .expect(this.btnEditRole(role_name).exists).ok();
    }

    async deleteRole(role_name="") {
        console.log(`Remove Role Name. : ${role_name}`)

        let isRoleExist = await this.btnRemoveRole(role_name).exists;
        
        console.log(isRoleExist)

        if (isRoleExist){
            await t
                .setNativeDialogHandler(() => true)
                .click(this.btnRemoveRole(role_name))
                .wait(1000)
                .expect(this.divRolebar(role_name).exists).notOk();
        }
    }

    async checkRole(role_name="", role_setting_obj={"move_menu": ["move"]}) {
        console.log(`Check Role Name. : ${role_name}`)
        				
        await t
            .expect(this.divRolebar(role_name).exists).ok()
            .click(this.btnEditRole(role_name))
            .wait(1000);
        
        // === create role ===
        await this.chooseBehaviorSelector(role_setting_obj, "check");

        await t
            .click(this.btnBack)
            .wait(1000);
        
        this.docking_element_index = 0;
        this.artifact_element_index = 0;
    }

    async renameRoleWithDuplicateName(role_name="role1") {
        console.log(`Check Duplicate Rename Role Name. : ${role_name}`)

        await t
            .click(this.btnEditRoleName)
            .typeText(this.inputEditingRoleName, role_name, { replace: true })
            .click(this.btnSave)
            .wait(1000)
            .expect(this.notificationTitle_Error.exists).ok()
            .wait(1000);
    }

    async renameRoleWithSpecialCharacter(role_name="test") {
        console.log(`Check Special Character Rename Role Name. : ${role_name}`)
        
        let pen_exist = await this.btnEditRoleName.child(".fa-pen").exists;

        if (pen_exist){
            await t
                .click(this.btnEditRoleName)
        }

        await t            
            .typeText(this.inputEditingRoleName, role_name, { replace: true })
            .click(this.btnSave)
            .wait(1000)
            .expect(this.notificationTitle_Error.exists).ok()
            .wait(1000);
    }

    async renameRole(rename_role_name=this.defaut_role_rename) {        
        let current_role_name = await this.spanEditingRoleName.innerText;
        let rename_name = rename_role_name;

        if (rename_role_name == this.defaut_role_rename && current_role_name == this.defaut_role){
            rename_name = this.defaut_role_rename;
        }else if(rename_role_name == this.defaut_role_rename && current_role_name == this.defaut_role_rename){
            rename_name = this.defaut_role;
        }

        console.log(`Rename Role Name. Rename to : ${rename_name}`)
        				
        await t
            .expect(this.spanEditingRoleName.exists).ok()
            .click(this.btnEditRoleName)
            .expect(this.inputEditingRoleName.exists).ok()
            .typeText(this.inputEditingRoleName, rename_name, { replace: true })
            .wait(1000)
            .click(this.btnSave)
            .wait(1000)
            .expect(this.notificationTitle_Success.exists).ok()
            .expect(this.spanEditingRoleName.innerText).eql(rename_name);

        await t
            .click(this.btnBack)
            .wait(1000)
            .hover(this.btnEditRole(rename_name))
            .expect(this.btnEditRole(rename_name).exists).ok();
    }

    async editRoleBehavior(role_name=this.defaut_role, role_setting_obj={"combination": ["docking@docking", "wait", "artifact@lift_module@lift"]}) {        
        console.log(`Add Behavior To Role Name. : ${role_name}`)
        
        await t
            .click(this.btnEditRole(role_name))
            .wait(1000)
        
        // === edit behavior ===
        await this.chooseBehaviorSelector(role_setting_obj, "add");

        await t
            .setNativeDialogHandler(() => true)
            .click(this.btnSave)
            .wait(2000)
            .click(this.btnBack)
            .wait(1000);
        
        this.docking_element_index = 0;
        this.artifact_element_index = 0;

        // check role
        await t
            .expect(this.btnEditRole(role_name).exists).ok();
    }

    async removeRoleBehavior(role_name=this.defaut_role, role_setting_obj={"combination": ["docking@docking", "wait", "artifact@lift_module@lift"]}) {        
        console.log(`Remove Behavior Role Name. : ${role_name}`)
        
        await t
            .click(this.btnEditRole(role_name))
            .wait(1000)
        
        // === create role ===
        await this.chooseBehaviorSelector(role_setting_obj, "remove");

        await t
            .setNativeDialogHandler(() => true)
            .click(this.btnSave)
            .wait(2000)
            .click(this.btnBack)
            .wait(1000);
        
        this.docking_element_index = 0;
        this.artifact_element_index = 0;

        // check role
        await t
            .expect(this.btnEditRole(role_name).exists).ok();
    }

    async adjustRoleBehaviorOrder(role_name=this.defaut_role, role_order_obj=[{'from': 1, 'to': 2}, {'from': 3, 'to': 1}]) {        
        console.log(`Remove Behavior Role Name. : ${role_name}`)
        
        await t
            .click(this.btnEditRole(role_name))
            .wait(1000);

        let total_behavior_list = [];
        let behavior_count = await this.behaviorSpan.count;
        
        for (var i = 0; i < behavior_count; i++) {
            let behavior = await this.behaviorSpan.nth(i).innerText;
            total_behavior_list.push(behavior);
        }
        console.log('Before adjust index')
        console.log(total_behavior_list)

        for (var i = 0; i < role_order_obj.length; i++) {
            let order_list = role_order_obj[i];
            let from = order_list['from'] - 1;
            let to = order_list['to'] - 1;
            let y_offset = 0;

            if (from > to){
                y_offset = -30;
            }else{
                y_offset = 30;
            }

            total_behavior_list = this.arraymove(total_behavior_list, from, to)

            await t
                .hover(this.divBehaviorDrag(from))
                .dragToElement(this.divBehaviorDrag(from), this.divBehaviorDrag(to), {destinationOffsetY: y_offset, speed: 0.5})
                .wait(3000);            
        }

        console.log('After adjust index')
        console.log(total_behavior_list)

        let after_behavior_count = await this.behaviorSpan.count;
        let after_behavior_list = [];
        for (var i = 0; i < after_behavior_count; i++) {
            let behavior = await this.behaviorSpan.nth(i).innerText;
            after_behavior_list.push(behavior);
        }
        const getAfterAdjustElementList = ClientFunction((after_li) => {return after_li});
        const after_behavior = await getAfterAdjustElementList(after_behavior_list);

        await t
            .expect(after_behavior).eql(total_behavior_list);

        await t
            .setNativeDialogHandler(() => true)
            .click(this.btnSave)
            .wait(2000)
            .click(this.btnBack)
            .wait(1000);

        // check role
        await t
            .expect(this.btnEditRole(role_name).exists).ok();
    }

    async chooseBehaviorSelector(behavior_setting_dict, mode) {
        for (const [menu, behavior_list] of Object.entries(behavior_setting_dict)) {
            let menu_selector = undefined;

            switch (menu) {
                case "move_menu":
                    menu_selector = this.moveMenu;
                    break;
                case "logic_menu":
                    menu_selector = this.logicMenu;
                    break;
                case "others_menu":
                    menu_selector = this.othersMenu;
                    break; 
                case "combination":
                    break;
                default:
                    break;
            }

            for (const [idx, behavior_str] of behavior_list.entries()) {						
                let behavior_selector = undefined;
                let check_behavior_selector = undefined;

                let select_docking_type = false;
                let select_artifact_type = false;
                
                let behavior = undefined;
                console.log(behavior_str)
                if (behavior_str.includes('@')){
                    behavior = behavior_str.split('@')[0];
                }else{
                    behavior = behavior_str;
                }

                switch (behavior) {
                    case "move":
                        menu_selector = this.moveMenu;
                        behavior_selector = this.move;
                        // check_behavior_selector = this.movelistItem;
                        break;
                    case "charge":
                        menu_selector = this.moveMenu;
                        behavior_selector = this.charge;
                        // check_behavior_selector = this.chargelistItem;
                        break;
                    case "docking":
                        menu_selector = this.moveMenu;
                        behavior_selector = this.docking;
                        // check_behavior_selector = this.dockinglistItem;
                        select_docking_type = true;                        
                        break;
                    case "rotate":
                        menu_selector = this.moveMenu;
                        behavior_selector = this.rotate;
                        // check_behavior_selector = this.rotatelistItem;
                        break;
                    case "wait":
                        menu_selector = this.logicMenu;
                        behavior_selector = this.wait;
                        // check_behavior_selector = this.waitlistItem;
                        break;
                    case "artifact":
                        menu_selector = this.othersMenu;
                        behavior_selector = this.artifact;
                        // check_behavior_selector = this.artifactlistItem;
                        select_artifact_type = true;
                        break;
                
                    default:
                        behavior_selector = this.move;
                        check_behavior_selector = this.movelistItem;
                        break;
                }
                if (mode == 'add'){
                    await t
                        .click(menu_selector)
                        .click(behavior_selector);
                    if (select_docking_type){
                        let docking_type = behavior_str.split('@')[1];
                        let docking_type_element = Selector(this.optionDockingTypes.nth(this.docking_element_index));
                        await t                            
                            .click(docking_type_element)
                            .click(docking_type_element.find('option').withText(docking_type));
                        
                        this.docking_element_index += 1;
                    }else if(select_artifact_type){
                        let artifact_type = behavior_str.split('@')[1];
                        let artifact_service = behavior_str.split('@')[2];

                        let artifact_type_element = Selector(this.optionArtifacTypes.nth(this.artifact_element_index));
                        let artifact_service_element = Selector(this.optionArtifactServices.nth(this.artifact_element_index));

                        await t
                            .click(artifact_type_element)
                            .click(artifact_type_element.find('option').withText(artifact_type))
                            .click(artifact_service_element)
                            .click(artifact_service_element.find('option').withText(artifact_service));

                        this.artifact_element_index += 1
                    }
                }else if (mode == 'check'){
                    let check_behavior_selector = this.behaviorSpan.nth(idx);
                    let behaviorSpanText = String(await Selector(check_behavior_selector).innerText).toLowerCase();
                    
                    await t
                        .hover(check_behavior_selector)
						.expect(behaviorSpanText).eql(behavior.toLowerCase())
                    if (select_docking_type){
                        let docking_type = behavior_str.split('@')[1];
                        let docking_type_element = this.optionDockingTypes.nth(this.docking_element_index);
                        await t                            
                            .expect(docking_type_element.value).eql(docking_type);
                        
                        this.docking_element_index += 1;
                    }else if(select_artifact_type){
                        let artifact_type = behavior_str.split('@')[1];
                        let artifact_service = behavior_str.split('@')[2];

                        let artifact_type_element = this.optionArtifacTypes.nth(this.artifact_element_index);
                        let artifact_service_element = this.optionArtifactServices.nth(this.artifact_element_index);

                        await t
                            .hover(artifact_type_element)
                            .expect(artifact_type_element.value).eql(artifact_type)
                            .hover(artifact_service_element)
                            .expect(artifact_service_element.value).eql(artifact_service);

                        this.artifact_element_index += 1
                    }
                }else if (mode == 'remove'){
                    let remove_selector = this.behaviorSpan.withText(behavior.charAt(0).toUpperCase() + behavior.slice(1)).parent().child('.item-trash');

                    await t
                        .hover(remove_selector)
                        .click(remove_selector)
                        .expect(remove_selector.exists).notOk();
                }
            }
        }
    }
}