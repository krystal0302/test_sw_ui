import { Selector } from 'testcafe';
import { getTestSetting } from '../util';
import { Login } from '../models/LoginModel';
import { Role } from '../models/RoleModel';

// --- Run test here ---
const login = new Login("./deploy_test_settings.json");
const role = new Role("./deploy_test_settings.json");

const page = "role";
const general_setting = getTestSetting(page, "./deploy_test_settings.json");
const home_url = general_setting["homeUrl"];

const role_page_url = `${home_url}/${page}.html`;

// const userDefault = Role(home_url, async t =>{
//     await login.login()
// }, { preserveUrl: true })

// fixture`Role Editor Role-Edit Feature`
// 	.page(home_url)
//     .beforeEach(async t =>{
//         await login.login()
//     });  // specify the entry page

// test("Duplicate Role Name", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();
    
//     await role.addRoleWithDuplicateName();
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Special Character", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();    

//     for (var i = 0; i < role.special_character_data.length; i++) {
//         await role.addRoleWithSpecialCharacter(role.special_character_data[i]);
//     }
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Add Role", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();    

//         for (const [idx, test_obj] of role.test_data.entries()) {
//             let role_name = test_obj["text_val"];
//             let setting_copy = Object.assign({}, test_obj);
// 			delete setting_copy["text_val"];

//             try {
//                 await role.addRole(role_name, setting_copy);
//                 await role.checkRole(role_name, setting_copy);
//             } catch (error) {
//                 console.log(error)
//                 console.log(`Error happen, so delete it`);
//                 await t
//                     .navigateTo(role_page_url)
//                     .wait(5000)
//                     .expect(role.btnCreateNewRole.exists).ok(); 
//                 await role.deleteRole(role_name);   
//             }            
//         }
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Delete Role", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();    

//         for (const [idx, test_obj] of role.test_data.entries()) {
//             let role_name = test_obj["text_val"];

//             await role.deleteRole(role_name);           
//         }
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Rename Duplicate Role Name", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();
    
//     let default_exsit = await role.btnEditRole(role.defaut_role).exists;
//     let default2_exsit = await role.btnEditRole(role.defaut_role_rename).exists;
//     let click_element = undefined;
    
//     if (default_exsit) {
//         click_element = role.btnEditRole(role.defaut_role);
//     }else if (default2_exsit) {
//         click_element = role.btnEditRole(role.defaut_role_rename);
//     }

//     await t
//         .click(click_element);
    
//     await role.renameRoleWithDuplicateName();
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Rename Special Character", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();

//     let default_exsit = await role.btnEditRole(role.defaut_role).exists;
//     let default2_exsit = await role.btnEditRole(role.defaut_role_rename).exists;
//     let click_element = undefined;
    
//     if (default_exsit) {
//         click_element = role.btnEditRole(role.defaut_role);
//     }else if (default2_exsit) {
//         click_element = role.btnEditRole(role.defaut_role_rename);
//     }

//     await t
//         .click(click_element);

//     for (var i = 0; i < role.special_character_data.length; i++) {
//         await role.renameRoleWithSpecialCharacter(role.special_character_data[i]);
//     }
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Rename Flow Nmae", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();    

//     let default_exsit = await role.btnEditRole(role.defaut_role).exists;
//     let default2_exsit = await role.btnEditRole(role.defaut_role_rename).exists;
//     let click_element = undefined;
    
//     if (default_exsit) {
//         click_element = role.btnEditRole(role.defaut_role);
//     }else if (default2_exsit) {
//         click_element = role.btnEditRole(role.defaut_role_rename);
//     }

//     await t
//         .click(click_element);

//     await role.renameRole();
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });

// test("Edit Flow Nmae", async t => {
//     await t
//         .navigateTo(role_page_url)
//         .wait(5000)
//         .expect(role.btnCreateNewRole.exists).ok();    

//     let default_exsit = await role.btnEditRole(role.defaut_role).exists;
//     let default2_exsit = await role.btnEditRole(role.defaut_role_rename).exists;
//     let click_role = undefined;
    
//     if (default_exsit) {
//         click_role = role.defaut_role;
//     }else if (default2_exsit) {
//         click_role = role.defaut_role_rename;
//     }

//     await role.editRoleBehavior(click_role);
//     await role.removeRoleBehavior(click_role);
//     await role.adjustRoleBehaviorOrder(click_role);
// })
// .timeouts({
//     pageLoadTimeout:    2000,
//     pageRequestTimeout: 60000,
//     ajaxRequestTimeout: 60000,
// });


