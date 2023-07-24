/*
 * Author: Angela Kao
 * Date: 4 Oct 2021
 * Description:
 **/

// ======================
//       Parameters 
// ======================
var $form = $('#create-user-form');
var selectedRows = [];
var userInfo = [];
var loggedInUserID = "";

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  initDataAsync();
});

async function initDataAsync() {
  // ------ register user activity detector ------
  userActivityDetector();

  // ------ get login status ------
  var statusData = await restLoginStatus();
  await getLoginStatus(statusData, 'userManagement');
  loggedInUserID = statusData.userID;

  // ------ init user information grid ------
  userInfo = await getUserInfo();
  genUserInfoGrid();

  // ------ init create user form ------
  await initUserForm();

  // ------ language switch ------
  await langAttachIdentifiers();
  await initLanguageSupport();

  // ------ binding events ------
  setTimeout(function () {
    showToastIfNeeded('deleteClicked', 'Users data are deleted!');
    showToastIfNeeded('saveClicked', 'Users data are saved!');
  }, 300);
  
  validateCreateUserInputEvent();
}

async function langAttachIdentifiers() {
  $('tr.jsgrid-header-row > th:contains("Edit")').attr('data-i18n', 'usr_man.col_Edit');
  $('tr.jsgrid-header-row > th:contains("User Name")').attr('data-i18n', 'usr_man.col_UserName');
  $('tr.jsgrid-header-row > th:contains("User Account")').attr('data-i18n', 'usr_man.col_UserAccount');
  $('tr.jsgrid-header-row > th:contains("Admin")').attr('data-i18n', 'usr_man.col_Admin');
  $('tr.jsgrid-header-row > th:contains("Phone Number")').attr('data-i18n', 'usr_man.col_PhoneNumber');
  $('tr.jsgrid-header-row > th:contains("E-mail")').attr('data-i18n', 'usr_man.col_Email');
  $('tr.jsgrid-header-row > th:contains("Member Since")').attr('data-i18n', 'usr_man.col_Membership');
  $('tr.jsgrid-header-row > th:contains("Options")').attr('data-i18n', 'usr_man.col_Options');
  
}

function getUserInfo() {
  return $.ajax({
    url: "/testdb/getUserInfo",
    dataType: "json"
  });
}

function genUserInfoGrid() {
  $("#user-jsGrid").jsGrid({
    width: "100%",
    height: "auto",
    sorting: true,
    autoload: true,
    paging: true,
    pageSize: 15,
    pageButtonCount: 10,
    pageIndex: 1,
    deleteConfirm: "Are you sure you want to delete this user?",
    noDataContent: "No user data found",

    controller: {
      loadData: getUserInfo,
      deleteItem: function (item) {
        var jsonString = JSON.stringify(item);
        var jsonObj = JSON.parse(jsonString);
        console.log(jsonObj.user_id);
        return restDeleteUserData([jsonObj.user_id]);
      },
    },

    fields: [
      {
        type: "checkbox", title: "Edit", sorting: false,
        itemTemplate: function (value, item) {
          return $("<input>").addClass('row_check').attr("type", "checkbox")
            .attr("checked", value || item.Checked)
            .attr("disabled", item.user_id == loggedInUserID)
            .on("change", function () {
              item.Checked = $(this).is(":checked");
              // save rowData to update user db data used
              var rowData = {
                ori_is_admin: _.find(userInfo, { user_id: item.user_id }).isAdmin || 'unknown',
                user_role: item.isAdmin === 'Y' ? "admin" : "general",
                user_id: item.user_id
              }
              var tempArray = selectedRows.filter(function (row) { return row.user_id !== item.user_id; });
              if (item.Checked) {
                tempArray.push(rowData);
              }
              selectedRows = tempArray;
              // console.log(selectedRows);

              var targetAdminSw = $("input.admin_sw." + item.user_id)[0];
              if (targetAdminSw) {
                // make admin_sw configurable only when this row is checked
                targetAdminSw.disabled = (item.Checked ? false : true);
              }
            });
        }
      },
      { name: "user_name", type: "text", title: "User Name", width: 200, align: "center" },
      { name: "user_id", type: "text", title: "User Account", width: 200, align: "center" },
      // { name: "isAdmin", type: "text", title: "isAdmin", align: "center" },
      {
        type: "text", title: "Admin", align: "center",
        itemTemplate: function (value, item) {
          var $input = $("<input>").addClass('admin_sw' + ' ' + item.user_id).attr("type", "checkbox")
            .attr("checked", item.isAdmin === 'Y')
            .attr("disabled", true)
            .on("change", function () {
              item.isAdmin = $(this).is(":checked") ? "Y" : "N";
              // update selectedRows array
              if (typeof item.Checked !== 'undefined' && item.Checked == true) {
                var idx = selectedRows.findIndex((row => row.user_id === item.user_id));
                selectedRows[idx].user_role = item.isAdmin === 'Y' ? "admin" : "general";
              }
            });
          var $span = $("<span class='slider round'>");
          return $("<label class='switch'>").append($input).append($span);
          // var switchElement =`<label class="switch">`;
          // if (item.isAdmin === 'Y') {
          //   switchElement += `  <input class="admin_sw" type="checkbox" checked>`;
          // } else {
          //   switchElement += `  <input class="admin_sw" type="checkbox">`;
          // }
          // switchElement += `      <span class="slider round"></span>
          //                     </label>`;
          // // console.log(jsGrid.fields);
          // return switchElement;
        }
      },
      { name: "user_phoneNum", type: "number", title: "Phone Number", width: 200, align: "center" },
      { name: "user_email", type: "text", title: "E-mail", width: 250, align: "center" },
      { name: "user_createdDate", type: "text", title: "Member Since", width: 130, align: "center" },
      {
        type: "control", editButton: false, width: 90,
        itemTemplate: function (value, item) {
          var $result = $([]);
          if (item.user_id != loggedInUserID) {
            $result = $result.add(this._createDeleteButton(item));
          }
          return $result;
        },
        headerTemplate: function () {
          return "Options";
        }
      }
    ]
  });
}

async function initUserForm() {
  $.validator.addMethod("regex", function (value, element, param) {
    return value.match(new RegExp("^" + param + "$"));
  });


  $.validator.addMethod("chkPasswordStrength", function (value, element) {
    let password = value;
    if (!(/^(?=.*[a-z])(?=.*[A-Z])(?=.*[0-9])(?=.*[~!@#$%^&*;:,.<>\[\]{}()\\|\-_])/.test(password))) {
      return false;
    }
    return true;
  }, function (value, element) {
    let password = $(element).val();
    if (!(/^(?=.*[A-Z])/.test(password))) {
      return 'Password must contain at least one uppercase';
    }
    else if (!(/^(?=.*[a-z])/.test(password))) {
      return 'Password must contain at least one lowercase';
    }
    else if (!(/^(?=.*[0-9])/.test(password))) {
      return 'Password must contain at least one number';
    }
    else if (!(/^(?=.*[~!@#$%^&*;:,.<>\[\]{}()\\|\-_])/.test(password))) {
      return "Password must contain at least one special character from ~!@#$%^&*;:,.<>[]{}()|-_";
    }
    return false;
  });

  $form.validate({
    rules: {
      accountID: {
        required: true,
        chkInputVal: true
      },
      accountPwd: {
        required: true,
        minlength: 4,
        chkPasswordStrength: true
      },
      userName: {
        required: true
      },
      phoneNum: {
        required: true,
        minlength: 8,
        regex: "^([0-9\(\)\/\+ \-]*)$"
      },
      email: {
        required: true,
        email: true
      }
    },
    messages: {
      accountID: {
        required: "Please enter account",
        chkInputVal: "Account include invalid characters"
      },
      accountPwd: {
        required: "Please enter password",
        minlength: "Your password must be at least 4 characters"
      },
      userName: {
        required: "Please enter name"
      },
      phoneNum: {
        required: "Please enter contact number",
        minlength: "Phone number must be at least 8 numbers",
        regex: "Invalid contact number format"
      },
      email: {
        required: "Please enter email",
        email: "Invalid email address"
      }
    },
    errorElement: 'span',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.closest('.form-group').append(error);
    },
    highlight: function (element, errorClass, validClass) {
      $(element).addClass('is-invalid');
    },
    unhighlight: function (element, errorClass, validClass) {
      $(element).removeClass('is-invalid');
    }
  });
  $form.on('submit', submitHandler);
}

function submitHandler(e) {
  if (!$form.valid()) { return; }

  e.preventDefault();

  $.ajax({
    url: '/testdb/addUserData',
    type: 'POST',
    data: $form.serialize(),
    success: function (data) {
      alert(data);
      window.location.href = "manage_user.html";
    },
    error: function (e) {
      alert(e.responseText);
    }
  });
}

function showToastIfNeeded(itemType, toastMsg) {
  if (localStorage.getItem(itemType)) {
    localStorage.removeItem(itemType);
    notificationMsg(0, toastMsg);
    // toast(toastMsg);
  }
}

$(document).on('change', '.row_check', function (e) {
  var checkedCount = $('.row_check:checkbox:checked').length;
  $('#delete-users-btn').prop("disabled", checkedCount == 0);
  $('#save-users-btn').prop("disabled", checkedCount == 0);
});

function saveUsers() {
  // var data = $("#user-jsGrid").jsGrid("option", "data");
  // console.log(data);
  if (selectedRows.length > 0) {
    var saveUsers = selectedRows.filter(row => row.user_id != loggedInUserID);
    saveUsers = _.map(saveUsers, user => _.omit(user, ['ori_is_admin']));
    restPostUsersRoleData(saveUsers);
    localStorage.setItem("saveClicked", true);
    window.location.reload();
  }
}

function deleteUsers() {
  if (selectedRows.length > 0) {
    var deleteUsers = selectedRows.filter(row => row.user_id != loggedInUserID).map(row => row.user_id);
    restDeleteUserData(deleteUsers);
    localStorage.setItem("deleteClicked", true);
    window.location.reload();
  }
}

// --- add input values validator ---
function validateCreateUserInputEvent() {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#create-user-btn');

  const idRule = new Rule('account-id', textNameValidation, interaction_element);
  const passwaordRule = new Rule('account-pwd', passwordValidation, interaction_element);
  const userNameRule = new Rule('user-name', textNameValidation, interaction_element);
  const phoneNameRule = new Rule('phoneNum', phoneValidation, interaction_element);
  const emailRule = new Rule('mail-address', emailValidation, interaction_element);

  validatorManager.addValidator(idRule);
  validatorManager.addValidator(passwaordRule);
  validatorManager.addValidator(userNameRule);
  validatorManager.addValidator(phoneNameRule);
  validatorManager.addValidator(emailRule);
  // console.log(role_validatorManager);

  /*
  // --- [CONFIG] 2. Define Validation Flow ---
  //                  * getValidator(): get the corresponding validator, 
  //                  * run()         : run the valiations, 
  //                  * do the styling and interaction logics
  **/
  function validationFlow(vm) {
    const validator = vm.getValidator(this.id);
    if (validator == undefined) { return; }

    const res = validator.run(this.value, false);
    let checkAllRes = vm.checkAllRuleClear();
    console.log(checkAllRes)

    // --- [STYLING] reflect validation result ---

    // --- [UX] Interaction Logics ---
    let err_still = []
    for (const [key, value] of Object.entries(checkAllRes)) {
      err_still.push(value);
    }
    if (err_still.includes(false)){
      interaction_element.prop('disabled', true);
    }else{
      interaction_element.prop('disabled', false);
    }        
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('create-user-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}