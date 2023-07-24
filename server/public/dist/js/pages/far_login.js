/*
 * Author: Angela Kao
 * Date: 23 Sep 2021
 * Description:
 **/

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'

  eulaStatusCheck();

  setLoginStatus(JSON.stringify({ "userID": '', 'loginStatus': false }));
  initLoginData();
  validateLoginInputEvent();
});

async function rmtTokenCheck() {
  if (rmtToken_ === undefined) {
    try {
      rmtToken_ = await fetchToken();
    } catch (err) {
      await sleep(5000);
      rmtTokenCheck();
    }
  }
}

async function licenseValidationCheck() {
  await rmtTokenCheck();
  let valid = false;
  let data = await fetchGetLicenseContainerInfo(rmtToken_);
  if (!data.hasOwnProperty('retCode')) { return false; }
  valid = (data.retCode === 0);

  data = await fetchGetLicenseInfo(rmtToken_);
  if (!data.hasOwnProperty('expirationTime')) { return false; }
  const expDate = new Date(data.expirationTime);
  valid = (new Date().getTime() <= expDate.getTime());

  return valid;
  // let validateState = await fetchGetLicenseValidation(rmtToken_);
  // console.log(validateState);
  // let licenseState = { valid: 1 };
  // if (!validateState.hasOwnProperty('license_check_result')) { licenseState.valid = 0; };

  // var resultObj = validateState.license_check_result;
  // for (let pkg in resultObj) {
  //   if (resultObj[pkg].includes("OK")) { continue; }
  //   licenseState.valid = 0;
  // }
  // console.log(licenseState);

  // return (licenseState.valid) ? true : false;
}

$('body').keyup(function (event) {
  if (event.keyCode === 13) {
    $("#user-login-btn").click();
  }
});

$('#user-login-btn').on('click', async function (event) {
  event.preventDefault();
  var user_id = $('#user-account').val();
  var user_pwd = $('#user-pwd').val();
  chkInputValStatus(user_id, "user-account");
  chkInputValStatus(user_pwd, "user-pwd");

  // --- empty account and/or password input case ---
  // error message
  var errorMsg = "Please fill in ";
  if (isEmptyString(user_id) || isEmptyString(user_pwd)) {
    if (isEmptyString(user_id) && isEmptyString(user_pwd)) {
      errorMsg += "account number and password."
    } else {
      errorMsg += (isEmptyString(user_id) ? "account number." : "");
      errorMsg += (isEmptyString(user_pwd) ? "password." : "");
    }
    showErrorInfo(errorMsg);
    return;
  } else {
    $('#error-info').css('display', 'none');
    // userLogin();
  }

  // --- license authorization check case ---
  let res = await licenseValidationCheck();
  console.log(res);
  if (false) {
    // if (!res && user_id === 'admin') {
    await setUIlocalStorage();
    // === redirect to validate license modal ===
    window.location.href = "settings_lic.html";

    // === show error message modal ===
    // // --- get hardware signature ---
    // let res = await restGetHardwareSignature();
    // // console.log(res);
    // // [ERROR HANDLING] get no hardware signature
    // let hwSig = (res?.hwSig) ? res.hwSig : "No hardware signature!";
    // $('#hw-sig').text(`HW SIG.: ${hwSig}`);

    // $('#license-hint-msgs').html("Invalid License!");
    // $('#error-license').css('display', '');
    return;
  }
  // } else {
  //   $('#error-info').css('display', 'none');
  //   $('#error-license').css('display', 'none');
  // }

  if (false) {
    // if (!res && user_id !== 'amdin') {
    showErrorInfo('Please use account with admin privilege to activate license');
    return;
  }

  console.log('--- start to login ---');
  $('#error-info').css('display', 'none');
  $('#error-license').css('display', 'none');
  setLoginStatus(JSON.stringify({ "userID": 'startLogin', 'loginStatus': false }));
  userLogin();
});

function chkInputValStatus(value, inputID) {
  if (isEmptyString(value)) {
    $('input[id=' + inputID + ']').addClass('is-invalid');
  } else {
    $('input[id=' + inputID + ']').removeClass('is-invalid');
  }
}

function showErrorInfo(msg) {
  $('#error-msg').html(msg);
  $('#error-info').css('display', '');
}

function userLogin() {
  $.ajax({
    url: '/testdb/login',
    type: 'POST',
    data: {
      userID: $('#user-account').val(),
      pwd: $('#user-pwd').val()
    },
    success: async function (data) {
      if ($('#remember-me-col').css("visibility") === "hidden") {
        localStorage.checkbox = true;
        console.log(`memorized user account: ${localStorage.userAccount}`);
      } else {
        // save user account if remember me is checked
        if ($('#remember-me').is(":checked")) {
          localStorage.userAccount = $('#user-account').val();
          localStorage.checkbox = $('#remember-me').val();
        } else {
          localStorage.userAccount = "";
          localStorage.checkbox = "";
        }
      }
      var statusData = await restLoginStatus();
      console.log(data)
      console.log(statusData)
      setLoginStatus(JSON.stringify({ "userID": statusData.userID, 'loginStatus': true }));
      await setUIlocalStorage();
      window.location.href = "index.html";
      // notice of changing the default password
      // if (data === 'Y') {
      //   window.location.href = "index.html";
      // } else {
      //   // alert(`You haven't changed your password before.\nPlease go to user settings page to change it.`);
      //   window.location.href = "settings2.html?tab=1";
      // }
    },
    error: function (e) {
      showErrorInfo(e.responseText);
    },
  });
}

async function initLoginData() {
  // console.log(localStorage.userAccount);
  if (localStorage.userAccount) {
    $('#user-account').val(localStorage.userAccount);
    $('#remember-me-col').css('visibility', 'hidden');
  }

  const isLogin = await checkLoginStatus();
  if (isLogin) {
    window.location.href = "index.html";
    return;
  }
  $('#remember-me-col').css('visibility', 'visible');
  if (localStorage.checkbox && localStorage.checkbox !== "") {
    $('#remember-me').attr('checked', true);
    $('#user-account').val(localStorage.userAccount);
  } else {
    $('#remember-me').removeAttr('checked');
    $('#user-account').val('');
  }

  // for (var key in localStorage) {
  //   console.log(key, ' = ', localStorage[key]);
  // }
}

async function checkLoginStatus() {
  const statusData = await restLoginStatus();
  if (statusData.hasOwnProperty('greetingMsg')) {
    setLoginStatus(JSON.stringify({ "userID": statusData.userID, 'loginStatus': true }));
  } else {
    setLoginStatus(JSON.stringify({ "userID": '', 'loginStatus': false }));
  }

  return statusData.hasOwnProperty('greetingMsg');
}


// --- upload file by XHR object ---
const url = "/upload-license";
// const form = document.querySelector('#upload-license');
const form = document.querySelector('#upload-lic');
console.log(form);

// form.addEventListener('submit', async (e) => {
form.addEventListener('click', async (e) => {
  // disable default action
  e.preventDefault();

  // collect files
  const files = document.querySelector('[name=fileElem]').files;
  const formData = new FormData();
  formData.append('license', files[0]);
  console.log(files);

  // POST form data
  const xhr = new XMLHttpRequest();

  // log response
  xhr.onload = async () => {
    console.log(xhr.responseText);
    res = JSON.parse(xhr.responseText);
    console.log(res);
    if (res.status) {
      // notificationMsg(1, 'License Uploaded!');

      // ---license check ---
      // await sysLicenseCheck();
      // await licenseValidationCheck();
      await sleep(1000);
      location.reload(true);
    }
  }

  // create and send the request
  xhr.open('POST', url);
  xhr.send(formData);
  // location.reload();

});

// --- add input values validator ---
function validateLoginInputEvent(rule_list = []) {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#user-login-btn');

  const useraccRule = new Rule('user-account', textNameValidation, interaction_element);
  // const pwsRule = new Rule('user-pwd', passwordValidation);


  validatorManager.addValidator(useraccRule);
  // validatorManager.addValidator(pwsRule);


  rule_list.forEach(element_rule => {
    validatorManager.addValidator(element_rule);
  });
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

    const res = validator.run(this.value);

    // --- [STYLING] reflect validation result ---

    // --- [UX] Interaction Logics ---
  }

  /*
  // --- [CONFIG] 3. Conduct Event-Bindings ---
  //                 * bind the validation procecedure to the target elements
  **/
  const targets = document.getElementsByClassName('login-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}

async function eulaStatusCheck() {
  await rmtTokenCheck();
  let slaData = await fetchGetSLAConfirmation(rmtToken_);
  if (!slaData.hasOwnProperty('confirmed')) return;
  if (!slaData.confirmed) {
    document.getElementById('eula-content').innerHTML = eulaHtmlContent;
    $('#eula-modal').modal('show');
  }
}

async function agreeEula() {
  await rmtTokenCheck();
  let res = await fetchPostSLAConfirmation(rmtToken_);
  if (res.ok) {
    $('#eula-modal').modal('hide');
  }
}

function disagreeEula() {
  alert('You can NOT access Login page until confirm the agreement!');
}