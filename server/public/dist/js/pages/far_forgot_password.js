/*
 * Author: Angela Kao
 * Date: 28 Apr 2022
 * Description:
 **/

var init_info_msg = "";
// ======================
//       Load Ready 
// ======================

$(function () {
  'use strict'
  validateforgotPWDInputEvent();
});

var btnsDiv = $('#send-action-btns').detach();
$('#search-email').on('click', function (event) {
  event.preventDefault();
  var user_id = $('#user-account').val();
  chkInputValStatus(user_id, "user-account");

  // error message
  if (isEmptyString(user_id)) {
    var errorMsg = "Please fill in account number.";
    showErrorInfo(errorMsg);
  } else {
    $('#error-info').css('display', 'none');
    searchUserEmail();
  }
});

$('#btns-div').on('click', '#send-email', function (event) {
  event.preventDefault();
  var user_id = $('#user-account').val();
  var user_email = $('#user-email').val();
  sendResetPasswordEmail(user_id, user_email);
});

$('#btns-div').on('click', '#cancel-send', showSearchAccountView);

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

function showSendEmailView(_inputText) {
  init_info_msg = $('#info-msg').text();
  $('#info-msg').text('We found the account email address below. Send an email to reset your password?');
  $('#user-account-input').hide();
  $('#user-email-input').show();
  $('#user-email').val(_inputText);
  $('#btns-div').append(btnsDiv);
  btnsDiv = $('#search-btn').detach();
}

function showSearchAccountView() {
  $('#info-msg').text(init_info_msg);
  $('#user-account-input').show();
  $('#user-email-input').hide();
  $('#btns-div').append(btnsDiv);
  btnsDiv = $('#send-action-btns').detach();
}

function searchUserEmail() {
  $.ajax({
    url: '/testdb/getUserEmail',
    type: 'POST',
    data: {
      userID: $('#user-account').val()
    },
    success: function (data) {
      if (isEmptyString(data)) {
        showErrorInfo('Email not found.');
      } else {
        // console.log(data);
        showSendEmailView(data);
      }
    },
    error: function (e) {
      console.log(e.responseText);
      showErrorInfo(e.responseText);
    },
  });
}

function sendResetPasswordEmail(_userAccount, _userEmail) {
  $.ajax({
    url: '/testdb/sendResetPasswordMail',
    type: 'POST',
    data: {
      userAccount: _userAccount,
      userMail: _userEmail
    },
    success: function (data) {
      alert(data);
    },
    error: function (e) {
      alert(e.responseText);
    },
  });
}

// --- add input values validator ---
function validateforgotPWDInputEvent(rule_list=[]) {
  /*
  // --- [CONFIG] 1. Setup Validation Configuration       ------
  //                  * create validatorManager and Rules 
  //                  * add rules into validatorManager   -
  **/
  const validatorManager = new ValidatorMananger();
  const interaction_element = $('#search-email');
  const useraccRule = new Rule('user-account', textNameValidation, interaction_element);
  

  validatorManager.addValidator(useraccRule);
  

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
  const targets = document.getElementsByClassName('forgot-pwd-input');
  for (let el of targets) {
    el.addEventListener("keyup", validationFlow.bind(el, validatorManager));
  }
}