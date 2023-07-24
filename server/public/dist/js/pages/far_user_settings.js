/*
 * Author: Angela Kao
 * Date: 7 Oct 2021,
 * Description:
 **/

// ======================
//       Parameters 
// ======================
var $form = $('#update-user-form');
var $chg_pwd_form = $('#change-pwd-form');
var modifiedPwd = {
  name: "modifiedPwd",
  value: ""
};

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  initDataAsync();
});

async function initDataAsync() {
  try {
    // ------ register user activity detector ------
    userActivityDetector();
    
    // ------ get login status ------
    var statusData = await restLoginStatus();
    getLoginStatus(statusData, 'settings', 'user_settings.html');

    var currUserData = await restGetCurrentUserData();
    var jsonData = JSON.parse(currUserData);
    // var obj = $.parseJSON(jsonData);
    // console.log(jsonData[0].user_id);
    parseUserCardInfo(jsonData[0]);

    initUpdateUserForm();

  } catch (err) {
    console.error(err);
  }
}

function parseUserCardInfo(_userInfo) {
  $('#profile-username').text(_userInfo.user_name);
  $('#user-account').val(_userInfo.user_id);
  $('#user-password').val('••••••••');
  $('#user-phone').val(_userInfo.user_phoneNum);
  $('#user-email').val(_userInfo.user_email);
}

function initUpdateUserForm() {
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

  $.validator.addMethod("chkConfirmPassword", function (value, element) {
    let password = value;
    if (password === $('#account-pwd').val()) {
      return true;
    }
    return false;
  });

  $form.validate({
    rules: {
      phoneNum: {
        required: true,
        minlength: 8,
        regex: "[0-9\-\(\)\s]+"
      },
      email: {
        required: true,
        email: true
      }
    },
    messages: {
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
      element.closest('.input-group').append(error);
    },
    highlight: function (element, errorClass, validClass) {
      $(element).addClass('is-invalid');
    },
    unhighlight: function (element, errorClass, validClass) {
      $(element).removeClass('is-invalid');
    }
  });
  $form.on('submit', submitHandler);

  $chg_pwd_form.validate({
    rules: {
      accountPwd: {
        required: true,
        minlength: 4,
        chkPasswordStrength: true
      },
      accountPwd2: {
        required: true,
        chkConfirmPassword: true
      }
    },
    messages: {
      accountPwd: {
        required: "Please enter password",
        minlength: "Your password must be at least 4 characters"
      },
      accountPwd2: {
        required: "Please enter confirm password",
        chkConfirmPassword: "Confirm password not match with the password"
      }
    },
    errorElement: 'span',
    errorPlacement: function (error, element) {
      error.addClass('invalid-feedback');
      element.closest('.input-group').append(error);
    },
    highlight: function (element, errorClass, validClass) {
      $(element).addClass('is-invalid');
    },
    unhighlight: function (element, errorClass, validClass) {
      $(element).removeClass('is-invalid');
    }
  });
  $chg_pwd_form.on('submit', submitChgPwdHandler);
}

function submitHandler(e) {
  if (!$form.valid()) { return; }
  
  e.preventDefault();

  var formArray = $form.serializeArray();
  // console.log(formArray);
  formArray.push(modifiedPwd);
  console.log(formArray);

  $.ajax({
    url: '/testdb/updateCurrentUserInfo',
    type: 'POST',
    data: formArray, //$form.serialize(),
    success: function (data) {
      toast(data);
    },
    error: function (e) {
      alert(e.responseText);
    }
  });
}

function submitChgPwdHandler(e) {
  if (!$chg_pwd_form.valid()) { return; }
  e.preventDefault();
  modifiedPwd.value = $('#account-pwd').val();
  $('#change-pwd-modal').modal('hide');
}

$('.switch-edit-mode').click(editButtonSwitch);
$('.switch-password-mode').click(pwdButtonSwitch);

function editButtonSwitch() {
  var readonly = $(this).prev('input').prop('readonly');
  $(this).prev('input').prop('readonly', !readonly);
  $(this).children().children('i').toggleClass('fa-pen');
  $(this).children().children('i').toggleClass('fa-eye');
}

function pwdButtonSwitch() {
  // var readonly = $(this).prev('input').prop('readonly');
  // $(this).prev('input').prop('readonly', !readonly);
  $(this).children().children('i').toggleClass("fa-eye fa-eye-slash");
  if ($(this).prev('input').attr('type') === 'text') {
    $(this).prev('input').attr('type', 'password');
  } else {
    $(this).prev('input').attr('type', 'text');
  }
}