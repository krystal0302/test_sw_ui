/*
 * Author: Angela Kao
 * Date: 3 May 2022
 * Description:
 **/

// ======================
//       Parameters 
// ======================
var $form = $('#reset-pwd-form');

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  initResetPasswordForm();
});

function initResetPasswordForm() {
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
      userPwd: {
        required: true,
        minlength: 4,
        chkPasswordStrength: true
      }
    },
    messages: {
      userPwd: {
        required: "Please enter password",
        minlength: "Your password must be at least 4 characters"
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
  $form.on('submit', submitResetPwdHandler);
}

function submitResetPwdHandler(e) {
  if (!$form.valid()) { return; }
  e.preventDefault();

  $.ajax({
    url: '/testdb/updatePassword',
    type: 'POST',
    data: $form.serialize(),
    success: function (data) {
      alert(data);
      window.location.href = "../login.html";
    },
    error: function (e) {
      alert(e.responseText);
    }
  });
}
