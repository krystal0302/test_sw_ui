/*
 * Author: none 
 * Date: 20 Sep. 22,
 * Description:
 **/

// ======================
//    Global Variables 
// ======================
let licenseStepper;

// ======================
//       Load Ready 
// ======================
$(function () {
  'use strict'
  initRundownAsync();
});

async function initRundownAsync() {
  try {
    // ------ get login status ------
    var statusData = await restLoginStatus();
    getLoginStatus(statusData, 'settings', 'settings_lic.html');

    // --- fetch RMT token initially --- 
    try {
      rmtToken_ = await fetchToken();
    } catch (err) {
      console.error(err);
    }

  } catch (err) {
    console.error(err);
  }

  // 7. license check and hardware signature
  // await sysLicenseCheck();
  // await sysHardwareSignature();
  await updateLicenseInfo();
  await initLicenseStepper();
}


async function sysLicenseCheck() {
  let validateState = await fetchGetLicenseValidation(rmtToken_);
  console.log(validateState);
  let licenseState = { valid: 1 };
  if (!validateState.hasOwnProperty('license_check_result')) { licenseState.valid = 0; };

  var resultObj = validateState.license_check_result;
  for (let pkg in resultObj) {
    if (resultObj[pkg].includes("OK")) { continue; }
    licenseState.valid = 0;
  }
  console.log(licenseState);
  if (licenseState.valid === 0) {
    $('#lic-expiry-date').text('No License');
    notificationMsg(3, 'Invalid License!');
  }
  if (licenseState.valid === 1) {
    let expiry;
    try {
      expiry = await restGetLicExpiryDate();
      console.log(expiry);
    } catch (e) {
      console.error(e);
    }

    // --- [protection] exceptional case ---
    if (expiry === undefined || expiry.hasOwnProperty('error')) {
      $('#lic-expiry-date').text('No License');
      notificationMsg(2, 'Can NOT get expiry date!');
      return;
    }

    // --- success case ---
    $('#lic-expiry-date').text(`Expired by ${expiry.date}`);
    notificationMsg(1, 'Swarm Service Authorized!');
  }
}

async function sysHardwareSignature() {
  // [API] get hardware signature
  let res = await restGetHardwareSignature();
  // console.log(res);

  // [ERROR HANDLING] get no hardware signature
  let hwSig = (res?.hwSig) ? res.hwSig : "No hardware signature!";

  $('#hw-sig').text(hwSig);
}

$('#upload-conf-modal').on('hidden.bs.modal', function (e) {
  $("#conf-file-input").val('');
});


// =============================================
//     License Upload and Signature Download 
// =============================================
// ------ license callbacks ------
// function showUploadZone() {
//   $('#drop-area').toggle();
// }

// async function removeLicense() {
//   console.log('remove license');
//   var res = await restDeleteLicense();
//   console.log(res);
//   if (res === 'OK') {
//     $('#lic-expiry-date').text('No License');
//     notificationMsg(1, 'License is deactivated!');
//   } else {
//     notificationMsg(3, 'Fail to deactivate license!');
//   }

//   // await sysLicenseCheck();
// }

// async function exportSignature() {
//   window.location = `/signature`;
// }

// // --- upload file by XHR object ---
// const url = "/upload-license";
// const form = document.querySelector('#upload-license');

// form.addEventListener('submit', async (e) => {
//   // disable default action
//   e.preventDefault();

//   // collect files
//   const files = document.querySelector('[name=fileElem]').files;
//   const formData = new FormData();
//   formData.append('license', files[0]);
//   console.log(files);


//   // ======== updated way ========
//   const test = new Promise((resolve, reject) => {
//     const xhr = new XMLHttpRequest()
//     const url = '/upload-license'
//     xhr.open('POST', url, true)
//     xhr.onload = () => {
//       if (xhr.status >= 200 && xhr.status < 400) {
//         // console.table(JSON.parse(xhr.response))
//         resolve(JSON.parse(xhr.response))
//       } else {
//         reject(new Error(`Conn. Fail: ${xhr.status}`))
//       }
//     }
//     xhr.send(formData)
//   })

//   test.then((result) => {
//     notificationMsg(1, 'License Uploaded!');
//     console.table(result)
//   }).catch((err) => {
//     notificationMsg(3, 'Fail to Upload License!');
//     console.error(err)
//   })

//   // console.log(res);
//   await sleep(1000);
//   await sysLicenseCheck();
// });

// ==================================
//     Wibu License Authorization 
// ==================================

async function updateLicenseInfo() {
  let data = await fetchGetLicenseContainerInfo(rmtToken_);
  const statusMsg = data.retCode === 0 ? 'authorized' : 'unauthorized';
  const boxMask = data.boxMask || '';
  const serialNo = data.serialNo || '';
  document.getElementById('lic-status').textContent = statusMsg;
  document.getElementById('container-id').textContent = `${boxMask}-${serialNo}`;
  document.getElementById('lic-type').textContent = data.firmCode;

  data = await fetchGetLicenseInfo(rmtToken_);
  const expDateString = data.expirationTime;
  let expDate = new Date(expDateString);
  expDate = moment(expDate).format("YYYY-MM-DD hh:mm:ss");
  if (isNaN(expDate)) return;
  document.getElementById('lic-exp-date').textContent = expDate;
}

async function initLicenseStepper() {
  licenseStepper = new Stepper(document.querySelector('#license-stepper'));

  if (window.innerWidth > PHONE_MAX_WIDTH) {
    document.getElementById('step1Label').textContent = "Step 1: Footprint exporting";
    document.getElementById('step2Label').textContent = "Step 2: License activation";
    document.getElementById('step3Label').textContent = "Step 3: Receipt downloading";
  }

  const uploadLicForm = document.querySelector('#upload-license-form');
  uploadLicForm.addEventListener('submit', async (e) => {

    e.preventDefault();

    const files = document.querySelector('[name=license-file]').files;
    const formData = new FormData();
    formData.append('license', files[0]);

    let res = await fetchPostLicense(rmtToken_, formData);
    console.log(res);

    if (res.retCode === 0) {
      nextStep();
    } else {
      notificationMsg(3, res.message);
    }
  });

  if (!isMobile()) return;

  const licenseFileInput = document.querySelector('#license-file');
  licenseFileInput.addEventListener('change', (e) => {
    const files = e.target.files;
    if (!files.length) return;
    document.getElementById('license-label').textContent = files[0].name;
  });
}

function exportFootprint() {
  genFootprintReceiptLink('footprint');
}

function downloadReceipt() {
  genFootprintReceiptLink('receipt');
}

async function genFootprintReceiptLink(_type) {
  let data = await fetchGetLicenseFootprint(rmtToken_);
  // console.log(_type);
  // console.log(data);
  if (data.retCode === 0) {
    const context = data.buffer;
    const blob = new Blob([context], { type: 'application/octet-stream' });
    const blobUrl = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = blobUrl;
    a.download = 'context.WiBuRaC';
    document.getElementById(`${_type}-link`).appendChild(a);
    a.click();
    URL.revokeObjectURL(blobUrl);

    const isFootprint = _type === 'footprint';
    notificationMsg(1, `${_type.charAt(0).toUpperCase() + _type.slice(1)} is ${isFootprint ? 'exported' : 'downloaded'}!`);
    if (isFootprint) {
      nextStep();
    }

  } else {
    notificationMsg(3, `Failed to export ${_type}!`);
  }
}

function nextStep() {
  setTimeout(function () {
    licenseStepper.next();
  }, 1000);
}

function prevStep() {
  licenseStepper.previous();
}