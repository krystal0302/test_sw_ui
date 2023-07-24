class ValidatorMananger {
  constructor() {
    this._rules = [];
  }
  addValidator(rule) {
    this._rules = [...this._rules, rule];
  }
  getValidator(name) {
    return this._rules.find((rule) => rule._name === name);
  }
  initValidateInput() {
    this._rules.forEach((element) => {
      const ele_id = `#${element["_name"]}`;

      $(ele_id).css('box-shadow', '');
      $(ele_id).css('border-color', '');
      $(ele_id).css('outline-color', '');
      $.powerTip.destroy(ele_id);
    });
  }
  checkAllRuleClear() {
    const check_dict = {};
    this._rules.forEach((element) => {
      if (element["_check_res"] != undefined) {
        check_dict[element["_name"]] = element["_check_res"]["bValid"];
      }
    });
    return check_dict;
  }
  makeNonClickableIfOneOfInputsFailed(interaction_element) {
    let validArr = [];
    this._rules.forEach((element) => {
      console.log(element);
      if (element._check_res !== undefined) {
        validArr.push(element._check_res.bValid);
      }
    });
    console.log(validArr);

    // const validator = this.getValidator(name);
    // if (!validator._interaction_element) { return; }
    // console.log(validator._interaction_element);

    const isEnable = validArr.every(v => v === true);
    interaction_element.prop('disabled', !isEnable);
  }
};

class Rule {
  constructor(name, handler, interaction_element) {
    this._name = name;
    this._handler = handler;
    this._interaction_element = interaction_element;
    this._check_res = undefined;
  }

  run(value, showInteractionLogicDefault = true) {
    let self = this;
    let testRes = this._handler(value);
    let validateRes = testRes.bValid;
    let validateMsg = testRes.strMsg;
    self.showPowerTip(validateMsg);
    self.showValidateInputResult(validateRes);
    if (showInteractionLogicDefault) {
      self.showInteractionElementLogic(validateRes);
    }
    this._check_res = testRes;
    return testRes;
  }

  showPowerTip(strMsg) {
    const element = $(document.getElementById(this._name));
    if (strMsg.length != 0 && strMsg[0].length != 0) {
      try {
        element.data('powertip', strMsg).powerTip();
        $.powerTip.show(element);
      } catch (error) {
        element.data('powertip', strMsg).powerTip();
        $.powerTip.show(element);
      }
    } else {
      $.powerTip.destroy(element);
    }
  }

  showValidateInputResult(bValid) {
    const element = $(document.getElementById(this._name));
    // console.log(element);
    element.css('box-shadow', (bValid) ? '' : '0 0 10px #CC0000');
    element.css('border-color', (bValid) ? '' : 'red');
    element.css('outline-color', (bValid) ? '' : 'red');
  }

  showInteractionElementLogic(bValid) {
    // console.log(bValid)
    // console.log(this._interaction_element)
    if (!this._interaction_element) { return; }
    this._interaction_element.prop('disabled', !bValid);
  }
}

// ------ generic validation rules ------
function textNameValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  const reg6 = /^[a-zA-Z0-9_]+$/;                         // must use numbers or A-Za-z or underscore (_)

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && reg6.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $ + - = ~ ` # %';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (reg5.test(inputVal)) ? '' : '- must end with number or A-Za-z';
  strMsg.push(msg5);
  const msg6 = (reg6.test(inputVal)) ? '' : '- only A-Za-z and number are allow';
  strMsg.push(msg6);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

function digitValueValidation(inputVal) {
  inputVal = inputVal.trim();
  const reg1 = /^-?([0]{1}\.{1}[0-9]+|[1-9]{1}[0-9]*\.{1}[0-9]+|[0-9]+|0)$/;                // (+|-)0-9 0.9
  const bRes = reg1.test(inputVal);
  let strMsg = "";
  strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters';

  return { bValid: bRes, strMsg: strMsg }
}

function numberValueValidation(inputVal) {
  inputVal = inputVal.trim();
  const reg1 = /^[+-]?[0-9.]+$/;           // (+|-)0-9
  const bRes = reg1.test(inputVal) && !isNaN(Number(inputVal));
  let strMsg = "";
  strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters';

  return { bValid: bRes, strMsg: strMsg }
}

function positivenumberValueValidation(inputVal) {
  const reg1 = /^[0-9.]+$/;                // (+|-)0-9
  const bRes = reg1.test(inputVal);
  let strMsg = "";
  strMsg += (reg1.test(inputVal)) ? strMsg : '- invalid number or includes invalid charaters';

  return { bValid: bRes, strMsg: strMsg }
}

function passwordValidation(inputVal) {
  const reg1 = /^(?=.*[0-9])(?=.*[!@#$%^&*])(?=.*[A-Z])[a-zA-Z0-9!@#$%^&*]{4,16}$/;     // contain only a-zA-Z0-9 and length is between 8 - 16,  at least a number, and at least a special character
  const reg2 = /\s/;                                      // space are not allow

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- password length should be 4 - 16 and need contain at least one number,<br /> one special character(ex: !@#$%^&*)<br /> and one capital character';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg2);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

function phoneValidation(inputVal) {
  const reg1 = /^([0-9\(\)\/\+ \-]{8,20})$/;     // phone format
  const reg2 = /\s/;                        // space are not allow

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- invalid phone format, please check you input format and length should be 8 - 20';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg2);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}

function emailValidation(inputVal) {
  const reg1 = /^[^\\/:\*\?"<>\|\$\+\-\=\`\~\#\%]+$/;     // forbidden characters \ / : * ? " < > | + - = ~ ` # %
  const reg2 = /^\./;                                     // cannot start with dot (.)
  const reg3 = /^(null|none|null[0-9]|none[0-9])(\.|$)/i; // forbidden names (null|none|null[0-9]|none[0-9])
  const reg4 = /\s/;                                      // space are not allow
  const reg5 = /.*[A-Za-z0-9]$/;                          // must end with number or A-Za-z
  const reg6 = /^(([^<>()\[\]\.,;:\s@\"]+(\.[^<>()\[\]\.,;:\s@\"]+)*)|(\".+\"))@(([^<>()\.,;\s@\"]+\.{0,1})+[^<>()\.,;:\s@\"]{2,})$/;

  const bRes = reg1.test(inputVal) && !reg2.test(inputVal) && !reg3.test(inputVal) && !reg4.test(inputVal) && reg5.test(inputVal) && reg6.test(inputVal);
  let strMsg = [];
  const msg1 = (reg1.test(inputVal)) ? '' : '- forbidden characters \ / : * ? " < > | $ + - = ~ ` # %';
  strMsg.push(msg1);
  const msg2 = (!reg2.test(inputVal)) ? '' : '- cannot start with dot (.)';
  strMsg.push(msg2);
  const msg3 = (!reg3.test(inputVal)) ? '' : '- forbidden names';
  strMsg.push(msg3);
  const msg4 = (!reg4.test(inputVal)) ? '' : '- cannot include space';
  strMsg.push(msg4);
  const msg5 = (reg5.test(inputVal)) ? '' : '- must end with number or A-Za-z';
  strMsg.push(msg5);
  const msg6 = (reg6.test(inputVal)) ? '' : '- must match email format';
  strMsg.push(msg6);
  strMsg = strMsg.filter(Boolean).join('<br />'); // line-break for title attirbute

  return { bValid: bRes, strMsg: strMsg }
}