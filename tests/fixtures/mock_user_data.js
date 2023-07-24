// ==========================
//   FAKE USER DATA FOR TEST
// ==========================

const adminUserAccount = [
  {
    "id": "admintest",
    "password": "Asdf1&2&3",
    "name": "admin test user",
    "phone": "0221234567",
    "phone2": "0233333333",
    "email": "admintest@farobottech.com",
    "email2": "testadmin@farobottech.com"
  },
  {
    "id": "admintest2",
    "password": "Asdf1&2&32",
    "name": "admin test user 2",
    "phone": "0227654321",
    "phone2": "0266666666",
    "email": "admintest2@farobottech.com",
    "email2": "testadmin2@farobottech.com"
  }
];

// [Rule] at least one JSON object
const invalidUserAccount = [
  {
    "special_symbol_id": "$test",
    "space_id": "te st",
    "chinese_id": "測試",
    "fullwidth_id": "ｔｅｓｔ",
    "phone": "abc",
    "email": "abc"
  },
  {
    "special_symbol_id": "test#",
    "space_id": " test",
    "chinese_id": "第二組測試",
    "fullwidth_id": "１２３",
    "phone": "０２２１２３４５６７",
    "email": "123@farobottech。com"
  }
]

module.exports = {
  adminUserAccount,
  invalidUserAccount
}
