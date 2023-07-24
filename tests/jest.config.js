module.exports = {
  // Automatically clear mock calls and instances between every test
  clearMocks: true,

  // The directory where Jest should output its coverage files
  coverageDirectory: "coverage",

  // A preset that is used as a base for Jest's configuration
  preset: "jest-puppeteer",
  setupFilesAfterEnv: ['./jest.setup.js'],
};
