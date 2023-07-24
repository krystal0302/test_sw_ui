const { setCellSizeByType } = require('../../server/public/dist/js/pages/far_map_util')
const { cellTypes, cellCollection } = require('../fixtures/mock_map_data')

describe('Cell update size by its types', () => {
	test('[Cell] update cell size by type', () => {
		setCellSizeByType(cellCollection, cellTypes);
		// expect(cellCollection).toEqual();
		var actualCell_1 = cellCollection.area1.find(c => c.cell_id === "cell_1");
		var expectCell_1 = { "type": "QRcode", "width": 1, "length": 1 };
		expect(actualCell_1).toMatchObject(expectCell_1);

		var actualCell_2 = cellCollection.area1.find(c => c.cell_id === "cell_2");
		var expectCell_2 = { "type": "RackDetection", "width": 2, "length": 6 };
		expect(actualCell_2).toMatchObject(expectCell_2);

		var actualCell_3 = cellCollection.area1.find(c => c.cell_id === "cell_3");
		var expectCell_3 = { "type": "ChargerDetection", "width": 2, "length": 2 };
		expect(actualCell_3).toMatchObject(expectCell_3);

		var actualCell_4 = cellCollection.area1.find(c => c.cell_id === "cell_4");
		var expectCell_4 = { "type": "Position", "width": 1, "length": 1 };
		expect(actualCell_4).toMatchObject(expectCell_4);

	})

})