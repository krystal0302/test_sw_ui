/*
 * Author: John Wu
 * Date: 28 Oct 21,
 * Description:
 **/

// =======================
//     Utility Modules
// =======================
/**
 * function to fill cell size by its type 
 * @param {object} _cells - The cells collection in JSON, update in-place
 * @param {object} _types - The types collection in JSON
 * @returns {} - none 
 */
function setCellSizeByType(_cells, _types) {
  console.log(_cells);
  console.log(_types);
  for (key in _cells) {
    _cells[key].forEach(c => {
      c.width = Number(_types[c.type].width);
      c.length = Number(_types[c.type].length);
    })
  }
}

let cellTypeAdaptor = {
  toUiCellType: function (_cellTypeObj, _cell) {
    // console.log(_cellTypeObj);
    // console.log(_cell);
    var targetArr = _cellTypeObj.filter(cto => cto.detectionType === _cell.detectionType);
    // console.log(targetArr);
    targetArr = targetArr.filter(ta => Number(ta.width) === Number(_cell.width));
    // console.log(targetArr);
    var target = targetArr.find(ta => Number(ta.length) === Number(_cell.length));
    // console.log(target);
    // var typeName = (target === undefined) ? (_cell.detectionType + _cell.width + _cell.length) : target.name
    return target;
  },
  toDetectionType: function (_cellTypeObj, _uiType) {
    var target = _cellTypeObj.find(cto => cto.name === _uiType);
    return target === undefined ? null : target.detectionType;
  },
  toUiCellType2: function (_funcTypeObj, _cell) {
    // console.log(_funcTypeObj);
    // console.log(_cell);
    // var funcTypes = _.flatten(Object.values(_funcTypeObj));
    // var targetArr = funcTypes.filter(cto => cto.detectionType === _cell.detectionType);
    // // console.log(targetArr);
    // targetArr = targetArr.filter(ta => Number(ta.width) === Number(_cell.width));
    // // console.log(targetArr);
    // var target = targetArr.find(ta => Number(ta.length) === Number(_cell.length));
    // // console.log(target);
    // // var typeName = (target === undefined) ? (_cell.detectionType + _cell.width + _cell.length) : target.name
    // return target;
  },
  toDetectionType2: function (_funcTypeObj, _uiType) {
    // console.log(_funcTypeObj);
    // console.log(_.flatten(Object.values(_funcTypeObj)));
    // var target = _.flatten(Object.values(_funcTypeObj)).find(ft=>ft.type===_uiType);
    const target = _funcTypeObj.find(ft => ft.type === _uiType);
    // var target = _cellTypeObj.find(cto => cto.name === _uiType);
    // return target === undefined ? null : target.detectionType;
    return target === undefined ? null : target.type;
  }

};

const defaultLoad = 'empty';
let cellLoadAdaptor = {
  toUiCellLoad: function (_cellLoad) {
    if (_cellLoad === 'rack') {
      return 'occupied';
    }
    return _cellLoad || defaultLoad;
  },
  toFileCellLoad: function (_cellLoad) {
    if (_cellLoad === 'occupied') {
      return 'rack';
    }
    return _cellLoad || defaultLoad;
  }
};

function updateCellsType(_cellCache, _cellTypes) {
  // console.log(_cellCache);
  console.log(_cellTypes);
  for (key in _cellCache) {
    console.log(_cellCache[key]);
    _cellCache[key].forEach(c => {
      console.log(c);
      console.log(`_cells > type: ${c.type}, width: ${c.width}, length: ${c.length}`);

      // --- temporarily type adaptor ---
      // console.log(c.type);
      var resTarget = cellTypeAdaptor.toUiCellType(_cellTypes, { 'detectionType': c.type, 'width': c.width, 'length': c.length });
      var rackName = (resTarget === undefined) ? (c.type + c.width + c.length) : resTarget.name;
      console.log(rackName);

      var target = _cellTypes.find(ct => ct.name == rackName);
      // console.log(target);
      if (target === undefined) {
        c.typeName = 'none';
        c.type = c.type;
        c.width = c.width;
        c.length = c.length;
      } else {
        c.typeName = target.name;
        c.type = target.detectionType;
        c.width = Number(target.width);
        c.length = Number(target.length);
      }

    })
  }

  console.log(_cellCache);
}

const defaultColor = {
  'tooltipColor': 'black',
  'color': 'black',
  'borderColor': 'black',
  'bgColor': 'gray',
  'focusColor': 'gray',
};

function getColor(_colorTable, _cellType, _colorType) {
  var target = _colorTable.find(cdt => cdt.name === _cellType);
  var colorString = (target === undefined) ? (defaultColor[_colorType] || 'black') : target.color[_colorType];

  return colorString;
}

function drawBoard(_ctx, _dim, _span = 50) {
  var bw = _dim.w;
  var bh = _dim.h;

  // --- vertical line ---
  for (var x = 0; x <= bw; x += _span) {
    _ctx.moveTo(0.5 + x, 0);
    _ctx.lineTo(0.5 + x, bh);
  }

  // --- horizontal line ---
  for (var y = 0; y <= bh; y += _span) {
    _ctx.moveTo(0, 0.5 + y);
    _ctx.lineTo(bw, 0.5 + y);
  }
  _ctx.strokeStyle = "transparent";
  _ctx.stroke();
}


if (typeof module !== 'undefined') {
  module.exports = {
    setCellSizeByType,
    getCellLoad,
    updateCellsType,
    getColor
  }
}
