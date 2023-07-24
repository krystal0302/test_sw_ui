// ==============================
//   [UX] Mobile Device Support 
// ==============================
let pinchZoom_ = false;
function bindTSPinch2Zoom() {
  if (!isTouchDevice()) return;
  fCanvas.on('touch:gesture', function (opt) {
    var e = opt.e;
    if (e.touches && e.touches.length == 2) {
      pinchZoom_ = true;
      // --- avoid origin scaling and rotation ---
      if (originSprite !== undefined) {
        fCanvas.remove(originSprite);
      }

      if (opt.self.state == "start") {
        zoomStartScale = fCanvas.getZoom();
      }
      var delta = zoomStartScale * (opt.self.scale);
      if (delta > 30) delta = 30;
      if (delta < 1) delta = 1;
      fCanvas.setZoom(delta);
    }
  });
}

function initMobileResizeEvts() {
  if (!isMobile()) return;
  window.addEventListener('resize', function () {
    refreshPopUps();
    updateToolsPosition();
  });

  function refreshPopUps() {
    updatePopUpPosition('node');
    updatePopUpPosition('cell');
    updatePopUpPosition('zone-node');
  }

  function updateToolsPosition() {
    // --- toggle edit properties tools and swap tools position ---
    if (window.innerWidth <= IPAD_PORTRAIT_WIDTH || window.innerHeight <= IPAD_PORTRAIT_WIDTH) {
      if (!$('#edit-tools-list').is(":hidden")) {
        $('#edit-tools-card').detach().appendTo('#edit-tools-section');
        $('#edit-tools-list').hide();
        $('#edit-tools-section').show();
      }
    } else {
      if (!$('#edit-tools-section').is(":hidden")) {
        $('#edit-tools-card').detach().appendTo('#edit-tools-list');
        $('#edit-tools-section').hide();
        $('#edit-tools-list').show();
      }
    }
  }
}

// ===========================
//   [UX] Cell State Styling 
// ===========================
// --- cell status color in live ---
function getLiveStatusColor(_colorCode) {
  // --- status: 'live-off' ---
  if (_colorCode === undefined) { return 'rgba(0,0,0,0.2)'; }

  // --- status: 'occupied' --- 
  if (Math.abs(_colorCode.r - 0.3) < 0.01 && _colorCode.g === 0 && _colorCode.b === 0) { return 'rgba(255,0,0,0.4)'; }

  // --- status: 'empty' ---
  if (_colorCode.r === 0 && Math.abs(_colorCode.g - 0.3) < 0.01 && _colorCode.b === 0) { return 'rgba(0,255,0,0.4)'; }

  // --- status: 'loaded' ---
  if (Math.abs(_colorCode.r - 1.0) > 0.01 && Math.abs(_colorCode.g - 1.0) < 0.01 && Math.abs(_colorCode.b - 1.0) < 0.01) { return 'rgba(98,91,87,0.4)'; }

  // --- status: 'unknown' ---
  return 'rgba(255,255,0,0.4)';
}