/*
 * Author: John 
 * Date: 29 Jul 2022
 * Description: 
 *   About Hollywood:
 *     1. styles controller 
 **/

function applyFontSpecs() {
	// --- Sidebar Row Title ---
	$(document).find('.nav-sidebar > .user-panel > .nav-item > .nav-link > p').each(function () {
		$(this).css("font-family", "Arial");
	});
	// TODO: move after fetching trigger tasks list
	// $(document).find('.nav-sidebar > .user-panel > .nav-item > .nav-treeview > .nav-item > .nav-link').each(function () {
	// 	$(this).css("font-family", "Arial");
	// });
	$(document).find('.nav-sidebar > .nav-item > .nav-link > p').each(function () {
		$(this).css("font-family", "Arial");
	});
	$(document).find('.sidebar-footer').each(function () {
		$(this).css("font-family", "Arial");
	});



	// --- Card Header Title ---
	$(document).find('.card-title > span').each(function () {
		// --- font-text, font-family spec. ---
		let title = $(this).text().trim().toLowerCase();
		title = title.replace(/\b\w/g, l => l.toUpperCase())
		$(this).text(title)
		$(this).css("font-family", "Arial");
	});

	// --- Content Header Title ---
	$(document).find('.row > .content-header').each(function () {
		// --- font-text, font-family spec. ---
		let title = $(this).text().trim().toLowerCase();
		title = title.replace(/\b\w/g, l => l.toUpperCase())
		$(this).find('h4').text(title)
		$(this).css("font-family", "Arial");
		// $(this).css("font-size", "110%");
	});

	// --- Card Tab Title ---
	$(document).find('.card-header > ul > li > a > span').each(function () {
		// --- font-text, font-family spec. ---
		let title = $(this).text().trim().toLowerCase();
		title = title.replace(/\b\w/g, l => l.toUpperCase())
		$(this).text(title)
		$(this).css("font-family", "Arial");
	});
}

function applyFontSize(opt, querySel = 'all') {
	var $selector = $(`${querySel} *`);
	if (querySel === 'all') {
		$selector = $('*');
	}
	let percent = getFontScalePercent(opt);
	$selector.each(function () {
		switch (this.tagName.toLowerCase()) {
			case "h1":
			case "h2":
			case "h3":
			case "h4":
			case "h5":
			case "h6":
				if ($(this).children().length > 0) return;
				let lastOpt = getSavedFontSize();
				var addVal = 0;
				if (isEmptyString(lastOpt)) return;
				if (lastOpt === opt) {
					if (opt === 'medium') return;
					addVal = (opt === 'small') ? -2 : 2;
				} else {
					if (lastOpt === 'small') {
						addVal = (opt === 'medium') ? 2 : 4;
					} else if (lastOpt === 'medium') {
						addVal = (opt === 'small') ? -2 : 2;
					} else if (lastOpt === 'large') {
						addVal = (opt === 'small') ? -4 : -2;
					}
				}
				var fontSize = parseFloat($(this).css('font-size'));
				fontSize = fontSize + addVal + "px";
				$(this).css('font-size', fontSize);
				break;
			case "input":
				if ($(this).attr('type') !== 'text' && $(this).attr('type') !== 'number') return;
				$(this).css('font-size', percent);
				break;
			case "button":
				if ($(this).children().length > 0) {
					let firstEl = $(this).children(":first")[0];
					if (firstEl.nodeName.toLowerCase() !== 'i') return;
					if (firstEl.nextSibling === null || isEmptyString(firstEl.nextSibling.textContent)) return;
				}
				// --- calculate font size according to button size class if included ---
				let btnSizeClass = _.intersection(Object.keys(btnClassFontSize), this.className.split(/\s+/));
				let btnFontSize = btnClassFontSize[btnSizeClass.toString()];
				if (isNaN(getFontScaleSize(btnFontSize))) {
					$(this).css('font-size', percent);
				} else {
					$(this).css('font-size', `${getFontScaleSize(btnFontSize)}px`);
				}
				break;
			case "li":
			case "th":
			case "td":
				if ($(this).children().length > 0) return;
				$(this).css('font-size', percent);
				break;
			case "span":
				if ($(this).children('span').length > 0) return;
				$(this).css('font-size', percent);
				break;
			case "p":
				if ($(this).children('label').length > 0) return;
				$(this).css('font-size', percent);
				break;
			case "a":
				const toggleType = ['tab', 'dropdown'];
				let isApplyToggleType = false;
				if ($(this).attr('data-toggle')) {
					isApplyToggleType = toggleType.some(type => $(this).attr('data-toggle').includes(type));
				}
				if (this.id === 'statusIndicator' || this.className === 'page-link' || isApplyToggleType) {
					$(this).css('font-size', percent);
				}
				break;
			case "div":
				const excludeIDs = ['save-placeholder', 'settings-btns-group', 'reset-div', 'sb-cell-size-deck'];
				let isExcludeID = excludeIDs.some(id => this.id.includes(id));
				if ($(this).children().length > 0 || isExcludeID) return;
				$(this).css('font-size', percent);
				// console.log(`${this.tagName}(${this.id || this.className}) >>> ${$(this).html()}`); 
				break;
			case "label":
			case "b":
			case "select":
				$(this).css('font-size', percent);
				break;
			default: {
				// console.log(this.tagName);
				break;
			}
		}
	});
	// --- after apply font size ---
	setLocalStorageByKey('fontSize', opt);
}

const btnClassFontSize = {
	"btn-sm": 14,
	"btn-lg": 20
}