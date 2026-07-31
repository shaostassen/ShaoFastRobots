// Sidebar nav behaviour. Progressive enhancement only — without this script the
// nav still renders and works, it is just less pleasant on small screens.
(function () {
	"use strict";

	var NARROW = 1100; // keep in sync with the breakpoint in docs.css

	function init() {
		var group = document.querySelector(".docs-nav-group");
		if (!group) return;

		if (window.innerWidth <= NARROW) {
			// Start collapsed on narrow screens, where an expanded 12-item list
			// would push the actual page content most of a screen down.
			group.open = false;
			return;
		}

		// Reveal the current lab inside the scrolling list. Scrolls the sidebar's
		// own scroll container only — never the page.
		var current = group.querySelector(".docs-nav > li.current");
		if (!current) return;

		var groupBox = group.getBoundingClientRect();
		var currentBox = current.getBoundingClientRect();

		if (currentBox.bottom > groupBox.bottom || currentBox.top < groupBox.top) {
			group.scrollTop += currentBox.top - groupBox.top - 8;
		}
	}

	if (document.readyState === "loading") {
		document.addEventListener("DOMContentLoaded", init);
	} else {
		init();
	}
})();
