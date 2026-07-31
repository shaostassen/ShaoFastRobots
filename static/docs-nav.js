// Sidebar nav behaviour. Progressive enhancement only — without this script the
// nav still renders and works, it is just less pleasant on small screens.
(function () {
	"use strict";

	var NARROW = 1100; // keep in sync with the breakpoint in docs.css
	var STORAGE_KEY = "docs-sidebar-hidden";

	function store(hidden) {
		try {
			localStorage.setItem(STORAGE_KEY, hidden ? "1" : "0");
		} catch (e) {
			/* private browsing / storage disabled — the toggle still works per-page */
		}
	}

	// Whole-sidebar show/hide. The preference is restored before paint by an
	// inline script in partials/head.html; this only handles the button and
	// persistence, so the choice survives navigating between labs.
	// Show/hide the whole sidebar. Two controls, each visible in exactly one
	// state: « in the sidebar header collapses it, and » pinned to the page edge
	// brings it back (a control inside the sidebar would vanish along with it).
	function initSidebarToggle() {
		var root = document.documentElement;
		var buttons = [
			document.querySelector(".docs-sidebar-collapse"),
			document.querySelector(".docs-sidebar-expand"),
		].filter(Boolean);
		if (!buttons.length) return;

		buttons.forEach(function (button) {
			button.addEventListener("click", function () {
				store(root.classList.toggle("sidebar-hidden"));
			});
		});
	}

	function init() {
		initSidebarToggle();

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
