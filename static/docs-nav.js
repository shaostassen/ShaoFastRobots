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
	function initSidebarToggle() {
		var root = document.documentElement;
		// Two entry points: the nav button (which also brings the sidebar back once
		// it is gone) and a collapse control in the sidebar header, which is where
		// people look for one first.
		var navButton = document.getElementById("sidebar-toggle-button");
		var collapseButton = document.querySelector(".docs-sidebar-collapse");
		var buttons = [navButton, collapseButton].filter(Boolean);
		if (!buttons.length) return;

		function sync() {
			var hidden = root.classList.contains("sidebar-hidden");
			if (navButton) {
				navButton.setAttribute("aria-pressed", String(hidden));
				navButton.title = hidden ? "Show sidebar" : "Hide sidebar";
			}
		}

		buttons.forEach(function (button) {
			button.addEventListener("click", function () {
				var hidden = root.classList.toggle("sidebar-hidden");
				store(hidden);
				sync();
			});
		});

		sync();
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
