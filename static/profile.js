// Landing-page enhancements. Loaded only on / via `extra.scripts` in
// content/_index.md. Everything here is progressive: with the script absent the
// page renders complete and static, just without the draw-on and the reveals.
(function () {
	"use strict";

	var reduced = window.matchMedia
		? window.matchMedia("(prefers-reduced-motion: reduce)").matches
		: false;

	// Fade sections up as they scroll into view. Unobserved once shown, so
	// scrolling back up does not replay them.
	function initReveals() {
		var targets = document.querySelectorAll(".reveal");
		if (!targets.length) return;

		if (reduced || !("IntersectionObserver" in window)) {
			for (var i = 0; i < targets.length; i++) {
				targets[i].classList.add("is-visible");
			}
			return;
		}

		var observer = new IntersectionObserver(
			function (entries) {
				entries.forEach(function (entry) {
					if (!entry.isIntersecting) return;
					entry.target.classList.add("is-visible");
					observer.unobserve(entry.target);
				});
			},
			{ rootMargin: "0px 0px -10% 0px", threshold: 0.1 }
		);

		targets.forEach(function (target) {
			observer.observe(target);
		});
	}

	function init() {
		initReveals();
	}

	if (document.readyState === "loading") {
		document.addEventListener("DOMContentLoaded", init);
	} else {
		init();
	}
})();
