// Landing-page enhancements. Loaded only on / via `extra.scripts` in
// content/_index.md. Everything here is progressive: with the script absent the
// page renders complete and static, just without the draw-on and the reveals.
(function () {
	"use strict";

	var reduced = window.matchMedia
		? window.matchMedia("(prefers-reduced-motion: reduce)").matches
		: false;

	// Measure a path's true length into a custom property, so the dash offset is
	// exact for whatever curve is in the markup rather than a hardcoded guess.
	function measure(selector, property) {
		var path = document.querySelector(selector);
		if (!path || typeof path.getTotalLength !== "function") return false;

		var length = path.getTotalLength();
		if (!length) return false;

		path.style.setProperty(property, length.toFixed(1));
		return true;
	}

	function initHeroPlot() {
		if (measure(".hero-plot-curve", "--trace-length")) {
			document.documentElement.classList.add("trace-ready");
		}
	}

	// The attractor is ~11k of path; measuring it is what lets it draw on when it
	// scrolls into view. Without this the CSS leaves it fully drawn.
	function initChaosPlot() {
		measure(".chaos-curve", "--chaos-length");
	}

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
		initHeroPlot();
		initChaosPlot();
		initReveals();
	}

	if (document.readyState === "loading") {
		document.addEventListener("DOMContentLoaded", init);
	} else {
		init();
	}
})();
