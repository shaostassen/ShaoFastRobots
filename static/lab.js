// Lab write-up page enhancements: scroll-spy for the right-hand contents rail,
// and click-to-zoom for figures. Both no-op on pages that lack the markup, so
// this can load site-wide. Nothing here is required to read the page.
(function () {
	"use strict";

	// Highlight the contents entry for whichever section is currently in view.
	function initScrollSpy() {
		var toc = document.getElementById("lab-toc");
		if (!toc || !("IntersectionObserver" in window)) return;

		var links = {};
		var headings = [];

		Array.prototype.forEach.call(toc.querySelectorAll("a[href*='#']"), function (link) {
			var id = decodeURIComponent(link.hash.slice(1));
			var heading = id && document.getElementById(id);
			if (!heading) return;
			links[id] = link;
			headings.push(heading);
		});
		if (!headings.length) return;

		var visible = {};

		function refresh() {
			// Pick the visible heading nearest the top of the viewport; if none
			// are on screen, keep whatever was last marked.
			var best = null;
			headings.forEach(function (heading) {
				if (!visible[heading.id]) return;
				var top = heading.getBoundingClientRect().top;
				if (best === null || top < best.top) best = { id: heading.id, top: top };
			});
			if (!best) return;

			Object.keys(links).forEach(function (id) {
				links[id].classList.toggle("is-current", id === best.id);
			});
		}

		var observer = new IntersectionObserver(
			function (entries) {
				entries.forEach(function (entry) {
					visible[entry.target.id] = entry.isIntersecting;
				});
				refresh();
			},
			// Bias the band towards the top so the entry changes as a section
			// reaches reading position, not when it merely peeks into view.
			{ rootMargin: "-10% 0px -70% 0px" }
		);

		headings.forEach(function (heading) {
			observer.observe(heading);
		});
	}

	// Click any figure image to see it full size. Plots and oscilloscope traces
	// are the reason: they are unreadable at inline width.
	function initLightbox() {
		var images = document.querySelectorAll("article figure img");
		if (!images.length) return;

		var box, boxImg, boxCaption;

		function build() {
			box = document.createElement("div");
			box.className = "lab-lightbox";
			box.setAttribute("role", "dialog");
			box.setAttribute("aria-modal", "true");

			boxImg = document.createElement("img");
			boxCaption = document.createElement("p");
			boxCaption.className = "lab-lightbox-caption";

			var close = document.createElement("button");
			close.className = "lab-lightbox-close";
			close.type = "button";
			close.setAttribute("aria-label", "Close");
			close.textContent = "✕";

			box.appendChild(boxImg);
			box.appendChild(boxCaption);
			box.appendChild(close);
			document.body.appendChild(box);

			box.addEventListener("click", hide);
		}

		function show(image) {
			if (!box) build();
			boxImg.src = image.currentSrc || image.src;
			boxImg.alt = image.alt || "";
			var caption = image.closest("figure");
			caption = caption && caption.querySelector("figcaption");
			boxCaption.textContent = caption ? caption.textContent.trim() : "";
			box.classList.add("is-open");
		}

		function hide() {
			if (box) box.classList.remove("is-open");
		}

		Array.prototype.forEach.call(images, function (image) {
			image.addEventListener("click", function () {
				show(image);
			});
		});

		document.addEventListener("keydown", function (event) {
			if (event.key === "Escape") hide();
		});
	}

	function init() {
		initScrollSpy();
		initLightbox();
	}

	if (document.readyState === "loading") {
		document.addEventListener("DOMContentLoaded", init);
	} else {
		init();
	}
})();
