	// Based on https://codeberg.org/daudix/duckquill/issues/101#issuecomment-2377169
	let searchSetup = false;
	let fuse;

	async function initIndex() {
		if (searchSetup) return;

		const url = document.getElementById("search-index").textContent;
		const response = await fetch(url);

		if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);

		const options = {
			includeScore: false,
			includeMatches: true,
			ignoreLocation: true,
			threshold: 0.15,
			keys: [
				{ name: "title", weight: 3 },
				{ name: "description", weight: 2 },
				{ name: "body", weight: 1 }
			]
		};

		fuse = new Fuse(await response.json(), options);
		searchSetup = true;

		console.log("Search index initialized successfully");
	}

	// The docs sidebar keeps the search box permanently visible, so the nav button
	// and the "/" shortcut move focus into it instead of toggling it open.
	function focusSearch() {
		initIndex();
		const searchBar = document.getElementById("search-bar");
		searchBar.removeAttribute("disabled");
		searchBar.focus();
		searchBar.select();
	}

	function debounce(actual_fn, wait) {
		let timeoutId;
	
		return (...args) => {
			clearTimeout(timeoutId);
	
			timeoutId = setTimeout(() => {
				actual_fn(...args);
			}, wait);
		};
	};

	function initSearch() {
		const searchBar = document.getElementById("search-bar");
		const searchResults = document.getElementById("search-results");
		const searchContainer = document.getElementById("search-container");
		const MAX_ITEMS = 10;
		const MAX_RESULTS = 4;

		let currentTerm = "";

		// Always-visible sidebar box: enable it up front and load the index on first
		// interaction. Rendering is factored out so it can be re-run once the index
		// resolves — otherwise anything typed while it was still loading is lost.
		searchBar.removeAttribute("disabled");

		function render() {
			if (!searchSetup) return;
			const searchVal = searchBar.value.trim();
			const results = searchVal ? fuse.search(searchVal, { limit: MAX_ITEMS }) : [];

			let html = "";
			for (const result of results) {
				html += makeTeaser(result, searchVal);
			}
			searchResults.innerHTML = html;
			searchResults.style.display = html ? "flex" : "none";
		}

		function loadThenRender() {
			initIndex().then(render).catch((e) => console.error("Search index failed:", e));
		}

		searchBar.addEventListener("focus", loadThenRender, { once: true });

		searchBar.addEventListener("keyup", () => {
			if (!searchSetup) {
				loadThenRender(); // no-op once loaded; initIndex() is idempotent
				return;
			}
			render();
		});

		function makeTeaser(result, searchVal) {
			const TEASER_SIZE = 20;
			let output = `<div class="search-result item"><a class="result-title" href="${result.item.url}">${result.item.title}</a>`;

			for (const match of result.matches) {
				if (match.key === "title") continue;

				const indices = match.indices.sort((a, b) => Math.abs(a[1] - a[0] - searchVal.length) - Math.abs(b[1] - b[0] - searchVal.length)).slice(0, MAX_RESULTS);
				const value = match.value;

				for (const ind of indices) {
					const start = Math.max(0, ind[0] - TEASER_SIZE);
					const end = Math.min(value.length - 1, ind[1] + TEASER_SIZE);
					output += "<span>"
						+ value.substring(start, ind[0])
						+ `<strong>${value.substring(ind[0], ind[1] + 1)}</strong>`
						+ value.substring(ind[1] + 1, end)
						+ "</span>";
				}

				if (match.indices.length > 4) {
                    const moreMatchesText = document.getElementById("more-matches-text").textContent;
					output += `<span class="more-matches">${moreMatchesText}</span>`.replace("$MATCHES", `+${match.indices.length - MAX_RESULTS}`);
				}
			}
			return output + "</div>";
		}

		/*window.addEventListener("click", function (event) {
			if (searchSetup && searchBar.getAttribute("disabled") === null && !searchContainer.contains(event.target)) {
				toggleSearch();
			}
		}, { passive: true });*/

		document.addEventListener("keydown", function(event) {
			// Don't hijack "/" while the user is already typing in a field.
			const tag = event.target.tagName;
			if (tag === "INPUT" || tag === "TEXTAREA" || event.target.isContentEditable) return;

			if (event.key === "/") {
				event.preventDefault();
				focusSearch();
			}
		});

		const searchToggle = document.getElementById("search-toggle");
		if (searchToggle) searchToggle.addEventListener("click", focusSearch);
	}

	if (document.readyState === "complete" ||
		(document.readyState !== "loading" && !document.documentElement.doScroll))
		initSearch();
	else
		document.addEventListener("DOMContentLoaded", initSearch);
