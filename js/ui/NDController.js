/**
 * NDController.js
 * 
 * Handles all ND panel UI interactions and binds them to NDView configuration
 */

import { STATE } from '../core/state.js';
import { 
    ndConfig, 
    setNDMode, 
    setNDRange, 
    setSelectedHeading,
    setFlightPlan,
    setCurrentWaypoint,
    setWindData,
    setVOR1,
    setVOR2,
    setILS,
    setNextWaypoint,
    toggleNDOption,
    FLIGHT_PLANS
} from './NDView.js';

let initialized = false;
let sidebarVisible = true;
let waypointUpdateInterval = null;

// Distance threshold to consider a waypoint "passed" (in NM)
const WAYPOINT_PASS_DISTANCE = 1.0;

/**
 * Initialize ND control panel event listeners
 */
export function initNDControls() {
    if (initialized) return;
    
    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', setupEventListeners);
    } else {
        setupEventListeners();
    }
    
    // Start continuous waypoint tracking
    startWaypointTracking();
    
    initialized = true;
}

function setupEventListeners() {
    // Toggle sidebar button
    const toggleBtn = document.getElementById('nd-toggle-panels');
    if (toggleBtn) {
        toggleBtn.addEventListener('click', toggleSidebar);
    }
    
    // Section collapse headers
    document.querySelectorAll('.nd-section-header').forEach(header => {
        header.addEventListener('click', (e) => {
            const section = header.closest('.nd-sidebar-section');
            if (section) {
                section.classList.toggle('collapsed');
            }
        });
    });
    
    // Mode buttons
    document.querySelectorAll('.nd-mode-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const mode = e.target.dataset.mode;
            if (mode) {
                setNDMode(mode);
                updateModeButtons(mode);
            }
        });
    });
    
    // Range buttons
    document.querySelectorAll('.nd-range-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const range = parseInt(e.target.dataset.range, 10);
            if (!isNaN(range)) {
                setNDRange(range);
                updateRangeButtons(range);
            }
        });
    });
    
    // Display options checkboxes
    setupDisplayOption('nd-show-wpt', 'showWpt');
    setupDisplayOption('nd-show-vordme', 'showVorDme');
    setupDisplayOption('nd-show-ndb', 'showNdb');
    setupDisplayOption('nd-show-arpt', 'showArpt');
    setupDisplayOption('nd-show-terr', 'showTerrain');
    setupDisplayOption('nd-show-wxr', 'showWxr');
    setupDisplayOption('nd-show-tfc', 'showTfc');
    
    // HDG controls
    setupHeadingControls();
    
    // VOR/ILS inputs
    setupNavInputs();
    
    // Wind inputs
    setupWindInputs();
    
    // Flight plan controls
    setupFlightPlanControls();
    
    // Collapse some sections by default to save space
    const sectionsToCollapse = ['vor1', 'vor2', 'ils', 'fplan'];
    sectionsToCollapse.forEach(sectionName => {
        const section = document.querySelector(`.nd-sidebar-section[data-section="${sectionName}"]`);
        if (section) {
            section.classList.add('collapsed');
        }
    });
    
    // Initial sidebar position update
    updateSidebarPosition();
    
    // Watch for split pane changes using MutationObserver
    const bodyObserver = new MutationObserver(() => {
        updateSidebarPosition();
    });
    bodyObserver.observe(document.body, { 
        attributes: true, 
        attributeFilter: ['data-split-left', 'data-split-right', 'class'] 
    });
}

/**
 * Update sidebar position based on which frame the ND is in
 */
function updateSidebarPosition() {
    const ndContainer = document.getElementById('nd-container');
    const toggleBtn = document.getElementById('nd-toggle-panels');
    if (!ndContainer) return;
    
    const body = document.body;
    const isNDOnLeft = body.dataset.splitLeft === 'nd';
    const isNDOnRight = body.dataset.splitRight === 'nd';
    
    // Position sidebar on the outer edge (left when ND is left, right when ND is right)
    if (isNDOnLeft) {
        ndContainer.classList.add('sidebar-left');
        // Update toggle button icon based on current state
        if (toggleBtn) {
            toggleBtn.textContent = sidebarVisible ? '▶' : '◀';
        }
    } else {
        ndContainer.classList.remove('sidebar-left');
        // Update toggle button icon based on current state
        if (toggleBtn) {
            toggleBtn.textContent = sidebarVisible ? '◀' : '▶';
        }
    }
}

function toggleSidebar() {
    const ndContainer = document.getElementById('nd-container');
    const toggleBtn = document.getElementById('nd-toggle-panels');
    
    if (!ndContainer) return;
    
    sidebarVisible = !sidebarVisible;
    
    const isLeftSide = ndContainer.classList.contains('sidebar-left');
    
    if (sidebarVisible) {
        ndContainer.classList.remove('sidebar-collapsed');
        if (toggleBtn) {
            toggleBtn.textContent = isLeftSide ? '▶' : '◀';
            toggleBtn.title = 'Hide ND Controls';
        }
    } else {
        ndContainer.classList.add('sidebar-collapsed');
        if (toggleBtn) {
            toggleBtn.textContent = isLeftSide ? '◀' : '▶';
            toggleBtn.title = 'Show ND Controls';
        }
    }
}

function updateModeButtons(activeMode) {
    document.querySelectorAll('.nd-mode-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.mode === activeMode);
    });
}

function updateRangeButtons(activeRange) {
    document.querySelectorAll('.nd-range-btn').forEach(btn => {
        btn.classList.toggle('active', parseInt(btn.dataset.range, 10) === activeRange);
    });
}

function setupDisplayOption(checkboxId, configKey) {
    const checkbox = document.getElementById(checkboxId);
    if (checkbox) {
        checkbox.checked = ndConfig[configKey];
        checkbox.addEventListener('change', () => {
            ndConfig[configKey] = checkbox.checked;
        });
    }
}

function setupHeadingControls() {
    const hdgInput = document.getElementById('nd-hdg-input');
    const managedBtn = document.getElementById('hdg-managed');
    const selectedBtn = document.getElementById('hdg-selected');
    
    // Inc/Dec buttons
    const hdgDec10 = document.getElementById('hdg-dec-10');
    const hdgDec1 = document.getElementById('hdg-dec-1');
    const hdgInc1 = document.getElementById('hdg-inc-1');
    const hdgInc10 = document.getElementById('hdg-inc-10');
    
    function updateHeading(delta) {
        if (ndConfig.selectedHdg === null) {
            ndConfig.selectedHdg = 0;
        }
        ndConfig.selectedHdg = ((ndConfig.selectedHdg + delta) % 360 + 360) % 360;
        if (hdgInput) {
            hdgInput.value = Math.round(ndConfig.selectedHdg);
        }
        
        // Switch to selected mode when adjusting
        if (managedBtn) managedBtn.classList.remove('active');
        if (selectedBtn) selectedBtn.classList.add('active');
    }
    
    if (hdgDec10) hdgDec10.addEventListener('click', () => updateHeading(-10));
    if (hdgDec1) hdgDec1.addEventListener('click', () => updateHeading(-1));
    if (hdgInc1) hdgInc1.addEventListener('click', () => updateHeading(1));
    if (hdgInc10) hdgInc10.addEventListener('click', () => updateHeading(10));
    
    if (hdgInput) {
        hdgInput.addEventListener('change', () => {
            const val = parseInt(hdgInput.value, 10);
            if (!isNaN(val)) {
                setSelectedHeading(val);
                if (managedBtn) managedBtn.classList.remove('active');
                if (selectedBtn) selectedBtn.classList.add('active');
            }
        });
    }
    
    if (managedBtn) {
        managedBtn.addEventListener('click', () => {
            setSelectedHeading(null);
            if (hdgInput) hdgInput.value = '';
            managedBtn.classList.add('active');
            if (selectedBtn) selectedBtn.classList.remove('active');
        });
    }
    
    if (selectedBtn) {
        selectedBtn.addEventListener('click', () => {
            if (ndConfig.selectedHdg === null) {
                ndConfig.selectedHdg = 0;
            }
            if (hdgInput) hdgInput.value = Math.round(ndConfig.selectedHdg);
            selectedBtn.classList.add('active');
            if (managedBtn) managedBtn.classList.remove('active');
        });
    }
}

function setupNavInputs() {
    // VOR 1
    const vor1Id = document.getElementById('vor1-id');
    const vor1Freq = document.getElementById('vor1-freq');
    const vor1Crs = document.getElementById('vor1-crs');
    
    function updateVOR1() {
        const id = vor1Id?.value || '---';
        const freq = vor1Freq?.value || '---.-';
        const crs = parseInt(vor1Crs?.value, 10) || 0;
        setVOR1(id, freq, crs, null);
    }
    
    if (vor1Id) vor1Id.addEventListener('change', updateVOR1);
    if (vor1Freq) vor1Freq.addEventListener('change', updateVOR1);
    if (vor1Crs) vor1Crs.addEventListener('change', updateVOR1);
    
    // VOR 2
    const vor2Id = document.getElementById('vor2-id');
    const vor2Freq = document.getElementById('vor2-freq');
    const vor2Crs = document.getElementById('vor2-crs');
    
    function updateVOR2() {
        const id = vor2Id?.value || '---';
        const freq = vor2Freq?.value || '---.-';
        const crs = parseInt(vor2Crs?.value, 10) || 0;
        setVOR2(id, freq, crs, null);
    }
    
    if (vor2Id) vor2Id.addEventListener('change', updateVOR2);
    if (vor2Freq) vor2Freq.addEventListener('change', updateVOR2);
    if (vor2Crs) vor2Crs.addEventListener('change', updateVOR2);
    
    // ILS
    const ilsId = document.getElementById('ils-id');
    const ilsFreq = document.getElementById('ils-freq');
    const ilsCrs = document.getElementById('ils-crs');
    
    function updateILS() {
        const id = ilsId?.value || '---';
        const freq = ilsFreq?.value || '---.-';
        const crs = parseInt(ilsCrs?.value, 10) || 0;
        setILS(id, freq, crs);
    }
    
    if (ilsId) ilsId.addEventListener('change', updateILS);
    if (ilsFreq) ilsFreq.addEventListener('change', updateILS);
    if (ilsCrs) ilsCrs.addEventListener('change', updateILS);
}

function setupWindInputs() {
    const windDir = document.getElementById('wind-dir');
    const windSpd = document.getElementById('wind-spd');
    
    function updateWind() {
        const dir = parseInt(windDir?.value, 10) || 0;
        const spd = parseInt(windSpd?.value, 10) || 0;
        setWindData(dir, spd);
    }
    
    if (windDir) windDir.addEventListener('change', updateWind);
    if (windSpd) windSpd.addEventListener('change', updateWind);
}

function setupFlightPlanControls() {
    const routeSelect = document.getElementById('fplan-route-select');
    const addBtn = document.getElementById('fplan-add');
    const clearBtn = document.getElementById('fplan-clear');
    const addForm = document.getElementById('fplan-add-form');
    const confirmBtn = document.getElementById('fplan-confirm');
    const cancelBtn = document.getElementById('fplan-cancel');
    const fplanList = document.getElementById('fplan-list');
    
    // Route selection dropdown
    if (routeSelect) {
        routeSelect.addEventListener('change', (e) => {
            const routeId = e.target.value;
            if (routeId && FLIGHT_PLANS[routeId]) {
                const route = FLIGHT_PLANS[routeId];
                ndConfig.flightPlan = route.waypoints.map(wpt => ({ ...wpt }));
                ndConfig.currentWptIndex = 0;
                updateFlightPlanList();
                updateNextWaypointInfo();
            }
        });
    }
    
    if (addBtn && addForm) {
        addBtn.addEventListener('click', () => {
            addForm.style.display = 'flex';
            document.getElementById('new-wpt-id')?.focus();
        });
    }
    
    if (cancelBtn && addForm) {
        cancelBtn.addEventListener('click', () => {
            addForm.style.display = 'none';
            clearAddForm();
        });
    }
    
    if (confirmBtn) {
        confirmBtn.addEventListener('click', () => {
            const idInput = document.getElementById('new-wpt-id');
            const latInput = document.getElementById('new-wpt-lat');
            const lonInput = document.getElementById('new-wpt-lon');
            
            const id = idInput?.value?.toUpperCase() || '';
            const lat = parseFloat(latInput?.value);
            const lon = parseFloat(lonInput?.value);
            
            if (id && !isNaN(lat) && !isNaN(lon)) {
                ndConfig.flightPlan.push({ id, lat, lon });
                updateFlightPlanList();
                updateNextWaypointInfo();
                
                addForm.style.display = 'none';
                clearAddForm();
            }
        });
    }
    
    if (clearBtn) {
        clearBtn.addEventListener('click', () => {
            ndConfig.flightPlan = [];
            ndConfig.currentWptIndex = 0;
            // Reset route selector
            if (routeSelect) routeSelect.value = '';
            updateFlightPlanList();
            setNextWaypoint('-----', 0, '---');
        });
    }
}

function clearAddForm() {
    const idInput = document.getElementById('new-wpt-id');
    const latInput = document.getElementById('new-wpt-lat');
    const lonInput = document.getElementById('new-wpt-lon');
    
    if (idInput) idInput.value = '';
    if (latInput) latInput.value = '';
    if (lonInput) lonInput.value = '';
}

function updateFlightPlanList() {
    const fplanList = document.getElementById('fplan-list');
    if (!fplanList) return;
    
    fplanList.innerHTML = '';
    
    ndConfig.flightPlan.forEach((wpt, idx) => {
        const wptEl = document.createElement('div');
        wptEl.className = 'fplan-wpt' + (idx === ndConfig.currentWptIndex ? ' active' : '');
        wptEl.innerHTML = `
            <span class="wpt-id">${wpt.id}</span>
            <button class="wpt-remove" data-index="${idx}" title="Remove waypoint">&times;</button>
        `;
        
        // Click on waypoint to make it active
        wptEl.addEventListener('click', (e) => {
            if (!e.target.classList.contains('wpt-remove')) {
                ndConfig.currentWptIndex = idx;
                updateFlightPlanList();
                updateNextWaypointInfo();
            }
        });
        
        // Remove button
        const removeBtn = wptEl.querySelector('.wpt-remove');
        if (removeBtn) {
            removeBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                ndConfig.flightPlan.splice(idx, 1);
                if (ndConfig.currentWptIndex >= ndConfig.flightPlan.length) {
                    ndConfig.currentWptIndex = Math.max(0, ndConfig.flightPlan.length - 1);
                }
                updateFlightPlanList();
                updateNextWaypointInfo();
            });
        }
        
        fplanList.appendChild(wptEl);
    });
}

/**
 * Calculate distance between two lat/lon points in nautical miles
 */
function calculateDistanceNM(lat1, lon1, lat2, lon2) {
    const R = 3440.065; // Earth radius in nautical miles
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}

/**
 * Calculate ETA based on distance and ground speed
 * Returns local time in the PC's timezone
 */
function calculateETA(distNM, gsKnots) {
    if (!gsKnots || gsKnots < 1) return '---';
    const hoursToWpt = distNM / gsKnots;
    const now = new Date();
    now.setTime(now.getTime() + hoursToWpt * 3600000);
    const hh = now.getHours().toString().padStart(2, '0');
    const mm = now.getMinutes().toString().padStart(2, '0');
    return `${hh}:${mm}`;
}

function updateNextWaypointInfo() {
    if (ndConfig.flightPlan.length === 0) {
        setNextWaypoint('-----', 0, '---');
        return;
    }
    
    const nextWpt = ndConfig.flightPlan[ndConfig.currentWptIndex];
    if (!nextWpt) {
        setNextWaypoint('-----', 0, '---');
        return;
    }
    
    // Calculate distance from current position to next waypoint
    const dist = calculateDistanceNM(STATE.lat, STATE.lon, nextWpt.lat, nextWpt.lon);
    
    // Calculate ETA based on ground speed
    const eta = calculateETA(dist, STATE.gs);
    
    // Update the display
    setNextWaypoint(nextWpt.id, dist, eta);
    
    // Check if waypoint is passed (within threshold distance)
    if (dist < WAYPOINT_PASS_DISTANCE && ndConfig.currentWptIndex < ndConfig.flightPlan.length - 1) {
        // Advance to next waypoint
        ndConfig.currentWptIndex++;
        updateFlightPlanList();
        // Recursively update for the new waypoint
        updateNextWaypointInfo();
    }
}

/**
 * Start continuous waypoint tracking updates
 */
export function startWaypointTracking() {
    if (waypointUpdateInterval) return;
    waypointUpdateInterval = setInterval(() => {
        if (ndConfig.flightPlan.length > 0) {
            updateNextWaypointInfo();
        }
    }, 1000); // Update every second
}

/**
 * Stop waypoint tracking
 */
export function stopWaypointTracking() {
    if (waypointUpdateInterval) {
        clearInterval(waypointUpdateInterval);
        waypointUpdateInterval = null;
    }
}

/**
 * Add some sample waypoints for testing/demo
 */
export function loadSampleFlightPlan() {
    ndConfig.flightPlan = [
        { id: 'LFPG', lat: 49.0097, lon: 2.5479 },    // Paris CDG
        { id: 'BIBAX', lat: 48.7500, lon: 3.1000 },
        { id: 'DEKOD', lat: 48.3333, lon: 4.0000 },
        { id: 'LSZH', lat: 47.4647, lon: 8.5492 },    // Zurich
        { id: 'LIMC', lat: 45.6306, lon: 8.7231 },    // Milan Malpensa
    ];
    ndConfig.currentWptIndex = 0;
    updateFlightPlanList();
    updateNextWaypointInfo();
}

// Export for external access
export { sidebarVisible, toggleSidebar };
