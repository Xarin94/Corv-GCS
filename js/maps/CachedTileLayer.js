/**
 * CachedTileLayer.js - Leaflet TileLayer with IndexedDB cache-first strategy
 * Drop-in replacement for L.tileLayer() with offline support.
 *
 * For cache misses, uses the native <img> tag (same as standard Leaflet) to
 * avoid CORS 403 errors from tile servers that block fetch() from Electron.
 * Opportunistic caching converts the loaded image to a blob via a canvas.
 */

import { getTile, putTile } from './TileCache.js';

/**
 * Create a cached tile layer.
 * @param {string} urlTemplate - Standard Leaflet tile URL template
 * @param {object} options - Leaflet TileLayer options + { provider: 'osm'|'esri' }
 * @returns {L.TileLayer} Leaflet tile layer with cache-first loading
 */
export function cachedTileLayer(urlTemplate, options = {}) {
    const provider = options.provider || 'osm';

    const layer = L.TileLayer.extend({
        createTile: function (coords, done) {
            const tile = document.createElement('img');
            tile.alt = '';
            tile.setAttribute('role', 'presentation');
            tile.crossOrigin = 'anonymous';

            const { x, y } = coords;
            const z = this._getZoomForUrl();

            // Try IndexedDB cache first
            getTile(provider, z, x, y).then(blob => {
                if (blob) {
                    // Cache hit — load from blob
                    const objUrl = URL.createObjectURL(blob);
                    tile.onload = () => {
                        URL.revokeObjectURL(objUrl);
                        done(null, tile);
                    };
                    tile.onerror = () => {
                        URL.revokeObjectURL(objUrl);
                        // Corrupted cache entry — fall back to network via <img>
                        this._loadFromNetwork(tile, coords, z, done);
                    };
                    tile.src = objUrl;
                } else {
                    // Cache miss — load from network via <img>
                    this._loadFromNetwork(tile, coords, z, done);
                }
            }).catch(() => {
                this._loadFromNetwork(tile, coords, z, done);
            });

            return tile;
        },

        /**
         * Load tile from network using native <img> src (avoids CORS fetch issues).
         * After successful load, opportunistically cache via canvas.toBlob().
         * If offline, generates a placeholder tile instead of failing silently.
         */
        _loadFromNetwork: function (tile, coords, z, done) {
            if (!navigator.onLine) {
                this._placeholderTile(tile, done);
                return;
            }

            const tileUrl = this.getTileUrl(coords);
            let retried = false;

            tile.onload = () => {
                done(null, tile);
                this._cacheFromImg(tile, provider, z, coords.x, coords.y);
            };
            tile.onerror = () => {
                if (!retried) {
                    retried = true;
                    setTimeout(() => { tile.src = tileUrl; }, 2000);
                    return;
                }
                this._placeholderTile(tile, done);
            };
            tile.src = tileUrl;
        },

        /**
         * Generate a dark placeholder tile with "OFFLINE" text for uncached areas.
         */
        _placeholderTile: function (tile, done) {
            const c = document.createElement('canvas');
            c.width = 256; c.height = 256;
            const ctx = c.getContext('2d');
            ctx.fillStyle = '#1a1a2e';
            ctx.fillRect(0, 0, 256, 256);
            ctx.strokeStyle = 'rgba(0,210,255,0.08)';
            ctx.strokeRect(0, 0, 256, 256);
            ctx.fillStyle = 'rgba(0,210,255,0.15)';
            ctx.font = '11px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('OFFLINE', 128, 132);
            tile.src = c.toDataURL();
            tile.onload = () => done(null, tile);
        },

        /**
         * Convert a loaded <img> to a blob via canvas and store in IndexedDB.
         * Fire-and-forget, errors are silently ignored.
         */
        _cacheFromImg: function (img, prov, z, x, y) {
            try {
                const canvas = document.createElement('canvas');
                canvas.width = img.naturalWidth || 256;
                canvas.height = img.naturalHeight || 256;
                const ctx = canvas.getContext('2d');
                ctx.drawImage(img, 0, 0);
                canvas.toBlob((blob) => {
                    if (blob) {
                        putTile(prov, z, x, y, blob).catch(() => {});
                    }
                }, 'image/png');
            } catch (e) {
                // CORS tainted canvas — skip caching silently
            }
        }
    });

    return new layer(urlTemplate, options);
}
