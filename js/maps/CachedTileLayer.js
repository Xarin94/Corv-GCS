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
         */
        _loadFromNetwork: function (tile, coords, z, done) {
            const tileUrl = this.getTileUrl(coords);

            tile.onload = () => {
                done(null, tile);
                // Opportunistic cache: draw to canvas → toBlob → IndexedDB
                this._cacheFromImg(tile, provider, z, coords.x, coords.y);
            };
            tile.onerror = (err) => {
                done(err || new Error('tile load error'), tile);
            };
            tile.src = tileUrl;
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
