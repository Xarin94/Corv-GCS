/**
 * LRUCache.js - Least Recently Used Cache Implementation
 * Generic cache with automatic eviction of oldest entries
 */

export class LRUCache {
    /**
     * Create an LRU cache
     * @param {number} maxSize - Maximum number of items in cache
     * @param {Function} onEvict - Callback when item is evicted
     */
    constructor(maxSize, onEvict = null) {
        this.maxSize = maxSize;
        this.cache = new Map();
        this.onEvict = onEvict;
    }

    /**
     * Get item from cache (marks as recently used)
     * @param {string} key - Cache key
     * @returns {*} Cached value or null
     */
    get(key) {
        if (!this.cache.has(key)) return null;
        const value = this.cache.get(key);
        // Move to end (most recently used)
        this.cache.delete(key);
        this.cache.set(key, value);
        return value;
    }

    /**
     * Set item in cache (evicts oldest if full)
     * @param {string} key - Cache key
     * @param {*} value - Value to cache
     */
    set(key, value) {
        if (this.cache.has(key)) {
            const oldValue = this.cache.get(key);
            this.cache.delete(key);
            if (this.onEvict) this.onEvict(oldValue);
        } else if (this.cache.size >= this.maxSize) {
            const oldestKey = this.cache.keys().next().value;
            const oldestValue = this.cache.get(oldestKey);
            this.cache.delete(oldestKey);
            if (this.onEvict) this.onEvict(oldestValue);
        }
        this.cache.set(key, value);
    }

    /**
     * Check if key exists in cache
     * @param {string} key - Cache key
     * @returns {boolean}
     */
    has(key) {
        return this.cache.has(key);
    }

    /**
     * Get current cache size
     * @returns {number}
     */
    size() {
        return this.cache.size;
    }

    /**
     * Clear the cache
     */
    clear() {
        if (this.onEvict) {
            for (const value of this.cache.values()) {
                this.onEvict(value);
            }
        }
        this.cache.clear();
    }
}
