/**
 * RingBuffer.js - High-performance circular buffer implementation
 * O(1) push/pop operations for time-series data
 */

export class RingBuffer {
    /**
     * Create a ring buffer with fixed capacity
     * @param {number} capacity - Maximum number of elements
     * @param {boolean} [useTypedArray=true] - Use Float64Array for numeric data
     */
    constructor(capacity, useTypedArray = true) {
        this.capacity = capacity;
        this.buffer = useTypedArray ? new Float64Array(capacity) : new Array(capacity);
        this.head = 0;      // Next write position
        this.tail = 0;      // First valid element
        this.length = 0;    // Current number of elements
        this.isTyped = useTypedArray;
    }

    /**
     * Push a value to the buffer
     * @param {number} value - Value to add
     */
    push(value) {
        this.buffer[this.head] = value;
        this.head = (this.head + 1) % this.capacity;

        if (this.length < this.capacity) {
            this.length++;
        } else {
            // Buffer full, advance tail (overwrites oldest)
            this.tail = (this.tail + 1) % this.capacity;
        }
    }

    /**
     * Get value at index (0 = oldest, length-1 = newest)
     * @param {number} index - Logical index
     * @returns {number} Value at index
     */
    get(index) {
        if (index < 0 || index >= this.length) return NaN;
        const realIndex = (this.tail + index) % this.capacity;
        return this.buffer[realIndex];
    }

    /**
     * Get the most recent value
     * @returns {number} Latest value
     */
    getLast() {
        if (this.length === 0) return NaN;
        const lastIdx = (this.head - 1 + this.capacity) % this.capacity;
        return this.buffer[lastIdx];
    }

    /**
     * Get the oldest value
     * @returns {number} First value
     */
    getFirst() {
        if (this.length === 0) return NaN;
        return this.buffer[this.tail];
    }

    /**
     * Convert to regular array (for Plotly compatibility)
     * @param {number} [startIndex=0] - Starting logical index
     * @param {number} [count] - Number of elements (default: all)
     * @returns {number[]} Array of values in order
     */
    toArray(startIndex = 0, count = undefined) {
        const len = count !== undefined ? Math.min(count, this.length - startIndex) : this.length - startIndex;
        if (len <= 0) return [];

        const result = new Array(len);
        for (let i = 0; i < len; i++) {
            result[i] = this.get(startIndex + i);
        }
        return result;
    }

    /**
     * Get a slice of the buffer as array
     * @param {number} start - Start index (can be negative from end)
     * @param {number} [end] - End index (exclusive)
     * @returns {number[]} Sliced array
     */
    slice(start, end) {
        const len = this.length;

        // Handle negative indices
        let s = start < 0 ? Math.max(0, len + start) : Math.min(start, len);
        let e = end === undefined ? len : (end < 0 ? Math.max(0, len + end) : Math.min(end, len));

        if (e <= s) return [];

        const result = new Array(e - s);
        for (let i = 0; i < result.length; i++) {
            result[i] = this.get(s + i);
        }
        return result;
    }

    /**
     * Find index of first element matching condition (searching from start)
     * @param {function} predicate - (value, index) => boolean
     * @returns {number} Index or -1 if not found
     */
    findIndex(predicate) {
        for (let i = 0; i < this.length; i++) {
            if (predicate(this.get(i), i)) return i;
        }
        return -1;
    }

    /**
     * Find index of first element >= target value (for sorted data like timestamps)
     * Uses binary search for O(log n) performance
     * @param {number} target - Target value
     * @returns {number} Index of first element >= target, or length if all smaller
     */
    lowerBound(target) {
        let lo = 0;
        let hi = this.length;

        while (lo < hi) {
            const mid = (lo + hi) >>> 1;
            if (this.get(mid) < target) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    /**
     * Clear all data
     */
    clear() {
        this.head = 0;
        this.tail = 0;
        this.length = 0;
        // Optionally zero the buffer for typed arrays
        if (this.isTyped) {
            this.buffer.fill(0);
        }
    }

    /**
     * Check if buffer is empty
     * @returns {boolean}
     */
    isEmpty() {
        return this.length === 0;
    }

    /**
     * Check if buffer is full
     * @returns {boolean}
     */
    isFull() {
        return this.length === this.capacity;
    }
}

/**
 * Multi-channel ring buffer for parallel time-series data
 * All channels share the same timeline
 */
export class MultiChannelRingBuffer {
    /**
     * Create multi-channel ring buffer
     * @param {number} capacity - Maximum number of samples
     * @param {number} channels - Number of data channels
     */
    constructor(capacity, channels) {
        this.capacity = capacity;
        this.channels = channels;
        this.timestamps = new RingBuffer(capacity, true);
        this.data = [];
        for (let i = 0; i < channels; i++) {
            this.data.push(new RingBuffer(capacity, true));
        }
    }

    /**
     * Push a sample with timestamp and multi-channel values
     * @param {number} timestamp - Timestamp
     * @param {number[]} values - Array of channel values
     */
    push(timestamp, values) {
        this.timestamps.push(timestamp);
        for (let i = 0; i < this.channels; i++) {
            this.data[i].push(values[i] !== undefined ? values[i] : NaN);
        }
    }

    /**
     * Get current length
     * @returns {number}
     */
    get length() {
        return this.timestamps.length;
    }

    /**
     * Get timestamps as array
     * @returns {number[]}
     */
    getTimestamps() {
        return this.timestamps.toArray();
    }

    /**
     * Get channel data as array
     * @param {number} channel - Channel index
     * @returns {number[]}
     */
    getChannel(channel) {
        if (channel < 0 || channel >= this.channels) return [];
        return this.data[channel].toArray();
    }

    /**
     * Get all data for a time window
     * @param {number} startTime - Start timestamp (inclusive)
     * @param {number} endTime - End timestamp (inclusive)
     * @returns {{ timestamps: number[], channels: number[][] }}
     */
    getWindow(startTime, endTime) {
        const startIdx = this.timestamps.lowerBound(startTime);
        const endIdx = this.timestamps.lowerBound(endTime + 1);

        const timestamps = this.timestamps.slice(startIdx, endIdx);
        const channels = [];
        for (let i = 0; i < this.channels; i++) {
            channels.push(this.data[i].slice(startIdx, endIdx));
        }

        return { timestamps, channels };
    }

    /**
     * Get last timestamp
     * @returns {number}
     */
    getLastTimestamp() {
        return this.timestamps.getLast();
    }

    /**
     * Get first timestamp
     * @returns {number}
     */
    getFirstTimestamp() {
        return this.timestamps.getFirst();
    }

    /**
     * Clear all channels
     */
    clear() {
        this.timestamps.clear();
        for (const ch of this.data) {
            ch.clear();
        }
    }
}

export default { RingBuffer, MultiChannelRingBuffer };
