// RingBuffer.h - fixed-capacity circular buffers (port of js/core/RingBuffer.js).
//
// O(1) push, no allocation after construction, binary search on a sorted
// channel (timestamps). dataBuffer of the Electron version is a
// MultiChannelRingBuffer of 1200 samples and 8 channels.
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace corv {

template <typename T>
class RingBuffer
{
public:
    explicit RingBuffer(std::size_t capacity = 1) : m_data(std::max<std::size_t>(1, capacity)) {}

    void push(const T &v)
    {
        m_data[(m_head + m_size) % m_data.size()] = v;
        if (m_size < m_data.size()) ++m_size;
        else m_head = (m_head + 1) % m_data.size();
    }

    /// i = 0 is the oldest sample.
    const T &at(std::size_t i) const { return m_data[(m_head + i) % m_data.size()]; }
    T &at(std::size_t i) { return m_data[(m_head + i) % m_data.size()]; }
    const T &last() const { return at(m_size - 1); }
    const T &first() const { return at(0); }

    std::size_t size() const { return m_size; }
    std::size_t capacity() const { return m_data.size(); }
    bool empty() const { return m_size == 0; }
    bool full() const { return m_size == m_data.size(); }
    void clear() { m_head = m_size = 0; }

    /// First index whose value is >= target (values must be ascending).
    std::size_t lowerBound(const T &target) const
    {
        std::size_t lo = 0, hi = m_size;
        while (lo < hi) {
            const std::size_t mid = (lo + hi) / 2;
            if (at(mid) < target) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }

    std::vector<T> toVector(std::size_t start = 0, std::size_t count = std::size_t(-1)) const
    {
        std::vector<T> out;
        const std::size_t end = std::min(m_size, count == std::size_t(-1) ? m_size : start + count);
        for (std::size_t i = start; i < end; ++i) out.push_back(at(i));
        return out;
    }

private:
    std::vector<T> m_data;
    std::size_t m_head = 0;
    std::size_t m_size = 0;
};

/// Synchronized channels sharing one timestamp column.
class MultiChannelRingBuffer
{
public:
    MultiChannelRingBuffer(std::size_t capacity, std::size_t channels)
        : m_time(capacity), m_channels(channels, RingBuffer<double>(capacity)) {}

    void push(double timestamp, const std::vector<double> &values)
    {
        m_time.push(timestamp);
        for (std::size_t c = 0; c < m_channels.size(); ++c)
            m_channels[c].push(c < values.size() ? values[c] : 0.0);
    }

    const RingBuffer<double> &channel(std::size_t c) const { return m_channels[c]; }
    std::size_t size() const { return m_time.size(); }

    /// Index range [first, last) of samples with startTime <= t <= endTime.
    std::pair<std::size_t, std::size_t> window(double startTime, double endTime) const
    {
        const std::size_t a = m_time.lowerBound(startTime);
        std::size_t b = a;
        while (b < m_time.size() && m_time.at(b) <= endTime) ++b;
        return {a, b};
    }

    void clear()
    {
        m_time.clear();
        for (auto &c : m_channels) c.clear();
    }

private:
    RingBuffer<double> m_time;
    std::vector<RingBuffer<double>> m_channels;
};

} // namespace corv
