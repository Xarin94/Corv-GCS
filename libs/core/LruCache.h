// LruCache.h - least-recently-used cache (port of js/core/LRUCache.js).
//
// The terrain keeps satellite textures here (capacity 1500): the evicted value
// is handed to a callback so its GPU resource can be released by its owner.
#pragma once

#include <functional>
#include <list>
#include <unordered_map>
#include <utility>

namespace corv {

template <typename K, typename V>
class LruCache
{
public:
    using Evict = std::function<void(const K &, V &)>;

    explicit LruCache(std::size_t capacity, Evict onEvict = {})
        : m_capacity(capacity), m_onEvict(std::move(onEvict)) {}

    V *get(const K &key)
    {
        auto it = m_index.find(key);
        if (it == m_index.end()) return nullptr;
        m_order.splice(m_order.begin(), m_order, it->second);
        return &it->second->second;
    }

    bool contains(const K &key) const { return m_index.count(key) != 0; }

    void put(const K &key, V value)
    {
        if (auto it = m_index.find(key); it != m_index.end()) {
            it->second->second = std::move(value);
            m_order.splice(m_order.begin(), m_order, it->second);
            return;
        }
        m_order.emplace_front(key, std::move(value));
        m_index[key] = m_order.begin();
        while (m_order.size() > m_capacity) {
            auto &back = m_order.back();
            if (m_onEvict) m_onEvict(back.first, back.second);
            m_index.erase(back.first);
            m_order.pop_back();
        }
    }

    bool remove(const K &key)
    {
        auto it = m_index.find(key);
        if (it == m_index.end()) return false;
        if (m_onEvict) m_onEvict(it->second->first, it->second->second);
        m_order.erase(it->second);
        m_index.erase(it);
        return true;
    }

    void clear()
    {
        if (m_onEvict)
            for (auto &kv : m_order) m_onEvict(kv.first, kv.second);
        m_order.clear();
        m_index.clear();
    }

    std::size_t size() const { return m_order.size(); }
    std::size_t capacity() const { return m_capacity; }

private:
    std::size_t m_capacity;
    Evict m_onEvict;
    std::list<std::pair<K, V>> m_order;
    std::unordered_map<K, typename std::list<std::pair<K, V>>::iterator> m_index;
};

} // namespace corv
