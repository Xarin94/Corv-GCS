// TerrainData.h - terrain as the renderer receives it, without GPU resources
// (the neutral contract of js/render/TerrainData.js and ARCHITETTURA-RENDERING.md).
//
// World: metres, X east, Y up, Z south, from the scene origin. A chunk is
// 1/30 degree of an HGT tile: its heights are the tile's samples at full
// resolution (Int16, voids filled), row 0 north, column 0 west. The vertex at
// (col, row) sits at (x0 + col*dx, h, z0 + row*dz).
//
// A chunk inside the satellite radius may also have a ChunkImage: its
// rectangle of satellite imagery, edge to edge, so the vertex at (col, row)
// samples it at (col, row) / (grid - 1).
#pragma once

#include <QByteArray>
#include <QList>
#include <QMutex>
#include <cstdint>
#include <memory>
#include <vector>

namespace corv {

struct ChunkKey {
    int latBase = 0, lonBase = 0;   // HGT tile (south-west corner)
    int cx = 0, cy = 0;             // column (east) and row (south) of the chunk in the tile
    bool operator==(const ChunkKey &) const = default;
    quint64 packed() const
    {
        return (quint64(quint16(latBase + 90)) << 40) | (quint64(quint16(lonBase + 180)) << 24) | (quint64(quint8(cx)) << 8) | quint8(cy);
    }
};

struct ChunkData {
    ChunkKey key;
    int grid = 0;                   // vertices per side (121 for SRTM1, 41 for SRTM3)
    std::vector<int16_t> heights;   // grid * grid
    double x0 = 0, z0 = 0;          // world position of vertex (0, 0)
    double dx = 0, dz = 0;          // spacing, metres
    int16_t minH = 0, maxH = 0;
    int originEpoch = 0;
};

/// Satellite imagery of one chunk: its lat/lon rectangle resampled to a square
/// power-of-two image (row 0 north, column 0 west), with the whole mip chain.
struct ChunkImage {
    quint64 chunk = 0;               // ChunkKey::packed()
    int size = 0;                    // pixels per side of level 0
    int band = 0;                    // distance band it was built for
    int zoom = 0;                    // tile zoom it was composed from
    bool bc1 = true;                 // BC1 blocks, else RGBA8
    std::vector<QByteArray> mips;    // level 0 first, down to 1 x 1
    qsizetype bytes() const
    {
        qsizetype n = 0;
        for (const QByteArray &m : mips) n += m.size();
        return n;
    }
};

/// Hand-over between the terrain manager (GUI thread) and the renderer (render thread,
/// GUI blocked during synchronize). Adds and removes since the last take.
class TerrainChanges
{
public:
    void add(std::shared_ptr<const ChunkData> c)
    {
        QMutexLocker l(&m_lock);
        m_added.push_back(std::move(c));
    }
    void remove(quint64 key)
    {
        QMutexLocker l(&m_lock);
        m_removed.push_back(key);
    }
    void addImage(std::shared_ptr<const ChunkImage> img)
    {
        QMutexLocker l(&m_lock);
        m_imagesAdded.push_back(std::move(img));
    }
    void removeImage(quint64 chunk)
    {
        QMutexLocker l(&m_lock);
        // An image still waiting for upload goes without ever reaching the GPU
        std::erase_if(m_imagesAdded, [chunk](const auto &i) { return i->chunk == chunk; });
        m_imagesRemoved.push_back(chunk);
    }
    void clearAll()
    {
        QMutexLocker l(&m_lock);
        m_added.clear();
        m_removed.clear();
        m_imagesAdded.clear();
        m_imagesRemoved.clear();
        m_clear = true;
    }
    /// Drop every image (satellite off), keep the chunks.
    void clearImages()
    {
        QMutexLocker l(&m_lock);
        m_imagesAdded.clear();
        m_imagesRemoved.clear();
        m_clearImages = true;
    }
    /// Renderer side. Takes at most maxAdds chunks and images up to budgetBytes
    /// (at least one); the rest stay for the next frame.
    void take(std::vector<std::shared_ptr<const ChunkData>> &added, std::vector<quint64> &removed, bool &clear, std::size_t maxAdds,
              std::vector<std::shared_ptr<const ChunkImage>> &images, std::vector<quint64> &imagesRemoved, bool &clearImages,
              qsizetype budgetBytes)
    {
        QMutexLocker l(&m_lock);
        clear = m_clear;
        m_clear = false;
        clearImages = m_clearImages;
        m_clearImages = false;
        removed.swap(m_removed);
        m_removed.clear();
        const std::size_t n = std::min(maxAdds, m_added.size());
        added.assign(m_added.begin(), m_added.begin() + qsizetype(n));
        m_added.erase(m_added.begin(), m_added.begin() + qsizetype(n));
        imagesRemoved.swap(m_imagesRemoved);
        m_imagesRemoved.clear();
        images.clear();
        qsizetype bytes = 0;
        std::size_t i = 0;
        for (; i < m_imagesAdded.size(); ++i) {
            const qsizetype b = m_imagesAdded[i]->bytes();
            if (i > 0 && bytes + b > budgetBytes) break;
            bytes += b;
            images.push_back(m_imagesAdded[i]);
        }
        m_imagesAdded.erase(m_imagesAdded.begin(), m_imagesAdded.begin() + qsizetype(i));
    }

private:
    QMutex m_lock;
    std::vector<std::shared_ptr<const ChunkData>> m_added;
    std::vector<quint64> m_removed;
    std::vector<std::shared_ptr<const ChunkImage>> m_imagesAdded;
    std::vector<quint64> m_imagesRemoved;
    bool m_clear = false;
    bool m_clearImages = false;
};

} // namespace corv
