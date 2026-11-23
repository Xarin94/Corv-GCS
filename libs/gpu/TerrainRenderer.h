// TerrainRenderer.h - terrain on QRhi (port of js/engine/three/TerrainRenderer.js).
//
// Per grid size (121 for SRTM1 chunks, 41 for SRTM3): one R16 texture array
// with a layer per chunk, one vertex buffer of (column, row), one index buffer
// per decimation step 1/2/4/8. Each frame the visible chunks (frustum test on
// their box) are grouped by grid, satellite image size and step, and drawn
// instanced: a handful of draw calls for ~500 chunks. The step comes from the
// screen-space error (6 px over the 30.9 m SRTM1 spacing), as lodStepForDistance().
//
// Satellite imagery (ChunkImage) lives in texture arrays too, BC1 with the whole
// mip chain (RGBA8 where the GPU has no BC1); a chunk's instance carries its
// layer, -1 for the height palette. The Electron version drew every textured
// chunk as a mesh of its own; here a textured chunk costs no more than an
// untextured one. Each image size (4096 down to 256) has pages: arrays of a
// fixed number of layers, about 20 MB at most. One more page is made when they
// are full and given back when it empties; nothing already on the GPU is
// uploaded again (an array that doubled cost a frame of 40 to 90 MB).
//
// Uploads per frame are capped: 24 chunks of heights and 2 MB of imagery
// (CHUNKS_PER_FRAME of the Electron version, in bytes as the guide asks). An
// image that is larger goes up in strips over some frames, coarse levels first,
// and the chunk changes to it when it is complete. Render thread only.
#pragma once

#include "scene/GeoZoneRaster.h"
#include "scene/TerrainData.h"
#include "terrain/SatelliteDetail.h"

#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <rhi/qrhi.h>
#include <deque>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace corv {

struct TerrainFrame {
    QMatrix4x4 viewProj;          // with the clip-space correction
    QVector3D camPos;
    float fovDeg = 60;
    float viewportHeight = 1080;
    float fogDensity = 0.00005f;
    QVector3D sunDir{0.3f, 0.8f, 0.2f};
    bool sunlight = true;
    float brightness = 0.85f;
    bool schematic = true;
    float schematicRadius = 30000;
    float triGrid = 0;            // 0..1
    QVector3D triCenter;          // x, z used
    bool lightTheme = false;
    bool zonesOn = false;         // UAS geographical zones painted
    QVector4D zoneMap;            // world x, z of the zones window's north-west corner; 1 / width, 1 / height
    DetailFrame detail;           // satellite imagery at zoom 17..20 around the aircraft
};

class TerrainRenderer
{
public:
    explicit TerrainRenderer(QRhi *rhi);
    ~TerrainRenderer();

    /// Apply the manager's adds / removes (capped per frame).
    void sync(TerrainChanges &changes, QRhiResourceUpdateBatch *u);
    /// Per-frame uniforms and instance data; call before the pass.
    void prepare(const TerrainFrame &f, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u);
    void draw(QRhiCommandBuffer *cb, const QSize &outputSize);
    /// SatelliteDetail's texture operations, in order (allocations, clears, tile uploads).
    void applyDetail(const std::vector<DetailOp> &ops, QRhiResourceUpdateBatch *u);
    /// The UAS zones raster (GeoZoneRaster), uploaded now.
    void setGeoZones(const std::shared_ptr<const GeoZoneRaster> &raster, QRhiResourceUpdateBatch *u);

    bool bc1Supported() const { return m_bc1; }
    int chunkCount() const { return int(m_chunks.size()); }
    int drawCalls() const { return m_drawCalls; }
    int visibleChunks() const { return m_visible; }
    qint64 triangles() const { return m_triangles; }
    int imageCount() const { return int(m_images.size()); }
    qint64 imageBytes() const { return m_imageBytes; }

private:
    struct Draw {
        int sat = 0;        // satellite array key, 0: none
        int step = 0;       // index into the decimation steps
        int first = 0, count = 0;
    };
    struct Grid {
        int size = 0;
        int capacity = 0;
        std::unique_ptr<QRhiTexture> heights;
        std::unique_ptr<QRhiBuffer> vbuf;
        std::unique_ptr<QRhiBuffer> ibuf[4];
        quint32 indexCount[4] = {};
        std::vector<int> freeLayers;
        std::unique_ptr<QRhiBuffer> instances;
        int instanceCapacity = 0;
        // One binding set per satellite array (0: none), with the layout the pipeline was made for
        std::map<int, std::unique_ptr<QRhiShaderResourceBindings>> srbs;
        std::unique_ptr<QRhiGraphicsPipeline> pipeline;
        std::vector<Draw> draws;
    };
    struct Chunk {
        std::shared_ptr<const ChunkData> data;
        int grid = 0;
        int layer = -1;
    };
    struct SatArray {       // a page
        int size = 0;
        bool bc1 = true;
        int layers = 0;
        std::unique_ptr<QRhiTexture> tex;
        std::vector<int> freeLayers;
    };
    struct ChunkSat {
        std::shared_ptr<const ChunkImage> image;
        int array = 0;      // SatArray key
        int layer = -1;
    };
    struct SatUpload {
        ChunkSat sat;
        int level = 0;      // the next level to send, down to 0
        int row = 0;        // and its next row (of blocks for BC1)
    };

    Grid *gridFor(int size, QRhiResourceUpdateBatch *u);
    void ensurePipeline(Grid &g, QRhiRenderPassDescriptor *rp, int sampleCount);
    QRhiShaderResourceBindings *srbFor(Grid &g, int satKey);
    std::unique_ptr<QRhiShaderResourceBindings> makeSrb(QRhiTexture *heights, QRhiTexture *sat);
    void growHeights(Grid &g, QRhiResourceUpdateBatch *u);
    void releaseChunk(Chunk &c);
    void addImage(const std::shared_ptr<const ChunkImage> &img);
    bool allocateLayer(ChunkSat &s);
    void releaseLayer(ChunkSat &s);
    void removeImage(quint64 chunk);
    void cancelUpload(quint64 chunk);
    void pumpUploads(QRhiResourceUpdateBatch *u);
    void trimPages();
    static constexpr int kMaxPages = 256;
    static int satKey(int size, bool bc1, int page) { return (size * 2 + (bc1 ? 1 : 0)) * kMaxPages + page; }

    QRhi *m_rhi;
    QShader m_vs, m_fs;
    std::unique_ptr<QRhiBuffer> m_ubuf;
    std::unique_ptr<QRhiSampler> m_sampler;
    std::unique_ptr<QRhiSampler> m_satSampler;
    std::unique_ptr<QRhiTexture> m_noSat;       // one grey layer bound where no imagery is drawn
    bool m_noSatReady = false;
    std::unique_ptr<QRhiTexture> m_zoneCov, m_zoneFloor;   // UAS zones (GeoZoneRaster)
    std::unique_ptr<QRhiSampler> m_zoneSampler;
    bool m_zonesReady = false;
    std::unique_ptr<QRhiTexture> m_detail[SatelliteDetail::kLevels];   // created when a level is first used
    bool m_mipsDirty[SatelliteDetail::kLevels] = {};
    int m_nextMips = 0;
    std::unique_ptr<QRhiTexture> m_noDetail;
    std::unique_ptr<QRhiSampler> m_detailSampler;
    std::unique_ptr<QRhiTexture> m_zeroTile;    // a transparent tile: what clears a slot, copied on the GPU
    bool m_zeroReady = false;
    std::unique_ptr<QRhiShaderResourceBindings> m_layoutSrb;   // never drawn with: what the pipelines are made for
    std::map<int, Grid> m_grids;
    std::unordered_map<quint64, Chunk> m_chunks;
    std::map<int, SatArray> m_sat;
    std::unordered_map<quint64, ChunkSat> m_images;
    std::deque<SatUpload> m_uploads;
    qint64 m_imageBytes = 0;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 1;
    QRhiTexture::Format m_heightFormat = QRhiTexture::R16;
    bool m_bc1 = false;
    int m_maxLayers = 2048;
    int m_drawCalls = 0, m_visible = 0;
    qint64 m_triangles = 0;
};

} // namespace corv
