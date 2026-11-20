#include "terrain/Hgt.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"

#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QThreadPool>
#include <QtEndian>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

QString HgtTile::name(int latBase, int lonBase)
{
    return u"%1%2%3%4"_s
        .arg(latBase >= 0 ? 'N'_L1 : 'S'_L1)
        .arg(std::abs(latBase), 2, 10, '0'_L1)
        .arg(lonBase >= 0 ? 'E'_L1 : 'W'_L1)
        .arg(std::abs(lonBase), 3, 10, '0'_L1);
}

std::shared_ptr<HgtTile> HgtTile::load(const QString &path, int latBase, int lonBase, QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) *error = f.errorString();
        return nullptr;
    }
    const qint64 bytes = f.size();
    int size = 0;
    if (bytes == 3601LL * 3601 * 2) size = 3601;
    else if (bytes == 1201LL * 1201 * 2) size = 1201;
    else {
        if (error) *error = u"not an SRTM size: %1 bytes"_s.arg(bytes);
        return nullptr;
    }
    auto t = std::make_shared<HgtTile>();
    t->latBase = latBase;
    t->lonBase = lonBase;
    t->size = size;
    t->data.resize(std::size_t(size) * size);
    if (f.read(reinterpret_cast<char *>(t->data.data()), bytes) != bytes) {
        if (error) *error = u"short read"_s;
        return nullptr;
    }
    // Big-endian on disk
    for (int16_t &v : t->data) v = qFromBigEndian(v);
    return t;
}

std::optional<double> HgtTile::elevation(double lat, double lon) const
{
    const double fy = (latBase + 1 - lat) * (size - 1);
    const double fx = (lon - lonBase) * (size - 1);
    if (fy < 0 || fx < 0 || fy > size - 1 || fx > size - 1) return std::nullopt;
    const int r0 = std::min(int(fy), size - 2), c0 = std::min(int(fx), size - 2);
    const double ty = fy - r0, tx = fx - c0;
    const int16_t a = at(r0, c0), b = at(r0, c0 + 1), c = at(r0 + 1, c0), d = at(r0 + 1, c0 + 1);
    if (a == kHgtVoid || b == kHgtVoid || c == kHgtVoid || d == kHgtVoid) return std::nullopt;
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

HgtStore &HgtStore::instance()
{
    static HgtStore s;
    return s;
}

QStringList HgtStore::searchDirs() const
{
    QStringList dirs;
    const QString install = DataRoot::installDir();
    dirs << install + u"/topography"_s << install + u"/topo"_s;
    dirs << DataRoot::terrainDir();
    {
        QMutexLocker l(&m_lock);
        dirs << m_extraDirs;
    }
#ifdef CORV_SOURCE_DIR
    dirs << QStringLiteral(CORV_SOURCE_DIR) + u"/topo"_s;
#endif
    for (const QString &d : qEnvironmentVariable("CORV_TOPO_DIRS").split(';'_L1, Qt::SkipEmptyParts)) dirs << d;
    return dirs;
}

QString HgtStore::find(int latBase, int lonBase) const
{
    const QString want = HgtTile::name(latBase, lonBase) + u".hgt"_s;
    for (const QString &d : searchDirs()) {
        const QDir dir(d);
        if (!dir.exists()) continue;
        // Names match whatever their case
        for (const QString &f : dir.entryList({u"*.hgt"_s, u"*.HGT"_s}, QDir::Files))
            if (f.compare(want, Qt::CaseInsensitive) == 0) return dir.filePath(f);
    }
    return {};
}

bool HgtStore::missing(int latBase, int lonBase) const
{
    QMutexLocker l(&m_lock);
    return m_missing.contains(key(latBase, lonBase));
}

bool HgtStore::loading(int latBase, int lonBase) const
{
    QMutexLocker l(&m_lock);
    return m_loading.contains(key(latBase, lonBase));
}

void HgtStore::forget(int latBase, int lonBase)
{
    QMutexLocker l(&m_lock);
    m_missing.remove(key(latBase, lonBase));
}

void HgtStore::addSearchDir(const QString &dir)
{
    {
        QMutexLocker l(&m_lock);
        if (dir.isEmpty() || m_extraDirs.contains(dir)) return;
        m_extraDirs << dir;
        // A tile not found before may be in the new folder
        m_missing.clear();
    }
    LOG_I("terrain") << "HGT folder added:" << dir;
}

int HgtStore::availableCount() const
{
    QSet<QString> names;
    for (const QString &d : searchDirs()) {
        const QDir dir(d);
        if (!dir.exists()) continue;
        for (const QString &f : dir.entryList({u"*.hgt"_s, u"*.HGT"_s}, QDir::Files)) names.insert(f.toUpper());
    }
    return int(names.size());
}

std::shared_ptr<const HgtTile> HgtStore::get(int latBase, int lonBase)
{
    const int k = key(latBase, lonBase);
    {
        QMutexLocker l(&m_lock);
        if (auto it = m_tiles.constFind(k); it != m_tiles.constEnd()) return *it;
        if (m_loading.contains(k) || m_missing.contains(k)) return nullptr;
        m_loading.insert(k);
    }
    QThreadPool::globalInstance()->start([this, k, latBase, lonBase] {
        const QString path = find(latBase, lonBase);
        std::shared_ptr<HgtTile> t;
        QString err;
        QElapsedTimer timer;
        timer.start();
        if (!path.isEmpty()) t = HgtTile::load(path, latBase, lonBase, &err);
        {
            QMutexLocker l(&m_lock);
            m_loading.remove(k);
            if (t) m_tiles.insert(k, t);
            else m_missing.insert(k);
        }
        if (t) {
            LOG_I("terrain") << "tile" << HgtTile::name(latBase, lonBase) << "loaded from" << path << "in" << timer.elapsed() << "ms";
            emit tileLoaded(latBase, lonBase);
        } else if (!path.isEmpty()) {
            LOG_W("terrain") << "tile" << path << "not usable:" << err;
        } else {
            LOG_D("terrain") << "tile" << HgtTile::name(latBase, lonBase) << "not found";
        }
    });
    return nullptr;
}

std::optional<double> HgtStore::elevation(double lat, double lon)
{
    const int latBase = int(std::floor(lat)), lonBase = int(std::floor(lon));
    const auto t = get(latBase, lonBase);
    return t ? t->elevation(lat, lon) : std::nullopt;
}

std::optional<HgtStore::Water> HgtStore::waterAt(double lat, double lon)
{
    const int latBase = int(std::floor(lat)), lonBase = int(std::floor(lon));
    const auto t = get(latBase, lonBase);
    if (!t) return std::nullopt;
    const int size = t->size;
    const int r = int(std::lround((1 - (lat - latBase)) * (size - 1)));
    const int c = int(std::lround((lon - lonBase) * (size - 1)));
    auto at = [&](int dr, int dc) { return t->at(std::clamp(r + dr, 0, size - 1), std::clamp(c + dc, 0, size - 1)); };
    const int16_t h = at(0, 0);
    if (h <= -12000) return std::nullopt;   // a void: no data
    if (h < 0) return Water{0, t->elevation(lat, lon)};
    static const int offs[12][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}, {0, 2}, {0, -2}, {2, 0}, {-2, 0}};
    for (const auto &o : offs)
        if (at(o[0], o[1]) != h) return std::nullopt;
    return Water{double(h), std::nullopt};
}

} // namespace corv
