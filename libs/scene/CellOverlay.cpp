#include "scene/CellOverlay.h"

#include <QMutex>

namespace corv::cellOverlay {

namespace {
QMutex g_lock;
CellOverlayPtr g_overlay = std::make_shared<const CellOverlay>();
} // namespace

CellOverlayPtr get()
{
    QMutexLocker l(&g_lock);
    return g_overlay;
}

void set(CellOverlayPtr overlay)
{
    QMutexLocker l(&g_lock);
    g_overlay = overlay ? std::move(overlay) : std::make_shared<const CellOverlay>();
}

} // namespace corv::cellOverlay
