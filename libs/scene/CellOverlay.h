// CellOverlay.h - the LTE cell sites the maps draw (getCellOverlay of
// js/maps/CellTowerLayer.js): live only while an LTEtelem module reports its
// cell, the serving site and the others within 10 km of the vehicle, and the
// colour of the link by its quality. Written by LteController, read by the
// mini-map (render thread) and the 3D view: an immutable snapshot behind a
// mutex, replaced as a whole.
#pragma once

#include <QColor>
#include <QString>
#include <memory>
#include <vector>

namespace corv {

struct CellRecord {
    unsigned eci = 0, tac = 0;
    double lat = 0, lon = 0;
    int range = 0, samples = 0;
    QString src;                 // ocid, ocid-2024, mls, beacondb, ocid-api, or a file name
};

/// The cells of one eNB of one network: the mast is the samples-weighted mean.
struct CellSite {
    int mcc = 0, mnc = 0;
    unsigned enb = 0;
    double lat = 0, lon = 0;
    std::vector<CellRecord> cells;
};

struct CellOverlay {
    bool live = false;
    std::vector<CellSite> sites;     // drawn: the serving one and those within 10 km
    int serving = -1;                // index into sites, -1: not found
    int ownMcc = 0, ownMnc = 0;      // the serving network: its sites are drawn brighter
    QColor linkColor;                // linkQuality(): green, orange, red
    int version = 0;                 // changes whenever the sites are picked again
};
using CellOverlayPtr = std::shared_ptr<const CellOverlay>;

namespace cellOverlay {
/// Any thread.
CellOverlayPtr get();
void set(CellOverlayPtr overlay);
} // namespace cellOverlay

} // namespace corv
