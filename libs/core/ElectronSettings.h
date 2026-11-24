// ElectronSettings.h - the settings of CORV GCS 1.x.
//
// The Electron version kept them in the browser's localStorage, which Chromium
// stores in a LevelDB folder of the user's profile ("Local Storage/leveldb"). This
// reads that folder (the write-ahead log and the sorted tables, plain or Snappy),
// without writing to it, and keeps the newest value of every key of the page.
// The keys are the ones settings.json uses; importInto() copies those 2.x knows.
#pragma once

#include <QDateTime>
#include <QHash>
#include <QString>
#include <QStringList>

namespace corv::electronSettings {

/// The 1.x profile of this user with the latest localStorage, or empty.
QString findStore();
/// When a store was last written.
QDateTime lastUsed(const QString &dir);

/// Every key of the page's localStorage (origin file://) with its newest value.
QHash<QString, QString> read(const QString &dir, QString *error = nullptr);

/// The keys 2.x takes over.
const QStringList &knownKeys();
/// Copy the known keys into Settings; returns how many.
int importInto(const QHash<QString, QString> &store);

/// LevelDB's block compression. Empty on malformed data. (Tests.)
QByteArray snappyDecompress(const QByteArray &in);

} // namespace corv::electronSettings
