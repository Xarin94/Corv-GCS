// UpdateCheck.h - which published release the running GCS should move to
// (the update check at start, UpdateController).
//
// The releases come from the GitHub REST list of a repository
// (GET /repos/{owner}/{repo}/releases): tag, draft and prerelease flags,
// notes, page and the attached files with their SHA-256 digest. A release is
// a candidate when its tag is a newer semantic version than the running one
// and it carries a package for this system: the NSIS setup on Windows, a DMG
// on macOS (universal, or the one for this processor), an AppImage on Linux.
// Pre-releases count while the running version is one, or on request.
// Pure logic, no network.
#pragma once

#include <QJsonArray>
#include <QList>
#include <QString>
#include <QStringList>
#include <optional>

namespace corv::update {

struct Version {
    int major = 0, minor = 0, patch = 0;
    QStringList pre;   // "alpha", "7" for 2.0.0-alpha.7; empty for a final release
    bool isPrerelease() const { return !pre.isEmpty(); }
    QString toString() const;
};

/// "v2.0.0-alpha.7", "2.0.0", "1.8.8" (build metadata after '+' ignored); none otherwise.
std::optional<Version> parseVersion(QString text);
/// Semantic-version precedence: < 0, 0, > 0.
int compare(const Version &a, const Version &b);

enum class Platform { Windows, MacOS, Linux };
/// What this build is: system and processor ("x86_64", "arm64").
Platform currentPlatform();
QString currentArch();

struct Asset {
    QString name, url;
    qint64 size = 0;
    QString sha256;   // lower-case hex from GitHub's "digest", empty when not given
};

struct Release {
    QString tag, name, notes, page;
    Version version;
    bool prerelease = false;
    QList<Asset> assets;
};

/// The package of a release for a system, or none.
std::optional<Asset> packageFor(const Release &r, Platform p, const QString &arch);

/// The newest release newer than `current` with a package for the system.
std::optional<Release> newest(const QJsonArray &releases, const Version &current, bool includePre, Platform p,
                              const QString &arch);

} // namespace corv::update
