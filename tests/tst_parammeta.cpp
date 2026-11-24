// What ArduPilot documents about the parameters: the table of a release over the newest one.
#include "vehicle/ParamMeta.h"

#include <QTest>

using namespace corv;
using namespace Qt::StringLiterals;

namespace {
const QByteArray kXml = R"(<?xml version="1.0" encoding="utf-8"?>
<!-- Generated from git tag Copter-4.6.3 -->
<paramfile><vehicles><parameters name="ArduCopter">
  <param humanName="My ground station number" name="ArduCopter:SYSID_MYGCS" documentation="Allows   restricting radio overrides" user="Advanced">
    <field name="Range">1 255</field><field name="Increment">1</field>
  </param>
</parameters></vehicles><libraries><parameters name="ARMING_">
  <param humanName="Arm Checks" name="ARMING_CHECK" documentation="Checks" user="Standard">
    <bitmask><bit code="1">Barometer</bit><bit code="0">All</bit></bitmask>
    <field name="Bitmask">0:All,1:Barometer</field>
  </param>
  <param humanName="Throttle failsafe" name="FS_THR_ENABLE" documentation="x" user="Standard">
    <values><value code="1">Enabled always RTL</value><value code="0">Disabled</value></values>
    <field name="RebootRequired">True</field><field name="Units">s</field>
  </param>
</parameters></libraries></paramfile>)";
} // namespace

class TestParamMeta : public QObject
{
    Q_OBJECT
private slots:
    void xml()
    {
        QHash<QString, ParamInfo> out;
        QString err;
        QVERIFY2(ParamMeta::parseXml(kXml, out, &err), qPrintable(err));
        QCOMPARE(out.size(), 3);
        const ParamInfo gcs = out.value(u"SYSID_MYGCS"_s);   // without the vehicle's prefix
        QCOMPARE(gcs.displayName, u"My ground station number"_s);
        QCOMPARE(gcs.description, u"Allows restricting radio overrides"_s);
        QVERIFY(gcs.hasRange && gcs.low == 1 && gcs.high == 255 && gcs.increment == 1);
        const ParamInfo arm = out.value(u"ARMING_CHECK"_s);
        QVERIFY(arm.isBitmask());
        QCOMPARE(arm.bitsLabel(3), u"All, Barometer"_s);
        const ParamInfo fs = out.value(u"FS_THR_ENABLE"_s);
        QCOMPARE(fs.valueLabel(1), u"Enabled always RTL"_s);
        QCOMPARE(fs.values.first().first, 0.0);
        QVERIFY(fs.rebootRequired && !fs.readOnly);
        QCOMPARE(fs.units, u"s"_s);
    }

    void notATable()
    {
        QHash<QString, ParamInfo> out;
        QString err;
        QVERIFY(!ParamMeta::parseXml("<html><body>404</body></html>", out, &err));
        QVERIFY(!err.isEmpty());
        QVERIFY(!ParamMeta::parseXml("<paramfile><vehicles>", out, &err));
        QVERIFY(!ParamMeta::parse("not json", out, nullptr, &err));
    }

    void versions()
    {
        ParamMeta &m = ParamMeta::instance();
        QCOMPARE(m.version(u"Copter"_s), QString());
        m.setVersion(u"Copter"_s, u"4.6.3"_s);
        QCOMPARE(m.version(u"Copter"_s), u"4.6.3"_s);
        QCOMPARE(m.version(u"Plane"_s), QString());
        m.setVersion(u"Copter"_s, QString());
    }
};

QTEST_GUILESS_MAIN(TestParamMeta)
#include "tst_parammeta.moc"
