/**
 * corv-mavlink.js - MAVLink messages of our own, outside the stock dialects
 *
 * node-mavlink drops a frame whose msgid has no magic number (CRC_EXTRA), so a
 * custom message needs both its class in the decode registry and its magic
 * number in every MavLinkPacketSplitter (createSplitter below).
 *
 * CORV_CELL_INFO (52200) comes from an LTEtelem cellular module, sysid 51 /
 * compid 68 like its RADIO_STATUS: the LTE cell the module is camped on, the
 * strongest neighbour cells and the number of cell changes since boot.
 * Definition: LTEtelem tools/mavlink/corv.xml (CRC_EXTRA 119, 63 bytes).
 */

const { MavLinkData, MavLinkPacketField, MavLinkPacketSplitter, MSG_ID_MAGIC_NUMBER } = require('node-mavlink');

class CorvCellInfo extends MavLinkData {
    constructor() {
        super();
        this.eci = 0;
        this.tac = 0;
        this.earfcn = 0;
        this.handovers = 0;
        this.nbEarfcn = [];
        this.mcc = 0;
        this.mnc = 0;
        this.pci = 0;
        this.rsrp = 0;
        this.rssi = 0;
        this.nbPci = [];
        this.nbRsrp = [];
        this.rsrq = 0;
        this.sinr = 0;
        this.band = 0;
        this.state = 0;
        this.nNeighbours = 0;
    }
}
CorvCellInfo.MSG_ID = 52200;
CorvCellInfo.MSG_NAME = 'CORV_CELL_INFO';
CorvCellInfo.PAYLOAD_LENGTH = 63;
CorvCellInfo.MAGIC_NUMBER = 119;
CorvCellInfo.FIELDS = [
    new MavLinkPacketField('eci', 'eci', 0, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('tac', 'tac', 4, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('earfcn', 'earfcn', 8, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('handovers', 'handovers', 12, false, 4, 'uint32_t', ''),
    new MavLinkPacketField('nb_earfcn', 'nbEarfcn', 16, false, 4, 'uint32_t[]', '', 4),
    new MavLinkPacketField('mcc', 'mcc', 32, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('mnc', 'mnc', 34, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('pci', 'pci', 36, false, 2, 'uint16_t', ''),
    new MavLinkPacketField('rsrp', 'rsrp', 38, false, 2, 'int16_t', 'dBm'),
    new MavLinkPacketField('rssi', 'rssi', 40, false, 2, 'int16_t', 'dBm'),
    new MavLinkPacketField('nb_pci', 'nbPci', 42, false, 2, 'uint16_t[]', '', 4),
    new MavLinkPacketField('nb_rsrp', 'nbRsrp', 50, false, 2, 'int16_t[]', 'dBm', 4),
    new MavLinkPacketField('rsrq', 'rsrq', 58, false, 1, 'int8_t', 'dB'),
    new MavLinkPacketField('sinr', 'sinr', 59, false, 1, 'int8_t', 'dB'),
    new MavLinkPacketField('band', 'band', 60, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('state', 'state', 61, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('n_neighbours', 'nNeighbours', 62, false, 1, 'uint8_t', ''),
];

const REGISTRY = { [CorvCellInfo.MSG_ID]: CorvCellInfo };

const MAGIC_NUMBERS = { ...MSG_ID_MAGIC_NUMBER };
for (const clazz of Object.values(REGISTRY)) MAGIC_NUMBERS[clazz.MSG_ID] = clazz.MAGIC_NUMBER;

/** A splitter that also passes (and CRC-checks) the messages above. */
function createSplitter() {
    return new MavLinkPacketSplitter({}, { magicNumbers: MAGIC_NUMBERS });
}

module.exports = { CorvCellInfo, REGISTRY, MAGIC_NUMBERS, createSplitter };
