/**
 * lte-bands.js - LTE carrier frequency from the EARFCN
 *
 * 3GPP TS 36.101 table 5.7.3-1, downlink: F_DL = F_DL_low + 0.1 (N_DL - N_Offs-DL)
 * for N_Offs-DL <= N_DL <= N_DL_max. The modem reports the downlink EARFCN;
 * for TDD bands (33-53) uplink and downlink share it.
 */

// [band, F_DL_low MHz, N_Offs-DL, N_DL max]
const BANDS = [
    [1, 2110, 0, 599], [2, 1930, 600, 1199], [3, 1805, 1200, 1949], [4, 2110, 1950, 2399],
    [5, 869, 2400, 2649], [6, 875, 2650, 2749], [7, 2620, 2750, 3449], [8, 925, 3450, 3799],
    [9, 1844.9, 3800, 4149], [10, 2110, 4150, 4749], [11, 1475.9, 4750, 4949], [12, 729, 5010, 5179],
    [13, 746, 5180, 5279], [14, 758, 5280, 5379], [17, 734, 5730, 5849], [18, 860, 5850, 5999],
    [19, 875, 6000, 6149], [20, 791, 6150, 6449], [21, 1495.9, 6450, 6599], [22, 3510, 6600, 7399],
    [23, 2180, 7500, 7699], [24, 1525, 7700, 8039], [25, 1930, 8040, 8689], [26, 859, 8690, 9039],
    [27, 852, 9040, 9209], [28, 758, 9210, 9659], [29, 717, 9660, 9769], [30, 2350, 9770, 9869],
    [31, 462.5, 9870, 9919], [32, 1452, 9920, 10359],
    [33, 1900, 36000, 36199], [34, 2010, 36200, 36349], [35, 1850, 36350, 36949], [36, 1930, 36950, 37549],
    [37, 1910, 37550, 37749], [38, 2570, 37750, 38249], [39, 1880, 38250, 38649], [40, 2300, 38650, 39649],
    [41, 2496, 39650, 41589], [42, 3400, 41590, 43589], [43, 3600, 43590, 45589], [44, 703, 45590, 46589],
    [45, 1447, 46590, 46789], [46, 5150, 46790, 54539], [47, 5855, 54540, 55239], [48, 3550, 55240, 56739],
    [49, 3550, 56740, 58239], [50, 1432, 58240, 59089], [51, 1427, 59090, 59139], [52, 3300, 59140, 60139],
    [53, 2483.5, 60140, 60254],
    [65, 2110, 65536, 66435], [66, 2110, 66436, 67335], [67, 738, 67336, 67535], [68, 753, 67536, 67835],
    [69, 2570, 67836, 68335], [70, 1995, 68336, 68585], [71, 617, 68586, 68935], [72, 461, 68936, 68985],
    [73, 460, 68986, 69035], [74, 1475, 69036, 69465], [75, 1432, 69466, 70315], [76, 1427, 70316, 70365],
    [85, 728, 70366, 70545], [87, 420, 70546, 70595], [88, 422, 70596, 70645],
];

/**
 * Downlink carrier of an EARFCN.
 * @returns {{band: number, mhz: number}|null} null when the EARFCN is in no band
 */
export function earfcnToFrequency(earfcn) {
    if (!Number.isInteger(earfcn) || earfcn < 0) return null;
    for (const [band, low, offs, max] of BANDS) {
        if (earfcn >= offs && earfcn <= max) return { band, mhz: Math.round((low + 0.1 * (earfcn - offs)) * 10) / 10 };
    }
    return null;
}

/** "1870.0 MHz" for the panels, "B3" when the EARFCN is unknown, fallback when neither is. */
export function frequencyText(earfcn, band, fallback = '---') {
    // EARFCN 0 is a real band 1 channel, but a modem that knows nothing sends 0 and band 0
    const f = earfcn || band === 1 ? earfcnToFrequency(earfcn) : null;
    return f ? `${f.mhz.toFixed(1)} MHz` : (band ? `B${band}` : fallback);
}
