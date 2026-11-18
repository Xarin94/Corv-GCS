# LTE cells shipped with CORV GCS

`cells-lte.csv.gz` holds the LTE cells of Italy, San Marino and Vatican City
(MCC 222, 292, 225), one per row, sorted by mcc, mnc, eci:

    mcc,mnc,tac,eci,lat,lon,range,samples,updated,src

`lat`/`lon` are estimates from crowd measurements, not surveyed mast positions.
`updated` is in Unix seconds. `src` names the source of the row:

| src         | source                                                        | licence             |
|-------------|---------------------------------------------------------------|---------------------|
| `ocid`      | OpenCelliD export of the date in `manifest.json` (cells seen in the 18 months before it) | CC BY-SA 4.0 |
| `ocid-2024` | OpenCelliD, June 2024 copy published as PMTiles by UN Smart Maps on Source Cooperative | CC BY-SA 4.0 |
| `mls`       | Mozilla Location Service, final full cell export (March 2024)  | Public domain (CC0) |

When a cell is in more than one source the row comes from the first in this table.

Cell tower data from OpenCelliD (https://opencellid.org), licensed under
CC BY-SA 4.0 (https://creativecommons.org/licenses/by-sa/4.0/). Contains data
from the Mozilla Location Service, public domain. This file, a merge of the two,
is distributed under CC BY-SA 4.0.

Rebuilt with `scripts/build-cell-bundle.js` (instructions at its top).
