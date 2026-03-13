# Flight Plans

This folder contains flight plan files in JSON format that can be loaded into the Navigation Display (ND).

## File Format

Each flight plan is a JSON file with the following structure:

```json
{
    "id": "ROUTE-ID",
    "name": "Human readable name",
    "departure": {
        "icao": "ICAO",
        "name": "Airport Name"
    },
    "arrival": {
        "icao": "ICAO",
        "name": "Airport Name"
    },
    "waypoints": [
        { "id": "WPT", "lat": 45.0000, "lon": 11.0000, "type": "airport|vor|ndb|waypoint" }
    ]
}
```

## Waypoint Types

- `airport` - Airport (shown with airport symbol)
- `vor` - VOR navaid (shown with VOR symbol)
- `ndb` - NDB navaid (shown with NDB symbol)
- `waypoint` - Standard waypoint (shown with triangle)

## Adding New Flight Plans

1. Create a new `.json` file in this folder
2. Follow the format above
3. The route will automatically appear in the ND flight plan selector

## Available Routes

- **IBK-BLQ.json** - Innsbruck (LOWI) to Bologna (LIPE)
- **BLQ-MXP.json** - Bologna (LIPE) to Milan Malpensa (LIMC)
- **DEMO.json** - Demo route over the Alps
