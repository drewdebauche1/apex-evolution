Centerline extraction tools

Usage:

Generate a synthetic test track and extract centerline:

```bash
py tools/centerline_extract.py --test
```

Extract centerline from an existing image:

```bash
py tools/centerline_extract.py --in path/to/track.png --out waypoints.json
```

Output is JSON: {"points":[[x,y],...]}

To convert into the game's `racingGuide` format, write a small converter that maps image coords to track coords and inserts the points as the `guide` waypoints.
