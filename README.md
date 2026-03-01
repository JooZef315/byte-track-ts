# byte-track-ts

ByteTrack implementation in TypeScript for object tracking with a small, dependency-light API.

**Features**

1. Pure TypeScript tracking core
2. Simple `Tracker` class interface
3. Works with any detector that provides `xywh`, `conf`, `cls`

**Install**

```bash
//npm
npm install github:JooZef315/byte-track-ts

//yarn
yarn add github:JooZef315/byte-track-ts

//bun
bun add github:JooZef315/byte-track-ts
```

**Quick Start**

```ts
import { Tracker } from "byte-track-ts";

const tracker = new Tracker({
  track_high_thresh: 0.5,
  track_low_thresh: 0.1,
  new_track_thresh: 0.6,
  track_buffer: 30,
  match_thresh: 0.8,
  fuse_score: true,
});

const detections = {
  xywh: [
    [100, 120, 80, 60],
    [220, 140, 90, 70],
  ],
  conf: [0.91, 0.84],
  cls: [2, 2],
};

const tracks = tracker.update(detections);
console.log(tracks);
```

**API**

1. `new Tracker(args)`
2. `update(detections, frame?) -> number[][]`
3. `reset()`

`detections` shape:

```
xywh: number[][]  // [x, y, w, h] per detection (center format)
conf: number[]    // confidence per detection
cls: number[]     // class per detection
xywhr?: number[][] // optional oriented boxes
```

**Track Output**

Each track row is:

```
[x1, y1, x2, y2, track_id, score, class_id, detection_index]
```

**Notes**

1. `track_buffer` directly controls `max_time_lost` (no frame-rate scaling).
2. Provide detector scores calibrated to your threshold settings.
