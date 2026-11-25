// Road standards: classes, vehicles and the design rules the kernel enforces.
// All slopes are ratios (rise / run). Widths are metres (full paved width).

export const ROAD_CLASSES = {
  primary: {
    id: 'primary',
    label: '主干道 / Primary',
    width: 11.0,
    minRadius: 42,
    designSpeed: 80 / 3.6,
    maxGrade: 0.055,
    maxGradeChange: 0.09, // change in grade per 30 m
    maxCurvatureRate: 0.012, // change of curvature (1/m) per metre, comfort bound
    shoulder: 1.2,
    sidewalk: 1.5,
    median: 0.0,
    surface: 'asphalt',
    cutSlope: 0.55,
    fillSlope: 0.5,
  },
  collector: {
    id: 'collector',
    label: '次干道 / Collector',
    width: 8.0,
    minRadius: 24,
    designSpeed: 50 / 3.6,
    maxGrade: 0.075,
    maxGradeChange: 0.11,
    maxCurvatureRate: 0.02,
    shoulder: 1.0,
    sidewalk: 1.2,
    median: 0.0,
    surface: 'asphalt',
    cutSlope: 0.6,
    fillSlope: 0.55,
  },
  local: {
    id: 'local',
    label: '支路 / Local',
    width: 6.2,
    minRadius: 14,
    designSpeed: 30 / 3.6,
    maxGrade: 0.09,
    maxGradeChange: 0.14,
    maxCurvatureRate: 0.05,
    shoulder: 0.6,
    sidewalk: 0.8,
    median: 0.0,
    surface: 'asphalt',
    cutSlope: 0.7,
    fillSlope: 0.6,
  },
  path: {
    id: 'path',
    label: '园路 / Path',
    width: 3.0,
    minRadius: 7,
    designSpeed: 15 / 3.6,
    maxGrade: 0.12,
    maxGradeChange: 0.2,
    maxCurvatureRate: 0.1,
    shoulder: 0.2,
    sidewalk: 0,
    median: 0,
    surface: 'gravel',
    cutSlope: 0.9,
    fillSlope: 0.8,
  },
};

export const VEHICLES = {
  sedan: {
    id: 'sedan',
    label: '轿车 / Sedan',
    length: 4.7,
    width: 1.85,
    wheelbase: 2.8,
    track: 1.58,
    frontOverhang: 0.9,
    rearOverhang: 1.0,
    maxSteer: 33 * Math.PI / 180,
    rideHeight: 0.16,
  },
  boxTruck: {
    id: 'boxTruck',
    label: '箱式货车 / Box truck',
    length: 8.6,
    width: 2.45,
    wheelbase: 5.2,
    track: 2.0,
    frontOverhang: 1.3,
    rearOverhang: 2.1,
    maxSteer: 30 * Math.PI / 180,
    rideHeight: 0.3,
  },
  bus: {
    id: 'bus',
    label: '公交 / Bus',
    length: 12.0,
    width: 2.55,
    wheelbase: 6.0,
    track: 2.1,
    frontOverhang: 2.6,
    rearOverhang: 3.4,
    maxSteer: 38 * Math.PI / 180,
    rideHeight: 0.34,
  },
};

// How much extra lane width a curve needs (AASHTO-style, simplified).
// Returns metres added per side beyond the class half width.
export function curveWidening(radius, vehicle) {
  if (!isFinite(radius) || radius <= 1) return 0;
  const v = vehicle || VEHICLES.sedan;
  const R = radius;
  // Off-tracking of the rear axle / swept path.
  const trackSweep = (v.wheelbase * v.wheelbase) / (2 * R);
  const frontSweep = Math.sqrt(R * R + v.wheelbase * v.wheelbase) - R;
  return Math.max(0, trackSweep * 0.35 + frontSweep * 0.15);
}

export function classById(id) {
  return ROAD_CLASSES[id] || ROAD_CLASSES.local;
}

export function vehicleById(id) {
  return VEHICLES[id] || VEHICLES.sedan;
}

// Buffered vehicle footprint used for swept-corridor checks.
export function vehicleHalfWidth(v) {
  return v.width / 2;
}

export const TOLERANCES = {
  seamGap: 0.06, // metres between roadway edge and graded terrain
  zFight: 0.015,
  gradeJumpPerSample: 0.22, // sudden vertical step fail
  corridorWarnFraction: 0.97,
};

// Radius of the outside front corner when the vehicle is at its tightest turn
// (used to size a junction pad that contains a low-speed turn). The front corner
// sits one wheelbase + front overhang ahead of the rear axle and half a width to
// the outside of the turn centre.
export function parkingTurnRadius(vehicleId) {
  const v = vehicleById(vehicleId);
  const r = v.wheelbase / Math.tan(v.maxSteer * 0.95);
  return Math.hypot(r + v.width * 0.5, v.wheelbase + v.frontOverhang);
}
