// Built-in sample projects.
// Each scene exercises a different part of the kernel:
// - arterial: trunk with a true T junction and a branch, plus forbidden zones
// - slope: road crossing a ridge through a saddle, with a spur that climbs
// - multibranch: a compact 4-way junction between two nearby 2-arm nodes

const baseTerrain = {
  id: 'meadow',
  seed: 4242,
  baseHeight: 0,
  roughness: 1.8,
  noiseScale: 160,
  features: [],
  obstacles: [],
  bounds: { minX: -260, maxX: 260, minZ: -260, maxZ: 260 },
};

export const SAMPLES = {
  arterial: () => ({
    id: 'arterial',
    name: '主支路 / Trunk + Branch (T)',
    seed: 20260401,
    vehicleId: 'sedan',
    controls: {
      resolution: 220,
      junctionRadius: 9,
      allowReroute: true,
      allowWiden: true,
      widenPreference: 0.6,
      taper: 10,
      maxWiden: 5.5,
    },
    terrain: {
      ...baseTerrain,
      id: 'arterial-terrain',
      seed: 8801,
      features: [
        { type: 'hill', x: -120, z: -140, radius: 90, height: 8 },
        { type: 'hill', x: 165, z: 135, radius: 110, height: 6 },
        { type: 'berm', a: { x: -200, y: 185 }, b: { x: 120, y: 165 }, height: 2.5, width: 24 },
      ],
      obstacles: [
        {
          id: 'lake',
          kind: 'water',
          label: '湖面',
          shape: 'polygon',
          points: [
            { x: 60, y: -130 },
            { x: 125, y: -150 },
            { x: 170, y: -95 },
            { x: 135, y: -55 },
            { x: 75, y: -70 },
          ],
        },
        { id: 'radio-tower', kind: 'structure', label: '无线电塔', x: -35, y: -105, radius: 14 },
      ],
    },
    nodes: [
      { id: 'W', x: -200, y: 20, kind: 'junction' },
      { id: 'J', x: 0, y: 5, kind: 'junction' },
      { id: 'E', x: 185, y: -8, kind: 'junction' },
      { id: 'N', x: 20, y: 135, kind: 'junction' },
    ],
    connections: [
      { id: 'trunk-west', name: '主干道 西段', a: 'W', b: 'J', classId: 'primary' },
      { id: 'trunk-east', name: '主干道 东段', a: 'J', b: 'E', classId: 'primary' },
      {
        id: 'branch-north',
        name: '北向支路',
        a: 'J',
        b: 'N',
        classId: 'local',
        controlPoints: [{ x: 10, y: 70 }],
      },
    ],
  }),

  slope: () => ({
    id: 'slope',
    name: '跨坡地 / Ridge Saddle Crossing',
    seed: 20260402,
    vehicleId: 'boxTruck',
    controls: {
      resolution: 220,
      junctionRadius: 9,
      allowReroute: true,
      allowWiden: true,
      widenPreference: 0.45,
      taper: 9,
      maxWiden: 6,
    },
    terrain: {
      ...baseTerrain,
      id: 'slope-terrain',
      seed: 9917,
      baseHeight: 2,
      features: [
        // An east-west ridge split by a low saddle around x in [-70, -10].
        // The spur crosses the ridge through the saddle; tapering the segment
        // ends is what actually lowers the pass (a gap alone still leaves a
        // full-height shoulder).
        {
          type: 'ridge',
          a: { x: -260, y: 142 },
          b: { x: -70, y: 126 },
          height: 18,
          width: 80,
          taper: 0.3,
        },
        {
          type: 'ridge',
          a: { x: -10, y: 114 },
          b: { x: 260, y: 132 },
          height: 17,
          width: 80,
          taper: 0.3,
        },
        // Cross-slope: ground rises toward the north-west.
        { type: 'slopePlane', dir: { x: -0.35, y: 0.94 }, grade: 0.035 },
        { type: 'plateau', x: -185, y: -40, radius: 40, falloff: 38, height: 7 },
        { type: 'valley', x: 120, y: -40, radius: 70, depth: 4 },
      ],
      obstacles: [
        {
          id: 'boulder',
          kind: 'rock',
          label: '巨石带',
          shape: 'polygon',
          points: [
            { x: 70, y: 105 },
            { x: 108, y: 92 },
            { x: 122, y: 120 },
            { x: 88, y: 138 },
          ],
        },
        { id: 'pond', kind: 'water', label: '水塘', x: -120, y: 135, radius: 24 },
      ],
    },
    nodes: [
      { id: 'W', x: -215, y: 5, kind: 'junction' },
      { id: 'P', x: 5, y: 20, kind: 'junction' },
      { id: 'E', x: 205, y: 10, kind: 'junction' },
      { id: 'N', x: -50, y: 130, kind: 'junction' },
    ],
    connections: [
      {
        id: 'cross-ridge',
        name: '跨脊-西段',
        a: 'W',
        b: 'P',
        classId: 'collector',
        // Cross the saddle high, then settle into P from the west so the
        // junction has a clean T (west / east / north bearings).
        controlPoints: [{ x: -150, y: 60 }, { x: -90, y: 52 }, { x: -45, y: 16 }],
      },
      {
        id: 'cross-ridge-east',
        name: '跨脊-东段',
        a: 'P',
        b: 'E',
        classId: 'collector',
        controlPoints: [{ x: 85, y: 35 }],
      },
      {
        id: 'ridge-spur',
        name: '山脊支线',
        a: 'P',
        b: 'N',
        classId: 'local',
        controlPoints: [{ x: -8, y: 78 }],
      },
    ],
  }),

  multibranch: () => ({
    id: 'multibranch',
    name: '近距离多岔 / Compact Multi-Branch',
    seed: 20260403,
    vehicleId: 'bus',
    controls: {
      resolution: 200,
      junctionRadius: 8,
      allowReroute: true,
      allowWiden: true,
      widenPreference: 0.75,
      taper: 8,
      maxWiden: 5,
    },
    terrain: {
      ...baseTerrain,
      id: 'multibranch-terrain',
      seed: 3311,
      roughness: 1.1,
      features: [
        { type: 'hill', x: 145, y: 160, radius: 85, height: 5 },
        { type: 'valley', x: -150, y: -95, radius: 60, depth: 4 },
      ],
      obstacles: [
        { id: 'mast', kind: 'structure', label: '天线阵', x: -38, y: 128, radius: 10 },
      ],
    },
    nodes: [
      { id: 'W', x: -200, y: 55, kind: 'junction' },
      { id: 'C1', x: -38, y: 55, kind: 'junction' },
      { id: 'C2', x: 0, y: 55, kind: 'junction' },
      { id: 'C3', x: 38, y: 55, kind: 'junction' },
      { id: 'E', x: 190, y: 55, kind: 'junction' },
      { id: 'N', x: 0, y: 190, kind: 'junction' },
      { id: 'S2', x: 0, y: -95, kind: 'junction' },
    ],
    connections: [
      { id: 'w-approach', name: '西侧进场', a: 'W', b: 'C1', classId: 'collector' },
      { id: 'c1-c2', name: 'C1-C2 短连接', a: 'C1', b: 'C2', classId: 'collector' },
      { id: 'c2-c3', name: 'C2-C3 短连接', a: 'C2', b: 'C3', classId: 'local' },
      { id: 'c3-east', name: '东侧出口', a: 'C3', b: 'E', classId: 'collector' },
      { id: 'c2-north', name: '北向支线', a: 'C2', b: 'N', classId: 'local' },
      { id: 'c2-south', name: '南向支线', a: 'C2', b: 'S2', classId: 'local' },
    ],
  }),
};

export function sampleProject(id) {
  const f = SAMPLES[id] || SAMPLES.arterial;
  return f();
}

export const SAMPLE_IDS = Object.keys(SAMPLES);
