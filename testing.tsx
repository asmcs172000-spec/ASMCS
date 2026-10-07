import * as satellite from "satellite.js";
import type { TleEntry } from "./satelliteRangeUtils";
import type { RectangleBounds } from "./useTargetPicker";
import { FREQUENCY_PRESETS, type FrequencySegment, type FrequencyPresetId } from "./FrequencyAndTime";
import { rectangleAreaKm2 } from "./areaFromBounds";

/** Single source of truth for the AOI area (spherical formula in areaFromBounds.ts). */
export { rectangleAreaKm2 };

/* ============================================================================
 * STRIP-CAPTURE PLANNER — merged version
 *
 * KEPT FROM THE ORIGINAL PLANNER
 *   - 9 representative AOI points trigger the AOS; the point that triggers it
 *     becomes the "capture point" that anchors the along/cross frame.
 *   - Multi-satellite formation: one leader pass, other satellites swept with
 *     their own offsets from the leader.
 *   - Per-segment, per-satellite forward-swept swath rectangles.
 *   - Coverage is measured on the REAL AOI with an equal-area cell grid; each
 *     cell records WHEN it was first covered. Later passes only count cells
 *     not already covered by earlier passes.
 *   - formatCoveragePercent() never rounds a small contribution to "0%".
 *
 * NEW IN THIS VERSION
 *   1. TIMES PER PASS (four, not two):
 *        dummyAos     = missionStart - aosLead (default 5 min)   [signal acquisition]
 *        missionStart = the real AOS: first moment an AOI point meets ALL limits
 *                       (elevation, slant range, tilt). t = 0 of the capture
 *                       timeline. The old `aos` field no longer exists.
 *        missionEnd   = missionStart + plan length, i.e. WHEN THE FREQUENCY PLAN ENDS
 *                       (plan length = end of the last frequency segment, capped
 *                       at maxMissionSeconds, 10 min). It is NOT cut at LOS.
 *        los          = REAL loss of signal: the first moment after missionStart
 *                       when NO AOI point meets the limits any more (replaces the
 *                       old fixed "AOS + 10 min"). LOS can fall before missionEnd;
 *                       the pass is then flagged (truncatedByLos) and any cell
 *                       the sensor reaches while outside the limits is NOT counted.
 *   2. TILT LOGIC. Tilt (off-nadir) is now a limit next to elevation and range.
 *        "flat"  : tilt = acos(H / slant range)         (range = H / cos(tilt))
 *        "exact" : true angle between nadir and the line of sight
 *      Defaults: H = 550 km, tilt 5..72.2 deg.  72.2 deg == acos(550/1800),
 *      so under the flat model max tilt and max range 1800 km are the same limit.
 *   3. IMAGEABILITY CHECK PER CELL. A cell only counts as covered if, at the
 *      moment the swept footprint reaches it, the real satellite meets the
 *      elevation / range / tilt limits for that cell. Coverage after LOS or
 *      outside the limits is therefore never counted.
 *   4. Equal-area cells: sample rows are spaced evenly in sin(latitude), so
 *      every cell really has the same area.
 * ==========================================================================*/

/* ------------------------------------------------------------------ */
/* Public configuration                                               */
/* ------------------------------------------------------------------ */

/* ===================== 1. LIMITS  (edit these) ===================== */
export const ORBIT_HEIGHT_KM = 550;     // satellite orbit height H
export const MIN_ELEVATION_DEG = 10;    // elevation of the AOI point as seen from the satellite
export const MIN_TILT_DEG = 5;          // off-nadir tilt, lower limit
export const MAX_TILT_DEG = 72.2;       // off-nadir tilt, upper limit  (= acos(550 / 1800) in the flat model)
export const MIN_RANGE_KM = 551;        // slant range, lower limit  (UI slider default)
export const MAX_RANGE_KM = 1800;       // slant range, upper limit  (UI slider default)

/* ===================== 2. TIMING  (edit these) ===================== */
export const AOS_LEAD_SECONDS = 5 * 60;      // dummy AOS = missionStart - this
export const MAX_MISSION_SECONDS = 10 * 60;  // plan length is capped here

/* ===================== 3. MODELS ===================== */
export const TILT_MODEL: TiltModel = "flat";    // "flat": tilt = acos(H / range)   "exact": true angle to nadir
export const SWATH_MODEL: SwathModel = "fixed"; // "fixed": preset swath   "cos": swath / cos(tilt)

/* Names kept so existing imports keep working. */
export const STRIP_ELEVATION_MIN_DEG = MIN_ELEVATION_DEG;
export const DEFAULT_MAX_MISSION_SECONDS = MAX_MISSION_SECONDS;
export const DEFAULT_AOS_LEAD_SECONDS = AOS_LEAD_SECONDS;
/** Fraction at (or above) which the AOI counts as fully covered. */
export const FULL_COVERAGE_FRACTION = 0.995;

export type TiltModel = "flat" | "exact";
export type SwathModel = "fixed" | "cos";

export type StripLimits = {
  minElevationDeg: number;
  minTiltDeg: number;
  maxTiltDeg: number;
  tiltModel: TiltModel;
  /** Orbit height H used by the flat tilt formula (km). */
  orbitHeightKm: number;
  /** Dummy AOS = mission start minus this lead (seconds). */
  aosLeadSeconds: number;
  /** Hard cap on mission length (seconds). */
  maxMissionSeconds: number;
  /** "fixed": swath = preset size. "cos": swath / cos(tilt) at the capture point (capped x4). */
  swathModel: SwathModel;
};

export const DEFAULT_STRIP_LIMITS: StripLimits = {
  minElevationDeg: MIN_ELEVATION_DEG,
  minTiltDeg: MIN_TILT_DEG,
  maxTiltDeg: MAX_TILT_DEG,
  tiltModel: TILT_MODEL,
  orbitHeightKm: ORBIT_HEIGHT_KM,
  aosLeadSeconds: AOS_LEAD_SECONDS,
  maxMissionSeconds: MAX_MISSION_SECONDS,
  swathModel: SWATH_MODEL,
};

const EARTH_RADIUS_KM = 6371.0088;
const COARSE_STEP_SECONDS = 30;
const REFINEMENT_MS = 500;
const LOS_SCAN_STEP_SECONDS = 10;
const LOS_HORIZON_SECONDS = 20 * 60;
const FORMATION_WINDOW_SECONDS = 180;
const FORMATION_STEP_SECONDS = 5;
const MAX_PASSES_SAFETY = 60;
const MARGIN_KM_PER_DEG = 10;
const SWATH_SCALE_CAP = 4;

/** Approximate number of equal-area cells used to measure AOI coverage. */
const TARGET_SAMPLE_COUNT = 24000;
const MAX_SAMPLES_PER_AXIS = 400;

const toDeg = (r: number) => (r * 180) / Math.PI;
const toRad = (d: number) => (d * Math.PI) / 180;
const clamp = (x: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, x));

/* ------------------------------------------------------------------ */
/* Types shared with the UI and the animation                         */
/* ------------------------------------------------------------------ */

type Vec3 = [number, number, number];
type LatLon = { lat: number; lon: number };

export type AoiCapturePoint = {
  pointIndex: number;
  lat: number;
  lon: number;
  elevationDeg: number;
  slantRangeKm: number;
  tiltDeg: number;
};

export type StripFootprint = {
  satelliteName: string;
  /** [0] alongMin/crossMin  [1] alongMin/crossMax  [2] alongMax/crossMax  [3] alongMax/crossMin */
  corners: LatLon[];
  /** Mission seconds at which the sensor reaches the two along-track ends. */
  tStartSec: number;
  tEndSec: number;
};

export type SegmentCoverage = {
  segmentId: string;
  presetId: FrequencyPresetId;
  startSeconds: number;
  durationSeconds: number;
  swathKm: number;
  footprints: StripFootprint[];
  /** Cells whose earliest coverage happened in this segment (no double counting). */
  areaKm2: number;
  coveredAreaKm2: number;
  coveredFraction: number;
  /** Same, but only cells not covered by earlier passes. */
  newlyCoveredFraction: number;
};

export type StripPass = {
  satelliteName: string;

  /** The four times of a pass. */
  dummyAos: Date;
  missionStart: Date;
  missionEnd: Date;
  los: Date;

  plannedSeconds: number;
  missionSeconds: number;
  /** true when LOS happens before the plan ends (capture after LOS is not counted). */
  truncatedByLos: boolean;

  newlyCoveredFraction: number;
  newlyCoveredAreaKm2: number;
  passCoveredFraction: number;
  passCoveredAreaKm2: number;
  cumulativeCoveredFraction: number;

  segmentBreakdown: SegmentCoverage[];
  /** Fraction of the AOI covered by this pass vs. mission second (index = second). */
  coverageCurve: number[];

  capturePoint: AoiCapturePoint;
  entrySideLabel: string;
  exitSideLabel: string;
  orientation: {
    headingDeg: number;
    rectangleTrackAngleDeg: number;
    crossTrackWidthKm: number;
    alongTrackLengthKm: number;
  } | null;

  /** Limits actually seen on the covered cells. */
  geometry: {
    tiltDeg: [number, number];
    rangeKm: [number, number];
    elevationDeg: [number, number];
    swathScale: number;
  } | null;

  otherSatellites: { name: string; missionStart: Date | null; offsetMs: number | null }[];
};

export type StripCapturePlanResult = {
  bounds: RectangleBounds;
  passes: StripPass[];
  fullyCovered: boolean;
  totalCoveredFraction: number;
  aoiAreaKm2: number;
  sampleCount: number;
  limits: StripLimits & { minRangeKm: number; maxRangeKm: number };
};

/* ------------------------------------------------------------------ */
/* Percentage formatting                                              */
/* ------------------------------------------------------------------ */

/**
 * Formats a 0..1 fraction as a percentage WITHOUT rounding small values to 0%.
 *   0.0023 -> "0.23%"   0.00004 -> "<0.01%"   0.037 -> "3.7%"
 *   0.47 -> "47%"       0.9996 -> "99.9%"     1 -> "100%"
 */
export function formatCoveragePercent(fraction: number): string {
  const pct = fraction * 100;
  if (!Number.isFinite(pct) || pct <= 0) return "0%";
  if (pct >= 100) return "100%";
  if (pct >= 99.5) return `${(Math.floor(pct * 10) / 10).toFixed(1)}%`;
  if (pct < 0.01) return "<0.01%";
  if (pct < 1) return `${pct.toFixed(2)}%`;
  if (pct < 10) return `${pct.toFixed(1)}%`;
  return `${pct.toFixed(0)}%`;
}

/* ------------------------------------------------------------------ */
/* Effective limits (for showing in the UI)                           */
/* ------------------------------------------------------------------ */

/**
 * Combines the tilt, range and elevation limits into the window that really
 * applies. Example (H = 550, flat model): tilt 5..72.2 deg  <->  range 552.1..1800 km.
 */
export function effectiveStripLimits(
  minRangeKm: number,
  maxRangeKm: number,
  limits: StripLimits = DEFAULT_STRIP_LIMITS,
) {
  const H = limits.orbitHeightKm;
  const R = EARTH_RADIUS_KM;
  const r = R + H;
  const flat = limits.tiltModel === "flat";

  const tiltAtRange = (p: number) =>
    flat
      ? toDeg(Math.acos(Math.min(1, H / p)))
      : toDeg(Math.acos(clamp((r * r + p * p - R * R) / (2 * r * p), -1, 1)));
  const rangeAtTilt = (t: number) => {
    const a = toRad(t);
    if (flat) return H / Math.cos(a);
    return r * Math.cos(a) - Math.sqrt(Math.max(0, R * R - r * r * Math.sin(a) ** 2));
  };
  const e = toRad(limits.minElevationDeg);
  const rangeAtMinEl = -R * Math.sin(e) + Math.sqrt((R * Math.sin(e)) ** 2 + 2 * R * H + H * H);

  const hi: [string, number][] = [
    ["max tilt", limits.maxTiltDeg],
    ["max range", tiltAtRange(maxRangeKm)],
    ["min elevation", tiltAtRange(rangeAtMinEl)],
  ];
  const lo: [string, number][] = [
    ["min tilt", limits.minTiltDeg],
    ["min range", tiltAtRange(minRangeKm)],
  ];
  const hiPick = hi.reduce((a, b) => (b[1] < a[1] ? b : a));
  const loPick = lo.reduce((a, b) => (b[1] > a[1] ? b : a));
  return {
    tiltMinDeg: loPick[1],
    tiltMaxDeg: hiPick[1],
    rangeMinKm: rangeAtTilt(loPick[1]),
    rangeMaxKm: rangeAtTilt(hiPick[1]),
    tiltMinBoundBy: loPick[0],
    tiltMaxBoundBy: hiPick[0],
  };
}

/* ------------------------------------------------------------------ */
/* Spherical helpers                                                  */
/* ------------------------------------------------------------------ */

function haversineDistanceKm(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const phi1 = toRad(lat1), phi2 = toRad(lat2);
  const dPhi = toRad(lat2 - lat1);
  const dLambda = toRad(lon2 - lon1);
  const a = Math.sin(dPhi / 2) ** 2 + Math.cos(phi1) * Math.cos(phi2) * Math.sin(dLambda / 2) ** 2;
  return EARTH_RADIUS_KM * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function initialBearingDeg(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const phi1 = toRad(lat1), phi2 = toRad(lat2);
  const dLambda = toRad(lon2 - lon1);
  const y = Math.sin(dLambda) * Math.cos(phi2);
  const x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLambda);
  return (toDeg(Math.atan2(y, x)) + 360) % 360;
}

/* ------------------------------------------------------------------ */
/* Satellite cache + propagation                                      */
/* ------------------------------------------------------------------ */

const satrecCache = new Map<string, satellite.SatRec>();
function getSatrec(tle: TleEntry): satellite.SatRec {
  const key = `${tle.tle1}|${tle.tle2}`;
  const cached = satrecCache.get(key);
  if (cached) return cached;
  const rec = satellite.twoline2satrec(tle.tle1, tle.tle2);
  satrecCache.set(key, rec);
  return rec;
}

type EciFrame = { position: satellite.EciVec3<number>; gmst: satellite.GMSTime; ecf: Vec3 };

function propagateFrame(satrec: satellite.SatRec, t: Date): EciFrame | null {
  const pv = satellite.propagate(satrec, t);
  if (!pv?.position || typeof pv.position === "boolean") return null;
  const gmst = satellite.gstime(t);
  const e = satellite.eciToEcf(pv.position, gmst);
  return { position: pv.position, gmst, ecf: [e.x, e.y, e.z] };
}

function subpointFromFrame(frame: EciFrame): LatLon {
  const geo = satellite.eciToGeodetic(frame.position, frame.gmst);
  return { lat: satellite.degreesLat(geo.latitude), lon: satellite.degreesLong(geo.longitude) };
}

/* ------------------------------------------------------------------ */
/* Look geometry: elevation, slant range, tilt                        */
/* ------------------------------------------------------------------ */

type PointGeo = LatLon & { ecf: Vec3; up: Vec3 };

function makePointGeo(lat: number, lon: number): PointGeo {
  const phi = toRad(lat), lam = toRad(lon);
  const e = satellite.geodeticToEcf({ latitude: phi, longitude: lam, height: 0 });
  return {
    lat, lon,
    ecf: [e.x, e.y, e.z],
    up: [Math.cos(phi) * Math.cos(lam), Math.cos(phi) * Math.sin(lam), Math.sin(phi)],
  };
}

type LookMetrics = { elevationDeg: number; rangeKm: number; tiltDeg: number };

function lookMetrics(sat: Vec3, p: PointGeo, limits: StripLimits): LookMetrics {
  const dx = sat[0] - p.ecf[0], dy = sat[1] - p.ecf[1], dz = sat[2] - p.ecf[2];
  const range = Math.hypot(dx, dy, dz);
  const elevationDeg = toDeg(Math.asin(clamp((dx * p.up[0] + dy * p.up[1] + dz * p.up[2]) / range, -1, 1)));
  let tiltDeg: number;
  if (limits.tiltModel === "flat") {
    tiltDeg = toDeg(Math.acos(Math.min(1, limits.orbitHeightKm / range)));
  } else {
    const sn = Math.hypot(sat[0], sat[1], sat[2]);
    tiltDeg = toDeg(Math.acos(clamp((dx * sat[0] + dy * sat[1] + dz * sat[2]) / (range * sn), -1, 1)));
  }
  return { elevationDeg, rangeKm: range, tiltDeg };
}

type Gate = { limits: StripLimits; minRangeKm: number; maxRangeKm: number };

function meetsLimits(m: LookMetrics, g: Gate): boolean {
  return (
    m.elevationDeg >= g.limits.minElevationDeg &&
    m.rangeKm >= g.minRangeKm && m.rangeKm <= g.maxRangeKm &&
    m.tiltDeg >= g.limits.minTiltDeg && m.tiltDeg <= g.limits.maxTiltDeg
  );
}

/* ------------------------------------------------------------------ */
/* AOS search (9 representative points) and LOS                       */
/* ------------------------------------------------------------------ */

function representativePoints(bounds: RectangleBounds): PointGeo[] {
  const midLat = (bounds.north + bounds.south) / 2;
  const midLon = (bounds.east + bounds.west) / 2;
  const raw: LatLon[] = [
    { lat: bounds.north, lon: bounds.west }, { lat: bounds.north, lon: midLon }, { lat: bounds.north, lon: bounds.east },
    { lat: midLat, lon: bounds.west }, { lat: midLat, lon: midLon }, { lat: midLat, lon: bounds.east },
    { lat: bounds.south, lon: bounds.west }, { lat: bounds.south, lon: midLon }, { lat: bounds.south, lon: bounds.east },
  ];
  return raw.map((p) => makePointGeo(p.lat, p.lon));
}

const POINT_LABELS = [
  "NW corner", "north edge", "NE corner",
  "west edge", "center", "east edge",
  "SW corner", "south edge", "SE corner",
];

function anyPointQualifies(frame: EciFrame, pts: PointGeo[], gate: Gate): boolean {
  for (const p of pts) if (meetsLimits(lookMetrics(frame.ecf, p, gate.limits), gate)) return true;
  return false;
}

function qualifyingPoints(frame: EciFrame, pts: PointGeo[], gate: Gate): AoiCapturePoint[] {
  const out: AoiCapturePoint[] = [];
  for (let i = 0; i < pts.length; i++) {
    const m = lookMetrics(frame.ecf, pts[i], gate.limits);
    if (meetsLimits(m, gate)) {
      out.push({ pointIndex: i, lat: pts[i].lat, lon: pts[i].lon, elevationDeg: m.elevationDeg, slantRangeKm: m.rangeKm, tiltDeg: m.tiltDeg });
    }
  }
  return out;
}

/** The point that JUST became qualifying (smallest margin to a limit) triggers the AOS. */
function pickTriggerPoint(atHi: AoiCapturePoint[], atLo: AoiCapturePoint[], gate: Gate): AoiCapturePoint | null {
  if (atHi.length === 0) return null;
  const loIdx = new Set(atLo.map((p) => p.pointIndex));
  const fresh = atHi.filter((p) => !loIdx.has(p.pointIndex));
  const pool = fresh.length > 0 ? fresh : atHi;
  const L = gate.limits;
  const margin = (p: AoiCapturePoint) =>
    Math.min(
      p.slantRangeKm - gate.minRangeKm,
      gate.maxRangeKm - p.slantRangeKm,
      (p.elevationDeg - L.minElevationDeg) * MARGIN_KM_PER_DEG,
      (p.tiltDeg - L.minTiltDeg) * MARGIN_KM_PER_DEG,
      (L.maxTiltDeg - p.tiltDeg) * MARGIN_KM_PER_DEG,
    );
  return pool.reduce((best, p) => (margin(p) < margin(best) ? p : best));
}

function qualifiesAt(satrec: satellite.SatRec, t: Date, pts: PointGeo[], gate: Gate): boolean {
  const frame = propagateFrame(satrec, t);
  return frame ? anyPointQualifies(frame, pts, gate) : false;
}

function bisectCrossing(
  satrec: satellite.SatRec, pts: PointGeo[], loTime: Date, hiTime: Date, gate: Gate,
): { missionStart: Date; capturePoint: AoiCapturePoint | null } {
  let lo = loTime.getTime();
  let hi = hiTime.getTime();
  while (hi - lo > REFINEMENT_MS) {
    const mid = Math.floor((lo + hi) / 2);
    if (qualifiesAt(satrec, new Date(mid), pts, gate)) hi = mid; else lo = mid;
  }
  const missionStart = new Date(hi);
  const hiFrame = propagateFrame(satrec, missionStart);
  const loFrame = propagateFrame(satrec, new Date(lo));
  const atHi = hiFrame ? qualifyingPoints(hiFrame, pts, gate) : [];
  const atLo = loFrame ? qualifyingPoints(loFrame, pts, gate) : [];
  return { missionStart, capturePoint: pickTriggerPoint(atHi, atLo, gate) };
}

type MissionStartCandidate = {
  tle: TleEntry;
  satrec: satellite.SatRec;
  missionStart: Date;
  capturePoint: AoiCapturePoint;
};

/** Lazily yields AOS candidates in time order (single continuous scan of the window). */
function* scanMissionStartCandidates(
  satRecs: { tle: TleEntry; satrec: satellite.SatRec }[],
  pts: PointGeo[], start: Date, end: Date, gate: Gate, stepSeconds: number,
): Generator<MissionStartCandidate> {
  const prevQualified = new Map<string, boolean>();
  satRecs.forEach(({ tle }) => prevQualified.set(tle.name, false));
  let prevTime: Date | null = null;

  for (let t = new Date(start); t <= end; t = new Date(t.getTime() + stepSeconds * 1000)) {
    let best: MissionStartCandidate | null = null;

    for (const { tle, satrec } of satRecs) {
      const frame = propagateFrame(satrec, t);
      const q = frame ? anyPointQualifies(frame, pts, gate) : false;

      if (q && !prevQualified.get(tle.name)) {
        let missionStart: Date;
        let capturePoint: AoiCapturePoint | null;
        if (prevTime) {
          const refined = bisectCrossing(satrec, pts, prevTime, t, gate);
          missionStart = refined.missionStart;
          capturePoint = refined.capturePoint;
        } else {
          missionStart = new Date(t);
          capturePoint = frame ? pickTriggerPoint(qualifyingPoints(frame, pts, gate), [], gate) : null;
        }
        if (capturePoint && (!best || missionStart.getTime() < best.missionStart.getTime())) {
          best = { tle, satrec, missionStart, capturePoint };
        }
      }
      prevQualified.set(tle.name, q);
    }

    if (best) yield best;
    prevTime = new Date(t);
  }
}

/**
 * REAL LOS: the last moment, after the AOS, at which some AOI point still meets
 * every limit (elevation, range, tilt). Replaces the old fixed "AOS + 10 min".
 */
function findLos(satrec: satellite.SatRec, pts: PointGeo[], missionStart: Date, gate: Gate): Date {
  const stepMs = LOS_SCAN_STEP_SECONDS * 1000;
  let lastOk = missionStart.getTime();
  for (let ms = stepMs; ms <= LOS_HORIZON_SECONDS * 1000; ms += stepMs) {
    const t = missionStart.getTime() + ms;
    if (qualifiesAt(satrec, new Date(t), pts, gate)) { lastOk = t; continue; }
    let lo = lastOk;
    let hi = t;
    while (hi - lo > REFINEMENT_MS) {
      const mid = Math.floor((lo + hi) / 2);
      if (qualifiesAt(satrec, new Date(mid), pts, gate)) lo = mid; else hi = mid;
    }
    return new Date(lo);
  }
  return new Date(lastOk);
}

/** When does another satellite qualify around the leader's AOS? (for the "other satellites" list) */
function findOtherMissionStart(satrec: satellite.SatRec, pts: PointGeo[], leaderMissionStart: Date, gate: Gate): Date | null {
  const t0 = leaderMissionStart.getTime() - FORMATION_WINDOW_SECONDS * 1000;
  const t1 = leaderMissionStart.getTime() + FORMATION_WINDOW_SECONDS * 1000;
  const step = FORMATION_STEP_SECONDS * 1000;
  let prev: number | null = null;
  for (let t = t0; t <= t1; t += step) {
    const q = qualifiesAt(satrec, new Date(t), pts, gate);
    if (q) {
      if (prev === null) return new Date(t);
      return bisectCrossing(satrec, pts, new Date(prev), new Date(t), gate).missionStart;
    }
    prev = t;
  }
  return null;
}

/* ------------------------------------------------------------------ */
/* AOI coverage samples (TRUE equal-area grid over the real AOI)       */
/* ------------------------------------------------------------------ */

function buildAoiSamples(bounds: RectangleBounds): { samples: PointGeo[]; cellAreaKm2: number; totalAreaKm2: number } {
  const totalAreaKm2 = rectangleAreaKm2(bounds);
  const midLat = (bounds.north + bounds.south) / 2;
  const heightKm = haversineDistanceKm(bounds.south, bounds.west, bounds.north, bounds.west);
  const widthKm = haversineDistanceKm(midLat, bounds.west, midLat, bounds.east);
  const cellKm = Math.sqrt(Math.max(totalAreaKm2, 1) / TARGET_SAMPLE_COUNT);
  const nu = Math.min(MAX_SAMPLES_PER_AXIS, Math.max(1, Math.ceil(widthKm / cellKm)));
  const nv = Math.min(MAX_SAMPLES_PER_AXIS, Math.max(1, Math.ceil(heightKm / cellKm)));

  // Rows evenly spaced in sin(lat) -> every cell has the same area on the sphere.
  const sN = Math.sin(toRad(bounds.north));
  const sS = Math.sin(toRad(bounds.south));
  const samples: PointGeo[] = [];
  for (let j = 0; j < nv; j++) {
    const lat = toDeg(Math.asin(sN - ((j + 0.5) / nv) * (sN - sS)));
    for (let i = 0; i < nu; i++) {
      const lon = bounds.west + ((i + 0.5) / nu) * (bounds.east - bounds.west);
      samples.push(makePointGeo(lat, lon));
    }
  }
  return { samples, cellAreaKm2: totalAreaKm2 / (nu * nv), totalAreaKm2 };
}

/* ------------------------------------------------------------------ */
/* Shared along/cross frame ANCHORED AT THE CAPTURE POINT              */
/* ------------------------------------------------------------------ */

function toLocalKm(lat: number, lon: number, originLat: number, originLon: number) {
  const kmPerDegLat = 111.32;
  const kmPerDegLon = 111.32 * Math.cos(toRad(originLat));
  return { eastKm: (lon - originLon) * kmPerDegLon, northKm: (lat - originLat) * kmPerDegLat };
}

function localToLatLon(originLat: number, originLon: number, eastKm: number, northKm: number): LatLon {
  const kmPerDegLat = 111.32;
  const kmPerDegLon = 111.32 * Math.cos(toRad(originLat));
  return { lat: originLat + northKm / kmPerDegLat, lon: originLon + eastKm / kmPerDegLon };
}

function projectToFrame(lat: number, lon: number, originLat: number, originLon: number, headingRad: number) {
  const { eastKm, northKm } = toLocalKm(lat, lon, originLat, originLon);
  return {
    along: eastKm * Math.sin(headingRad) + northKm * Math.cos(headingRad),
    cross: eastKm * Math.cos(headingRad) - northKm * Math.sin(headingRad),
  };
}

function frameToLatLon(originLat: number, originLon: number, headingRad: number, along: number, cross: number): LatLon {
  const eastKm = along * Math.sin(headingRad) + cross * Math.cos(headingRad);
  const northKm = along * Math.cos(headingRad) - cross * Math.sin(headingRad);
  return localToLatLon(originLat, originLon, eastKm, northKm);
}

type SatelliteFrameInfo = {
  name: string;
  alongOffset: number;
  crossOffset: number;
  groundSpeedKmPerSec: number;
};

type PassGeometry = {
  originLat: number;
  originLon: number;
  headingDeg: number;
  headingRad: number;
  /** AOI bounding box in the frame. Used ONLY to limit how far a footprint is drawn. */
  alongMin: number; alongMax: number;
  crossMin: number; crossMax: number;
  rectangleTrackAngleDeg: number;
  perSatellite: SatelliteFrameInfo[];
  sampleAlong: Float64Array;
  sampleCross: Float64Array;
  repAlong: number[];
  repCross: number[];
};

function computePassGeometry(
  satRecs: { tle: TleEntry; satrec: satellite.SatRec }[],
  leaderSatrec: satellite.SatRec,
  bounds: RectangleBounds,
  missionStart: Date,
  capturePoint: AoiCapturePoint,
  samples: PointGeo[],
  reps: PointGeo[],
): PassGeometry | null {
  const leaderF0 = propagateFrame(leaderSatrec, missionStart);
  const leaderF1 = propagateFrame(leaderSatrec, new Date(missionStart.getTime() + 1000));
  if (!leaderF0 || !leaderF1) return null;
  const s0 = subpointFromFrame(leaderF0);
  const s1 = subpointFromFrame(leaderF1);

  const headingDeg = initialBearingDeg(s0.lat, s0.lon, s1.lat, s1.lon);
  const headingRad = toRad(headingDeg);
  const originLat = capturePoint.lat;
  const originLon = capturePoint.lon;

  const corners = [
    { lat: bounds.north, lon: bounds.west }, { lat: bounds.north, lon: bounds.east },
    { lat: bounds.south, lon: bounds.east }, { lat: bounds.south, lon: bounds.west },
  ];
  let alongMin = Infinity, alongMax = -Infinity, crossMin = Infinity, crossMax = -Infinity;
  for (const c of corners) {
    const { along, cross } = projectToFrame(c.lat, c.lon, originLat, originLon, headingRad);
    alongMin = Math.min(alongMin, along); alongMax = Math.max(alongMax, along);
    crossMin = Math.min(crossMin, cross); crossMax = Math.max(crossMax, cross);
  }

  const rectEdgeBearing = initialBearingDeg(bounds.south, bounds.west, bounds.north, bounds.west);
  let angle = Math.abs(rectEdgeBearing - headingDeg) % 180;
  if (angle > 90) angle = 180 - angle;

  // Rigid look offset: the leader's swath centre starts exactly on the capture point.
  const leaderSub = projectToFrame(s0.lat, s0.lon, originLat, originLon, headingRad);
  const lookAlong = -leaderSub.along;
  const lookCross = -leaderSub.cross;

  const perSatellite: SatelliteFrameInfo[] = satRecs.map(({ tle, satrec }) => {
    const isLeader = satrec === leaderSatrec;
    const f0 = isLeader ? leaderF0 : propagateFrame(satrec, missionStart);
    const f1 = isLeader ? leaderF1 : propagateFrame(satrec, new Date(missionStart.getTime() + 1000));
    if (!f0 || !f1) return { name: tle.name, alongOffset: 0, crossOffset: 0, groundSpeedKmPerSec: 0 };
    const p0 = subpointFromFrame(f0);
    const p1 = subpointFromFrame(f1);
    const sub = projectToFrame(p0.lat, p0.lon, originLat, originLon, headingRad);
    return {
      name: tle.name,
      alongOffset: sub.along + lookAlong,
      crossOffset: sub.cross + lookCross,
      groundSpeedKmPerSec: haversineDistanceKm(p0.lat, p0.lon, p1.lat, p1.lon),
    };
  });

  const sampleAlong = new Float64Array(samples.length);
  const sampleCross = new Float64Array(samples.length);
  for (let k = 0; k < samples.length; k++) {
    const p = projectToFrame(samples[k].lat, samples[k].lon, originLat, originLon, headingRad);
    sampleAlong[k] = p.along;
    sampleCross[k] = p.cross;
  }
  const repAlong: number[] = [];
  const repCross: number[] = [];
  for (const r of reps) {
    const p = projectToFrame(r.lat, r.lon, originLat, originLon, headingRad);
    repAlong.push(p.along);
    repCross.push(p.cross);
  }

  return {
    originLat, originLon, headingDeg, headingRad,
    alongMin, alongMax, crossMin, crossMax,
    rectangleTrackAngleDeg: angle,
    perSatellite,
    sampleAlong, sampleCross, repAlong, repCross,
  };
}

/* ------------------------------------------------------------------ */
/* Footprints + TRUE-AOI coverage for ONE pass                         */
/* ------------------------------------------------------------------ */

type Rect = { alongMin: number; alongMax: number; crossMin: number; crossMax: number };

function clipToDrawBox(r: Rect, geom: PassGeometry): Rect | null {
  const alongMin = Math.max(r.alongMin, geom.alongMin);
  const alongMax = Math.min(r.alongMax, geom.alongMax);
  const crossMin = Math.max(r.crossMin, geom.crossMin);
  const crossMax = Math.min(r.crossMax, geom.crossMax);
  if (alongMax <= alongMin || crossMax <= crossMin) return null;
  return { alongMin, alongMax, crossMin, crossMax };
}

type PlannedRect = {
  satIndex: number;
  segIndex: number;
  rect: Rect;
  tStart: number;
  tEnd: number;
};

type PlanContext = {
  bounds: RectangleBounds;
  segments: FrequencySegment[];
  plannedSeconds: number;
  gate: Gate;
  satRecs: { tle: TleEntry; satrec: satellite.SatRec }[];
  reps: PointGeo[];
  samples: PointGeo[];
  cellAreaKm2: number;
  totalAreaKm2: number;
  /** cells covered by earlier passes */
  covered: Uint8Array;
};

function evaluatePass(ctx: PlanContext, cand: MissionStartCandidate): StripPass | null {
  const { limits } = ctx.gate;
  const N = ctx.samples.length;

  // ---- the four times -------------------------------------------------------
  const missionStart = cand.missionStart;
  const los = findLos(cand.satrec, ctx.reps, missionStart, ctx.gate);
  const availableSeconds = Math.floor((los.getTime() - missionStart.getTime()) / 1000);
  const missionSeconds = ctx.plannedSeconds;                 // mission ends when the frequency plan ends
  if (missionSeconds < 1) return null;
  const truncatedByLos = availableSeconds < missionSeconds;  // LOS arrives before the plan ends
  const missionEnd = new Date(missionStart.getTime() + missionSeconds * 1000);
  const dummyAos = new Date(missionStart.getTime() - limits.aosLeadSeconds * 1000);

  // ---- shared frame ---------------------------------------------------------
  const geom = computePassGeometry(ctx.satRecs, cand.satrec, ctx.bounds, missionStart, cand.capturePoint, ctx.samples, ctx.reps);
  if (!geom) return null;

  const swathScale =
    limits.swathModel === "cos"
      ? Math.min(SWATH_SCALE_CAP, 1 / Math.max(Math.cos(toRad(cand.capturePoint.tiltDeg)), 1 / SWATH_SCALE_CAP))
      : 1;

  // ---- planned rectangles: per segment, per satellite -----------------------
  const planned: PlannedRect[] = [];
  const segBreakdown: SegmentCoverage[] = ctx.segments.map((seg) => ({
    segmentId: seg.id,
    presetId: seg.presetId,
    startSeconds: seg.startSeconds,
    durationSeconds: seg.durationSeconds,
    swathKm: FREQUENCY_PRESETS[seg.presetId].swathKm * swathScale,
    footprints: [],
    areaKm2: 0, coveredAreaKm2: 0, coveredFraction: 0, newlyCoveredFraction: 0,
  }));

  ctx.segments.forEach((seg, segIndex) => {
    const preset = FREQUENCY_PRESETS[seg.presetId];
    if (!preset) return;
    const tA = seg.startSeconds;
    const tB = Math.min(seg.startSeconds + seg.durationSeconds, missionSeconds);   // no sweep past mission end
    if (tB <= tA) return;
    const half = (preset.swathKm * swathScale) / 2;

    geom.perSatellite.forEach((s, satIndex) => {
      if (s.groundSpeedKmPerSec <= 0) return;
      const raw: Rect = {
        alongMin: s.alongOffset + s.groundSpeedKmPerSec * tA,
        alongMax: s.alongOffset + s.groundSpeedKmPerSec * tB,
        crossMin: s.crossOffset - half,
        crossMax: s.crossOffset + half,
      };
      const clipped = clipToDrawBox(raw, geom);
      if (!clipped) return;
      const tStart = clamp((clipped.alongMin - s.alongOffset) / s.groundSpeedKmPerSec, tA, tB);
      const tEnd = clamp((clipped.alongMax - s.alongOffset) / s.groundSpeedKmPerSec, tA, tB);
      planned.push({ satIndex, segIndex, rect: clipped, tStart, tEnd });

      const ll = (a: number, c: number) => frameToLatLon(geom.originLat, geom.originLon, geom.headingRad, a, c);
      segBreakdown[segIndex].footprints.push({
        satelliteName: s.name,
        corners: [
          ll(clipped.alongMin, clipped.crossMin), ll(clipped.alongMin, clipped.crossMax),
          ll(clipped.alongMax, clipped.crossMax), ll(clipped.alongMax, clipped.crossMin),
        ],
        tStartSec: tStart,
        tEndSec: tEnd,
      });
    });
  });

  // ---- coverage on the REAL AOI, with the imageability check per cell --------
  const firstTime = new Float64Array(N).fill(Infinity);
  const firstSeg = new Int16Array(N).fill(-1);
  const tiltArr = new Float32Array(N);
  const rangeArr = new Float32Array(N);
  const elArr = new Float32Array(N);

  const frameCache: Map<number, Vec3 | null>[] = ctx.satRecs.map(() => new Map());
  const satEcfAt = (satIndex: number, sec: number): Vec3 | null => {
    const cache = frameCache[satIndex];
    if (cache.has(sec)) return cache.get(sec)!;
    const f = propagateFrame(ctx.satRecs[satIndex].satrec, new Date(missionStart.getTime() + sec * 1000));
    const v = f ? f.ecf : null;
    cache.set(sec, v);
    return v;
  };

  for (const pr of planned) {
    const s = geom.perSatellite[pr.satIndex];
    for (let k = 0; k < N; k++) {
      const along = geom.sampleAlong[k];
      const cross = geom.sampleCross[k];
      if (along < pr.rect.alongMin || along > pr.rect.alongMax || cross < pr.rect.crossMin || cross > pr.rect.crossMax) continue;
      const t = (along - s.alongOffset) / s.groundSpeedKmPerSec;
      if (t >= firstTime[k]) continue;
      const sec = clamp(Math.round(t), 0, missionSeconds);
      const ecf = satEcfAt(pr.satIndex, sec);
      if (!ecf) continue;
      const m = lookMetrics(ecf, ctx.samples[k], limits);
      if (!meetsLimits(m, ctx.gate)) continue;      // outside elevation / range / tilt -> not imageable
      firstTime[k] = t;
      firstSeg[k] = pr.segIndex;
      tiltArr[k] = m.tiltDeg; rangeArr[k] = m.rangeKm; elArr[k] = m.elevationDeg;
    }
  }

  // ---- aggregate -------------------------------------------------------------
  const segCount = new Array(ctx.segments.length).fill(0);
  const segNew = new Array(ctx.segments.length).fill(0);
  const curveBins = new Array(missionSeconds + 1).fill(0);
  let coveredCount = 0;
  let newCount = 0;
  let tiltLo = Infinity, tiltHi = -Infinity, rngLo = Infinity, rngHi = -Infinity, elLo = Infinity, elHi = -Infinity;

  for (let k = 0; k < N; k++) {
    if (!Number.isFinite(firstTime[k])) continue;
    coveredCount++;
    segCount[firstSeg[k]]++;
    const isNew = ctx.covered[k] === 0;
    if (isNew) { newCount++; segNew[firstSeg[k]]++; }
    curveBins[clamp(Math.ceil(firstTime[k]), 0, missionSeconds)]++;
    tiltLo = Math.min(tiltLo, tiltArr[k]); tiltHi = Math.max(tiltHi, tiltArr[k]);
    rngLo = Math.min(rngLo, rangeArr[k]); rngHi = Math.max(rngHi, rangeArr[k]);
    elLo = Math.min(elLo, elArr[k]); elHi = Math.max(elHi, elArr[k]);
  }
  if (newCount === 0) return null;   // pass adds nothing new -> discard

  let running = 0;
  const coverageCurve = curveBins.map((c: number) => { running += c; return running / N; });

  segBreakdown.forEach((sb, i) => {
    sb.areaKm2 = segCount[i] * ctx.cellAreaKm2;
    sb.coveredAreaKm2 = sb.areaKm2;
    sb.coveredFraction = segCount[i] / N;
    sb.newlyCoveredFraction = segNew[i] / N;
  });

  for (let k = 0; k < N; k++) if (Number.isFinite(firstTime[k])) ctx.covered[k] = 1;
  let cum = 0;
  for (let k = 0; k < N; k++) cum += ctx.covered[k];

  // ---- labels / orientation --------------------------------------------------
  const leaderInfo = geom.perSatellite[ctx.satRecs.findIndex((r) => r.satrec === cand.satrec)];
  const halfMax = Math.max(0, ...ctx.segments.map((s) => (FREQUENCY_PRESETS[s.presetId]?.swathKm ?? 0) * swathScale / 2));
  let exitIdx = -1;
  let exitAlong = -Infinity;
  for (let i = 0; i < ctx.reps.length; i++) {
    const inBand = leaderInfo ? Math.abs(geom.repCross[i] - leaderInfo.crossOffset) <= halfMax : true;
    if (inBand && geom.repAlong[i] > exitAlong) { exitAlong = geom.repAlong[i]; exitIdx = i; }
  }
  if (exitIdx < 0) {
    for (let i = 0; i < ctx.reps.length; i++) if (geom.repAlong[i] > exitAlong) { exitAlong = geom.repAlong[i]; exitIdx = i; }
  }

  const otherSatellites = ctx.satRecs
    .filter((r) => r.satrec !== cand.satrec)
    .map((r) => {
      const a = findOtherMissionStart(r.satrec, ctx.reps, missionStart, ctx.gate);
      return { name: r.tle.name, missionStart: a, offsetMs: a ? a.getTime() - missionStart.getTime() : null };
    });

  return {
    satelliteName: cand.tle.name,
    dummyAos, missionStart, missionEnd, los,
    plannedSeconds: ctx.plannedSeconds, missionSeconds, truncatedByLos,
    newlyCoveredFraction: newCount / N,
    newlyCoveredAreaKm2: newCount * ctx.cellAreaKm2,
    passCoveredFraction: coveredCount / N,
    passCoveredAreaKm2: coveredCount * ctx.cellAreaKm2,
    cumulativeCoveredFraction: cum / N,
    segmentBreakdown: segBreakdown,
    coverageCurve,
    capturePoint: cand.capturePoint,
    entrySideLabel: POINT_LABELS[cand.capturePoint.pointIndex] ?? "AOI",
    exitSideLabel: exitIdx >= 0 ? POINT_LABELS[exitIdx] : "AOI",
    orientation: {
      headingDeg: geom.headingDeg,
      rectangleTrackAngleDeg: geom.rectangleTrackAngleDeg,
      crossTrackWidthKm: geom.crossMax - geom.crossMin,
      alongTrackLengthKm: geom.alongMax - geom.alongMin,
    },
    geometry: {
      tiltDeg: [tiltLo, tiltHi],
      rangeKm: [rngLo, rngHi],
      elevationDeg: [elLo, elHi],
      swathScale,
    },
    otherSatellites,
  };
}

/* ------------------------------------------------------------------ */
/* PUBLIC ENTRY POINT                                                  */
/* ------------------------------------------------------------------ */

export function planStripCapture(
  tles: TleEntry[],
  bounds: RectangleBounds,
  start: Date,
  end: Date,
  minRangeKm: number,
  maxRangeKm: number,
  segments: FrequencySegment[],
  options: Partial<StripLimits> = {},
): StripCapturePlanResult {
  const limits: StripLimits = { ...DEFAULT_STRIP_LIMITS, ...options };
  const gate: Gate = { limits, minRangeKm, maxRangeKm };
  const { samples, cellAreaKm2, totalAreaKm2 } = buildAoiSamples(bounds);

  const empty: StripCapturePlanResult = {
    bounds, passes: [], fullyCovered: false, totalCoveredFraction: 0,
    aoiAreaKm2: totalAreaKm2, sampleCount: samples.length,
    limits: { ...limits, minRangeKm, maxRangeKm },
  };

  const usable = segments.filter((s) => s.durationSeconds > 0 && FREQUENCY_PRESETS[s.presetId]);
  if (tles.length === 0 || usable.length === 0) return empty;

  // planned plan length = end of the last segment, capped at the mission limit
  const plannedSeconds = Math.min(
    limits.maxMissionSeconds,
    Math.max(...usable.map((s) => s.startSeconds + s.durationSeconds)),
  );

  const satRecs = tles.map((tle) => ({ tle, satrec: getSatrec(tle) }));
  const reps = representativePoints(bounds);
  const ctx: PlanContext = {
    bounds, segments: usable, plannedSeconds, gate, satRecs, reps, samples,
    cellAreaKm2, totalAreaKm2, covered: new Uint8Array(samples.length),
  };

  const passes: StripPass[] = [];
  let skipUntilMs = -Infinity;
  let fully = false;
  let total = 0;

  for (const cand of scanMissionStartCandidates(satRecs, reps, start, end, gate, COARSE_STEP_SECONDS)) {
    if (cand.missionStart.getTime() < skipUntilMs) continue;           // same orbit pass as the previous one
    const pass = evaluatePass(ctx, cand);
    skipUntilMs = (pass ? Math.max(pass.los.getTime(), pass.missionEnd.getTime()) : cand.missionStart.getTime()) + 1000;
    if (!pass) continue;                                      // no new area -> discard
    passes.push(pass);
    total = pass.cumulativeCoveredFraction;
    if (total >= FULL_COVERAGE_FRACTION) { fully = true; break; }
    if (passes.length >= MAX_PASSES_SAFETY) break;
  }

  return { ...empty, passes, fullyCovered: fully, totalCoveredFraction: total };
}