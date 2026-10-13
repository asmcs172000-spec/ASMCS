import * as satellite from "satellite.js";

// ─── Geometry ─────────────────────────────────────────────────────────────
//
// SPOT-MODE CAPTURE MODEL
// -----------------------
// The satellite flies along its ground track. The user drops a marker (target
// lat/lon). The satellite TILTS (off-nadir angle, η) to point at the marker,
// and the slant range ρ says how far away the marker is. Tilt and range are
// tied together by the Earth-center / satellite / target triangle:
//
//   Rs = Re + h            (satellite distance from Earth's center)
//   Tilt  -> Range : ρ = Rs·cosη − √(Re² − Rs²·sin²η)
//   Range -> Tilt  : cosη = (Rs² + ρ² − Re²) / (2·Rs·ρ)
//   Tilt  -> Elev  : cos(el) = Rs·sinη / Re
//
// (The flat-Earth shortcut ρ = h / cosη is NOT used.)
//
// In the scan itself, nothing is assumed: every step propagates the real TLE
// (SGP4), converts to ECEF, and measures elevation, slant range, and tilt
// (angle between nadir and the line of sight) directly from the vectors.
//
// MISSION TIMELINE PER PASS
// -------------------------
//   dummyAos     = missionStart − 5 min   (preparation marker)
//   missionStart = REAL AOS: first instant where ALL of these hold
//                    • elevation ≥ minElevationDeg
//                    • slant range within [minRangeKm, maxRangeKm]
//                    • tilt within [minTiltDeg, maxTiltDeg]
//   missionEnd   = missionStart + missionDurationSeconds
//                  (= sum of the frequency segment durations, ≤ 10 min)
//   los          = natural LOS: instant elevation drops below minElevationDeg
//
// If los < missionEnd the pass is SHORT (`shortPass = true`). Use
// `planMissionAcrossPasses()` to chain the following passes until the full
// mission duration is covered.
//
// Ground-station comms passes (`findPasses`) are unaffected.

// ── General usable link/imaging envelope ──
export const MIN_LINK_RANGE_KM = 551;   // nadir floor
export const MAX_LINK_RANGE_KM = 1800;  // usable range ceiling

export const EARTH_RADIUS_KM = 6378.137; // WGS84 equatorial radius
export const NOMINAL_ALTITUDE_KM = 550;

export const DEFAULT_MIN_ELEVATION_DEG = 0;
export const DEFAULT_MIN_TILT_DEG = 5;
export const DEFAULT_MAX_TILT_DEG = 90; // range limit is what normally bounds max tilt
export const DEFAULT_MISSION_DURATION_SECONDS = 10 * 60;
export const MAX_MISSION_DURATION_SECONDS = 10 * 60;
export const DUMMY_AOS_LEAD_MINUTES = 5;

// ─── Types ────────────────────────────────────────────────────────────────

export type TleEntry = {
  name: string;
  tle1: string;
  tle2: string;
  color?: string;
};

/** Any point on the ground the satellite's range can be measured against —
 *  used for ground-station passes and for Spot/Area targets. */
export type GroundPoint = {
  name: string;
  lat: number;
  lon: number;
  color?: string;
};

/** @deprecated alias kept for compatibility. */
export type GroundStation = GroundPoint;

export type Pass = {
  aos: Date;
  los: Date;
  maxElevationDeg: number;
  rangeAtMaxElevationKm: number;
};

export const DEFAULT_GROUND_STATIONS: GroundPoint[] = [
  { name: "Rawalpindi GS", lat: 33.6167, lon: 73.0992, color: "#7dd3fc" },
  { name: "Azerbaijan GS", lat: 40.39, lon: 49.29, color: "#fef08a" },
  { name: "Turkey GS", lat: 41.1037, lon: 29.0261, color: "#f472b6" },
];

export const OTHER_SAT_FALLBACK_COLORS = [
  "#7dd3fc", "#fbbf24", "#f472b6", "#a78bfa", "#34d399", "#fb7185", "#facc15", "#60a5fa",
];

const toDeg = (rad: number) => (rad * 180) / Math.PI;
const toRad = (deg: number) => (deg * Math.PI) / 180;

// 30s coarse step; every boundary is refined to ~1s by bisection.
const STEP_SECONDS = 30;
// If a pass is still in progress when the scan window ends, keep stepping
// (up to this long) so its real LOS can be found.
const LOS_EXTENSION_SECONDS = 30 * 60;

// ─── Range / tilt / elevation formulas ────────────────────────────────────

/** Tilt (off-nadir, degrees) needed to see a target at slant range `rangeKm`.
 *  `satRadiusKm` = Re + altitude. cosη = (Rs² + ρ² − Re²) / (2·Rs·ρ). */
export function tiltFromRangeDeg(
  rangeKm: number,
  satRadiusKm: number = EARTH_RADIUS_KM + NOMINAL_ALTITUDE_KM,
): number {
  const Rs = satRadiusKm;
  const Re = EARTH_RADIUS_KM;
  if (rangeKm <= 0) return 0;
  const cosEta = (Rs * Rs + rangeKm * rangeKm - Re * Re) / (2 * Rs * rangeKm);
  return toDeg(Math.acos(Math.max(-1, Math.min(1, cosEta))));
}

/** Slant range (km) for a given tilt. ρ = Rs·cosη − √(Re² − Rs²·sin²η).
 *  Returns NaN if the tilt looks past the Earth's limb. */
export function rangeFromTiltKm(
  tiltDeg: number,
  satRadiusKm: number = EARTH_RADIUS_KM + NOMINAL_ALTITUDE_KM,
): number {
  const Rs = satRadiusKm;
  const Re = EARTH_RADIUS_KM;
  const eta = toRad(tiltDeg);
  const disc = Re * Re - Rs * Rs * Math.sin(eta) ** 2;
  if (disc < 0) return NaN;
  return Rs * Math.cos(eta) - Math.sqrt(disc);
}

/** Target elevation (degrees) for a given tilt. cos(el) = Rs·sinη / Re. */
export function elevationFromTiltDeg(
  tiltDeg: number,
  satRadiusKm: number = EARTH_RADIUS_KM + NOMINAL_ALTITUDE_KM,
): number {
  const x = (satRadiusKm * Math.sin(toRad(tiltDeg))) / EARTH_RADIUS_KM;
  if (x > 1) return NaN;
  return toDeg(Math.acos(Math.max(-1, Math.min(1, x))));
}

/** Convert the user's range limits into the tilt limits they imply.
 *  Handy for showing "derived tilt" in the UI. */
export function deriveTiltLimitsDeg(
  minRangeKm: number,
  maxRangeKm: number,
  altitudeKm: number = NOMINAL_ALTITUDE_KM,
): { minTiltDeg: number; maxTiltDeg: number } {
  const Rs = EARTH_RADIUS_KM + altitudeKm;
  return {
    minTiltDeg: tiltFromRangeDeg(minRangeKm, Rs),
    maxTiltDeg: tiltFromRangeDeg(maxRangeKm, Rs),
  };
}

// ─── Snapshot (real SGP4 look geometry) ───────────────────────────────────

type Snapshot = {
  elevationDeg: number;
  rangeKm: number;
  azimuthDeg: number;
  /** Off-nadir angle at the satellite toward the target, degrees. */
  tiltDeg: number;
  /** Satellite distance from Earth's center, km. */
  satRadiusKm: number;
};

/** @deprecated name kept for older imports. */
export type RangeSnapshot = Snapshot;

// ── satrec cache ── (re-parsing the same TLE is wasted work)
const satrecCache = new Map<string, satellite.SatRec>();

function getSatrec(tle: TleEntry): satellite.SatRec {
  const key = `${tle.tle1}|${tle.tle2}`;
  const cached = satrecCache.get(key);
  if (cached) return cached;
  const satrec = satellite.twoline2satrec(tle.tle1, tle.tle2);
  satrecCache.set(key, satrec);
  return satrec;
}

/** Single shared low-level lookup: elevation, slant range, azimuth AND tilt
 *  of a propagated satrec as seen from `point` at instant `t`. */
function snapshotAt(satrec: satellite.SatRec, point: GroundPoint, t: Date): Snapshot | null {
  const geodetic = {
    latitude: satellite.degreesToRadians(point.lat),
    longitude: satellite.degreesToRadians(point.lon),
    height: 0,
  };
  const pv = satellite.propagate(satrec, t);
  if (!pv?.position || typeof pv.position === "boolean") return null;

  const gmst = satellite.gstime(t);
  const satEcf = satellite.eciToEcf(pv.position, gmst);
  const look = satellite.ecfToLookAngles(geodetic, satEcf);

  // Tilt = angle between nadir (−r_sat) and line of sight (target − r_sat).
  const tgtEcf = satellite.geodeticToEcf(geodetic);
  const losX = tgtEcf.x - satEcf.x;
  const losY = tgtEcf.y - satEcf.y;
  const losZ = tgtEcf.z - satEcf.z;
  const losLen = Math.hypot(losX, losY, losZ);
  const satLen = Math.hypot(satEcf.x, satEcf.y, satEcf.z);
  let tiltDeg = 0;
  if (losLen > 0 && satLen > 0) {
    const dot = -(satEcf.x * losX + satEcf.y * losY + satEcf.z * losZ);
    tiltDeg = toDeg(Math.acos(Math.max(-1, Math.min(1, dot / (satLen * losLen)))));
  }

  return {
    elevationDeg: toDeg(look.elevation),
    rangeKm: look.rangeSat,
    azimuthDeg: toDeg(look.azimuth),
    tiltDeg,
    satRadiusKm: satLen,
  };
}

/** Bisect to a boundary between a known FALSE instant and a known TRUE
 *  instant (either order). Returns the TRUE-side instant, ~1s precision. */
function bisectBoundary(
  isTrue: (t: Date) => boolean,
  falseTime: Date,
  trueTime: Date,
): Date {
  let a = falseTime.getTime();
  let b = trueTime.getTime();
  while (Math.abs(b - a) > 1000) {
    const mid = (a + b) / 2;
    if (isTrue(new Date(mid))) b = mid;
    else a = mid;
  }
  return new Date(b);
}

// ─── Ground-station passes (unchanged behaviour) ──────────────────────────

/** Every AOS/LOS pass of `tle` over `point` within [start, end] (horizon 0°).
 *  Used for the ground-station comms-pass list. */
export function findPasses(
  tle: TleEntry,
  point: GroundPoint,
  start: Date,
  end: Date,
  stepSeconds: number = STEP_SECONDS,
): Pass[] {
  const satrec = getSatrec(tle);
  const passes: Pass[] = [];
  const aboveHorizon = (t: Date) => (snapshotAt(satrec, point, t)?.elevationDeg ?? -180) >= 0;

  let inPass = false;
  let passStart: Date | null = null;
  let maxElev = -90;
  let rangeAtMax = 0;
  let prevTime: Date | null = null;

  for (let t = new Date(start); t <= end; t = new Date(t.getTime() + stepSeconds * 1000)) {
    const snap = snapshotAt(satrec, point, t);
    const elev = snap?.elevationDeg ?? -180;

    if (elev >= 0 && !inPass) {
      inPass = true;
      passStart = prevTime ? bisectBoundary(aboveHorizon, prevTime, t) : new Date(t);
      maxElev = elev;
      rangeAtMax = snap?.rangeKm ?? 0;
    }
    if (inPass && snap && elev > maxElev) {
      maxElev = elev;
      rangeAtMax = snap.rangeKm;
    }
    if (elev < 0 && inPass) {
      inPass = false;
      const los = prevTime ? bisectBoundary(aboveHorizon, t, prevTime) : new Date(t);
      if (passStart) {
        passes.push({ aos: passStart, los, maxElevationDeg: maxElev, rangeAtMaxElevationKm: rangeAtMax });
      }
      passStart = null;
      maxElev = -90;
    }
    prevTime = new Date(t);
  }

  return passes;
}

// ─── Spot-mode capture passes (Georeference target selection) ─────────────

export type SpotScanOptions = {
  /** Minimum target elevation (deg) for the pass to count. Default 0. */
  minElevationDeg?: number;
  /** Minimum tilt (deg). Default 5. */
  minTiltDeg?: number;
  /** Maximum tilt (deg). Default 90 (range limit normally binds first). */
  maxTiltDeg?: number;
  /** Total mission duration = sum of frequency segment durations. Max 600 s. */
  missionDurationSeconds?: number;
  /** Coarse scan step. Default 30 s. */
  stepSeconds?: number;
};

/** One upcoming capture opportunity.
 *
 *  Compatibility: `aos` = missionStart (the real AOS) and `los` = natural LOS,
 *  so existing UI that reads `aos` / `los` keeps working. */
export type SpotOrbit = Pass & {
  /** Slant range (km) to the target at mission start. */
  rangeAtAosKm: number;

  /** missionStart − 5 min. Preparation marker, not a real AOS. */
  dummyAos: Date;
  /** REAL AOS: elevation, range and tilt all in limits. */
  missionStart: Date;
  /** missionStart + missionDurationSeconds (planned end). */
  missionEnd: Date;

  /** Geometry at mission start. */
  rangeAtStartKm: number;
  tiltAtStartDeg: number;
  elevationAtStartDeg: number;

  /** Requested mission length (s). */
  missionDurationSeconds: number;
  /** Seconds of the mission this pass can actually deliver (≤ duration). */
  availableSeconds: number;
  /** True if LOS comes before missionEnd. */
  shortPass: boolean;
  /** Seconds still needed from later passes (0 if complete). */
  remainingSeconds: number;
};

function clampDuration(sec: number | undefined): number {
  const v = sec ?? DEFAULT_MISSION_DURATION_SECONDS;
  if (!Number.isFinite(v)) return DEFAULT_MISSION_DURATION_SECONDS;
  return Math.max(0, Math.min(MAX_MISSION_DURATION_SECONDS, v));
}

export function findRangeGatedOrbits(
  tle: TleEntry,
  target: GroundPoint,
  start: Date,
  end: Date,
  minRangeKm: number,
  maxRangeKm: number,
  options: SpotScanOptions = {},
): SpotOrbit[] {
  const satrec = getSatrec(tle);
  const orbits: SpotOrbit[] = [];

  const minEl = options.minElevationDeg ?? DEFAULT_MIN_ELEVATION_DEG;
  const minTilt = options.minTiltDeg ?? DEFAULT_MIN_TILT_DEG;
  const maxTilt = options.maxTiltDeg ?? DEFAULT_MAX_TILT_DEG;
  const stepSeconds = options.stepSeconds ?? STEP_SECONDS;
  const durationSec = clampDuration(options.missionDurationSeconds);

  /** All three capture conditions together. */
  const qualifies = (s: Snapshot | null): boolean =>
    !!s &&
    s.elevationDeg >= minEl &&
    s.rangeKm >= minRangeKm &&
    s.rangeKm <= maxRangeKm &&
    s.tiltDeg >= minTilt &&
    s.tiltDeg <= maxTilt;

  const isQualifying = (t: Date) => qualifies(snapshotAt(satrec, target, t));
  const isAboveMinEl = (t: Date) => (snapshotAt(satrec, target, t)?.elevationDeg ?? -180) >= minEl;

  let inPass = false;
  let missionStart: Date | null = null;
  let startSnap: Snapshot | null = null;
  let maxElev = -90;
  let rangeAtMax = 0;
  let prevTime: Date | null = null;
  let prevQualified = false;

  const endMs = end.getTime();
  const hardEndMs = endMs + LOS_EXTENSION_SECONDS * 1000;

  for (let ms = start.getTime(); ms <= hardEndMs; ms += stepSeconds * 1000) {
    // Past the scan window: only keep going to close a pass already open.
    if (ms > endMs && !inPass) break;

    const t = new Date(ms);
    const snap = snapshotAt(satrec, target, t);
    const elev = snap?.elevationDeg ?? -180;
    const onPass = elev >= minEl;
    const qualified = qualifies(snap);

    if (onPass && !inPass) {
      inPass = true;
      missionStart = null;
      startSnap = null;
      maxElev = elev;
      rangeAtMax = snap?.rangeKm ?? 0;
    }

    if (inPass && onPass && snap) {
      if (elev > maxElev) {
        maxElev = elev;
        rangeAtMax = snap.rangeKm;
      }
      // First instant all conditions hold in this pass = mission start (real AOS).
      if (!missionStart && qualified) {
        missionStart =
          prevTime && !prevQualified ? bisectBoundary(isQualifying, prevTime, t) : new Date(t);
        startSnap = snapshotAt(satrec, target, missionStart) ?? snap;
      }
    }

    if (!onPass && inPass) {
      inPass = false;
      // Natural LOS: elevation falls below min elevation.
      const los = prevTime ? bisectBoundary(isAboveMinEl, t, prevTime) : new Date(t);

      if (missionStart && startSnap) {
        const startMs = missionStart.getTime();
        const missionEnd = new Date(startMs + durationSec * 1000);
        const passSeconds = Math.max(0, (los.getTime() - startMs) / 1000);
        const availableSeconds = Math.min(durationSec, passSeconds);

        orbits.push({
          aos: missionStart,
          los,
          maxElevationDeg: maxElev,
          rangeAtMaxElevationKm: rangeAtMax,
          rangeAtAosKm: startSnap.rangeKm,

          dummyAos: new Date(startMs - DUMMY_AOS_LEAD_MINUTES * 60 * 1000),
          missionStart,
          missionEnd,

          rangeAtStartKm: startSnap.rangeKm,
          tiltAtStartDeg: startSnap.tiltDeg,
          elevationAtStartDeg: startSnap.elevationDeg,

          missionDurationSeconds: durationSec,
          availableSeconds,
          shortPass: los.getTime() < missionEnd.getTime(),
          remainingSeconds: Math.max(0, durationSec - availableSeconds),
        });
      }

      missionStart = null;
      startSnap = null;
      maxElev = -90;
    }

    prevTime = new Date(t);
    prevQualified = qualified;
  }

  return orbits;
}

// ─── Multi-pass mission planning ──────────────────────────────────────────

export type MissionAllocation = {
  /** Index into the orbits array passed in. */
  orbitIndex: number;
  /** Capture starts here (this pass's mission start). */
  start: Date;
  /** Capture ends here (mission-time used on this pass). */
  end: Date;
  durationSeconds: number;
  /** Seconds into the overall mission plan where this chunk begins. Use it
   *  to split the frequency segments across passes in order. */
  planOffsetSeconds: number;
};

export type MissionPlan = {
  allocations: MissionAllocation[];
  totalSeconds: number;
  coveredSeconds: number;
  remainingSeconds: number;
  /** True when the whole mission duration is covered. */
  completed: boolean;
};

/** Chain passes (chronological, from `startIndex`) until the full mission
 *  duration is covered. Each pass contributes up to its available time
 *  (missionStart → LOS). A short pass is followed by the next qualifying
 *  pass for the remaining duration. */
export function planMissionAcrossPasses(
  orbits: SpotOrbit[],
  startIndex: number = 0,
  totalSeconds?: number,
): MissionPlan {
  const total = clampDuration(totalSeconds ?? orbits[startIndex]?.missionDurationSeconds);
  const allocations: MissionAllocation[] = [];
  let covered = 0;

  for (let i = startIndex; i < orbits.length && covered < total; i++) {
    const o = orbits[i];
    const passSeconds = Math.max(0, (o.los.getTime() - o.missionStart.getTime()) / 1000);
    const take = Math.min(passSeconds, total - covered);
    if (take <= 0) continue;
    allocations.push({
      orbitIndex: i,
      start: o.missionStart,
      end: new Date(o.missionStart.getTime() + take * 1000),
      durationSeconds: take,
      planOffsetSeconds: covered,
    });
    covered += take;
  }

  return {
    allocations,
    totalSeconds: total,
    coveredSeconds: covered,
    remainingSeconds: Math.max(0, total - covered),
    completed: covered >= total - 1e-6,
  };
}

// ─── Helpers ──────────────────────────────────────────────────────────────

export function fallbackColorForName(name: string, palette: string[]): string {
  let hash = 0;
  for (let i = 0; i < name.length; i++) {
    hash = (hash * 31 + name.charCodeAt(i)) >>> 0;
  }
  return palette[hash % palette.length];
}

export function formatUtc(d: Date): string {
  const day = d.getUTCDate().toString().padStart(2, "0");
  const month = d.toLocaleString("en-US", { month: "short", timeZone: "UTC" });
  const year = d.getUTCFullYear();
  const h = d.getUTCHours().toString().padStart(2, "0");
  const m = d.getUTCMinutes().toString().padStart(2, "0");
  const s = d.getUTCSeconds().toString().padStart(2, "0");
  return `${day} ${month} ${year}, ${h}:${m}:${s} UTC`;
}
