use crate::constraints::{optimistic_eta_hours, violates_constraints};
use crate::envelope::{build_arrival_envelopes, simplify_envelope_boundary};
use crate::grid::{resolve_grid_spec, GridBestTracker, RoutingGrid};
use crate::grib::GribProvider;
use crate::landmask::Landmask;
use crate::objective::{step_cost, CostComponents, ObjectiveWeights};
use crate::polar::{angle_au_vent, Polar, MIN_ANGLE_AU_VENT_DEG};
use crate::route::build_route_legs;
use crate::sea_state::{DefaultSeaStateModifier, SeaStatePolarModifier};
use crate::types::*;
use chrono::{DateTime, Duration as ChronoDuration, Utc};
use ordered_float::OrderedFloat;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Optional runtime stats when `AI_ISOCHRONE_PROFILE=1`.
#[derive(Debug, Default)]
pub struct RoutingProfile {
    pub layers: usize,
    pub nodes_expanded: usize,
    pub expansions: usize,
    pub successors_raw: usize,
    pub successors_kept: usize,
    pub landmask_checks: usize,
    pub visited_cells: usize,
    pub expand_wall: Duration,
    pub merge_wall: Duration,
    pub backtrack_wall: Duration,
}

struct ProfileCounters {
    layers: AtomicUsize,
    nodes: AtomicUsize,
    expansions: AtomicUsize,
    succ_raw: AtomicUsize,
    succ_kept: AtomicUsize,
    landmask: AtomicUsize,
}

impl ProfileCounters {
    fn enabled() -> bool {
        std::env::var("AI_ISOCHRONE_PROFILE").is_ok()
    }
}

impl Default for ProfileCounters {
    fn default() -> Self {
        Self {
            layers: AtomicUsize::new(0),
            nodes: AtomicUsize::new(0),
            expansions: AtomicUsize::new(0),
            succ_raw: AtomicUsize::new(0),
            succ_kept: AtomicUsize::new(0),
            landmask: AtomicUsize::new(0),
        }
    }
}

fn print_routing_profile(p: &RoutingProfile) {
    let total = p.expand_wall + p.merge_wall + p.backtrack_wall;
    eprintln!("\n=== Routing profile ===");
    eprintln!("Layers (10-min steps): {}", p.layers);
    eprintln!("Nodes expanded:         {}", p.nodes_expanded);
    eprintln!("Expansion calls:        {}", p.expansions);
    eprintln!("Successors generated:   {} → {} kept after landmask", p.successors_raw, p.successors_kept);
    eprintln!("Landmask point checks:  {}", p.landmask_checks);
    eprintln!("Unique visited cells:   {}", p.visited_cells);
    eprintln!("Wall time expand phase: {:.2?} ({:.0}%)", p.expand_wall, pct(p.expand_wall, total));
    eprintln!("Wall time merge phase:  {:.2?} ({:.0}%)", p.merge_wall, pct(p.merge_wall, total));
    eprintln!("Wall time backtrack:    {:.2?} ({:.0}%)", p.backtrack_wall, pct(p.backtrack_wall, total));
    eprintln!(
        "Avg per layer: {:.1} ms expand, {:.1} ms merge",
        p.expand_wall.as_secs_f64() * 1000.0 / p.layers.max(1) as f64,
        p.merge_wall.as_secs_f64() * 1000.0 / p.layers.max(1) as f64,
    );
}

fn pct(part: Duration, total: Duration) -> f64 {
    if total.is_zero() {
        0.0
    } else {
        part.as_secs_f64() / total.as_secs_f64() * 100.0
    }
}

/// Cached environment for time-invariant GRIB providers.
#[derive(Clone, Copy)]
struct EnvSnapshot {
    wind: Wind,
    current: Current,
    sea_state: SeaState,
    current_vx: f64,
    current_vy: f64,
}

/// Precomputed boat speed + heading trig for one compass direction.
#[derive(Clone, Copy)]
struct HeadingKinematics {
    boat_speed: f64,
    sin_h: f64,
    cos_h: f64,
}

/// Lightweight successor for layer merge (Copy — no clone in fold/reduce).
#[derive(Clone, Copy)]
struct SuccCandidate {
    cell_key: NodeKey,
    point: Point,
    heading: f64,
    time: f64,
    cost: CostComponents,
    dist_from_start: f64,
    wind: Wind,
    sea_state: SeaState,
}

struct LayerExpansion {
    key: NodeKey,
    point: Point,
    heading: f64,
    time: f64,
    successors: Vec<SuccCandidate>,
    arrival: Option<(NodeKey, f64, f64)>,
}

/// SOTA multi-criteria isochrone router
pub struct SotaIsochroneRouter {
    config: SotaRoutingConfig,
    weights: ObjectiveWeights,
    landmask: Landmask,
    polar: Box<dyn Polar + Send + Sync>,
    grib: Box<dyn GribProvider + Send + Sync>,
    sea_modifier: DefaultSeaStateModifier,
    start_time: DateTime<Utc>,
    scenario_id: Option<String>,
    max_boat_speed_ms: f64,
}

type NodeKey = (i32, i32);

#[derive(Clone, Copy)]
struct CellRecord {
    time: f64,
    point: Point,
    heading: f64,
    parent: Option<NodeKey>,
    wind: Wind,
    sea_state: SeaState,
}

#[derive(Debug, Clone)]
struct SotaNode {
    point: Point,
    time: f64,
    heading: f64,
    cost: CostComponents,
    parent_key: Option<NodeKey>,
    /// Precomputed spatial hash key (avoids haversine in merge).
    cell_key: NodeKey,
    /// Accumulated hop distance from start (for fast grid precision without haversine).
    dist_from_start: f64,
}

impl PartialEq for SotaNode {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
            && (OrderedFloat(self.point.lat), OrderedFloat(self.point.lon))
                == (OrderedFloat(other.point.lat), OrderedFloat(other.point.lon))
    }
}

impl Eq for SotaNode {}

impl PartialOrd for SotaNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.time.partial_cmp(&other.time)
    }
}

impl Ord for SotaNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl SotaNode {
    #[allow(dead_code)]
    fn register_arrival(
        &self,
        key: NodeKey,
        dest: Option<Point>,
        arrival_radius_m: f64,
        weights: &ObjectiveWeights,
        arrival_records: &mut Vec<(NodeKey, f64, f64)>,
        best_arrival_time: &mut f64,
    ) {
        if let Some(dest_pt) = dest {
            if self.point.distance_to(&dest_pt) <= arrival_radius_m {
                let total = self.cost.total(weights);
                arrival_records.push((key, self.time, total));
                if self.time < *best_arrival_time {
                    *best_arrival_time = self.time;
                }
            }
        }
    }
}

impl SotaIsochroneRouter {
    pub fn new(
        config: SotaRoutingConfig,
        weights: ObjectiveWeights,
        landmask: Landmask,
        polar: Box<dyn Polar + Send + Sync>,
        grib: Box<dyn GribProvider + Send + Sync>,
        start_time: DateTime<Utc>,
    ) -> Self {
        Self {
            config,
            weights,
            landmask,
            polar,
            grib,
            sea_modifier: DefaultSeaStateModifier::new(0.35),
            start_time,
            scenario_id: None,
            max_boat_speed_ms: 8.0,
        }
    }

    pub fn with_scenario_id(mut self, id: impl Into<String>) -> Self {
        self.scenario_id = Some(id.into());
        self
    }

    pub fn with_sea_modifier(mut self, modifier: DefaultSeaStateModifier) -> Self {
        self.sea_modifier = modifier;
        self
    }

    pub fn calculate(&self) -> SotaRoutingResult {
        let base = &self.config.base;
        let step_seconds = base.simulation_step_seconds();
        let step_hours = step_seconds / 3600.0;
        let time_limit = base.time_limit_hours * 3600.0;
        let track_cost = self.config.optimize_cost;
        let build_isos = self.config.build_isochrones;

        let mut tracker = if build_isos {
            let grid_spec = resolve_grid_spec(
                self.grib.as_ref(),
                base.start,
                base.destination,
                base.grid_step_deg,
            );
            let routing_grid = RoutingGrid::from_spec(grid_spec, &self.landmask);
            Some(GridBestTracker::new(routing_grid))
        } else {
            None
        };

        let mut cells: FxHashMap<NodeKey, CellRecord> =
            FxHashMap::with_capacity_and_hasher(1_048_576, Default::default());

        let start_key = self.point_key(&base.start);
        let start_node = SotaNode {
            point: base.start,
            time: 0.0,
            heading: 0.0,
            cost: CostComponents::default(),
            parent_key: None,
            cell_key: start_key,
            dist_from_start: 0.0,
        };
        let start_env = self.resolve_env(base.start, 0.0);
        let mut layer = vec![start_node];
        cells.insert(
            start_key,
            CellRecord {
                time: 0.0,
                point: base.start,
                heading: 0.0,
                parent: None,
                wind: start_env.wind,
                sea_state: start_env.sea_state,
            },
        );
        if let Some(t) = tracker.as_mut() {
            let _ = t.try_update(base.start, 0.0, &self.landmask);
        }

        let mut isochrones: Vec<Isochrone> = Vec::new();
        let mut next_iso_time = base.isochrone_step_hours * 3600.0;

        let mut arrival_records: Vec<(NodeKey, f64, f64)> = Vec::new();
        let dest = base.destination;
        let mut best_arrival_time = f64::INFINITY;

        let max_nodes =
            ((base.time_limit_hours * 120_000.0) as usize).clamp(500_000, 25_000_000);
        let mut nodes_explored = 0usize;
        let iso_band = base.isochrone_step_hours * 3600.0;
        let mut current_time = 0.0;

        let headings: Vec<f64> = (0..base.num_directions)
            .map(|i| i as f64 * base.direction_step_degrees())
            .collect();

        let env = self.resolve_env(base.start, 0.0);
        let time_invariant = self.grib.is_time_invariant();
        let cached_kinematics = self.build_heading_kinematics(&headings, &env);

        let profile_on = ProfileCounters::enabled();
        let profile = Arc::new(ProfileCounters::default());
        let mut expand_wall = Duration::ZERO;
        let mut merge_wall = Duration::ZERO;
        let mut seen_layer =
            FxHashSet::with_capacity_and_hasher(32_768, Default::default());

        while current_time <= time_limit && !layer.is_empty() {
            nodes_explored += layer.len();
            if nodes_explored > max_nodes {
                break;
            }

            if profile_on {
                profile.layers.fetch_add(1, Ordering::Relaxed);
                profile.nodes.fetch_add(layer.len(), Ordering::Relaxed);
            }

            let prune_before = best_arrival_time;
            let layer_kinematics;
            let heading_kinematics: &[HeadingKinematics] = if time_invariant {
                &cached_kinematics
            } else {
                layer_kinematics =
                    self.build_heading_kinematics(&headings, &self.resolve_env(layer[0].point, current_time));
                &layer_kinematics
            };
            let t_expand = Instant::now();
            let profile_ref = Arc::clone(&profile);
            let expansions: Vec<LayerExpansion> = layer
                .par_iter()
                .filter_map(|node| {
                    self.expand_layer_node(
                        node,
                        &headings,
                        &heading_kinematics,
                        step_seconds,
                        time_limit,
                        dest,
                        &env,
                        track_cost,
                        prune_before,
                        profile_on.then_some(profile_ref.as_ref()),
                    )
                })
                .collect();

            // One batched landmask pass per layer (~80k points) instead of ~10k nested rayon jobs.
            let all_succ_points: Vec<Point> = expansions
                .iter()
                .flat_map(|exp| exp.successors.iter().map(|s| s.point))
                .collect();
            if profile_on {
                profile
                    .landmask
                    .fetch_add(all_succ_points.len(), Ordering::Relaxed);
            }
            let are_sea = self.landmask.are_sea(&all_succ_points);
            let mut sea_idx = 0usize;
            let mut expansions = expansions;
            for exp in &mut expansions {
                exp.successors.retain(|_| {
                    let keep = are_sea[sea_idx];
                    sea_idx += 1;
                    keep
                });
                if profile_on {
                    profile
                        .succ_kept
                        .fetch_add(exp.successors.len(), Ordering::Relaxed);
                }
            }
            expand_wall += t_expand.elapsed();

            let t_merge = Instant::now();

            for exp in &expansions {
                if let Some(record) = exp.arrival {
                    arrival_records.push(record);
                    if record.1 < best_arrival_time {
                        best_arrival_time = record.1;
                    }
                }
            }

            // Layer-local dedupe + global insert in one serial pass (FxHashSet avoids
            // parallel fold/reduce overhead at ~10k expansions/layer).
            seen_layer.clear();
            let mut next_layer: Vec<SotaNode> = Vec::with_capacity(layer.len() * headings.len() / 2);

            for exp in &expansions {
                for succ in &exp.successors {
                    if !seen_layer.insert(succ.cell_key) {
                        continue;
                    }
                    use std::collections::hash_map::Entry;
                    if let Entry::Vacant(e) = cells.entry(succ.cell_key) {
                        e.insert(CellRecord {
                            time: succ.time,
                            point: succ.point,
                            heading: succ.heading,
                            parent: Some(exp.key),
                            wind: succ.wind,
                            sea_state: succ.sea_state,
                        });
                        if let Some(t) = tracker.as_mut() {
                            t.try_update_sea(succ.point, succ.time);
                        }
                        next_layer.push(SotaNode {
                            point: succ.point,
                            time: succ.time,
                            heading: succ.heading,
                            cost: succ.cost,
                            parent_key: Some(exp.key),
                            cell_key: succ.cell_key,
                            dist_from_start: succ.dist_from_start,
                        });
                    }
                }
            }
            merge_wall += t_merge.elapsed();

            if build_isos {
                while next_iso_time <= current_time + step_seconds + 1e-6 {
                    if let Some(t) = tracker.as_ref() {
                        let iso = t.build_isochrone_envelope(
                            next_iso_time,
                            iso_band,
                            base.start,
                            base.envelope_sector_deg,
                        );
                        if !iso.points.is_empty() {
                            isochrones.push(iso);
                        }
                    }
                    next_iso_time += base.isochrone_step_hours * 3600.0;
                    if next_iso_time > time_limit + iso_band {
                        break;
                    }
                }
            }

            if self.config.stop_on_arrival && best_arrival_time.is_finite() {
                break;
            }

            layer = next_layer;
            current_time += step_seconds;
        }

        if build_isos {
            if next_iso_time <= time_limit + iso_band {
                if let Some(t) = tracker.as_ref() {
                    let iso = t.build_isochrone_envelope(
                        next_iso_time,
                        iso_band,
                        base.start,
                        base.envelope_sector_deg,
                    );
                    if !iso.points.is_empty() {
                        isochrones.push(iso);
                    }
                }
            }
        }

        let t_back = Instant::now();
        let (best_route, best_route_headings, best_eta_hours, best_cost, route_legs) =
            self.reconstruct_best_route(
            &arrival_records,
            &cells,
            step_hours,
            self.config.optimize_cost,
        );
        let backtrack_wall = t_back.elapsed();

        if profile_on {
            let report = RoutingProfile {
                layers: profile.layers.load(Ordering::Relaxed),
                nodes_expanded: profile.nodes.load(Ordering::Relaxed),
                expansions: profile.expansions.load(Ordering::Relaxed),
                successors_raw: profile.succ_raw.load(Ordering::Relaxed),
                successors_kept: profile.succ_kept.load(Ordering::Relaxed),
                landmask_checks: profile.landmask.load(Ordering::Relaxed),
                visited_cells: cells.len(),
                expand_wall,
                merge_wall,
                backtrack_wall,
            };
            print_routing_profile(&report);
        }

        let mut arrival_envelopes = if self.config.build_arrival_envelopes {
            if let Some(dest_pt) = dest {
                build_arrival_envelopes(
                    &isochrones,
                    dest_pt,
                    self.config.arrival_radius_m,
                    self.config.envelope_step_minutes,
                )
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        };

        for env in &mut arrival_envelopes {
            simplify_envelope_boundary(env);
        }

        SotaRoutingResult {
            isochrones,
            arrival_envelopes,
            best_route,
            best_route_headings,
            route_legs,
            best_eta_hours,
            best_cost,
            scenario_id: self.scenario_id.clone(),
            eta_percentiles: None,
        }
    }

    fn resolve_env(&self, at: Point, sim_time: f64) -> EnvSnapshot {
        let t = self.start_time + ChronoDuration::seconds(sim_time as i64);
        let (wind, current, sea_state) = if self.grib.is_time_invariant() {
            let (wind, current, sea_state) = self.grib.get_environment(&at, self.start_time);
            (
                wind.unwrap_or(Wind::new(270.0, 10.0)),
                current.unwrap_or(Current::new(90.0, 0.5)),
                sea_state.unwrap_or(SeaState::new(1.0, 8.0, 270.0)),
            )
        } else {
            let (wind, current, sea_state) = self.grib.get_environment(&at, t);
            (
                wind.unwrap_or(Wind::new(270.0, 10.0)),
                current.unwrap_or(Current::new(90.0, 0.5)),
                sea_state.unwrap_or(SeaState::new(1.0, 8.0, 270.0)),
            )
        };
        let current_dir_rad = current.direction.to_radians();
        EnvSnapshot {
            wind,
            current,
            sea_state,
            current_vx: current.speed * current_dir_rad.sin(),
            current_vy: current.speed * current_dir_rad.cos(),
        }
    }

    fn build_heading_kinematics(&self, headings: &[f64], env: &EnvSnapshot) -> Vec<HeadingKinematics> {
        headings
            .iter()
            .map(|&heading| {
                let angle = angle_au_vent(heading, env.wind.direction);
                let base_speed = self.polar.speed_ms(angle, env.wind.speed);
                let factor = self.sea_modifier.speed_factor(angle, &env.sea_state);
                let h_rad = heading.to_radians();
                HeadingKinematics {
                    boat_speed: base_speed * factor,
                    sin_h: h_rad.sin(),
                    cos_h: h_rad.cos(),
                }
            })
            .collect()
    }

    fn env_at(&self, point: Point, time: f64, cached: EnvSnapshot) -> EnvSnapshot {
        if self.grib.is_time_invariant() {
            cached
        } else {
            self.resolve_env(point, time)
        }
    }

    fn expand_layer_node(
        &self,
        node: &SotaNode,
        headings: &[f64],
        heading_kinematics: &[HeadingKinematics],
        step_seconds: f64,
        time_limit: f64,
        dest: Option<Point>,
        cached_env: &EnvSnapshot,
        track_cost: bool,
        best_arrival_time: f64,
        profile: Option<&ProfileCounters>,
    ) -> Option<LayerExpansion> {
        if node.time > time_limit {
            return None;
        }

        if self.config.enable_destination_prune {
            if let Some(dest_pt) = dest {
                if best_arrival_time.is_finite() {
                    let optimistic =
                        optimistic_eta_hours(&node.point, &dest_pt, self.max_boat_speed_ms);
                    if node.time / 3600.0 + optimistic
                        > best_arrival_time / 3600.0 + self.config.destination_prune_slack_hours
                    {
                        return None;
                    }
                }
            }
        }

        let key = node.cell_key;
        let env = self.env_at(node.point, node.time, *cached_env);

        if violates_constraints(&self.config.constraints, &env.wind, &env.sea_state) {
            return None;
        }

        let max_distance = self.config.base.max_distance_meters;
        let new_time = node.time + step_seconds;
        if new_time > time_limit {
            let arrival = self.arrival_record(node, key, dest);
            return Some(LayerExpansion {
                key,
                point: node.point,
                heading: node.heading,
                time: node.time,
                successors: Vec::new(),
                arrival,
            });
        }

        let mut successors = Vec::with_capacity(headings.len());
        for (i, &heading) in headings.iter().enumerate() {
            if let Some(succ) = self.expand_heading(
                node,
                heading,
                heading_kinematics[i],
                step_seconds,
                new_time,
                max_distance,
                &env,
                track_cost,
            ) {
                successors.push(succ);
            }
        }

        if let Some(p) = profile {
            p.expansions.fetch_add(1, Ordering::Relaxed);
            p.succ_raw.fetch_add(successors.len(), Ordering::Relaxed);
        }

        if successors.is_empty() {
            let arrival = self.arrival_record(node, key, dest);
            return Some(LayerExpansion {
                key,
                point: node.point,
                heading: node.heading,
                time: node.time,
                successors: Vec::new(),
                arrival,
            });
        }

        let arrival = self.arrival_record(node, key, dest);

        Some(LayerExpansion {
            key,
            point: node.point,
            heading: node.heading,
            time: node.time,
            successors,
            arrival,
        })
    }

    fn arrival_record(
        &self,
        node: &SotaNode,
        key: NodeKey,
        dest: Option<Point>,
    ) -> Option<(NodeKey, f64, f64)> {
        let dest_pt = dest?;
        if node.point.distance_to(&dest_pt) <= self.config.arrival_radius_m {
            Some((key, node.time, node.cost.total(&self.weights)))
        } else {
            None
        }
    }

    fn expand_heading(
        &self,
        node: &SotaNode,
        heading: f64,
        _kin: HeadingKinematics,
        step_seconds: f64,
        new_time: f64,
        max_distance: f64,
        env: &EnvSnapshot,
        track_cost: bool,
    ) -> Option<SuccCandidate> {
        let angle = angle_au_vent(heading, env.wind.direction);
        if angle + 1e-6 < MIN_ANGLE_AU_VENT_DEG {
            return None;
        }
        let boat_speed = self.polar.speed_ms(angle, env.wind.speed)
            * self.sea_modifier.speed_factor(angle, &env.sea_state);
        if boat_speed < 0.05 {
            return None;
        }

        let h_rad = heading.to_radians();
        let boat_vx = boat_speed * h_rad.sin();
        let boat_vy = boat_speed * h_rad.cos();
        let eff_vx = boat_vx + env.current_vx;
        let eff_vy = boat_vy + env.current_vy;
        let eff_speed = (eff_vx * eff_vx + eff_vy * eff_vy).sqrt();
        if eff_speed < 0.05 {
            return None;
        }

        let distance = eff_speed * step_seconds;
        if distance > max_distance {
            return None;
        }

        let eff_dir = eff_vx.atan2(eff_vy).to_degrees().rem_euclid(360.0);
        let new_point = crate::geometry::move_from_point_fast(&node.point, eff_dir, distance);
        let dist_from_start = node.dist_from_start + distance;
        let cell_key = point_key_from_dist(&new_point, dist_from_start);

        let cost = if track_cost {
            let step = step_cost(
                step_seconds,
                Some(node.heading),
                heading,
                &env.wind,
                &env.current,
                &env.sea_state,
                &self.weights,
            );
            node.cost.add(&step)
        } else {
            CostComponents {
                eta_seconds: new_time,
                ..Default::default()
            }
        };

        Some(SuccCandidate {
            point: new_point,
            time: new_time,
            heading,
            cost,
            cell_key,
            dist_from_start,
            wind: env.wind,
            sea_state: env.sea_state,
        })
    }

    fn point_key(&self, point: &Point) -> NodeKey {
        let distance = self.config.base.start.distance_to(point);
        point_key_from_dist(point, distance)
    }

    fn reconstruct_best_route(
        &self,
        arrivals: &[(NodeKey, f64, f64)],
        cells: &FxHashMap<NodeKey, CellRecord>,
        step_hours: f64,
        optimize_cost: bool,
    ) -> (
        Option<Vec<Point>>,
        Vec<f64>,
        Option<f64>,
        Option<f64>,
        Vec<RouteLeg>,
    ) {
        if arrivals.is_empty() {
            return (None, Vec::new(), None, None, Vec::new());
        }

        let best = if optimize_cost {
            arrivals
                .iter()
                .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
        } else {
            arrivals
                .iter()
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        }
        .unwrap();

        let eta = best.1 / 3600.0;
        let cost = best.2;

        let raw_chain = backtrack_from_cells(cells, best.0);
        let route: Vec<Point> = raw_chain.iter().map(|c| c.point).collect();
        let headings: Vec<f64> = raw_chain.iter().map(|c| c.heading).collect();
        let wind_samples: Vec<(Wind, SeaState)> = raw_chain
            .iter()
            .map(|c| (c.wind, c.sea_state))
            .collect();

        let legs = build_route_legs(&route, &headings, step_hours, &wind_samples);

        let best_route = if route.is_empty() { None } else { Some(route) };

        (best_route, headings, Some(eta), Some(cost), legs)
    }
}

#[inline]
fn grid_precision_for_dist(dist_m: f64) -> f64 {
    if dist_m < 50_000.0 {
        0.0063
    } else if dist_m < 200_000.0 {
        0.0081
    } else {
        0.0099
    }
}

#[inline]
fn point_key_from_dist(point: &Point, dist_from_start: f64) -> NodeKey {
    let precision = grid_precision_for_dist(dist_from_start);
    (
        (point.lat / precision).round() as i32,
        (point.lon / precision).round() as i32,
    )
}

fn backtrack_from_cells(cells: &FxHashMap<NodeKey, CellRecord>, arrival_key: NodeKey) -> Vec<CellRecord> {
    let mut chain = Vec::new();
    let mut current = Some(arrival_key);
    let mut guard = 0usize;
    while let Some(k) = current {
        let Some(cell) = cells.get(&k) else {
            break;
        };
        chain.push(*cell);
        current = cell.parent;
        guard += 1;
        if guard > 50_000 {
            break;
        }
    }
    chain.reverse();
    chain
}

/// Public entry point for SOTA routing
pub fn calculate_sota_routing(
    config: SotaRoutingConfig,
    weights: ObjectiveWeights,
    landmask: Landmask,
    polar: Box<dyn Polar + Send + Sync>,
    grib: Box<dyn GribProvider + Send + Sync>,
    start_time: DateTime<Utc>,
) -> SotaRoutingResult {
    SotaIsochroneRouter::new(config, weights, landmask, polar, grib, start_time).calculate()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grib::SimpleGribProvider;
    use crate::polar::SimplePolar;

    #[test]
    fn sota_routing_produces_isochrones() {
        let config = SotaRoutingConfig {
            base: IsochroneConfig {
                time_limit_hours: 2.0,
                isochrone_step_hours: 1.0,
                ..IsochroneConfig::default()
            },
            ..SotaRoutingConfig::default()
        };
        let landmask = Landmask::new().unwrap();
        let result = calculate_sota_routing(
            config,
            ObjectiveWeights::default(),
            landmask,
            Box::new(SimplePolar::default_voilier()),
            Box::new(SimpleGribProvider::default()),
            Utc::now(),
        );
        assert!(!result.isochrones.is_empty());
    }

    #[test]
    fn constraints_reduce_expansion() {
        let base_config = SotaRoutingConfig {
            base: IsochroneConfig {
                time_limit_hours: 1.0,
                ..IsochroneConfig::default()
            },
            enable_destination_prune: false,
            ..SotaRoutingConfig::default()
        };
        let landmask = Landmask::new().unwrap();
        let polar = Box::new(SimplePolar::default_voilier());
        let grib = Box::new(SimpleGribProvider::default());

        let mut loose = base_config.clone();
        loose.constraints = RoutingConstraints::unlimited();
        let loose_result = calculate_sota_routing(
            loose,
            ObjectiveWeights::default(),
            landmask.clone(),
            polar.clone(),
            grib.clone(),
            Utc::now(),
        );

        let mut strict = base_config;
        strict.constraints = RoutingConstraints {
            max_true_wind_ms: Some(5.0),
            max_significant_wave_m: Some(0.5),
            min_depth_m: None,
        };
        let strict_result = calculate_sota_routing(
            strict,
            ObjectiveWeights::default(),
            landmask,
            polar,
            grib,
            Utc::now(),
        );

        let loose_points: usize = loose_result
            .isochrones
            .iter()
            .map(|iso| iso.points.len())
            .sum();
        let strict_points: usize = strict_result
            .isochrones
            .iter()
            .map(|iso| iso.points.len())
            .sum();

        assert!(!loose_result.isochrones.is_empty());
        assert!(strict_points <= loose_points);
    }
}
