use crate::constraints::{optimistic_eta_hours, violates_constraints};
use crate::envelope::{build_arrival_envelopes, simplify_envelope_boundary};
use crate::geometry::*;
use crate::grid::{resolve_grid_spec, CellKey, GridBestTracker, RoutingGrid};
use crate::grib::GribProvider;
use crate::landmask::Landmask;
use crate::objective::{step_cost, CostComponents, ObjectiveWeights};
use crate::polar::{angle_au_vent, Polar};
use crate::route::{backtrack_route, build_route_legs, simplify_route};
use crate::sea_state::{DefaultSeaStateModifier, SeaStatePolarModifier};
use crate::types::*;
use chrono::{DateTime, Duration, Utc};
use ordered_float::OrderedFloat;
use rayon::prelude::*;
use std::collections::{BinaryHeap, HashMap};

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

#[derive(Debug, Clone)]
struct SotaNode {
    point: Point,
    time: f64,
    heading: f64,
    cost: CostComponents,
    weighted_cost: f64,
    parent_key: Option<CellKey>,
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
        OrderedFloat(other.weighted_cost).partial_cmp(&OrderedFloat(self.weighted_cost))
    }
}

impl Ord for SotaNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl SotaNode {
    fn prune_key(&self, optimize_cost: bool) -> f64 {
        if optimize_cost {
            self.weighted_cost
        } else {
            self.time
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

        let grid_spec = resolve_grid_spec(
            self.grib.as_ref(),
            base.start,
            base.destination,
            base.grid_step_deg,
        );
        let routing_grid = RoutingGrid::from_spec(grid_spec, &self.landmask);
        let mut tracker = GridBestTracker::new(routing_grid);
        let grid = tracker.grid().clone();

        let mut visited: HashMap<CellKey, f64> = HashMap::new();
        let mut parent_map: HashMap<CellKey, CellKey> = HashMap::new();
        let mut point_map: HashMap<CellKey, Point> = HashMap::new();
        let mut heading_map: HashMap<CellKey, f64> = HashMap::new();
        let mut frontier = BinaryHeap::new();

        let start_cell = grid.cell_containing(&base.start);
        let start_node = SotaNode {
            point: base.start,
            time: 0.0,
            heading: 0.0,
            cost: CostComponents::default(),
            weighted_cost: 0.0,
            parent_key: None,
        };
        visited.insert(start_cell, 0.0);
        point_map.insert(start_cell, base.start);
        let _ = tracker.try_update(base.start, 0.0, &self.landmask);

        frontier.push(start_node);

        let mut isochrones: Vec<Isochrone> = Vec::new();
        let mut next_iso_time = base.isochrone_step_hours * 3600.0;

        let mut arrival_records: Vec<(CellKey, f64, f64)> = Vec::new();
        let dest = base.destination;
        let mut best_arrival_time = f64::INFINITY;

        let max_nodes = 500_000;
        let mut nodes_explored = 0usize;
        let iso_band = base.isochrone_step_hours * 3600.0;

        while let Some(node) = frontier.pop() {
            nodes_explored += 1;
            if nodes_explored > max_nodes {
                break;
            }

            let cell_key = grid.cell_containing(&node.point);
            let node_key_cost = node.prune_key(self.config.optimize_cost);

            if visited.get(&cell_key).copied().unwrap_or(f64::INFINITY) + 1e-6 < node_key_cost {
                continue;
            }
            visited.insert(cell_key, node_key_cost);

            if node.time > time_limit {
                continue;
            }

            // Destination cone pruning
            if self.config.enable_destination_prune {
                if let Some(dest_pt) = dest {
                    let optimistic = optimistic_eta_hours(
                        &node.point,
                        &dest_pt,
                        self.max_boat_speed_ms,
                    );
                    if node.time / 3600.0 + optimistic
                        > best_arrival_time / 3600.0 + self.config.destination_prune_slack_hours
                    {
                        continue;
                    }
                }
            }

            // Exclude land points; arrival time is kept at exact coordinates.
            if node.time > 0.0 && !self.landmask.is_sea(&node.point) {
                continue;
            }

            if node.time > 0.0 {
                tracker.try_update(node.point, node.time, &self.landmask);
            }

            point_map.insert(cell_key, node.point);
            heading_map.insert(cell_key, node.heading);
            if let Some(pk) = node.parent_key {
                parent_map.insert(cell_key, pk);
            }

            if let Some(dest_pt) = dest {
                if node.point.distance_to(&dest_pt) <= self.config.arrival_radius_m {
                    let total = node.cost.total(&self.weights);
                    arrival_records.push((cell_key, node.time, total));
                    if node.time < best_arrival_time {
                        best_arrival_time = node.time;
                    }
                }
            }

            if node.time >= next_iso_time {
                let iso = tracker.build_isochrone_envelope(
                    next_iso_time,
                    iso_band,
                    base.start,
                    base.envelope_sector_deg,
                );
                if !iso.points.is_empty() {
                    isochrones.push(iso);
                }
                next_iso_time += base.isochrone_step_hours * 3600.0;
            }

            for succ in self.expand(&node, &grid) {
                if succ.time > 0.0 && !self.landmask.is_sea(&succ.point) {
                    continue;
                }
                let skey = grid.cell_containing(&succ.point);
                let succ_key_cost = succ.prune_key(self.config.optimize_cost);
                if succ_key_cost < visited.get(&skey).copied().unwrap_or(f64::INFINITY) {
                    frontier.push(SotaNode {
                        parent_key: Some(cell_key),
                        ..succ
                    });
                }
            }
        }

        if next_iso_time <= time_limit + iso_band {
            let iso = tracker.build_isochrone_envelope(
                next_iso_time,
                iso_band,
                base.start,
                base.envelope_sector_deg,
            );
            if !iso.points.is_empty() {
                isochrones.push(iso);
            }
        }

        let (best_route, best_eta_hours, best_cost, route_legs) = self.reconstruct_best_route(
            &arrival_records,
            &parent_map,
            &point_map,
            &heading_map,
            &grid,
            step_hours,
        );

        let mut arrival_envelopes = if let Some(dest_pt) = dest {
            build_arrival_envelopes(
                &isochrones,
                dest_pt,
                self.config.arrival_radius_m,
                self.config.envelope_step_minutes,
            )
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
            route_legs,
            best_eta_hours,
            best_cost,
            scenario_id: self.scenario_id.clone(),
            eta_percentiles: None,
        }
    }

    fn expand(&self, node: &SotaNode, _grid: &RoutingGrid) -> Vec<SotaNode> {
        let base = &self.config.base;
        let step_seconds = base.simulation_step_seconds();
        let current_time = self.start_time + Duration::seconds(node.time as i64);
        let direction_step = base.direction_step_degrees();

        let (wind, current, sea_state) = self.grib.get_environment(&node.point, current_time);
        let wind = wind.unwrap_or(Wind::new(270.0, 10.0));
        let current = current.unwrap_or(Current::new(90.0, 0.5));
        let sea_state = sea_state.unwrap_or(SeaState::new(1.0, 8.0, wind.direction));

        if violates_constraints(&self.config.constraints, &wind, &sea_state) {
            return Vec::new();
        }

        let directions: Vec<f64> = (0..base.num_directions)
            .map(|i| i as f64 * direction_step)
            .collect();

        let candidates: Vec<SotaNode> = directions
            .par_iter()
            .filter_map(|&heading| {
                let angle = angle_au_vent(heading, wind.direction);
                let base_speed = self.polar.speed_ms(angle, wind.speed);
                let factor = self.sea_modifier.speed_factor(angle, &sea_state);
                let boat_speed = base_speed * factor;

                if boat_speed < 0.05 {
                    return None;
                }

                let (eff_speed, eff_dir) =
                    calculate_effective_velocity(boat_speed, heading, &current);
                let distance = eff_speed * step_seconds;

                if distance > base.max_distance_meters {
                    return None;
                }

                let new_point = move_from_point(&node.point, eff_dir, distance);
                if node.time > 0.0 && !self.landmask.is_sea(&new_point) {
                    return None;
                }

                let step = step_cost(
                    step_seconds,
                    Some(node.heading),
                    heading,
                    &wind,
                    &current,
                    &sea_state,
                    &self.weights,
                );
                let new_cost = node.cost.add(&step);
                let weighted = new_cost.total(&self.weights);

                Some(SotaNode {
                    point: new_point,
                    time: node.time + step_seconds,
                    heading,
                    cost: new_cost,
                    weighted_cost: weighted,
                    parent_key: None,
                })
            })
            .collect();

        candidates
    }

    fn reconstruct_best_route(
        &self,
        arrivals: &[(CellKey, f64, f64)],
        parent_map: &HashMap<CellKey, CellKey>,
        point_map: &HashMap<CellKey, Point>,
        heading_map: &HashMap<CellKey, f64>,
        grid: &RoutingGrid,
        step_hours: f64,
    ) -> (Option<Vec<Point>>, Option<f64>, Option<f64>, Vec<RouteLeg>) {
        if arrivals.is_empty() {
            return (None, None, None, Vec::new());
        }

        let best = arrivals
            .iter()
            .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();

        let eta = best.1 / 3600.0;
        let cost = best.2;

        let raw_route = backtrack_route(
            self.config.base.start,
            best.0,
            parent_map,
            point_map,
        );

        let route = if raw_route.len() > 200 {
            simplify_route(&raw_route, 200)
        } else {
            raw_route
        };

        let headings: Vec<f64> = route
            .iter()
            .filter_map(|p| {
                let key = grid.cell_containing(p);
                heading_map.get(&key).copied()
            })
            .collect();

        let wind_samples: Vec<(Wind, SeaState)> = route
            .iter()
            .map(|p| {
                let t = self.start_time;
                let (w, _, s) = self.grib.get_environment(p, t);
                (
                    w.unwrap_or(Wind::new(270.0, 10.0)),
                    s.unwrap_or(SeaState::default()),
                )
            })
            .collect();

        let legs = build_route_legs(&route, &headings, step_hours, &wind_samples);

        let best_route = if route.is_empty() { None } else { Some(route) };

        (best_route, Some(eta), Some(cost), legs)
    }
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
