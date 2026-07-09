use crate::envelope::{build_arrival_envelopes, simplify_envelope_boundary};
use crate::isochrone::simplify_isochrone_public;
use crate::geometry::*;
use crate::grib::GribProvider;
use crate::landmask::Landmask;
use crate::objective::{step_cost, CostComponents, ObjectiveWeights};
use crate::polar::{angle_au_vent, Polar};
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
}

#[derive(Debug, Clone)]
struct SotaNode {
    point: Point,
    time: f64,
    heading: f64,
    cost: CostComponents,
    weighted_cost: f64,
    parent_key: Option<(i32, i32)>,
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
        // Min-heap on weighted cost
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
        }
    }

    pub fn with_sea_modifier(mut self, modifier: DefaultSeaStateModifier) -> Self {
        self.sea_modifier = modifier;
        self
    }

    pub fn calculate(&self) -> SotaRoutingResult {
        let base = &self.config.base;
        let step_seconds = base.simulation_step_seconds();
        let time_limit = base.time_limit_hours * 3600.0;

        let mut visited: HashMap<(i32, i32), f64> = HashMap::new();
        let mut parent_map: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
        let mut heading_map: HashMap<(i32, i32), f64> = HashMap::new();
        let mut cost_map: HashMap<(i32, i32), CostComponents> = HashMap::new();
        let mut frontier = BinaryHeap::new();

        let start_key = self.point_key(&base.start);
        let start_node = SotaNode {
            point: base.start,
            time: 0.0,
            heading: 0.0,
            cost: CostComponents::default(),
            weighted_cost: 0.0,
            parent_key: None,
        };
        frontier.push(start_node);
        visited.insert(start_key, 0.0);
        cost_map.insert(start_key, CostComponents::default());

        let mut isochrones: Vec<Isochrone> = Vec::new();
        let mut next_iso_time = base.isochrone_step_hours * 3600.0;
        let mut current_iso_points: Vec<Point> = Vec::new();

        let mut arrival_records: Vec<(Point, f64, f64)> = Vec::new(); // point, eta, cost
        let dest = base.destination;

        let max_nodes = 500_000;

        let mut nodes_explored = 0usize;

        while let Some(node) = frontier.pop() {
            nodes_explored += 1;
            if nodes_explored > max_nodes {
                break;
            }

            let key = self.point_key(&node.point);
            let node_key_cost = node.prune_key(self.config.optimize_cost);

            if visited.get(&key).copied().unwrap_or(f64::INFINITY) + 1e-6 < node_key_cost {
                continue;
            }
            visited.insert(key, node_key_cost);

            if node.time > time_limit {
                continue;
            }

            let is_sea = if node.time == 0.0 {
                true
            } else {
                self.landmask.is_sea(&node.point)
            };

            if !is_sea && node.time > 0.0 {
                continue;
            }

            heading_map.insert(key, node.heading);
            cost_map.insert(key, node.cost);
            if let Some(pk) = node.parent_key {
                parent_map.insert(key, pk);
            }

            // Check arrival at destination
            if let Some(dest_pt) = dest {
                if node.point.distance_to(&dest_pt) <= self.config.arrival_radius_m {
                    let total = node.cost.total(&self.weights);
                    arrival_records.push((node.point, node.time, total));
                }
            }

            if node.time >= next_iso_time - step_seconds {
                current_iso_points.push(node.point);
            }
            if node.time >= next_iso_time {
                if !current_iso_points.is_empty() {
                    isochrones.push(Isochrone {
                        time_hours: next_iso_time / 3600.0,
                        points: current_iso_points.clone(),
                    });
                    current_iso_points.clear();
                }
                next_iso_time += base.isochrone_step_hours * 3600.0;
            }

            let successors = self.expand(&node);
            for succ in successors {
                let skey = self.point_key(&succ.point);
                let succ_key_cost = succ.prune_key(self.config.optimize_cost);

                if succ_key_cost < visited.get(&skey).copied().unwrap_or(f64::INFINITY) {
                    frontier.push(succ);
                }
            }
        }

        if !current_iso_points.is_empty() {
            isochrones.push(Isochrone {
                time_hours: next_iso_time / 3600.0,
                points: current_iso_points,
            });
        }

        // Simplify isochrones
        isochrones.par_iter_mut().for_each(|iso| {
            simplify_isochrone_public(iso);
        });

        // Best route reconstruction
        let (best_route, best_eta_hours, best_cost) =
            self.reconstruct_best_route(&arrival_records, &parent_map, dest);

        // Build arrival envelopes
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
            best_eta_hours,
            best_cost,
        }
    }

    fn expand(&self, node: &SotaNode) -> Vec<SotaNode> {
        let base = &self.config.base;
        let step_seconds = base.simulation_step_seconds();
        let current_time = self.start_time + Duration::seconds(node.time as i64);
        let direction_step = base.direction_step_degrees();

        let (wind, current, sea_state) = self.grib.get_environment(&node.point, current_time);
        let wind = wind.unwrap_or(Wind::new(270.0, 10.0));
        let current = current.unwrap_or(Current::new(90.0, 0.5));
        let sea_state = sea_state.unwrap_or(SeaState::new(1.0, 8.0, wind.direction));

        let directions: Vec<f64> = (0..base.num_directions)
            .map(|i| i as f64 * direction_step)
            .collect();

        let parent_key = Some(self.point_key(&node.point));

        let mut candidates: Vec<SotaNode> = directions
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
                    parent_key,
                })
            })
            .collect();

        if !candidates.is_empty() {
            let points: Vec<Point> = candidates.iter().map(|n| n.point).collect();
            let are_sea = self.landmask.are_sea(&points);
            candidates = candidates
                .into_iter()
                .enumerate()
                .filter_map(|(i, c)| {
                    if are_sea.get(i).copied().unwrap_or(false) || c.time == step_seconds {
                        Some(c)
                    } else {
                        None
                    }
                })
                .collect();
        }

        candidates
    }

    fn point_key(&self, point: &Point) -> (i32, i32) {
        let distance = self.config.base.start.distance_to(point);
        let precision = if distance < 50_000.0 {
            0.0063
        } else if distance < 200_000.0 {
            0.0081
        } else {
            0.0099
        };
        (
            (point.lat / precision).round() as i32,
            (point.lon / precision).round() as i32,
        )
    }

    fn reconstruct_best_route(
        &self,
        arrivals: &[(Point, f64, f64)],
        _parent_map: &HashMap<(i32, i32), (i32, i32)>,
        dest: Option<Point>,
    ) -> (Option<Vec<Point>>, Option<f64>, Option<f64>) {
        if arrivals.is_empty() {
            return (None, None, None);
        }

        let best = arrivals
            .iter()
            .min_by(|a, b| {
                a.2.partial_cmp(&b.2)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();

        let eta = best.1 / 3600.0;
        let cost = best.2;

        // Simple route: start -> destination (full backtracking would need stored paths)
        let route = dest.map(|d| vec![self.config.base.start, d]);

        (route, Some(eta), Some(cost))
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
}
