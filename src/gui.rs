#[cfg(feature = "gui")]
mod gui_impl {
    use crate::{
        calculate_dual_routing, calculate_sota_routing, default_scenarios, route_to_gpx,
        tack_decision_eta, BufrGribGridProvider, DualRoutingResult, GribProvider, Isochrone,
        Landmask, ObjectiveWeights, OpponentState, Point, Polar, SotaRoutingConfig,
        SotaRoutingResult, ArrivalEnvelope, WeatherScenario, default_routing_polar,
    };
    use chrono::Utc;
    use eframe::egui;
    use poll_promise::Promise;
    use walkers::{
        lon_lat, sources::OpenStreetMap, HttpTiles, Map, MapMemory, Plugin, Projector,
    };

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum ViewLayer {
        Isochrones,
        Envelopes,
        Both,
    }

    enum ComputeJob {
        Single(Promise<Result<SotaRoutingResult, String>>),
        Dual(Promise<Result<DualRoutingResult, String>>),
    }

    struct RoutingMapPlugin {
        isochrones: Vec<Isochrone>,
        opponent_isochrones: Vec<Isochrone>,
        scenario_routes: Vec<Vec<Point>>,
        envelopes: Vec<ArrivalEnvelope>,
        start: Point,
        opponent_pos: Option<Point>,
        destination: Option<Point>,
        best_route: Option<Vec<Point>>,
        opponent_route: Option<Vec<Point>>,
        show_isochrones: bool,
        show_envelopes: bool,
        show_opponent: bool,
        show_scenario_fan: bool,
    }

    impl Plugin for RoutingMapPlugin {
        fn run(
            self: Box<Self>,
            ui: &mut egui::Ui,
            _response: &egui::Response,
            projector: &Projector,
            _map_memory: &MapMemory,
        ) {
            let painter = ui.painter();

            if self.show_scenario_fan {
                for route in &self.scenario_routes {
                    if route.len() >= 2 {
                        let screen: Vec<egui::Pos2> = route
                            .iter()
                            .map(|p| {
                                let v = projector.project(lon_lat(p.lon, p.lat));
                                egui::pos2(v.x, v.y)
                            })
                            .collect();
                        for w in screen.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(1.5, egui::Color32::from_rgba_unmultiplied(255, 255, 100, 80)),
                            );
                        }
                    }
                }
            }

            if self.show_isochrones {
                let colors = [
                    egui::Color32::from_rgba_unmultiplied(255, 50, 50, 160),
                    egui::Color32::from_rgba_unmultiplied(255, 120, 30, 160),
                    egui::Color32::from_rgba_unmultiplied(255, 200, 50, 160),
                    egui::Color32::from_rgba_unmultiplied(120, 220, 50, 160),
                    egui::Color32::from_rgba_unmultiplied(50, 180, 255, 160),
                ];
                for (idx, iso) in self.isochrones.iter().enumerate() {
                    let color = colors[idx % colors.len()];
                    for pt in &iso.points {
                        let pos = projector.project(lon_lat(pt.lon, pt.lat));
                        painter.circle_filled(egui::pos2(pos.x, pos.y), 2.0, color);
                    }
                }
            }

            if self.show_opponent {
                for iso in &self.opponent_isochrones {
                    for pt in &iso.points {
                        let pos = projector.project(lon_lat(pt.lon, pt.lat));
                        painter.circle_filled(
                            egui::pos2(pos.x, pos.y),
                            2.0,
                            egui::Color32::from_rgba_unmultiplied(255, 80, 200, 140),
                        );
                    }
                }
                if let Some(route) = &self.opponent_route {
                    if route.len() >= 2 {
                        let screen: Vec<egui::Pos2> = route
                            .iter()
                            .map(|p| {
                                let v = projector.project(lon_lat(p.lon, p.lat));
                                egui::pos2(v.x, v.y)
                            })
                            .collect();
                        for w in screen.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(2.5, egui::Color32::from_rgb(255, 100, 180)),
                            );
                        }
                    }
                }
            }

            if self.show_envelopes {
                for env in self.envelopes.iter() {
                    let color = egui::Color32::from_rgba_unmultiplied(160, 80, 255, 180);
                    if env.boundary_points.len() >= 2 {
                        let screen: Vec<egui::Pos2> = env
                            .boundary_points
                            .iter()
                            .map(|p| {
                                let v = projector.project(lon_lat(p.lon, p.lat));
                                egui::pos2(v.x, v.y)
                            })
                            .collect();
                        for w in screen.windows(2) {
                            painter.line_segment([w[0], w[1]], egui::Stroke::new(2.0, color));
                        }
                        if let (Some(first), Some(last)) = (screen.first(), screen.last()) {
                            painter.line_segment([*last, *first], egui::Stroke::new(1.5, color));
                        }
                    }
                }
            }

            if let Some(route) = &self.best_route {
                if route.len() >= 2 {
                    let screen: Vec<egui::Pos2> = route
                        .iter()
                        .map(|p| {
                            let v = projector.project(lon_lat(p.lon, p.lat));
                            egui::pos2(v.x, v.y)
                        })
                        .collect();
                    for w in screen.windows(2) {
                        painter.line_segment(
                            [w[0], w[1]],
                            egui::Stroke::new(3.0, egui::Color32::from_rgb(0, 220, 255)),
                        );
                    }
                }
            }

            let start_v = projector.project(lon_lat(self.start.lon, self.start.lat));
            let start_pos = egui::pos2(start_v.x, start_v.y);
            painter.circle_filled(start_pos, 8.0, egui::Color32::GREEN);

            if let Some(opp) = self.opponent_pos {
                let v = projector.project(lon_lat(opp.lon, opp.lat));
                painter.circle_filled(egui::pos2(v.x, v.y), 7.0, egui::Color32::from_rgb(255, 120, 200));
            }

            if let Some(dest) = self.destination {
                let v = projector.project(lon_lat(dest.lon, dest.lat));
                painter.circle_filled(egui::pos2(v.x, v.y), 8.0, egui::Color32::RED);
            }
        }
    }

    pub struct SotaRoutingApp {
        config: SotaRoutingConfig,
        weights: ObjectiveWeights,
        result: Option<SotaRoutingResult>,
        dual_result: Option<DualRoutingResult>,
        opponent: OpponentState,
        opponent_enabled: bool,
        start: Point,
        destination: Option<Point>,
        tiles: Option<HttpTiles>,
        map_memory: MapMemory,
        view_layer: ViewLayer,
        computing: bool,
        compute_job: Option<ComputeJob>,
        status_message: String,
        use_bufr_grid: bool,
        scenarios: Vec<(WeatherScenario, bool)>,
        show_scenario_fan: bool,
    }

    impl SotaRoutingApp {
        pub fn new(start: Point, destination: Option<Point>) -> Self {
            let mut config = SotaRoutingConfig::default();
            config.base.start = start;
            config.base.destination = destination;
            config.base.time_limit_hours = 8.0;

            let scenarios: Vec<(WeatherScenario, bool)> = default_scenarios()
                .into_iter()
                .map(|s| {
                    let on = s.id == "baseline" || s.id == "front_early";
                    (s, on)
                })
                .collect();

            Self {
                config,
                weights: ObjectiveWeights::default(),
                result: None,
                dual_result: None,
                opponent: OpponentState::default(),
                opponent_enabled: true,
                start,
                destination,
                tiles: None,
                map_memory: MapMemory::default(),
                view_layer: ViewLayer::Both,
                computing: false,
                compute_job: None,
                status_message: "Ready — Compute or Dual (opponent + scenarios).".into(),
                use_bufr_grid: true,
                scenarios,
                show_scenario_fan: true,
            }
        }

        fn build_grib(start: Point, dest: Option<Point>, use_bufr: bool) -> Box<dyn GribProvider + Send + Sync> {
            if use_bufr {
                let min_lat = start.lat.min(dest.map(|d| d.lat).unwrap_or(start.lat)) - 2.0;
                let max_lat = start.lat.max(dest.map(|d| d.lat).unwrap_or(start.lat)) + 2.0;
                let min_lon = start.lon.min(dest.map(|d| d.lon).unwrap_or(start.lon)) - 3.0;
                let max_lon = start.lon.max(dest.map(|d| d.lon).unwrap_or(start.lon)) + 3.0;
                Box::new(BufrGribGridProvider::synthetic_mediterranean(
                    min_lat, max_lat, min_lon, max_lon, 0.5,
                ))
            } else {
                Box::new(crate::SimpleGribProvider::default())
            }
        }

        fn run_single(
            config: SotaRoutingConfig,
            weights: ObjectiveWeights,
            start: Point,
            use_bufr: bool,
        ) -> Result<SotaRoutingResult, String> {
            let landmask = Landmask::new().map_err(|e| e.to_string())?;
            let grib = Self::build_grib(start, config.base.destination, use_bufr);
            Ok(calculate_sota_routing(
                config,
                weights,
                landmask,
                default_routing_polar(),
                grib,
                Utc::now(),
            ))
        }

        fn run_dual(
            config: SotaRoutingConfig,
            weights: ObjectiveWeights,
            start: Point,
            opponent: OpponentState,
            use_bufr: bool,
            scenarios: Vec<WeatherScenario>,
        ) -> Result<DualRoutingResult, String> {
            let landmask = Landmask::new().map_err(|e| e.to_string())?;
            let grib = Self::build_grib(start, config.base.destination, use_bufr);
            Ok(calculate_dual_routing(
                config, weights, landmask, grib, opponent, Utc::now(), &scenarios,
            ))
        }

        fn spawn_single(&mut self) {
            let config = self.config.clone();
            let weights = self.weights.clone();
            let start = self.start;
            let use_bufr = self.use_bufr_grid;
            self.computing = true;
            self.compute_job = Some(ComputeJob::Single(spawn_promise(move || {
                Self::run_single(config, weights, start, use_bufr)
            })));
        }

        fn spawn_dual(&mut self) {
            let config = self.config.clone();
            let weights = self.weights.clone();
            let start = self.start;
            let opponent = self.opponent.clone();
            let use_bufr = self.use_bufr_grid;
            let scenarios: Vec<WeatherScenario> = self
                .scenarios
                .iter()
                .filter(|(_, on)| *on)
                .map(|(s, _)| s.clone())
                .collect();
            self.computing = true;
            self.compute_job = Some(ComputeJob::Dual(spawn_promise(move || {
                Self::run_dual(config, weights, start, opponent, use_bufr, scenarios)
            })));
        }

        fn poll_compute(&mut self) {
            let ready_single = match &self.compute_job {
                Some(ComputeJob::Single(p)) => p.ready().cloned(),
                _ => None,
            };
            if let Some(result) = ready_single {
                self.compute_job = None;
                self.computing = false;
                match result {
                    Ok(r) => {
                        self.status_message = format!("Done: {} isochrones, {} legs", r.isochrones.len(), r.route_legs.len());
                        self.result = Some(r);
                        self.dual_result = None;
                    }
                    Err(e) => self.status_message = format!("Error: {}", e),
                }
                return;
            }

            let ready_dual = match &self.compute_job {
                Some(ComputeJob::Dual(p)) => p.ready().cloned(),
                _ => None,
            };
            if let Some(result) = ready_dual {
                self.compute_job = None;
                self.computing = false;
                match result {
                    Ok(d) => {
                        let delta = d.eta_delta_hours.map(|h| format!("{:.2}h", h)).unwrap_or_else(|| "?".into());
                        self.status_message = format!("Dual done — ETA Δ: {} | {} scenarios", delta, d.scenario_results.len());
                        self.result = Some(d.mine.clone());
                        self.dual_result = Some(d);
                    }
                    Err(e) => self.status_message = format!("Error: {}", e),
                }
            }
        }

        fn show_controls(&mut self, ui: &mut egui::Ui) {
            ui.heading("SOTA Routing");
            ui.separator();

            ui.collapsing("Start / Mark", |ui| {
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.start.lat).speed(0.01).prefix("Start lat "));
                    ui.add(egui::DragValue::new(&mut self.start.lon).speed(0.01).prefix("lon "));
                });
                if let Some(ref mut dest) = self.destination {
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut dest.lat).speed(0.01).prefix("Mark lat "));
                        ui.add(egui::DragValue::new(&mut dest.lon).speed(0.01).prefix("lon "));
                    });
                }
                self.config.base.start = self.start;
                self.config.base.destination = self.destination;
            });

            ui.collapsing("Opponent", |ui| {
                ui.checkbox(&mut self.opponent_enabled, "Show opponent");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.opponent.position.lat).speed(0.01).prefix("lat "));
                    ui.add(egui::DragValue::new(&mut self.opponent.position.lon).speed(0.01).prefix("lon "));
                });
                ui.add(egui::Slider::new(&mut self.opponent.polar_scale, 0.5..=1.2).text("Polar scale"));
                if let Some(ref d) = self.dual_result {
                    if let Some(delta) = d.eta_delta_hours {
                        ui.label(format!("ETA delta (me−opp): {:.2} h", delta));
                    }
                    if let Some(p) = d.combined_eta_percentiles {
                        ui.label(format!("P10/P50/P90: {:.1}/{:.1}/{:.1} h", p.p10_hours, p.p50_hours, p.p90_hours));
                    }
                }
            });

            ui.collapsing("Scenarios", |ui| {
                ui.checkbox(&mut self.show_scenario_fan, "Route fan on map");
                for (scenario, enabled) in &mut self.scenarios {
                    ui.checkbox(enabled, &scenario.label);
                }
            });

            ui.collapsing("Constraints", |ui| {
                opt_drag(ui, &mut self.config.constraints.max_true_wind_ms, "Max wind m/s", 0.5);
                opt_drag(ui, &mut self.config.constraints.max_significant_wave_m, "Max Hs m", 0.1);
                ui.checkbox(&mut self.config.enable_destination_prune, "Cone prune");
            });

            ui.collapsing("Lambda", |ui| {
                ui.add(egui::Slider::new(&mut self.weights.lambda_wave_risk, 0.0..=5.0).text("λ₁"));
                ui.add(egui::Slider::new(&mut self.weights.lambda_comfort, 0.0..=5.0).text("λ₂"));
                ui.add(egui::Slider::new(&mut self.weights.lambda_manoeuvre, 0.0..=5.0).text("λ₃"));
                ui.add(egui::Slider::new(&mut self.weights.lambda_safety, 0.0..=5.0).text("λ₄"));
            });

            ui.horizontal(|ui| {
                if ui.add_enabled(!self.computing, egui::Button::new("Compute")).clicked() {
                    self.spawn_single();
                }
                if ui.add_enabled(!self.computing, egui::Button::new("Dual")).clicked() {
                    self.spawn_dual();
                }
                if self.computing { ui.spinner(); }
            });

            ui.label(&self.status_message);

            if let Some(ref r) = self.result {
                if let Some(eta) = r.best_eta_hours {
                    ui.label(format!("ETA: {:.2} h | {} tacks", eta, r.route_legs.iter().filter(|l| l.is_tack).count()));
                }
                if let Some(dest) = self.destination {
                    let (h, t) = tack_decision_eta(self.start, dest, 5.0);
                    ui.label(format!("Tack hint hold/tack: {:.1}h / {:.1}h", h, t));
                }
                #[cfg(not(target_arch = "wasm32"))]
                if ui.button("Export GPX").clicked() {
                    if let Some(route) = &r.best_route {
                        let _ = std::fs::write("sota-route.gpx", route_to_gpx("sota-route", route));
                        self.status_message = "Wrote sota-route.gpx".into();
                    }
                }
            }
        }
    }

    fn opt_drag(ui: &mut egui::Ui, value: &mut Option<f64>, label: &str, speed: f64) {
        ui.horizontal(|ui| {
            ui.label(label);
            let mut v = value.unwrap_or(0.0);
            ui.add(egui::DragValue::new(&mut v).speed(speed));
            *value = Some(v);
        });
    }

    fn spawn_promise<T: Send + 'static>(
        f: impl FnOnce() -> Result<T, String> + Send + 'static,
    ) -> Promise<Result<T, String>> {
        #[cfg(not(target_arch = "wasm32"))]
        { Promise::spawn_thread("routing", f) }
        #[cfg(target_arch = "wasm32")]
        { Promise::spawn_local(async move { f() }) }
    }

    impl eframe::App for SotaRoutingApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            self.poll_compute();
            if self.tiles.is_none() {
                self.tiles = Some(HttpTiles::new(OpenStreetMap, ctx.clone()));
            }

            egui::SidePanel::left("controls").default_width(300.0).show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.show_controls(ui));
            });

            egui::CentralPanel::default().show(ctx, |ui| {
                let (isochrones, envelopes, best_route) = self.result.as_ref()
                    .map(|r| (r.isochrones.clone(), r.arrival_envelopes.clone(), r.best_route.clone()))
                    .unwrap_or_default();

                let (opp_iso, opp_route, scenario_routes) = self.dual_result.as_ref()
                    .map(|d| (
                        d.opponent.isochrones.clone(),
                        d.opponent.best_route.clone(),
                        d.scenario_results.iter().filter_map(|s| s.best_route.clone()).collect(),
                    ))
                    .unwrap_or_default();

                let (show_iso, show_env) = match self.view_layer {
                    ViewLayer::Isochrones => (true, false),
                    ViewLayer::Envelopes => (false, true),
                    ViewLayer::Both => (true, true),
                };

                if let Some(ref mut tiles) = self.tiles {
                    Map::new(
                        Some(tiles as &mut dyn walkers::Tiles),
                        &mut self.map_memory,
                        lon_lat(self.start.lon, self.start.lat),
                    )
                    .with_plugin(RoutingMapPlugin {
                        isochrones,
                        opponent_isochrones: if self.opponent_enabled { opp_iso } else { vec![] },
                        scenario_routes: if self.show_scenario_fan { scenario_routes } else { vec![] },
                        envelopes,
                        start: self.start,
                        opponent_pos: if self.opponent_enabled { Some(self.opponent.position) } else { None },
                        destination: self.destination,
                        best_route,
                        opponent_route: if self.opponent_enabled { opp_route } else { None },
                        show_isochrones: show_iso,
                        show_envelopes: show_env,
                        show_opponent: self.opponent_enabled,
                        show_scenario_fan: self.show_scenario_fan,
                    })
                    .show(ui, |_ui, _r, _p, _m| {});
                }
            });
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn run_native(start: Point, destination: Option<Point>) -> eframe::Result<()> {
        eframe::run_native(
            "SOTA Isochrone Routing",
            eframe::NativeOptions {
                viewport: egui::ViewportBuilder::default().with_inner_size([1400.0, 900.0]),
                ..Default::default()
            },
            Box::new(move |_cc| Ok(Box::new(SotaRoutingApp::new(start, destination)))),
        )
    }
}

#[cfg(feature = "gui")]
pub use gui_impl::*;
