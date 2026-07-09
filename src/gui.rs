#[cfg(feature = "gui")]
mod gui_impl {
    use crate::{
        calculate_sota_routing, BufrGribGridProvider, GribProvider, Isochrone, Landmask,
        ObjectiveWeights, Point, Polar, SotaRoutingConfig, SotaRoutingResult,
        ArrivalEnvelope, SimplePolar,
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

    /// Plugin drawing isochrones, envelopes, route on map
    struct RoutingMapPlugin {
        isochrones: Vec<Isochrone>,
        envelopes: Vec<ArrivalEnvelope>,
        start: Point,
        destination: Option<Point>,
        best_route: Option<Vec<Point>>,
        show_isochrones: bool,
        show_envelopes: bool,
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

            if self.show_isochrones {
                let colors = [
                    egui::Color32::from_rgba_unmultiplied(255, 50, 50, 180),
                    egui::Color32::from_rgba_unmultiplied(255, 120, 30, 180),
                    egui::Color32::from_rgba_unmultiplied(255, 200, 50, 180),
                    egui::Color32::from_rgba_unmultiplied(120, 220, 50, 180),
                    egui::Color32::from_rgba_unmultiplied(50, 180, 255, 180),
                ];
                for (idx, iso) in self.isochrones.iter().enumerate() {
                    let color = colors[idx % colors.len()];
                    for pt in &iso.points {
                        let pos = projector.project(lon_lat(pt.lon, pt.lat));
                        painter.circle_filled(egui::pos2(pos.x, pos.y), 2.5, color);
                    }
                }
            }

            if self.show_envelopes {
                let env_colors = [
                    egui::Color32::from_rgba_unmultiplied(180, 80, 255, 200),
                    egui::Color32::from_rgba_unmultiplied(120, 50, 220, 200),
                    egui::Color32::from_rgba_unmultiplied(80, 30, 180, 200),
                ];
                for (idx, env) in self.envelopes.iter().enumerate() {
                    let color = env_colors[idx % env_colors.len()];
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
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(2.5, color),
                            );
                        }
                        if let (Some(first), Some(last)) = (screen.first(), screen.last()) {
                            painter.line_segment(
                                [*last, *first],
                                egui::Stroke::new(2.0, color),
                            );
                        }
                    }
                    if let Some(&first) = env.boundary_points.first() {
                        let v = projector.project(lon_lat(first.lon, first.lat));
                        painter.text(
                            egui::pos2(v.x, v.y),
                            egui::Align2::CENTER_BOTTOM,
                            format!(
                                "{:.1}-{:.1}h",
                                env.min_eta_hours, env.max_eta_hours
                            ),
                            egui::FontId::proportional(11.0),
                            egui::Color32::WHITE,
                        );
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
                            egui::Stroke::new(3.0, egui::Color32::from_rgb(0, 200, 255)),
                        );
                    }
                }
            }

            let start_v = projector.project(lon_lat(self.start.lon, self.start.lat));
            let start_pos = egui::pos2(start_v.x, start_v.y);
            painter.circle_filled(start_pos, 8.0, egui::Color32::GREEN);
            painter.circle_stroke(
                start_pos,
                10.0,
                egui::Stroke::new(2.0, egui::Color32::DARK_GREEN),
            );

            if let Some(dest) = self.destination {
                let v = projector.project(lon_lat(dest.lon, dest.lat));
                let pos = egui::pos2(v.x, v.y);
                painter.circle_filled(pos, 8.0, egui::Color32::RED);
                painter.circle_stroke(pos, 10.0, egui::Stroke::new(2.0, egui::Color32::DARK_RED));
            }
        }
    }

    /// Shared SOTA routing GUI application (native + wasm)
    pub struct SotaRoutingApp {
        config: SotaRoutingConfig,
        weights: ObjectiveWeights,
        result: Option<SotaRoutingResult>,
        start: Point,
        destination: Option<Point>,
        tiles: Option<HttpTiles>,
        map_memory: MapMemory,
        view_layer: ViewLayer,
        computing: bool,
        compute_promise: Option<Promise<Result<SotaRoutingResult, String>>>,
        status_message: String,
        use_bufr_grid: bool,
    }

    impl SotaRoutingApp {
        pub fn new(start: Point, destination: Option<Point>) -> Self {
            let mut config = SotaRoutingConfig::default();
            config.base.start = start;
            config.base.destination = destination;
            config.base.time_limit_hours = 12.0;
            config.base.isochrone_step_hours = 1.0;
            config.envelope_step_minutes = 30.0;

            Self {
                config,
                weights: ObjectiveWeights::default(),
                result: None,
                start,
                destination,
                tiles: None,
                map_memory: MapMemory::default(),
                view_layer: ViewLayer::Both,
                computing: false,
                compute_promise: None,
                status_message: "Ready. Configure parameters and click Compute.".into(),
                use_bufr_grid: true,
            }
        }

        pub fn with_result(
            start: Point,
            destination: Option<Point>,
            result: SotaRoutingResult,
        ) -> Self {
            let mut app = Self::new(start, destination);
            app.result = Some(result);
            app.status_message = "Loaded routing result.".into();
            app
        }

        fn run_routing_compute(
            config: SotaRoutingConfig,
            weights: ObjectiveWeights,
            start: Point,
            use_bufr: bool,
        ) -> Result<SotaRoutingResult, String> {
            let landmask = Landmask::new().map_err(|e| e.to_string())?;
            let polar: Box<dyn Polar + Send + Sync> = Box::new(SimplePolar::default_voilier());
            let grib: Box<dyn GribProvider + Send + Sync> = if use_bufr {
                let dest = config.base.destination;
                let min_lat = start
                    .lat
                    .min(dest.map(|d| d.lat).unwrap_or(start.lat))
                    - 2.0;
                let max_lat = start
                    .lat
                    .max(dest.map(|d| d.lat).unwrap_or(start.lat))
                    + 2.0;
                let min_lon = start
                    .lon
                    .min(dest.map(|d| d.lon).unwrap_or(start.lon))
                    - 3.0;
                let max_lon = start
                    .lon
                    .max(dest.map(|d| d.lon).unwrap_or(start.lon))
                    + 3.0;
                Box::new(BufrGribGridProvider::synthetic_mediterranean(
                    min_lat, max_lat, min_lon, max_lon, 0.5,
                ))
            } else {
                Box::new(crate::SimpleGribProvider::default())
            };

            Ok(calculate_sota_routing(
                config,
                weights,
                landmask,
                polar,
                grib,
                Utc::now(),
            ))
        }

        fn spawn_compute(&mut self) {
            let config = self.config.clone();
            let weights = self.weights.clone();
            let start = self.start;
            let use_bufr = self.use_bufr_grid;

            self.computing = true;
            self.status_message = "Computing SOTA isochrones...".into();

            #[cfg(not(target_arch = "wasm32"))]
            {
                self.compute_promise = Some(Promise::spawn_thread("sota-routing", move || {
                    Self::run_routing_compute(config, weights, start, use_bufr)
                }));
            }
            #[cfg(target_arch = "wasm32")]
            {
                self.compute_promise = Some(Promise::spawn_local(async move {
                    Self::run_routing_compute(config, weights, start, use_bufr)
                }));
            }
        }

        fn poll_compute(&mut self) {
            let ready = self
                .compute_promise
                .as_ref()
                .and_then(|p| p.ready().cloned());
            if let Some(result) = ready {
                self.compute_promise = None;
                self.computing = false;
                match result {
                    Ok(r) => {
                        let n_iso = r.isochrones.len();
                        let n_env = r.arrival_envelopes.len();
                        self.status_message = format!(
                            "Done: {} isochrones, {} arrival envelopes",
                            n_iso, n_env
                        );
                        self.result = Some(r);
                    }
                    Err(e) => {
                        self.status_message = format!("Error: {}", e);
                    }
                }
            }
        }

        fn show_controls(&mut self, ui: &mut egui::Ui) {
            ui.heading("SOTA Isochrone Routing");
            ui.label(format!("J = ETA + λ₁·wave + λ₂·comfort + λ₃·manoeuvre + λ₄·safety"));
            ui.separator();

            ui.collapsing("Start / Destination", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Start lat");
                    ui.add(egui::DragValue::new(&mut self.start.lat).speed(0.01));
                    ui.label("lon");
                    ui.add(egui::DragValue::new(&mut self.start.lon).speed(0.01));
                });
                if let Some(ref mut dest) = self.destination {
                    ui.horizontal(|ui| {
                        ui.label("Dest lat");
                        ui.add(egui::DragValue::new(&mut dest.lat).speed(0.01));
                        ui.label("lon");
                        ui.add(egui::DragValue::new(&mut dest.lon).speed(0.01));
                    });
                }
                self.config.base.start = self.start;
                self.config.base.destination = self.destination;
            });

            ui.collapsing("Objective weights (λ)", |ui| {
                ui.horizontal(|ui| {
                    ui.label("λ₁ wave");
                    ui.add(egui::DragValue::new(&mut self.weights.lambda_wave_risk).speed(0.05));
                });
                ui.horizontal(|ui| {
                    ui.label("λ₂ comfort");
                    ui.add(egui::DragValue::new(&mut self.weights.lambda_comfort).speed(0.05));
                });
                ui.horizontal(|ui| {
                    ui.label("λ₃ manoeuvre");
                    ui.add(egui::DragValue::new(&mut self.weights.lambda_manoeuvre).speed(0.05));
                });
                ui.horizontal(|ui| {
                    ui.label("λ₄ safety");
                    ui.add(egui::DragValue::new(&mut self.weights.lambda_safety).speed(0.05));
                });
            });

            ui.collapsing("Simulation", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Time limit (h)");
                    ui.add(egui::DragValue::new(&mut self.config.base.time_limit_hours).speed(0.5));
                });
                ui.horizontal(|ui| {
                    ui.label("Isochrone step (h)");
                    ui.add(egui::DragValue::new(&mut self.config.base.isochrone_step_hours).speed(0.25));
                });
                ui.horizontal(|ui| {
                    ui.label("Sim step (min)");
                    ui.add(egui::DragValue::new(&mut self.config.base.simulation_step_minutes).speed(0.5));
                });
                ui.horizontal(|ui| {
                    ui.label("Directions");
                    ui.add(egui::DragValue::new(&mut self.config.base.num_directions).speed(1));
                });
                ui.horizontal(|ui| {
                    ui.label("Envelope band (min)");
                    ui.add(egui::DragValue::new(&mut self.config.envelope_step_minutes).speed(5.0));
                });
                ui.checkbox(&mut self.config.optimize_cost, "Optimize composite cost J");
                ui.checkbox(&mut self.use_bufr_grid, "Use GRIB/BUFR grid provider");
            });

            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.view_layer, ViewLayer::Isochrones, "Isochrones");
                ui.selectable_value(&mut self.view_layer, ViewLayer::Envelopes, "Envelopes");
                ui.selectable_value(&mut self.view_layer, ViewLayer::Both, "Both");
            });

            ui.horizontal(|ui| {
                let btn = ui.add_enabled(!self.computing, egui::Button::new("Compute"));
                if btn.clicked() {
                    self.spawn_compute();
                }
                if self.computing {
                    ui.spinner();
                }
            });

            ui.label(&self.status_message);

            if let Some(ref r) = self.result {
                ui.separator();
                ui.label(format!("Isochrones: {}", r.isochrones.len()));
                ui.label(format!("Arrival envelopes: {}", r.arrival_envelopes.len()));
                if let Some(eta) = r.best_eta_hours {
                    ui.label(format!("Best ETA: {:.2} h", eta));
                }
                if let Some(cost) = r.best_cost {
                    ui.label(format!("Best J: {:.0} s-eq", cost));
                }
            }
        }
    }

    impl eframe::App for SotaRoutingApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            self.poll_compute();

            if self.tiles.is_none() {
                self.tiles = Some(HttpTiles::new(OpenStreetMap, ctx.clone()));
            }

            egui::SidePanel::left("controls")
                .default_width(280.0)
                .resizable(true)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        self.show_controls(ui);
                    });
                });

            egui::CentralPanel::default().show(ctx, |ui| {
                let start_pos = lon_lat(self.start.lon, self.start.lat);

                let (isochrones, envelopes, best_route) = self
                    .result
                    .as_ref()
                    .map(|r| {
                        (
                            r.isochrones.clone(),
                            r.arrival_envelopes.clone(),
                            r.best_route.clone(),
                        )
                    })
                    .unwrap_or_default();

                let (show_iso, show_env) = match self.view_layer {
                    ViewLayer::Isochrones => (true, false),
                    ViewLayer::Envelopes => (false, true),
                    ViewLayer::Both => (true, true),
                };

                if let Some(ref mut tiles) = self.tiles {
                    let plugin = RoutingMapPlugin {
                        isochrones,
                        envelopes,
                        start: self.start,
                        destination: self.destination,
                        best_route,
                        show_isochrones: show_iso,
                        show_envelopes: show_env,
                    };
                    Map::new(
                        Some(tiles as &mut dyn walkers::Tiles),
                        &mut self.map_memory,
                        start_pos,
                    )
                    .with_plugin(plugin)
                    .show(ui, |_ui, _r, _p, _m| {});
                }
            });
        }
    }

    /// Launch native GUI (not available on wasm)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn run_native(start: Point, destination: Option<Point>) -> eframe::Result<()> {
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1400.0, 900.0])
                .with_title("SOTA Isochrone Routing"),
            ..Default::default()
        };
        eframe::run_native(
            "SOTA Isochrone Routing",
            options,
            Box::new(move |_cc| Ok(Box::new(SotaRoutingApp::new(start, destination)))),
        )
    }
}

#[cfg(feature = "gui")]
pub use gui_impl::*;