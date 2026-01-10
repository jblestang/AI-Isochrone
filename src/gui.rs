#[cfg(feature = "gui")]
mod gui_impl {
    use eframe::egui;
    use crate::types::{Point, Isochrone};
    use walkers::{Map, MapMemory, HttpTiles, sources::OpenStreetMap, Plugin, Projector, lon_lat};

    /// Source de tuiles personnalisée pour OpenSeaMap avec fallback OpenStreetMap
    pub struct OpenSeaMapSource {
        osm: OpenStreetMap,
    }

    impl OpenSeaMapSource {
        pub fn new() -> Self {
            Self {
                osm: OpenStreetMap,
            }
        }
    }

    impl walkers::sources::TileSource for OpenSeaMapSource {
        fn tile_url(&self, tile_id: walkers::TileId) -> String {
            // Essayer OpenSeaMap d'abord
            format!("https://tiles.openseamap.org/seamark/{}/{}/{}.png", tile_id.zoom, tile_id.x, tile_id.y)
        }
        
        fn attribution(&self) -> walkers::sources::Attribution {
            // Utiliser l'attribution d'OpenStreetMap en fallback
            self.osm.attribution()
        }
        
        fn tile_size(&self) -> u32 {
            256
        }
        
        fn max_zoom(&self) -> u8 {
            19
        }
    }

    /// Plugin pour dessiner les isochrones sur la carte
    struct IsochronePlugin {
        isochrones: Vec<Isochrone>,
        start: Point,
        destination: Option<Point>,
    }

    impl Plugin for IsochronePlugin {
        fn run(
            self: Box<Self>,
            ui: &mut egui::Ui,
            _response: &egui::Response,
            projector: &Projector,
            _map_memory: &MapMemory,
        ) {
            let painter = ui.painter();
            
            let colors = [
                egui::Color32::from_rgb(255, 0, 0),     // Rouge pour 1h
                egui::Color32::from_rgb(255, 100, 0),   // Orange pour 2h
                egui::Color32::from_rgb(255, 200, 0),   // Jaune pour 3h
                egui::Color32::from_rgb(200, 255, 0),   // Vert-jaune pour 4h
                egui::Color32::from_rgb(100, 255, 0),   // Vert pour 5h+
            ];
            
            // Dessiner les isochrones comme des points (dots)
            for (idx, isochrone) in self.isochrones.iter().enumerate() {
                if isochrone.points.is_empty() {
                    continue;
                }
                
                // Couleur basée sur l'heure (cyclique)
                let color_idx = idx % colors.len();
                let dot_color = colors[color_idx];
                
                // Dessiner chaque point de l'isochrone
                for point in &isochrone.points {
                    // Convertir Point (lat, lon) en Position (lon, lat) pour walkers
                    let position = lon_lat(point.lon, point.lat);
                    let vec = projector.project(position);
                    let screen_pos = egui::Pos2::new(vec.x, vec.y);
                    
                    // Dessiner un point (cercle rempli)
                    painter.circle_filled(screen_pos, 3.0, dot_color);
                }
                
                // Afficher l'heure si c'est la dernière isochrone ou toutes les 4h
                if idx == self.isochrones.len() - 1 || idx % 4 == 0 {
                    // Calculer le centre de l'isochrone pour afficher le texte
                    if let Some(&first_point) = isochrone.points.first() {
                        let center_position = lon_lat(first_point.lon, first_point.lat);
                        let center_vec = projector.project(center_position);
                        let center_pos = egui::Pos2::new(center_vec.x, center_vec.y);
                        
                        painter.text(
                            center_pos,
                            egui::Align2::CENTER_CENTER,
                            format!("{:.0}h", isochrone.time_hours),
                            egui::FontId::proportional(12.0),
                            egui::Color32::BLACK,
                        );
                    }
                }
            }
            
            // Dessiner le point de départ
            let start_position = lon_lat(self.start.lon, self.start.lat);
            let start_vec = projector.project(start_position);
            let start_pos = egui::Pos2::new(start_vec.x, start_vec.y);
            painter.circle_filled(start_pos, 8.0, egui::Color32::GREEN);
            painter.circle_stroke(start_pos, 10.0, egui::Stroke::new(2.0, egui::Color32::DARK_GREEN));
            
            // Dessiner le point d'arrivée si spécifié
            if let Some(dest) = self.destination {
                let dest_position = lon_lat(dest.lon, dest.lat);
                let dest_vec = projector.project(dest_position);
                let dest_pos = egui::Pos2::new(dest_vec.x, dest_vec.y);
                painter.circle_filled(dest_pos, 8.0, egui::Color32::RED);
                painter.circle_stroke(dest_pos, 10.0, egui::Stroke::new(2.0, egui::Color32::DARK_RED));
            }
            
            // Note: self est consommé ici (Box<Self>)
        }
    }

    /// Application GUI pour visualiser les isochrones
    pub struct IsochroneApp {
        isochrones: Vec<Isochrone>,
        start: Point,
        destination: Option<Point>,
        tiles: Option<HttpTiles>,
        map_memory: MapMemory,
    }

    impl IsochroneApp {
        pub fn new(isochrones: Vec<Isochrone>, start: Point, destination: Option<Point>) -> Self {
            Self {
                tiles: None, // Sera initialisé dans update() avec le contexte egui
                map_memory: MapMemory::default(),
                isochrones,
                start,
                destination,
            }
        }
    }

    impl eframe::App for IsochroneApp {
        fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
            // Initialiser les tuiles la première fois avec le contexte egui
            if self.tiles.is_none() {
                // Utiliser OpenStreetMap comme source (plus fiable que OpenSeaMap)
                // On pourrait utiliser OpenSeaMapSource::new() mais OpenStreetMap est plus stable
                self.tiles = Some(HttpTiles::new(OpenStreetMap, ctx.clone()));
            }
            
            egui::CentralPanel::default().show(ctx, |ui| {
                // Contrôles en haut
                ui.horizontal(|ui| {
                    ui.label(format!("Isochrones: {}", self.isochrones.len()));
                    
                    ui.separator();
                    
                    if ui.button("Réinitialiser vue").clicked() {
                        // Réinitialiser la vue sur le point de départ
                        // MapMemory utilise la position passée à Map::new pour la position initiale
                        // Pour réinitialiser, on peut réinitialiser le zoom et la position sera recalculée
                        let _ = self.map_memory.set_zoom(9.0); // Zoom par défaut
                    }
                });
                
                ui.separator();
                
                // Afficher la carte avec walkers
                let start_position = lon_lat(self.start.lon, self.start.lat);
                
                // Créer la carte avec le plugin pour dessiner les isochrones
                if let Some(ref mut tiles) = self.tiles {
                    // Créer le plugin pour chaque frame
                    let plugin = IsochronePlugin {
                        isochrones: self.isochrones.clone(),
                        start: self.start,
                        destination: self.destination,
                    };
                    
                    // Créer la carte avec le plugin
                    Map::new(
                        Some(tiles as &mut dyn walkers::Tiles),
                        &mut self.map_memory,
                        start_position,
                    )
                    .with_plugin(plugin)
                    .show(ui, |_ui, _response, _projector, _memory| {
                        // Contenu vide - le plugin gère le dessin des isochrones
                    });
                }
            });
        }
    }
}

#[cfg(feature = "gui")]
pub use gui_impl::*;
