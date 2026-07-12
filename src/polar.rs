/// Polaire de bateau : vitesse en fonction de l'angle au vent et de la force du vent
/// 
/// La polaire définit la vitesse du bateau (en nœuds) en fonction de:
/// - L'angle au vent (angle entre le cap du bateau et la direction d'où vient le vent)
/// - La force du vent (en m/s ou Beaufort)
pub trait Polar {
    /// Retourne la vitesse du bateau en nœuds pour un angle au vent et une force de vent donnés
    /// angle_au_vent: angle entre le cap du bateau et la direction du vent (0° = vent de face, 180° = vent arrière)
    /// wind_speed_ms: vitesse du vent en m/s
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64;

    /// When using a multi-sail polar, returns the index of the sail delivering best speed.
    fn active_sail_index(&self, angle_au_vent: f64, wind_speed_ms: f64) -> Option<usize> {
        let _ = (angle_au_vent, wind_speed_ms);
        None
    }

    /// Speed in knots for a specific sail index, if valid at `(TWA, wind)`.
    fn speed_for_sail(
        &self,
        sail_index: usize,
        angle_au_vent: f64,
        wind_speed_ms: f64,
    ) -> Option<f64> {
        let _ = (sail_index, angle_au_vent, wind_speed_ms);
        None
    }

    /// Sail plan for one routing step: `(sail_index, speed_knots)`.
    /// Multi-sail polars keep the current sail unless another is materially faster.
    fn select_sail_plan(
        &self,
        prev_sail: Option<usize>,
        angle_au_vent: f64,
        wind_speed_ms: f64,
    ) -> Option<(usize, f64)> {
        let _ = prev_sail;
        let speed = self.speed_knots(angle_au_vent, wind_speed_ms);
        if speed < 0.01 {
            return None;
        }
        self.active_sail_index(angle_au_vent, wind_speed_ms)
            .map(|idx| (idx, speed))
    }
    
    /// Retourne la vitesse en m/s
    fn speed_ms(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        self.speed_knots(angle_au_vent, wind_speed_ms) / 1.944
    }
}

/// Polaire simple basée sur des tables de valeurs
/// Utilise une interpolation bilinéaire
#[derive(Clone, Debug)]
pub struct SimplePolar {
    /// Angles au vent en degrés (clés de la table)
    angles: Vec<f64>,
    /// Forces de vent en m/s (clés de la table)
    wind_speeds: Vec<f64>,
    /// Table [angle_index][wind_index] -> vitesse en nœuds
    speed_table: Vec<Vec<f64>>,
}

impl SimplePolar {
    /// Crée une nouvelle polaire simple avec une table de valeurs
    pub fn new(angles: Vec<f64>, wind_speeds: Vec<f64>, speed_table: Vec<Vec<f64>>) -> Self {
        Self {
            angles,
            wind_speeds,
            speed_table,
        }
    }

    /// Crée une polaire par défaut pour un voilier type (exemple)
    /// Cette polaire est un exemple simplifié et devrait être remplacée par des données réelles
    pub fn default_voilier() -> Self {
        // Angles au vent: 30° (no-go limit), 45°, 60°, 90°, 120°, 135°, 150°, 180° (vent arrière)
        let angles = vec![30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0];
        
        // Forces de vent en m/s: 2.5, 5, 7.5, 10, 12.5, 15 m/s (~5, 10, 15, 20, 25, 30 nœuds)
        let wind_speeds = vec![2.5, 5.0, 7.5, 10.0, 12.5, 15.0];
        
        // Table de vitesses en nœuds [wind][angle]
        // IMPORTANT: Le nombre de lignes doit correspondre exactement au nombre d'éléments dans wind_speeds
        // Exemple simplifié: plus rapide au largue, limité au près et vent arrière
        let speed_table = vec![
            // 30° 45°  60°   90°  120° 135° 150° 180°
            vec![0.0, 4.0, 5.0, 6.0, 7.0, 6.5, 5.5, 4.0], // 2.5 m/s (index 0)
            vec![0.0, 5.5, 6.5, 8.0, 9.5, 9.0, 7.5, 5.5], // 5.0 m/s (index 1)
            vec![0.0, 6.5, 8.0, 9.5, 11.0, 10.5, 9.0, 6.5], // 7.5 m/s (index 2)
            vec![0.0, 7.5, 9.0, 10.5, 12.0, 11.5, 10.0, 7.0], // 10.0 m/s (index 3)
            vec![0.0, 8.0, 9.5, 11.0, 12.5, 12.0, 10.5, 7.5], // 12.5 m/s (index 4)
            vec![0.0, 8.5, 10.0, 11.5, 13.0, 12.5, 11.0, 8.0], // 15.0 m/s (index 5)
        ];
        
        Self::new(angles, wind_speeds, speed_table)
    }

    pub fn angles(&self) -> &[f64] {
        &self.angles
    }

    pub fn wind_speeds(&self) -> &[f64] {
        &self.wind_speeds
    }

    /// Scale all speeds in the table by `factor`.
    pub fn scaled(from: &Self, factor: f64) -> Self {
        Self {
            angles: from.angles.clone(),
            wind_speeds: from.wind_speeds.clone(),
            speed_table: from
                .speed_table
                .iter()
                .map(|row| row.iter().map(|v| v * factor).collect())
                .collect(),
        }
    }
}

/// One sail configuration with its own polar table and valid TWA / wind range.
#[derive(Clone, Debug)]
pub struct SailConfig {
    pub name: &'static str,
    pub polar: SimplePolar,
    pub min_twa_deg: f64,
    pub max_twa_deg: f64,
    pub min_wind_ms: f64,
    pub max_wind_ms: f64,
}

impl SailConfig {
    pub fn is_active(&self, twa_deg: f64, wind_ms: f64) -> bool {
        twa_deg + 1e-6 >= self.min_twa_deg
            && twa_deg <= self.max_twa_deg + 1e-6
            && wind_ms + 1e-6 >= self.min_wind_ms
            && wind_ms <= self.max_wind_ms + 1e-6
    }

    /// Expanded domain while this sail is already set — avoids boundary flip-flop.
    pub fn is_active_sticky(&self, twa_deg: f64, wind_ms: f64) -> bool {
        twa_deg + 1e-6 >= (self.min_twa_deg - SAIL_HYSTERESIS_TWA_DEG).max(0.0)
            && twa_deg <= self.max_twa_deg + SAIL_HYSTERESIS_TWA_DEG + 1e-6
            && wind_ms + 1e-6 >= (self.min_wind_ms - SAIL_HYSTERESIS_WIND_MS).max(0.0)
            && wind_ms <= self.max_wind_ms + SAIL_HYSTERESIS_WIND_MS + 1e-6
    }
}

/// Composite polar: pick the fastest applicable sail at each (TWA, wind).
#[derive(Clone, Debug)]
pub struct MultiSailPolar {
    sails: Vec<SailConfig>,
}

impl MultiSailPolar {
    pub fn new(sails: Vec<SailConfig>) -> Self {
        Self { sails }
    }

    pub fn sails(&self) -> &[SailConfig] {
        &self.sails
    }

    /// Fastest applicable sail at `(twa, wind)` using strict domain limits.
    pub fn best_sail_at(&self, twa_deg: f64, wind_ms: f64) -> Option<(usize, f64)> {
        self.best_sail_at_domain(twa_deg, wind_ms, false)
    }

    fn best_sail_at_domain(
        &self,
        twa_deg: f64,
        wind_ms: f64,
        sticky: bool,
    ) -> Option<(usize, f64)> {
        if twa_deg + 1e-6 < MIN_ANGLE_AU_VENT_DEG {
            return None;
        }
        self.sails
            .iter()
            .enumerate()
            .filter(|(_, s)| {
                if sticky {
                    s.is_active_sticky(twa_deg, wind_ms)
                } else {
                    s.is_active(twa_deg, wind_ms)
                }
            })
            .map(|(i, s)| (i, s.polar.speed_knots(twa_deg, wind_ms)))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .filter(|(_, spd)| *spd >= MIN_SAIL_SPEED_KNOTS)
    }

    fn speed_for_sail_domain(
        &self,
        sail_index: usize,
        twa_deg: f64,
        wind_ms: f64,
        sticky: bool,
    ) -> Option<f64> {
        let sail = self.sails.get(sail_index)?;
        let spd = sail.polar.speed_knots(twa_deg, wind_ms);

        if sticky {
            if sail.is_active_sticky(twa_deg, wind_ms) && spd >= MIN_SAIL_SPEED_KNOTS {
                return Some(spd);
            }
            if self.retention_speed_ok(sail_index, twa_deg) {
                if spd >= MIN_SAIL_SPEED_KNOTS {
                    return Some(spd);
                }
                // Sail still hoisted but polar reads zero off-design — proxy via genoa table.
                let fallback = self
                    .sails
                    .first()
                    .map(|s| s.polar.speed_knots(twa_deg, wind_ms))
                    .unwrap_or(0.0);
                if fallback >= MIN_SAIL_SPEED_KNOTS {
                    return Some(fallback);
                }
            }
            return None;
        }

        if sail.is_active(twa_deg, wind_ms) && spd >= MIN_SAIL_SPEED_KNOTS {
            Some(spd)
        } else {
            None
        }
    }

    /// While a sail is set, keep it through tacks unless TWA leaves this retention band.
    fn retention_speed_ok(&self, sail_index: usize, twa_deg: f64) -> bool {
        match sail_index {
            0 => twa_deg <= 118.0,              // Genoa + main
            1 => twa_deg <= 140.0,              // Reef-1 + jib
            2 => twa_deg <= 110.0,              // Storm jib
            3 => twa_deg >= 78.0,               // Gennaker — douse below ~78° TWA
            4 => twa_deg >= 115.0,              // Spinnaker
            _ => false,
        }
    }

    /// Sticky sail selection: keep current sail unless another is clearly faster or current is unusable.
    pub fn select_sail_plan(
        &self,
        prev_sail: Option<usize>,
        twa_deg: f64,
        wind_ms: f64,
    ) -> Option<(usize, f64)> {
        if let Some(prev_idx) = prev_sail {
            if let Some(prev_speed) = self.speed_for_sail_domain(prev_idx, twa_deg, wind_ms, true) {
                let best = self.best_sail_at(twa_deg, wind_ms)?;
                if self.should_switch_sail(prev_idx, best.0, twa_deg, prev_speed, best.1) {
                    return Some(best);
                }
                if prev_speed >= best.1 - SAIL_SWITCH_MARGIN_KNOTS {
                    return Some((prev_idx, prev_speed));
                }
                return Some((prev_idx, prev_speed));
            }
        }
        self.best_sail_at(twa_deg, wind_ms)
    }

    /// Asymmetric thresholds between overlapping sails (Schmitt-trigger style).
    fn should_switch_sail(
        &self,
        from: usize,
        to: usize,
        twa_deg: f64,
        from_speed: f64,
        to_speed: f64,
    ) -> bool {
        if from == to {
            return false;
        }
        let margin_ok = to_speed >= from_speed + SAIL_SWITCH_MARGIN_KNOTS;

        // Genoa (0) ↔ Gennaker (3): wide overlap — require clear point-of-sail shift.
        if (from == 0 && to == 3) || (from == 3 && to == 0) {
            if from == 0 {
                return twa_deg >= 120.0 && margin_ok;
            }
            // Douse gennaker when close-hauled — do not keep because proxy speed ties genoa.
            return twa_deg <= 82.0;
        }

        // Gennaker (3) ↔ Spinnaker (4): downwind overlap.
        if (from == 3 && to == 4) || (from == 4 && to == 3) {
            if from == 3 {
                return twa_deg >= 150.0 && margin_ok;
            }
            return twa_deg <= 120.0 && margin_ok;
        }

        // Genoa (0) ↔ Reef (1): wind-driven; allow when strictly faster by margin.
        margin_ok
    }

    /// Index of the sail delivering `speed_knots`, if any (for labelling / diagrams).
    pub fn active_sail_index(&self, twa_deg: f64, wind_ms: f64) -> Option<usize> {
        if twa_deg + 1e-6 < MIN_ANGLE_AU_VENT_DEG {
            return None;
        }
        let speed = self.speed_knots(twa_deg, wind_ms);
        if speed < 0.01 {
            return None;
        }
        self.sails.iter().enumerate().find_map(|(i, sail)| {
            if !sail.is_active(twa_deg, wind_ms) {
                return None;
            }
            let s = sail.polar.speed_knots(twa_deg, wind_ms);
            if (s - speed).abs() < 0.05 {
                Some(i)
            } else {
                None
            }
        })
    }

    pub fn active_sail_name(&self, twa_deg: f64, wind_ms: f64) -> Option<&'static str> {
        self.active_sail_index(twa_deg, wind_ms)
            .map(|i| self.sails[i].name)
    }

    pub fn sail_name(&self, index: usize) -> &'static str {
        self.sails
            .get(index)
            .map(|s| s.name)
            .unwrap_or("Unknown")
    }

    /// RGB color for map route segments per sail index.
    pub fn sail_color_rgb(index: Option<usize>) -> [u8; 3] {
        match index {
            Some(0) => [80, 160, 255],  // Genoa + main
            Some(1) => [255, 170, 60],  // Reef-1 + jib
            Some(2) => [255, 80, 80],   // Storm jib
            Some(3) => [100, 220, 90],  // Gennaker
            Some(4) => [200, 100, 255], // Spinnaker
            _ => [200, 200, 200],
        }
    }

    /// Default offshore setup: genoa/main, reef, storm, gennaker, spinnaker.
    pub fn default_voilier() -> Self {
        let angles = std_angles();
        let winds = std_winds();

        let jib_main = SimplePolar::default_voilier();

        // Reef-1 main + working jib: same angles, ~82% speed, usable in breeze.
        let reef1 = SimplePolar::new(
            angles.clone(),
            winds.clone(),
            scale_rows(
                &[
                    vec![0.0, 3.3, 4.1, 5.0, 5.8, 5.4, 4.6, 3.4],
                    vec![0.0, 4.5, 5.3, 6.6, 7.8, 7.4, 6.2, 4.6],
                    vec![0.0, 5.3, 6.6, 7.8, 9.0, 8.6, 7.4, 5.4],
                    vec![0.0, 6.2, 7.4, 8.6, 9.8, 9.4, 8.2, 5.8],
                    vec![0.0, 6.6, 7.8, 9.0, 10.2, 9.8, 8.6, 6.2],
                    vec![0.0, 7.0, 8.2, 9.4, 10.6, 10.2, 8.8, 6.6],
                ],
                1.0,
            ),
        );

        // Storm jib + trysail: limited angles, strong wind only.
        let storm = SimplePolar::new(
            angles.clone(),
            winds.clone(),
            vec![
                vec![0.0, 2.5, 3.0, 3.2, 3.0, 2.5, 0.0, 0.0],
                vec![0.0, 3.0, 3.6, 4.0, 3.6, 3.0, 0.0, 0.0],
                vec![0.0, 3.4, 4.0, 4.4, 4.0, 3.4, 0.0, 0.0],
                vec![0.0, 3.6, 4.2, 4.6, 4.2, 3.6, 0.0, 0.0],
                vec![0.0, 3.8, 4.4, 4.8, 4.4, 3.8, 0.0, 0.0],
                vec![0.0, 4.0, 4.6, 5.0, 4.6, 4.0, 0.0, 0.0],
            ],
        );

        // Asymmetric gennaker: reach to broad reach.
        let gennaker = SimplePolar::new(
            angles.clone(),
            winds.clone(),
            vec![
                vec![0.0, 0.0, 0.0, 0.0, 6.5, 7.5, 8.0, 7.0],
                vec![0.0, 0.0, 0.0, 0.0, 8.5, 9.5, 10.5, 9.0],
                vec![0.0, 0.0, 0.0, 0.0, 10.0, 11.5, 12.5, 11.0],
                vec![0.0, 0.0, 0.0, 0.0, 11.0, 12.5, 13.5, 12.0],
                vec![0.0, 0.0, 0.0, 0.0, 10.5, 12.0, 13.0, 11.5],
                vec![0.0, 0.0, 0.0, 0.0, 9.0, 10.0, 11.0, 9.5],
            ],
        );

        // Symmetric spinnaker: deep downwind in light/medium air.
        let spinnaker = SimplePolar::new(
            angles,
            winds,
            vec![
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 5.5, 7.0, 7.5],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 7.5, 9.5, 10.5],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 9.0, 11.5, 12.5],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 12.5, 13.5],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 9.5, 12.0, 13.0],
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 10.0, 11.0],
            ],
        );

        Self::new(vec![
            SailConfig {
                name: "Genoa + main",
                polar: jib_main,
                min_twa_deg: 30.0,
                max_twa_deg: 110.0,
                min_wind_ms: 2.5,
                max_wind_ms: 13.0,
            },
            SailConfig {
                name: "Reef-1 + jib",
                polar: reef1,
                min_twa_deg: 32.0,
                max_twa_deg: 130.0,
                min_wind_ms: 9.0,
                max_wind_ms: 18.0,
            },
            SailConfig {
                name: "Storm jib",
                polar: storm,
                min_twa_deg: 35.0,
                max_twa_deg: 95.0,
                min_wind_ms: 14.0,
                max_wind_ms: 25.0,
            },
            SailConfig {
                name: "Gennaker",
                polar: gennaker,
                min_twa_deg: 110.0,
                max_twa_deg: 170.0,
                min_wind_ms: 4.0,
                max_wind_ms: 14.0,
            },
            SailConfig {
                name: "Spinnaker",
                polar: spinnaker,
                min_twa_deg: 135.0,
                max_twa_deg: 180.0,
                min_wind_ms: 3.0,
                max_wind_ms: 11.0,
            },
        ])
    }
}

impl Polar for MultiSailPolar {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        let angle = angle_au_vent.min(180.0).max(0.0);
        if angle + 1e-6 < MIN_ANGLE_AU_VENT_DEG {
            return 0.0;
        }
        self.sails
            .iter()
            .filter(|s| s.is_active(angle, wind_speed_ms))
            .map(|s| s.polar.speed_knots(angle, wind_speed_ms))
            .fold(0.0, f64::max)
    }

    fn active_sail_index(&self, angle_au_vent: f64, wind_speed_ms: f64) -> Option<usize> {
        MultiSailPolar::active_sail_index(self, angle_au_vent, wind_speed_ms)
    }

    fn select_sail_plan(
        &self,
        prev_sail: Option<usize>,
        angle_au_vent: f64,
        wind_speed_ms: f64,
    ) -> Option<(usize, f64)> {
        MultiSailPolar::select_sail_plan(self, prev_sail, angle_au_vent, wind_speed_ms)
    }

    fn speed_for_sail(
        &self,
        sail_index: usize,
        angle_au_vent: f64,
        wind_speed_ms: f64,
    ) -> Option<f64> {
        MultiSailPolar::speed_for_sail_domain(self, sail_index, angle_au_vent, wind_speed_ms, true)
    }
}

/// Default polar used by routing binaries and GUI.
pub fn default_routing_polar() -> Box<dyn Polar + Send + Sync> {
    Box::new(MultiSailPolar::default_voilier())
}

fn std_angles() -> Vec<f64> {
    vec![30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0]
}

fn std_winds() -> Vec<f64> {
    vec![2.5, 5.0, 7.5, 10.0, 12.5, 15.0]
}

fn scale_rows(rows: &[Vec<f64>], factor: f64) -> Vec<Vec<f64>> {
    rows.iter()
        .map(|row| row.iter().map(|v| v * factor).collect())
        .collect()
}

/// Polar scaled by a factor (opponent slower/faster boat).
#[derive(Clone)]
pub struct ScaledPolar<P: Polar + Clone> {
    inner: P,
    scale: f64,
}

impl<P: Polar + Clone> ScaledPolar<P> {
    pub fn new(inner: P, scale: f64) -> Self {
        Self { inner, scale }
    }
}

impl<P: Polar + Clone> Polar for ScaledPolar<P> {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        self.inner.speed_knots(angle_au_vent, wind_speed_ms) * self.scale
    }

    fn active_sail_index(&self, angle_au_vent: f64, wind_speed_ms: f64) -> Option<usize> {
        self.inner.active_sail_index(angle_au_vent, wind_speed_ms)
    }

    fn select_sail_plan(
        &self,
        prev_sail: Option<usize>,
        angle_au_vent: f64,
        wind_speed_ms: f64,
    ) -> Option<(usize, f64)> {
        self.inner
            .select_sail_plan(prev_sail, angle_au_vent, wind_speed_ms)
            .map(|(idx, kt)| (idx, kt * self.scale))
    }
}

impl SimplePolar {
    fn interpolate_1d(&self, x: f64, x_values: &[f64], y_values: &[f64]) -> f64 {
        if x <= x_values[0] {
            return y_values[0];
        }
        if x >= x_values[x_values.len() - 1] {
            return y_values[y_values.len() - 1];
        }

        for i in 0..x_values.len() - 1 {
            if x >= x_values[i] && x <= x_values[i + 1] {
                let t = (x - x_values[i]) / (x_values[i + 1] - x_values[i]);
                return y_values[i] * (1.0 - t) + y_values[i + 1] * t;
            }
        }
        y_values[y_values.len() - 1]
    }
}

impl Polar for SimplePolar {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        let angle = angle_au_vent.min(180.0).max(0.0);
        if angle < MIN_ANGLE_AU_VENT_DEG {
            return 0.0;
        }
        
        // Vérifier que la table a le bon nombre de lignes
        if self.speed_table.len() < self.wind_speeds.len() {
            eprintln!("⚠️  Erreur polaire: speed_table a {} lignes mais wind_speeds a {} valeurs", 
                     self.speed_table.len(), self.wind_speeds.len());
            // Fallback: retourner une vitesse minimale
            return 2.0;
        }
        
        // Interpolation bilinéaire
        // 1. Interpoler pour chaque angle au niveau des vitesses de vent
        let mut speeds_at_winds = Vec::new();
        for wind_idx in 0..self.wind_speeds.len() {
            if wind_idx >= self.speed_table.len() {
                eprintln!("⚠️  Erreur polaire: wind_idx {} >= speed_table.len() {}", wind_idx, self.speed_table.len());
                // Utiliser la dernière ligne disponible
                if !self.speed_table.is_empty() {
                    let last_idx = self.speed_table.len() - 1;
                    let mut speeds_at_angle = Vec::new();
                    for angle_idx in 0..self.angles.len() {
                        if angle_idx < self.speed_table[last_idx].len() {
                            speeds_at_angle.push(self.speed_table[last_idx][angle_idx]);
                        } else {
                            speeds_at_angle.push(0.0);
                        }
                    }
                    let speed = self.interpolate_1d(angle, &self.angles, &speeds_at_angle);
                    speeds_at_winds.push(speed);
                } else {
                    speeds_at_winds.push(2.0); // Fallback
                }
                continue;
            }
            
            let mut speeds_at_angle = Vec::new();
            for angle_idx in 0..self.angles.len() {
                if angle_idx < self.speed_table[wind_idx].len() {
                    speeds_at_angle.push(self.speed_table[wind_idx][angle_idx]);
                } else {
                    eprintln!("⚠️  Erreur polaire: angle_idx {} >= speed_table[{}].len() {}", 
                             angle_idx, wind_idx, self.speed_table[wind_idx].len());
                    speeds_at_angle.push(0.0);
                }
            }
            let speed = self.interpolate_1d(angle, &self.angles, &speeds_at_angle);
            speeds_at_winds.push(speed);
        }
        
        // 2. Interpoler selon la vitesse du vent
        let result = self.interpolate_1d(wind_speed_ms, &self.wind_speeds, &speeds_at_winds);

        result.max(0.0)
    }
}

/// Minimum angle au vent (no-go zone). Polars are zero for TWA strictly below 30°.
pub const MIN_ANGLE_AU_VENT_DEG: f64 = 30.0;

/// Wall-clock penalty when hoisting/dousing a different sail plan (default 30 min).
pub const DEFAULT_SAIL_CHANGE_PENALTY_SECONDS: f64 = 1800.0;

/// New sail must beat current sail by at least this margin (knots) to justify a switch.
pub const SAIL_SWITCH_MARGIN_KNOTS: f64 = 3.0;

/// Minimum sailing time between optional sail swaps (6 h); forced swaps outside retention band.
pub const MIN_SAIL_CHANGE_INTERVAL_SECONDS: f64 = 21600.0;

/// TWA / wind expansion while retaining the current sail (hysteresis band).
pub const SAIL_HYSTERESIS_TWA_DEG: f64 = 12.0;
pub const SAIL_HYSTERESIS_WIND_MS: f64 = 2.0;

/// Ignore sail plans slower than this (knots) — treat as unusable.
pub const MIN_SAIL_SPEED_KNOTS: f64 = 0.5;

/// Calcule l'angle au vent à partir du cap du bateau et de la direction du vent.
/// Retourne l'angle au vent en degrés (0-180).
pub fn angle_au_vent(boat_heading: f64, wind_direction: f64) -> f64 {
    let diff = (boat_heading - wind_direction).abs() % 360.0;
    diff.min(360.0 - diff).min(180.0)
}

/// Signed angle from wind direction to boat heading in (-180, 180].
/// Positive = wind on starboard, negative = wind on port.
pub fn signed_wind_side(boat_heading: f64, wind_direction: f64) -> f64 {
    let diff = (boat_heading - wind_direction).rem_euclid(360.0);
    if diff > 180.0 {
        diff - 360.0
    } else {
        diff
    }
}

/// Compass headings plus close-hauled candidates on both tacks (for beating upwind).
pub fn routing_headings(num_compass_dirs: usize, wind_direction: f64) -> Vec<f64> {
    let n = num_compass_dirs.max(1);
    let step = 360.0 / n as f64;
    let mut headings: Vec<f64> = (0..n).map(|i| i as f64 * step).collect();
    for twa in [MIN_ANGLE_AU_VENT_DEG, 38.0, 45.0, 52.0, 60.0] {
        headings.push((wind_direction + twa).rem_euclid(360.0));
        headings.push((wind_direction - twa).rem_euclid(360.0));
    }
    headings.sort_by(|a, b| a.partial_cmp(b).unwrap());
    headings.dedup_by(|a, b| (*a - *b).abs() < 1.5);
    headings
}

/// Minimum heading change to count as a manoeuvre (tack or gybe).
pub const TACK_GYBE_MIN_HEADING_DELTA_DEG: f64 = 35.0;

/// True when a significant heading change crosses the wind (tack or gybe).
pub fn is_tack_or_gybe(prev_heading: f64, new_heading: f64, wind_direction: f64) -> bool {
    use crate::geometry::angle_difference;

    if angle_difference(prev_heading, new_heading).abs() < TACK_GYBE_MIN_HEADING_DELTA_DEG {
        return false;
    }
    let prev_side = signed_wind_side(prev_heading, wind_direction);
    let next_side = signed_wind_side(new_heading, wind_direction);
    prev_side.signum() != next_side.signum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_au_vent() {
        // Vent de 0° (Nord), bateau au 90° (Est) -> angle au vent = 90°
        assert!((angle_au_vent(90.0, 0.0) - 90.0).abs() < 1e-10);
        
        // Vent de 0°, bateau au 180° (Sud) -> angle au vent = 180° (vent arrière)
        assert!((angle_au_vent(180.0, 0.0) - 180.0).abs() < 1e-10);
        
        // Vent de 0°, bateau au 0° -> angle au vent = 0° (vent de face)
        assert!((angle_au_vent(0.0, 0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn no_go_zone_returns_zero_speed() {
        let polar = SimplePolar::default_voilier();
        assert_eq!(polar.speed_knots(20.0, 10.0), 0.0);
        assert_eq!(polar.speed_knots(29.0, 10.0), 0.0);
        assert!(polar.speed_ms(35.0, 10.0) > 0.05);
    }

    #[test]
    fn test_polar_interpolation() {
        let polar = SimplePolar::default_voilier();
        let speed = polar.speed_knots(90.0, 5.0);
        assert!(speed > 0.0);
        assert!(speed < 20.0); // Vérification raisonnable
    }

    #[test]
    fn tack_crosses_wind_side() {
        // West wind, tack from NW (starboard) to SW (port)
        assert!(is_tack_or_gybe(315.0, 225.0, 270.0));
    }

    #[test]
    fn bear_away_on_same_tack_is_not_a_manoeuvre() {
        // West wind, bear away from NW toward N on starboard tack
        assert!(!is_tack_or_gybe(315.0, 340.0, 270.0));
    }

    #[test]
    fn small_heading_adjustment_is_not_a_manoeuvre() {
        assert!(!is_tack_or_gybe(315.0, 330.0, 270.0));
    }

    #[test]
    fn gybe_crosses_wind_side_downwind() {
        // North wind, gybe from SE to SW
        assert!(is_tack_or_gybe(135.0, 225.0, 0.0));
    }

    #[test]
    fn routing_headings_cover_both_tacks() {
        let h = routing_headings(8, 254.0);
        assert!(h.len() >= 12);
        assert!(h.iter().any(|&hdg| {
            let twa = angle_au_vent(hdg, 254.0);
            twa >= 30.0 && twa <= 45.0
        }));
    }

    #[test]
    fn sticky_sail_keeps_current_when_margin_small() {
        let multi = MultiSailPolar::default_voilier();
        // Upwind: genoa is best and should stay on genoa
        let (idx, _) = multi.select_sail_plan(None, 45.0, 8.0).unwrap();
        assert_eq!(idx, 0);
        let (idx2, _) = multi.select_sail_plan(Some(0), 50.0, 8.0).unwrap();
        assert_eq!(idx2, 0);
    }

    #[test]
    fn sticky_sail_blocks_genoa_gennaker_flip_flop() {
        let multi = MultiSailPolar::default_voilier();
        // Hoist gennaker on a broad reach
        let (idx, _) = multi.select_sail_plan(None, 130.0, 8.0).unwrap();
        assert_eq!(idx, 3, "expected gennaker downwind");
        // TWA dips to 95° on a tack — gennaker stays hoisted
        let (idx2, _) = multi.select_sail_plan(Some(3), 95.0, 8.5).unwrap();
        assert_eq!(idx2, 3, "gennaker should stay through a tack");
        // Only switch back when firmly close-hauled
        let (idx3, _) = multi.select_sail_plan(Some(3), 82.0, 8.5).unwrap();
        assert_eq!(idx3, 0, "genoa when firmly upwind");
        // Tack to 89° — gennaker stays (polar reads zero but retention applies)
        let (idx4, spd4) = multi.select_sail_plan(Some(3), 89.0, 8.5).unwrap();
        assert_eq!(idx4, 3, "gennaker retained through tack at 89°");
        assert!(spd4 >= 3.0, "proxy speed while gennaker hoisted");
    }

    #[test]
    fn sticky_sail_switches_genoa_to_gennaker_on_broad_reach() {
        let multi = MultiSailPolar::default_voilier();
        let (idx, _) = multi.select_sail_plan(Some(0), 130.0, 8.0).unwrap();
        assert_eq!(idx, 3, "gennaker when TWA clearly broad");
    }

    #[test]
    fn multi_sail_downwind_beats_jib_main() {
        let multi = MultiSailPolar::default_voilier();
        let jib = SimplePolar::default_voilier();
        let twa = 150.0;
        let wind = 8.0;
        assert!(multi.speed_knots(twa, wind) > jib.speed_knots(twa, wind));
        assert_eq!(multi.active_sail_name(twa, wind), Some("Gennaker"));
    }

    #[test]
    fn multi_sail_upwind_uses_jib_main() {
        let multi = MultiSailPolar::default_voilier();
        assert_eq!(multi.active_sail_name(45.0, 8.0), Some("Genoa + main"));
    }

    #[test]
    fn multi_sail_respects_no_go() {
        let multi = MultiSailPolar::default_voilier();
        assert_eq!(multi.speed_knots(20.0, 10.0), 0.0);
    }
}
