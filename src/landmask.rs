use crate::types::Point;

/// Wrapper for land/sea checks (native: roaring-landmask, wasm: simplified stub)
pub struct Landmask {
    #[cfg(not(target_arch = "wasm32"))]
    mask: std::sync::Arc<roaring_landmask::RoaringMask>,
}

impl Landmask {
    pub fn new() -> Result<Self, String> {
        #[cfg(not(target_arch = "wasm32"))]
        {
            use roaring_landmask::{LandmaskProvider, RoaringMask};
            let mask = RoaringMask::new(LandmaskProvider::Gshhg)
                .map_err(|e| format!("Erreur lors de la création du landmask: {:?}", e))?;
            Ok(Self {
                mask: std::sync::Arc::new(mask),
            })
        }
        #[cfg(target_arch = "wasm32")]
        {
            Ok(Self {})
        }
    }

    pub fn is_land(&self, point: &Point) -> bool {
        if !point.lat.is_finite() || !point.lon.is_finite() {
            return true;
        }
        let lat = point.lat.clamp(-89.999, 89.999);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.mask.contains(point.lon, lat)
        }
        #[cfg(target_arch = "wasm32")]
        {
            // Simplified landmask for wasm demo: approximate Mediterranean + Atlantic coast boxes
            wasm_approx_is_land(point)
        }
    }

    pub fn is_sea(&self, point: &Point) -> bool {
        !self.is_land(point)
    }

    pub fn are_land(&self, points: &[Point]) -> Vec<bool> {
        if points.len() <= 32 {
            points.iter().map(|p| self.is_land(p)).collect()
        } else {
            #[cfg(not(target_arch = "wasm32"))]
            {
                use rayon::prelude::*;
                points.par_iter().map(|p| self.is_land(p)).collect()
            }
            #[cfg(target_arch = "wasm32")]
            {
                points.iter().map(|p| self.is_land(p)).collect()
            }
        }
    }

    pub fn are_sea(&self, points: &[Point]) -> Vec<bool> {
        if points.len() <= 32 {
            points.iter().map(|p| self.is_sea(p)).collect()
        } else {
            #[cfg(not(target_arch = "wasm32"))]
            {
                use rayon::prelude::*;
                points.par_iter().map(|p| self.is_sea(p)).collect()
            }
            #[cfg(target_arch = "wasm32")]
            {
                points.iter().map(|p| self.is_sea(p)).collect()
            }
        }
    }
}

impl Default for Landmask {
    fn default() -> Self {
        Self::new().expect("Impossible de créer le landmask")
    }
}

impl Clone for Landmask {
    fn clone(&self) -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self {
                mask: std::sync::Arc::clone(&self.mask),
            }
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self {}
        }
    }
}

/// Rough land boxes for wasm demo routing (France/Spain coast approximation)
#[cfg(target_arch = "wasm32")]
fn wasm_approx_is_land(point: &Point) -> bool {
    let lat = point.lat;
    let lon = point.lon;

    // Brittany peninsula
    if lat > 47.5 && lat < 48.8 && lon > -5.0 && lon < -1.5 {
        return true;
    }
    // French mainland south coast strip
    if lat > 42.0 && lat < 47.5 && lon > -2.0 && lon < 8.5 && lat > 43.0 && lon < 3.5 {
        return true;
    }
    // Corsica
    if lat > 41.3 && lat < 43.1 && lon > 8.4 && lon < 9.6 {
        return true;
    }
    // Spain north coast
    if lat > 43.0 && lat < 44.0 && lon > -10.0 && lon < -1.0 {
        return true;
    }
    false
}
