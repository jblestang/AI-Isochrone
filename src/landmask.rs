use crate::types::Point;
use roaring_landmask::{RoaringMask, LandmaskProvider};
use std::sync::Arc;

/// Wrapper pour roaring-landmask permettant de vérifier si un point est sur terre ou en mer
pub struct Landmask {
    mask: Arc<RoaringMask>,
}

impl Landmask {
    /// Crée un nouveau landmask
    pub fn new() -> Result<Self, String> {
        // RoaringMask nécessite un LandmaskProvider
        // On utilise GSHHG (Global Self-consistent Hierarchical High-resolution Geography)
        let provider = LandmaskProvider::Gshhg;
        let mask = RoaringMask::new(provider)
            .map_err(|e| format!("Erreur lors de la création du landmask: {:?}", e))?;
        Ok(Self {
            mask: Arc::new(mask),
        })
    }

    /// Vérifie si un point est sur terre
    pub fn is_land(&self, point: &Point) -> bool {
        // RoaringMask utilise contains avec lon, lat
        self.mask.contains(point.lon, point.lat)
    }

    /// Vérifie si un point est en mer (pas sur terre)
    pub fn is_sea(&self, point: &Point) -> bool {
        !self.is_land(point)
    }

    /// Vérifie si plusieurs points sont sur terre (parallélisé avec rayon)
    pub fn are_land(&self, points: &[Point]) -> Vec<bool> {
        use rayon::prelude::*;
        points
            .par_iter()
            .map(|p| self.is_land(p))
            .collect()
    }

    /// Vérifie si plusieurs points sont en mer (parallélisé avec rayon)
    pub fn are_sea(&self, points: &[Point]) -> Vec<bool> {
        use rayon::prelude::*;
        points
            .par_iter()
            .map(|p| self.is_sea(p))
            .collect()
    }
}

impl Default for Landmask {
    fn default() -> Self {
        Self::new().expect("Impossible de créer le landmask")
    }
}

impl Clone for Landmask {
    fn clone(&self) -> Self {
        Self {
            mask: Arc::clone(&self.mask),
        }
    }
}
