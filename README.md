# AI-Isochrone - Calculateur d'isochrones pour bateau

Logiciel de calcul d'isochrones pour bateau en Rust, prenant en compte :
- **GRIBS/Courants réels** : données météorologiques et océaniques
- **Polaire du bateau** : performances du bateau selon l'angle au vent
- **Trait de côte** : utilisation de `roaring-landmask` pour éviter les terres
- **Multi-core** : parallélisation avec `rayon`

## Caractéristiques

- **Pas d'isochrone** : 1 heure
- **Pas de simulation intermédiaire** : 5 minutes
- **Limite de temps** : 24 heures (configurable)
- **Parallélisation** : utilisation automatique de tous les cœurs disponibles

## Installation

```bash
cargo build --release
```

## Utilisation

### Interface en ligne de commande (CLI)

Exécution par défaut (Lorient → Toulon) :

```bash
cargo run --release
```

### Interface graphique (GUI) avec egui

Pour visualiser les isochrones sur une carte interactive avec OpenSeaMap/OpenStreetMap :

```bash
# Compiler avec la feature gui
cargo build --release --features gui

# Lancer l'interface graphique
cargo run --release --bin ai-isochrone-gui --features gui

# Ou calculer les isochrones depuis la GUI (optionnel, sinon utilise les arguments)
cargo run --release --bin ai-isochrone-gui --features gui -- \
  --start-lat 47.75 \
  --start-lon -3.37 \
  --dest-lat 43.12 \
  --dest-lon 5.93 \
  --time-limit-hours 24.0

# Ou charger depuis un fichier JSON
cargo run --release --bin ai-isochrone-gui --features gui -- \
  --input results.json \
  --start-lat 47.75 \
  --start-lon -3.37
```

**Fonctionnalités de l'interface graphique :**
- Carte interactive avec tuiles OpenSeaMap (fallback OpenStreetMap)
- Visualisation des isochrones avec couleurs différentes par heure
- Points de départ (vert) et d'arrivée (rouge)
- Zoom avec molette de la souris ou boutons +/-
- Déplacement de la carte par glisser-déposer
- Bouton pour réinitialiser la vue

### Options de ligne de commande

```bash
cargo run --release -- \
  --start-lat 47.75 \
  --start-lon -3.37 \
  --dest-lat 43.12 \
  --dest-lon 5.93 \
  --time-limit-hours 24.0 \
  --isochrone-step-hours 1.0 \
  --simulation-step-minutes 5.0 \
  --num-directions 16 \
  --output results.json
```

### Paramètres

- `--start-lat`, `--start-lon` : Coordonnées du point de départ (défaut: Lorient 47.75, -3.37)
- `--dest-lat`, `--dest-lon` : Coordonnées du point d'arrivée (défaut: Toulon 43.12, 5.93)
- `--time-limit-hours` : Temps maximum de simulation en heures (défaut: 24.0)
- `--isochrone-step-hours` : Intervalle entre les isochrones en heures (défaut: 1.0)
- `--simulation-step-minutes` : Pas de simulation en minutes (défaut: 5.0)
- `--num-directions` : Nombre de directions explorées (défaut: 16 = 22.5° entre chaque)
- `--output` : Fichier de sortie JSON (optionnel)

## Architecture

Le projet est organisé en modules :

- **`types`** : Types de base (Point, Wind, Current, Isochrone, etc.)
- **`geometry`** : Calculs géométriques (distance, bearing, déplacement, vitesse effective)
- **`landmask`** : Wrapper pour `roaring-landmask` pour éviter les terres
- **`polar`** : Gestion de la polaire du bateau (vitesse selon angle au vent)
- **`grib`** : Interface pour charger les données GRIB/courants
- **`isochrone`** : Algorithme principal de calcul d'isochrones

## Algorithme

L'algorithme utilise une approche de type "wavefront expansion" (expansion de front d'onde) :

1. **Initialisation** : Point de départ et configuration
2. **Exploration** : Pour chaque pas de temps (5 min), exploration de toutes les directions possibles
3. **Filtrage** : Élimination des positions sur terre avec `roaring-landmask`
4. **Calcul de vitesse** : 
   - Vitesse du bateau depuis la polaire (angle au vent + force du vent)
   - Vitesse effective en tenant compte du courant
5. **Génération d'isochrones** : Regroupement des points atteignables par heure
6. **Parallélisation** : Utilisation de `rayon` pour paralléliser l'exploration des directions

## Configuration de la polaire

La polaire par défaut (`SimplePolar::default_voilier()`) est un exemple simplifié. Pour utiliser une polaire réelle, implémentez le trait `Polar` avec vos propres données :

```rust
struct MaPolaire {
    // Vos données
}

impl Polar for MaPolaire {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        // Votre logique de calcul
    }
}
```

## Chargement de données GRIB

Par défaut, le système utilise un provider simple avec des valeurs constantes. Pour charger des données GRIB réelles, implémentez le trait `GribProvider` :

```rust
struct MonGribProvider {
    // Vos données GRIB
}

impl GribProvider for MonGribProvider {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        // Lecture depuis vos fichiers GRIB
    }
    
    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        // Lecture depuis vos fichiers GRIB
    }
}
```

Note : Pour une intégration complète avec des fichiers GRIB réels, vous pouvez utiliser des bibliothèques comme `grib-rs` ou `eccodes`.

## Performance

Le calcul est optimisé pour le multi-core :
- Exploration parallèle des directions avec `rayon`
- Vérification parallèle des points sur terre/en mer
- Cache pour les données GRIB (à implémenter dans le provider)

## Format de sortie JSON

Si `--output` est spécifié, les résultats sont sauvegardés au format JSON :

```json
{
  "isochrones": [
    {
      "time_hours": 1.0,
      "num_points": 150,
      "points": [
        {"lat": 47.75, "lon": -3.37},
        ...
      ]
    },
    ...
  ]
}
```

## Limitations actuelles

- Provider GRIB par défaut : valeurs constantes (à remplacer par un vrai loader GRIB)
- Polaire par défaut : exemple simplifié (à remplacer par une vraie polaire)
- Pas de gestion du temps réel : utilisation du temps système actuel
- Pas d'optimisation de route : calcul uniquement des isochrones

## Développement futur

- Intégration avec `grib-rs` pour charger des fichiers GRIB réels
- Chargement de polaires depuis des fichiers
- Optimisation de route avec A* ou Dijkstra
- Support du temps réel avec interpolation des données GRIB
- Export en formats standards (GPX, KML, etc.)

## Licence

MIT
