# POC GreenMove Logistics — Python

## Objectif
- Module ZFE : test point-dans-polygone avec Ray Casting et préfiltre Bounding Box.
- Module Safety : lissage exponentiel de l'axe longitudinal Y, puis seuil `Y < -2.5 m/s²`.
- Sorties : `output/zfe_alerts.json` et `output/daily_score.json`.

## Prérequis
Python 3.10 ou supérieur. Aucune dépendance externe : bibliothèque standard uniquement.

## Lancer en local
Depuis le dossier `gml_poc` :

```bash
python main.py
```

Lancer un seul module :

```bash
python main.py --module zfe
python main.py --module safety
```

Personnaliser les fichiers d'entrée :

```bash
python main.py --module all --polygon data/lyon_polygon.json --gps data/truck_gps.json --csv data/accelerometer_data.csv
```

## Algorithmes
### ZFE
1. Lecture locale du polygone et des positions GPS.
2. Calcul de la Bounding Box (latitude/longitude min/max).
3. Si le point est hors de la boîte, il est immédiatement classé `OUT`.
4. Sinon, Ray Casting : un rayon horizontal part du point et le nombre d'intersections avec les arêtes du polygone détermine l'intérieur (nombre impair) ou l'extérieur (nombre pair).
5. Un point sur le contour est classé `IN`. Une alerte est créée lors du passage `OUT` vers `IN`.

Le coût du Ray Casting est O(N) par point pour N sommets. La Bounding Box réduit les calculs pour les points manifestement extérieurs. La méthode est adaptée au polygone simplifié du POC ; un produit européen devra gérer les systèmes de coordonnées, les mises à jour cartographiques, les trous/polygones multiples et la précision GPS.

### Safety
1. Lecture des colonnes `timestamp, acc_x, acc_y, acc_z`.
2. L'axe Y est longitudinal selon l'énoncé.
3. Filtre exponentiel : `y_filtré = alpha * y_brut + (1-alpha) * y_filtré_précédent`, avec `alpha=0.8`.
4. Détection si `y_filtré < -2.5 m/s²`.
5. Export des événements et d'un score POC dans `daily_score.json`.

L'extrait de l'énoncé contient un freinage à `-3.45 m/s²`. Le score fourni est uniquement une illustration technique : sa formule doit être validée avec l'assureur et des données terrain avant usage contractuel.

## Gestion des erreurs
- Fichier absent ou JSON invalide : message explicite et arrêt contrôlé.
- Point GPS ou ligne CSV mal formés : ligne ignorée et journalisée.
- En-têtes CSV manquants : erreur explicite.
- Les résultats sont conservés dans le dossier `output/`.

## Limites du POC
- Les fichiers fournis dans `data/` reprennent les exemples de l'annexe de l'énoncé ; remplacer le CSV par le fichier complet fourni pour l'évaluation, le cas échéant.
- Le POC ne remplace pas une validation sur véhicule, un étalonnage capteur, ni une validation juridique/assurantielle.
- L'énoncé indique 100 Hz dans l'annexe 1 ; le CSV d'exemple est échantillonné à 1 seconde. Le code traite chaque ligne sans supposer une fréquence particulière.
