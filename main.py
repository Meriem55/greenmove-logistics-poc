
"""POC GreenMove Logistics : moteur ZFE autonome + détection de freinage."""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent
DATA = ROOT
OUTPUT = ROOT / "output"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("GML-POC")


def load_json(path: Path) -> Any:
    """Charge un JSON et signale clairement les erreurs de fichier/format."""
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"Fichier introuvable : {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"JSON invalide dans {path} : {exc}") from exc


def point_on_segment(lat: float, lon: float, a: dict, b: dict,
                     eps: float = 1e-10) -> bool:
    """Vérifie si un point est sur un segment (tolérance numérique)."""
    cross = (lon - a["lon"]) * (b["lat"] - a["lat"]) - \
            (lat - a["lat"]) * (b["lon"] - a["lon"])
    if abs(cross) > eps:
        return False
    dot = (lon - a["lon"]) * (b["lon"] - a["lon"]) + \
          (lat - a["lat"]) * (b["lat"] - a["lat"])
    if dot < -eps:
        return False
    length_sq = (b["lon"] - a["lon"]) ** 2 + (b["lat"] - a["lat"]) ** 2
    return dot <= length_sq + eps


def point_in_polygon(lat: float, lon: float, polygon: list[dict]) -> bool:
    """
    Ray Casting : compte les intersections d'un rayon horizontal.
    La boîte englobante (Bounding Box) élimine rapidement les points extérieurs.
    Les points exactement sur le contour sont considérés dans la zone.
    """
    if len(polygon) < 3:
        raise ValueError("Le polygone doit contenir au moins 3 sommets.")

    min_lat = min(p["lat"] for p in polygon)
    max_lat = max(p["lat"] for p in polygon)
    min_lon = min(p["lon"] for p in polygon)
    max_lon = max(p["lon"] for p in polygon)

    if not (min_lat <= lat <= max_lat and min_lon <= lon <= max_lon):
        return False

    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        a, b = polygon[j], polygon[i]

        if point_on_segment(lat, lon, a, b):
            return True

        crosses_lat = (a["lat"] > lat) != (b["lat"] > lat)
        if crosses_lat:
            intersection_lon = (
                (b["lon"] - a["lon"]) * (lat - a["lat"]) /
                (b["lat"] - a["lat"]) + a["lon"]
            )
            if lon < intersection_lon:
                inside = not inside
        j = i
    return inside


def run_zfe(polygon_path: Path, gps_path: Path) -> list[dict]:
    polygon = load_json(polygon_path)
    gps_points = load_json(gps_path)

    if not isinstance(polygon, list) or not isinstance(gps_points, list):
        raise ValueError("Les fichiers JSON doivent contenir des listes.")

    events = []
    was_inside = False

    for point in gps_points:
        try:
            lat, lon = float(point["lat"]), float(point["lon"])
            inside = point_in_polygon(lat, lon, polygon)
            status = "IN" if inside else "OUT"

            logger.info(
                "GPS #%s %s -> %s",
                point.get("id", "?"),
                point.get("timestamp", ""),
                status
            )

            if inside and not was_inside:
                event = {
                    "type": "ALERT ZFE",
                    "point_id": point.get("id"),
                    "timestamp": point.get("timestamp"),
                    "latitude": lat,
                    "longitude": lon,
                    "message": "Entrée dans le périmètre ZFE détectée hors ligne."
                }
                events.append(event)

                logger.warning(
                    "[ALERT ZFE] Entrée détectée au point %s",
                    point.get("id", "?")
                )

            was_inside = inside

        except (KeyError, TypeError, ValueError) as exc:
            logger.error("Point GPS ignoré (%s) : %s", point, exc)

    OUTPUT.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT / "zfe_alerts.json"

    out_path.write_text(
        json.dumps(events, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    logger.info("Résultat ZFE enregistré : %s", out_path)
    return events


def run_safety(csv_path: Path, threshold: float = -2.5,
               alpha: float = 0.8) -> dict:
    """
    Analyse l'axe longitudinal Y.
    Le filtre exponentiel lisse le bruit.
    Un freinage prolongé n'est comptabilisé qu'une seule fois.
    Le seuil utilisé est Y < -2,5 m/s².
    """
    if not 0 < alpha <= 1:
        raise ValueError("alpha doit être compris entre 0 et 1.")

    events = []
    smoothed_y = None
    samples = 0
    was_braking = False

    try:
        with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)

            required = {"timestamp", "acc_x", "acc_y", "acc_z"}
            if not required.issubset(set(reader.fieldnames or [])):
                raise ValueError(
                    "Colonnes attendues : timestamp, acc_x, acc_y, acc_z"
                )

            for row_number, row in enumerate(reader, start=2):
                try:
                    timestamp = float(row["timestamp"])
                    ax = float(row["acc_x"])
                    ay = float(row["acc_y"])
                    az = float(row["acc_z"])

                    if not all(math.isfinite(v) for v in (timestamp, ax, ay, az)):
                        raise ValueError("valeur non finie")

                    samples += 1

                    if smoothed_y is None:
                        smoothed_y = ay
                    else:
                        smoothed_y = alpha * ay + (1 - alpha) * smoothed_y

                    # Détection d'un nouveau freinage uniquement.
                    if smoothed_y < threshold and not was_braking:
                        events.append({
                            "type": "HARSH_BRAKING",
                            "timestamp": timestamp,
                            "row": row_number,
                            "acc_y_raw_m_s2": round(ay, 3),
                            "acc_y_filtered_m_s2": round(smoothed_y, 3),
                            "threshold_m_s2": threshold
                        })

                        was_braking = True

                        logger.warning(
                            "[FREINAGE VIOLENT] ligne %s : Y brut=%.2f, filtré=%.2f",
                            row_number, ay, smoothed_y
                        )

                    # Réarmement lorsque l'accélération repasse au-dessus du seuil.
                    elif smoothed_y >= threshold:
                        was_braking = False

                except (KeyError, TypeError, ValueError) as exc:
                    logger.error(
                        "Ligne CSV %s ignorée : %s",
                        row_number, exc
                    )

    except FileNotFoundError as exc:
        raise FileNotFoundError(
            f"CSV introuvable : {csv_path}"
        ) from exc

    # Score de démonstration, non homologué pour une décision d'assurance.
    score = max(0, 100 - 10 * len(events))

    result = {
        "date_generation_utc": datetime.now(timezone.utc).isoformat(),
        "samples_analyses": samples,
        "threshold_m_s2": threshold,
        "filter": {
            "type": "exponential_moving_average",
            "alpha": alpha
        },
        "harsh_braking_count": len(events),
        "safety_score_poc": score,
        "events": events,
        "note": (
            "Score de démonstration non homologué "
            "pour une décision d'assurance."
        )
    }

    OUTPUT.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT / "daily_score.json"

    out_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    logger.info("Score journalier enregistré : %s", out_path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="POC Edge GreenMove Logistics"
    )

    parser.add_argument(
        "--module",
        choices=["zfe", "safety", "all"],
        default="all"
    )
    parser.add_argument(
        "--polygon",
        type=Path,
        default=DATA / "lyon_polygon.json"
    )
    parser.add_argument(
        "--gps",
        type=Path,
        default=DATA / "truck_gps.json"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DATA / "accelerometer_data.csv"
    )

    args = parser.parse_args()

    try:
        if args.module in ("zfe", "all"):
            run_zfe(args.polygon, args.gps)

        if args.module in ("safety", "all"):
            run_safety(args.csv)

        return 0

    except (OSError, ValueError) as exc:
        logger.error("Arrêt contrôlé : %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
