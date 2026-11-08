"""Turn raw 10 Hz node readings into one feature vector per window."""
import numpy as np

# Order matters: the anomaly model is trained on exactly these columns.
FEATURE_NAMES = [
    "loud_max", "loud_mean", "loud_std",
    "light_mean", "light_delta",
    "pir_frac", "vib_hits",
    "flame_any", "water_any",
    "temp", "hum",
]


def window_features(readings, prev_light=None):
    """readings: list of dicts straight from the Arduino JSON lines."""
    if not readings:
        return None
    lm = np.array([r.get("loud_max", 0) for r in readings], float)
    lmean = np.array([r.get("loud_mean", 0) for r in readings], float)
    light = np.array([r.get("light", 0) for r in readings], float)
    temps = [r["temp"] for r in readings if "temp" in r]
    hums = [r["hum"] for r in readings if "hum" in r]
    light_mean = float(light.mean())
    return {
        "loud_max": float(lm.max()),
        "loud_mean": float(lmean.mean()),
        "loud_std": float(lm.std()),
        "light_mean": light_mean,
        "light_delta": 0.0 if prev_light is None else light_mean - prev_light,
        "pir_frac": float(np.mean([r.get("pir", 0) for r in readings])),
        "vib_hits": float(sum(r.get("vib", 0) for r in readings)),
        "flame_any": float(any(r.get("flame", 0) for r in readings)),
        "water_any": float(any(r.get("water", 0) for r in readings)),
        # DHT is optional; 0 means "not wired", the model learns that as normal
        "temp": float(temps[-1]) if temps else 0.0,
        "hum": float(hums[-1]) if hums else 0.0,
    }


def to_vector(feat):
    return np.array([feat[k] for k in FEATURE_NAMES], float)
