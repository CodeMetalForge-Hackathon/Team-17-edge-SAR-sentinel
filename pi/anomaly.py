"""Unsupervised anomaly model for node feature windows. No labels needed.

Two detectors, both fit on NORMAL data only:
  1. Robust z-score per feature (median / MAD). Catches a single sensor going off-scale
     (a tap, a shake, PIR firing in a quiet zone) and tells you WHICH feature did it.
  2. IsolationForest on all features together. Catches odd COMBINATIONS that no single
     feature flags. Threshold is calibrated on the training data, not the sklearn default.
A window is anomalous if either fires.

Train on a few minutes of NORMAL room data (ambient noise, people walking past normally):
    python node_bridge.py n1 --log        # leave it ~5 min, then Ctrl+C
    python anomaly.py train               # writes node_iforest.joblib
    python anomaly.py train --synthetic   # no hardware yet? simulated normal data
Train on the laptop or the Pi; copy the .joblib to the Pi if trained elsewhere.
"""
import sys

import numpy as np

import config
from features import FEATURE_NAMES

Z_THRESH = 6.0        # robust z above this = anomalous. Raise if you get false alarms on site.
IF_QUANTILE = 0.999   # IsolationForest threshold = this quantile of training scores
IF_MARGIN = 0.02


class NodeAnomalyModel:
    def __init__(self, med, scale, iforest, if_thr):
        self.med, self.scale, self.iforest, self.if_thr = med, scale, iforest, if_thr

    @classmethod
    def train(cls, X, seed=0):
        from sklearn.ensemble import IsolationForest
        X = np.asarray(X, float)
        med = np.median(X, axis=0)
        dev = np.abs(X - med)
        mad = 1.4826 * np.median(dev, axis=0)
        # Calibrate so 99.9% of NORMAL windows stay under the threshold on every feature
        # (matters for spiky counts like vib_hits whose MAD is 0).
        tail = np.quantile(dev, 0.999, axis=0) / (0.9 * Z_THRESH)
        # 0.1 floor: features that never move in normal data (flame, water, pir) get z = 10 on a 0 -> 1 jump.
        scale = np.maximum.reduce([mad, tail, np.full(X.shape[1], 0.1)])
        iforest = IsolationForest(n_estimators=200, random_state=seed).fit(X)
        train_scores = -iforest.decision_function(X)
        if_thr = float(np.quantile(train_scores, IF_QUANTILE) + IF_MARGIN)
        return cls(med, scale, iforest, if_thr)

    def assess(self, x):
        """-> (score, anomalous, top_feature). score > 1.0 means anomalous by the z test."""
        x = np.asarray(x, float)
        z = np.abs(x - self.med) / self.scale
        i = int(np.argmax(z))
        zscore = float(z[i] / Z_THRESH)
        if_s = float(-self.iforest.decision_function(x.reshape(1, -1))[0])
        anomalous = zscore > 1.0 or if_s > self.if_thr
        top = FEATURE_NAMES[i] if zscore > 1.0 else ("combination" if anomalous else None)
        return zscore, anomalous, top

    def save(self, path=config.MODEL_FILE):
        import joblib
        joblib.dump({"med": self.med, "scale": self.scale, "iforest": self.iforest,
                     "if_thr": self.if_thr, "features": FEATURE_NAMES}, path)

    @classmethod
    def load(cls, path=config.MODEL_FILE):
        import joblib
        d = joblib.load(path)
        assert d["features"] == FEATURE_NAMES, "feature list changed since training, retrain"
        return cls(d["med"], d["scale"], d["iforest"], d["if_thr"])


def load_normal_csv(path=config.NORMAL_LOG):
    import csv
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return np.array([[float(r[k]) for k in FEATURE_NAMES] for r in rows])


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "train":
        if "--synthetic" in sys.argv:
            from simulate import synthetic_normal_windows
            X = synthetic_normal_windows(2000)
        else:
            X = load_normal_csv()
        m = NodeAnomalyModel.train(X)
        m.save()
        print(f"trained on {len(X)} windows -> {config.MODEL_FILE}")
    else:
        print(__doc__)
