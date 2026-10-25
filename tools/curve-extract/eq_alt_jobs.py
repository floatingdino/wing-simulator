"""Random parameter sets for eq_alt_oracle.py: eq_alt_jobs.py <n> <seed> [models,..] [mix] > jobs.json"""
import json
import random
import sys

random.seed(int(sys.argv[2]) if len(sys.argv) > 2 else 1)
N = int(sys.argv[1]) if len(sys.argv) > 1 else 200


def knob(lo, hi):
    r = random.random()
    if r < 0.1:
        return lo
    if r < 0.2:
        return hi
    if r < 0.3 and lo < 0:
        return 0.0
    return round(random.uniform(lo, hi), 3)


K10 = lambda: knob(0, 10)  # noqa: E731
G5 = lambda: knob(-5, 5)  # noqa: E731
B = lambda: random.randint(0, 1)  # noqa: E731
E = lambda *v: (lambda: random.choice(v))  # noqa: E731

M = {
    "SOUL": {"lf": K10, "lg": G5, "lmf": K10, "lmf3": B, "lmq": K10, "lmg": G5, "hmf": K10, "hmf3": B,
             "hmq": K10, "hmg": G5, "hf": K10, "hg": G5},
    "E88": {"lf": K10, "lg": G5, "lq": E("LOW", "HIGH"), "lt": E("BELL", "SHELV"), "lmf": K10, "lmg": G5,
            "lmq": K10, "hmf": K10, "hmg": G5, "hmq": K10, "hf": K10, "hg": G5, "hq": E("LOW", "HIGH"),
            "ht": E("BELL", "SHELV")},
    "E84": {"g": lambda: knob(-20, 20), "lf": E("OFF", "35", "60", "110", "220"), "lg": G5,
            "mf": E("OFF", "350", "700", "1k6", "3k2", "4k8", "7k2"), "mg": G5, "mq": E("LOW", "HIGH"),
            "hf": E("10k", "12k", "16k", "OFF"), "hg": G5},
    "F110": {"peq": B, "lmf": K10, "lmg": G5, "lmq": K10, "lmf3": B, "hmf": K10, "hmg": G5, "hmq": K10,
             "hmf3": B, "shv": B, "lf": E("33", "56", "95", "160", "270", "460"), "lg": G5,
             "hf": E("3k3", "4k7", "6k8", "10k", "15k", "18k"), "hg": G5, "g": lambda: knob(-18, 18)},
    "PULSAR": {"eq1": B, "1lb": K10, "1latt": K10, "1lf": E("20", "30", "60", "100"), "1hw": K10, "1hb": K10,
               "1hf": E("3k", "4k", "5k", "8k", "10k", "12k", "16k"), "1hatt": K10,
               "1hattf": E("5k", "10k", "20k"), "eq5": B, "5lb": K10, "5lf": E("200", "300", "500", "700", "1k"),
               "5md": K10, "5mf": E("200", "300", "500", "700", "1k", "1k5", "2k", "3k", "4k", "5k", "7k"),
               "5hb": K10, "5hf": E("1k5", "2k", "3k", "4k", "5k")},
    "MACH4": {"sub": G5, "40": G5, "160": G5, "650": G5, "2k5": G5, "air": K10,
              "airm": E("OFF", "2k5", "5k", "10k", "20k", "40k"), "again": B},
    "PIA": dict({"g": lambda: knob(-12, 12)},
                **{b: (lambda: knob(-12, 12)) for b in ["31", "63", "125", "250", "500", "1k", "2k", "4k", "8k", "16k"]}),
}
models = sys.argv[3].split(",") if len(sys.argv) > 3 else list(M)
jobs = []
for m in models:
    sets = []
    for _ in range(N):
        d = {k: f() for k, f in M[m].items()}
        if len(sys.argv) > 4:
            d["mix"] = round(random.uniform(0, 125), 1)
        sets.append(d)
    jobs.append({"model": m, "sets": sets})
json.dump(jobs, sys.stdout)
