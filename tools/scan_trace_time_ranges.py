import glob
import os


def scan(pattern: str, *, x_abs_min: float = 1.0) -> None:
    files = sorted(glob.glob(pattern))
    print(f"files {len(files)}")

    mins = []
    maxs = []
    for fp in files:
        mn = None
        mx = None
        with open(fp, "r", encoding="utf-8") as f:
            for line in f:
                toks = line.split()
                if len(toks) < 6:
                    continue
                for i in range(0, len(toks), 6):
                    if i + 1 >= len(toks):
                        break
                    try:
                        ts = float(toks[i])
                        x = float(toks[i + 1])
                    except Exception:
                        continue
                    if abs(x) < float(x_abs_min):
                        continue
                    mn = ts if mn is None else min(mn, ts)
                    mx = ts if mx is None else max(mx, ts)

        mins.append(mn)
        maxs.append(mx)
        dur_s = None
        if mn is not None and mx is not None:
            dur_s = (mx - mn) / 1000.0
        print(os.path.basename(fp), "min", mn, "max", mx, "dur_s", dur_s)

    print("---continuity---")
    for i in range(1, len(files)):
        if mins[i] is None or maxs[i - 1] is None:
            continue
        gap_s = (mins[i] - maxs[i - 1]) / 1000.0
        print(os.path.basename(files[i - 1]), "->", os.path.basename(files[i]), "gap_s", gap_s)


if __name__ == "__main__":
    scan("trace_data/K615+703TV023_*.txt")
