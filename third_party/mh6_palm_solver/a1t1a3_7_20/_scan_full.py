import argparse
import sys, os, csv, time

solver_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(solver_dir)
sys.path.insert(0, solver_dir)
from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver

s = MH6PalmSolver()

parser = argparse.ArgumentParser(description="Scan normalized MH6 workspace coverage")
parser.add_argument("--mode", choices=(
    s.MAPPING_WORKSPACE_CONDITIONAL,
    s.MAPPING_WORKSPACE_INDEPENDENT,
    s.MAPPING_MOTOR_RANGE,
), default=s.MAPPING_WORKSPACE_CONDITIONAL)
parser.add_argument("--step", type=float, default=0.05)
parser.add_argument("--output", help="optional CSV output path")
args = parser.parse_args()

step = args.step
if not 0.0 < step <= 1.0:
    parser.error("--step must lie in (0,1]")
n_steps = int(1.0 / step) + 1
values = [round(i * step, 4) for i in range(n_steps)]

total = n_steps ** 3
count = 0
valid = []

print(f"Scanning {total} points (mode={args.mode}, step={step})...")
t0 = time.time()

for u1 in values:
    for u2 in values:
        for u3 in values:
            sols = s.solve_from_normalized(u1, u2, u3, mode=args.mode)
            n = len(sols)
            if n > 0:
                valid.append((u1, u2, u3, n))
            count += 1
            if count % 1000 == 0:
                print(f"  {count}/{total} ({count/total*100:.0f}%), found {len(valid)} valid so far")

t1 = time.time()
print(f"\nDone in {t1-t0:.1f}s")

# Save
step_label = f"{step:g}".replace(".", "p")
default_name = f"workspace_scan_{args.mode}_{step_label}.csv"
out_path = os.path.abspath(args.output) if args.output else os.path.join(solver_dir, default_name)
with open(out_path, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["u1", "u2", "u3", "n_solutions"])
    w.writerows(valid)
print(f"Saved {len(valid)} valid points to {out_path}")

# Summary
print(f"\n=== Summary ===")
print(f"Total: {total} points")
print(f"Valid: {len(valid)} ({len(valid)/total*100:.1f}%)")
if valid:
    u1s = [v[0] for v in valid]
    u2s = [v[1] for v in valid]
    u3s = [v[2] for v in valid]
    print(f"u1 range: [{min(u1s):.2f}, {max(u1s):.2f}]")
    print(f"u2 range: [{min(u2s):.2f}, {max(u2s):.2f}]")
    print(f"u3 range: [{min(u3s):.2f}, {max(u3s):.2f}]")
    # Check continuity: for each u1, what u2/u3 ranges have solutions
    for u1 in sorted(set(u1s)):
        u2_at_u1 = sorted(set(v[1] for v in valid if abs(v[0]-u1)<1e-6))
        u3_at_u1 = sorted(set(v[2] for v in valid if abs(v[0]-u1)<1e-6))
        print(f"  u1={u1:.2f}: u2=[{min(u2_at_u1):.2f},{max(u2_at_u1):.2f}]  u3=[{min(u3_at_u1):.2f},{max(u3_at_u1):.2f}]  ({len(u2_at_u1)} u2 steps, {len(u3_at_u1)} u3 steps)")
