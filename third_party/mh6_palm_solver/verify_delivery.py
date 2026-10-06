"""Verify the archived release and the solver used by the project."""

from pathlib import Path
import hashlib


def main() -> int:
    directory = Path(__file__).resolve().parent
    package = directory / "a1t1a3_7_20"
    manifest = package / "MH6改进版_SHA256SUMS.txt"
    count = 0
    for line in manifest.read_text(encoding="utf-8").splitlines():
        expected, name = line.split(maxsplit=1)
        relative = Path(name.lstrip("*"))
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("invalid delivery manifest path")
        actual = hashlib.sha256((package / relative).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"checksum mismatch: {relative}")
        count += 1
    runtime = directory.parents[1] / "DHandControl/scripts/mh6_palm_solver_v2.py"
    if runtime.read_bytes() != (package / "solve_from_arpha2_arpha3_theta1.py").read_bytes():
        raise ValueError("runtime solver differs from the archived source")
    print(f"Verified {count} delivery checksums; runtime solver matches archived source.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
