"""``ct-unknowns`` — what we still need to measure, and where each number plugs in.

Three modes:

    ct-unknowns                      the table, grouped by who is expected to produce it
    ct-unknowns --check rig_sim      which entries are still at their placeholder
    ct-unknowns --format md          writes docs/unknowns.md, for sending to the team
"""

from __future__ import annotations

import argparse

from ct.cli._common import header, print_table
from ct.config import REPO_ROOT, RunConfig
from ct.control.state import ProcedureState
from ct.unknowns import UNKNOWNS, blocking, by_owner, outstanding, to_markdown


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="List the physical constants the rig controller still needs.",
    )
    parser.add_argument("--check", metavar="CONFIG", default=None,
                        help="report which entries are still placeholders in this config")
    parser.add_argument("--format", choices=["table", "md", "detail"], default="table")
    parser.add_argument("--owner", default=None,
                        help="restrict to one owner (mech, electrical, sensing, controls, "
                             "procedure, clinical)")
    parser.add_argument("--out", default=None,
                        help="where to write markdown (default docs/unknowns.md)")
    parser.add_argument("--blocks", metavar="STATE", default=None,
                        help="only entries that block this procedure state on hardware")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    entries = list(UNKNOWNS)
    if args.owner:
        entries = [u for u in entries if u.owner == args.owner]
    if args.blocks:
        try:
            wanted = ProcedureState(args.blocks)
        except ValueError:
            print(f"unknown state '{args.blocks}'. "
                  f"Known: {', '.join(s.value for s in ProcedureState)}")
            return 2
        entries = [u for u in entries if wanted in u.blocks]

    cfg_dict = None
    if args.check:
        cfg_dict = RunConfig.from_yaml(args.check).to_dict()

    if args.format == "md":
        path = args.out or (REPO_ROOT / "docs" / "unknowns.md")
        text = to_markdown(cfg_dict)
        with open(path, "w") as fh:
            fh.write(text)
        print(f"wrote {path} ({len(UNKNOWNS)} entries)")
        return 0

    if args.format == "detail":
        for owner, group in by_owner(entries).items():
            header(owner)
            for u in group:
                mark = ""
                if cfg_dict is not None:
                    mark = "  [PLACEHOLDER]" if u.is_placeholder(cfg_dict) else "  [measured]"
                print(f"\n{u.key}{mark}")
                print(f"  what   {u.what}")
                print(f"  units  {u.units}")
                print(f"  why    {u.why}")
                print(f"  how    {u.how_to_measure}")
                if u.blocks:
                    print(f"  blocks {', '.join(s.value for s in u.blocks)}")
        return 0

    for owner, group in by_owner(entries).items():
        header(owner)
        rows = []
        for u in group:
            row = {
                "key": u.key,
                "units": u.units,
                "blocks": ",".join(s.value for s in u.blocks) or "-",
            }
            if cfg_dict is not None:
                row["status"] = "PLACEHOLDER" if u.is_placeholder(cfg_dict) else "measured"
                row["value"] = u.resolve(cfg_dict)
            rows.append(row)
        print_table(rows)

    if cfg_dict is not None:
        still = outstanding(cfg_dict, entries)
        header("summary")
        print(f"{len(still)} of {len(entries)} still at placeholder values.")
        hard_blocks = blocking(cfg_dict, list(ProcedureState), entries)
        if hard_blocks:
            print(
                f"{len(hard_blocks)} of those block a procedure state on hardware, so a run "
                "with a real bus backend will refuse to start until they are measured "
                "(or --allow-placeholders is passed)."
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
