#!/usr/bin/env python3

"""
This is a helper script to run test binaries in parallel with two probes.

Build the binaries before running this script::

    cargo test -p testsuite --target thumbv7em-none-eabi --bins --no-run
"""

from typing import List, NamedTuple, Optional, Tuple
import argparse
import asyncio
import os
import sys
import time


class TestResult(NamedTuple):
    rc: int
    elf_abs_path: str
    probe: str
    elapsed: float

    def elf_filename(self) -> str:
        return os.path.basename(self.elf_abs_path)

    def test_name(self) -> str:
        # assumes the ELF is in the form of "rtc-1794a848b0c66d5d"
        return self.elf_filename().split("-")[0]

    def status(self) -> str:
        return "PASS" if self.passed() else "FAIL"

    def passed(self) -> bool:
        return self.rc == 0


def find_elfs(elf_dir: str) -> Tuple[List[str], str]:
    """
    ``cargo`` does not output stable names for the test binaries.

    This scans for ELF files, returning a tuple of
    ``(test_elf_files, subghz_test_elf_file)``.
    """
    test_elf_files: List[str] = []
    subghz_test_elf_file: Optional[str] = None

    for file in os.listdir(elf_dir):
        # runnable ELFs will not have an extension
        if os.path.splitext(file)[1] != "":
            continue

        file_abs_path = os.path.join(elf_dir, file)
        with open(file_abs_path, "rb") as f:
            magic = f.read(4)

        if magic == b"\x7FELF":
            if "subghz" in file:
                if subghz_test_elf_file is not None:
                    raise Exception(
                        "subghz_test_elf_file already set "
                        f"subghz_test_elf_file={subghz_test_elf_file}"
                    )

                subghz_test_elf_file = file_abs_path
            else:
                test_elf_files.append(file_abs_path)

    if subghz_test_elf_file is None:
        raise Exception("subghz test ELF file not found")

    return (test_elf_files, subghz_test_elf_file)


async def probe_run(elf_path: str, probe: str, log_prefix: str) -> TestResult:
    print(f"[{log_prefix}] Running {elf_path}")
    start = time.monotonic()
    proc = await asyncio.create_subprocess_exec(
        "probe-run",
        "--chip",
        "STM32WLE5JCIx",
        "--connect-under-reset",
        elf_path,
        "--probe",
        probe,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    rc = await proc.wait()
    elapsed = time.monotonic() - start

    for line in (
        (await proc.stdout.read())
        .decode(encoding="UTF-8", errors="backslashreplace")
        .splitlines()
    ):
        print(f"[{log_prefix}] {line.rstrip()}")

    return TestResult(rc=rc, elf_abs_path=elf_path, probe=probe, elapsed=elapsed)


async def probe_worker(
    test_elf_queue: asyncio.Queue, probe: str, log_prefix: str
) -> List[TestResult]:
    print(f"[{log_prefix}] Using probe {probe}")

    ret = []
    while True:
        try:
            test_elf_path = test_elf_queue.get_nowait()
        except asyncio.QueueEmpty:
            return ret
        else:
            result = await probe_run(test_elf_path, probe, log_prefix)
            ret.append(result)

    return ret


def flatten(t: List[list]) -> list:
    return [item for sublist in t for item in sublist]


async def main() -> int:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(this_dir, ".."))
    default_elf_dir = os.path.join(
        repo_root, "target", "thumbv7em-none-eabi", "debug", "deps"
    )
    parser = argparse.ArgumentParser(description="Helper to run all on-target tests")
    parser.add_argument(
        "-d",
        "--elf-dir",
        default=default_elf_dir,
        type=str,
        help="Path to directory containing ELFs to flash",
    )
    parser.add_argument("probe", nargs=2, help="Serial number of the probes to use")
    args = parser.parse_args()

    (test_elf_files, subghz_test_elf_file) = find_elfs(args.elf_dir)

    print("Running subghz test")
    subghz_results = await asyncio.gather(
        probe_run(subghz_test_elf_file, args.probe[0], "A"),
        probe_run(subghz_test_elf_file, args.probe[1], "B"),
    )

    num_tests: int = len(test_elf_files)
    print(f"Running {num_tests} tests")

    test_elf_queue = asyncio.Queue()
    for test in test_elf_files:
        test_elf_queue.put_nowait(test)

    test_results = await asyncio.gather(
        probe_worker(test_elf_queue, args.probe[0], "A"),
        probe_worker(test_elf_queue, args.probe[1], "B"),
    )
    test_results = flatten(test_results)
    test_results.extend(subghz_results)

    fail: bool = False
    for result in test_results:
        if not result.passed():
            fail = True

        print(
            f"{result.status()} "
            f"{result.test_name()} "
            f"{result.elapsed:.3f}s "
            f"{result.probe}"
        )

    if fail:
        return 1
    else:
        return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
