#!/usr/bin/env python3
"""Check that CDC receiver ownership rejects previously unsound safe callers.

Run with the repository's Rust toolchain and thumbv7em-none-eabihf target.
Creates a temporary example in this checkout and removes it even on failure.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
base = "#![no_std]\n#![no_main]\nextern crate flipperzero_rt;\nuse flipperzero::cdc::UsbCdc;\n"
cases = {
    "borrowed capture": (
        "fn check(cdc:UsbCdc){let mut count=0;let _r=cdc.async_receiver(|b|count+=b.len());}",
        "closure may outlive",
    ),
    "reused owner": (
        "fn check(cdc:UsbCdc){let _r=cdc.async_receiver(|_|{});let _s=cdc.async_receiver(|_|{});}",
        "use of moved value",
    ),
    "non-Send capture": (
        "fn check(cdc:UsbCdc){let p=core::ptr::null::<u8>();let _r=cdc.async_receiver(move |_|{core::hint::black_box(p);});}",
        "cannot be sent between threads",
    ),
}
with tempfile.NamedTemporaryFile(
    prefix="cdc-contract-", suffix=".rs", dir=root / "crates/flipperzero/examples"
) as fixture:
    path = pathlib.Path(fixture.name)
    for name, (body, expected) in cases.items():
        path.write_text(base + body)
        result = subprocess.run(
            ["cargo", "check", "-p", "flipperzero", "--example", path.stem],
            cwd=root / "crates", capture_output=True, text=True,
        )
        if result.returncode == 0 or expected not in result.stderr:
            raise RuntimeError(f"{name}: unexpected compiler result\n{result.stderr}")
        print(f"{name}: rejected as expected")
