#!/usr/bin/env python3
"""Run firmware-independent tests for the actual CDC TX transition module."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
module = root / "crates/flipperzero/src/cdc/tx_progress.rs"
with tempfile.TemporaryDirectory(prefix="cdc-tx-tests-") as directory:
    path = pathlib.Path(directory)
    source = path / "tests.rs"
    # Rust raw string permits spaces/backslashes in checkout paths.
    source.write_text(f'#[path = r#"{module}"#]\nmod tx_progress;\n')
    executable = path / "tests"
    subprocess.run(["rustc", "--edition=2024", "--test", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
