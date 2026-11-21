#!/usr/bin/env python3
"""Compile-negative tests for the BLE session ownership contract; no hardware."""
import pathlib
import subprocess
import tempfile
root = pathlib.Path(__file__).resolve().parents[1]
base = '#![no_std]\n#![no_main]\nextern crate flipperzero_rt;\nuse flipperzero::bluetooth::serial::BleSerial;\n'
cases = {
    'global ownership requires unsafe': ('fn check(){let _ = BleSerial::start();}', 'requires unsafe'),
    'session cannot cross threads': ('fn send<T: Send>(){} fn check(){send::<BleSerial>();}', 'cannot be sent between threads'),
    'session cannot be shared': ('fn sync<T: Sync>(){} fn check(){sync::<BleSerial>();}', 'cannot be shared between threads'),
    'I/O requires mutable session': ('fn check(s:&BleSerial){let _=s.read(&mut [0;1], flipperzero::furi::time::FuriDuration::ZERO);}', 'cannot borrow'),
}
with tempfile.NamedTemporaryFile(prefix='ble-contract-', suffix='.rs', dir=root/'crates/flipperzero/examples') as fixture:
    path = pathlib.Path(fixture.name)
    for name, (body, expected) in cases.items():
        path.write_text(base+body)
        result = subprocess.run(['cargo','check','-p','flipperzero','--example',path.stem],cwd=root/'crates',capture_output=True,text=True)
        if result.returncode == 0 or expected not in result.stderr:
            raise RuntimeError(f'{name}: unexpected compiler result\n{result.stderr}')
        print(f'{name}: rejected as expected')
