#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Styves Goupil
"""tools/fm1_rescue.py's chip check against a simulated UBOOT (no hardware, no pyusb): both flash parts an FM-1 ships
with (Puya 856014, UC25HQ80 B36014) pass the check and get their backup; another part, or another chip key, stops it
before anything is read or written. Run from the repository's root:
  python3 tests/rescue_test.py"""
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import fm1_rescue as R  # noqa: E402

failed = 0


def ok(cond, what):
    global failed
    print(("  ok   " if cond else "  FAIL ") + what)
    failed += not cond


class FakeUboot:
    """The loader's answers for a chip key, a flash type and a JEDEC id; the flash already holds the image."""

    def __init__(self, key, typ, fid, image):
        self.answer, self.flash, self.reads, self.writes = (key, typ, fid), image + bytes(R.FLASH_SIZE - R.APP_END), 0, 0

    def inquiry(self):
        return "WL82", "UBOOT1.00"

    def start_loader(self, blob):
        pass

    def chip(self):
        return self.answer

    def read(self, addr, n):
        self.reads += 1
        return self.flash[addr:addr + n]

    def write_sector(self, addr, data):
        self.writes += 1


def run(key, typ, fid):
    image = bytes(range(256)) * (R.APP_END // 256)
    dev = FakeUboot(key, typ, fid, image)
    with tempfile.TemporaryDirectory() as d:
        try:
            wrote = R.rescue(dev, image, b"", write=False, backup_dir=d, say=lambda *a, **k: None)
            err = None
        except R.Fail as e:
            wrote, err = None, str(e)
        return dev, wrote, err, len(os.listdir(d))


for fid, name in ((0x856014, "Puya P25Q80"), (0xB36014, "UC25HQ80")):
    dev, wrote, err, backups = run(0x980F, 3, fid)
    ok(err is None and wrote is False and backups == 1 and dev.writes == 0,
       f"{fid:06X} ({name}): the check passes, a backup is saved, nothing written")

for key, typ, fid, what in ((0x980F, 3, 0xEF4014, "another flash part (Winbond EF4014)"),
                            (0x980F, 3, 0xB36015, "the UC25HQ80's maker, another size"),
                            (0x980F, 3, 0x000000, "no id"),
                            (0x1234, 3, 0x856014, "another chip key"),
                            (0x980F, 2, 0xB36014, "another flash type")):
    dev, wrote, err, backups = run(key, typ, fid)
    ok(err is not None and "nothing written" in err and dev.reads == 0 and backups == 0 and dev.writes == 0,
       f"{what}: refused before any read")

print("rescue: all checks passed" if not failed else f"rescue: {failed} failed")
sys.exit(1 if failed else 0)
