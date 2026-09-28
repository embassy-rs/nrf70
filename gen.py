#!/usr/bin/env python3
"""Regenerates fw/bindings.rs and fw/nrf70.bin from Nordic's sources.

Usage: gen.py NRF_WIFI_DIR NRF70_BIN

NRF_WIFI_DIR is a checkout of https://github.com/zephyrproject-rtos/nrf_wifi at the
revision the nRF Connect SDK pins in its west.yml, and NRF70_BIN the matching
firmware, nrf_wifi/bin/zephyr/default/nrf70.bin in sdk-nrfxlib at the SDK's tag.
The two must come from the same SDK release: the firmware refuses a host whose
command layouts differ from its own, and the driver checks the version in the
firmware header against the one in the bindings.
"""

import re
import shutil
import subprocess
import sys

if len(sys.argv) != 3:
    sys.exit(__doc__)
nrf_wifi, nrf70_bin = sys.argv[1], sys.argv[2]

subprocess.run(
    [
        "bindgen",
        "gen_wrapper.h",
        "--output=fw/bindings.rs",
        "--use-core",
        "--ignore-functions",
        "--default-enum-style=rust",
        "--no-prepend-enum-name",
        "--no-layout-tests",
        "--blocklist-item=RPU_ADDR_MAP_MCU",
        "--",
        f"-I{nrf_wifi}/fw_if/umac_if/inc/fw",
        f"-I{nrf_wifi}/hw_if/hal/inc",
        f"-I{nrf_wifi}/hw_if/hal/inc/system",
    ],
    check=True,
)

h = open("fw/bindings.rs").read()
h = re.sub(r"= (\d+);", lambda m: "= 0x{:x};".format(int(m[1])), h)
h = h.replace("pub enum", "#[derive(num_enum::TryFromPrimitive)] pub enum")
h = h.replace("NRF_WIFI_802", "IEEE_802")
h = h.replace("NRF_WIFI_", "")
h = h.replace("nrf_wifi_", "")
open("fw/bindings.rs", "w").write(h)

subprocess.run(
    [
        "rustfmt",
        "--edition=2021",
        "fw/bindings.rs",
    ],
    check=True,
)

shutil.copyfile(nrf70_bin, "fw/nrf70.bin")
