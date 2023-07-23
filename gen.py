#!/usr/bin/env python3
import re
import glob
import os
import subprocess

for f in glob.glob("fw/*.bin"):
    os.remove(f)

subprocess.run(
    [
        "bindgen",
        "gen_wrapper.h",
        "--output=fw/bindings.rs",
        "--use-core",
        "--ignore-functions",
        "--default-enum-style=rust",
        "--no-prepend-enum-name",
        "--",
        "-I./sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/inc/fw/",
        "-I./sdk-nrf/drivers/wifi/nrf700x/osal/hw_if/hal/inc/fw/",
    ],
    check=True,
)

h = open("fw/bindings.rs").read()
h = re.sub("= (\d+);", lambda m: "= 0x{:x};".format(int(m[1])), h)
open("fw/bindings.rs", "w").write(h)


h = open(
    "sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/inc/fw/rpu_fw_patches.h"
).read()

flavors = {}
flavors["_radiotest"] = re.search(
    re.compile("#ifdef CONFIG_NRF700X_RADIO_TEST(.*)#else", re.MULTILINE | re.DOTALL), h
)[1]
flavors[""] = re.search(re.compile("#else(.*)#endif", re.MULTILINE | re.DOTALL), h)[1]

for suffix, code in flavors.items():
    for fw in re.findall(
        re.compile(
            "const unsigned char __aligned\\(4\\)\\s+([a-z0-9_]+)\\[\\] = \\{([a-f0-9x, \t\r\n]+)\\}",
            re.MULTILINE,
        ),
        code,
    ):
        name = fw[0].removeprefix("wifi_nrf_") + suffix + ".bin"
        data = bytes.fromhex(
            "".join(c for c in fw[1].replace("0x", "") if c in "0123456789abcdef")
        )
        open("fw/" + name, "wb").write(data)
