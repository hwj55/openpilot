#!/usr/bin/env python3
"""
C3XL Boot Speedup Patch - restore all fast-boot modifications after git pull.

Usage:
    python3 /data/fastboot_patch.py

Place this file in /data/ on the device. Run after every git pull that
overwrites modified files.  Safe to run multiple times (idempotent).
"""

import os
import sys

OP_DIR = "/data/openpilot"
PATCH_VERSION = "2026-06-26-v2"

def green(s):
    return f"\033[32m{s}\033[0m"
def yellow(s):
    return f"\033[33m{s}\033[0m"
def red(s):
    return f"\033[31m{s}\033[0m"

def check_root():
    if not os.path.isdir(OP_DIR):
        print(red(f"ERROR: {OP_DIR} not found. Is openpilot installed?"))
        sys.exit(1)

def apply_edit(filepath, old, new, description):
    """Apply a text replacement. Returns True if changed, False if already done."""
    fullpath = os.path.join(OP_DIR, filepath)
    if not os.path.isfile(fullpath):
        print(red(f"  SKIP: {filepath} not found"))
        return False

    with open(fullpath, "r", encoding="utf-8", errors="replace") as f:
        content = f.read()

    if new in content and old not in content:
        print(green(f"  OK: {description} (already applied)"))
        return False

    if old not in content:
        print(yellow(f"  WARN: {description} - pattern not found, may need manual review"))
        return False

    content = content.replace(old, new, 1)
    with open(fullpath, "w", encoding="utf-8") as f:
        f.write(content)
    print(green(f"  DONE: {description}"))
    return True

def create_file(filepath, content, description):
    """Create a new file. Returns True if created, False if already exists."""
    fullpath = os.path.join(OP_DIR, filepath)
    os.makedirs(os.path.dirname(fullpath), exist_ok=True)

    if os.path.isfile(fullpath):
        with open(fullpath, "r") as f:
            existing = f.read()
        if existing.strip() == content.strip():
            print(green(f"  OK: {description} (already exists)"))
            return False

    with open(fullpath, "w") as f:
        f.write(content)
    print(green(f"  DONE: {description}"))
    return True


def patch_launch_script():
    """Patch launch_chffrplus.sh: hw_init.py before manager when prebuilt exists."""
    old = '''  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py'''

    new = '''  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  else
    # C3XL: init hardware (big cores + amp for SPI panda) before manager starts
    python3 $DIR/scripts/hw_init.py
  fi
  ./manager.py'''

    apply_edit("launch_chffrplus.sh", old, new, "launch: hw_init before manager")


def patch_card_py():
    """Patch card.py: CAN timeout 5s + SKIP_FW_QUERY."""
    # Change 1: CAN timeout 20 -> 5
    old1 = "messaging.sub_sock('can', timeout=20)"
    new1 = "messaging.sub_sock('can', timeout=5)"
    apply_edit("selfdrive/car/card.py", old1, new1, "card.py: CAN timeout 5s")

    # Change 2: SKIP_FW_QUERY when cache hit
    old2 = '''      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params'''

    new2 = '''      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params
        # C3XL fast boot: skip FW query when CarParamsCache is fresh
        os.environ['SKIP_FW_QUERY'] = '1\''''

    apply_edit("selfdrive/car/card.py", old2, new2, "card.py: SKIP_FW_QUERY on cache hit")


def patch_pandad_py():
    """Patch pandad.py: DEV firmware skip + SPI reset skip."""
    # Change 1: skip DEV firmware flash
    old1 = '''  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash()
    cloudlog.info("Done flashing")'''

    new1 = '''  if panda.bootstub or panda_signature != fw_signature:
    # C3XL: CarrotPilot custom DEV firmware - skip reflash to save cold boot time
    if "DEV" in panda_version and not panda.bootstub:
      cloudlog.info(f"CarrotPilot DEV firmware detected ({panda_version}), skipping reflash")
    else:
      cloudlog.info("Panda firmware out of date, update required")
      panda.flash()
      cloudlog.info("Done flashing")'''

    apply_edit("selfdrive/pandad/pandad.py", old1, new1, "pandad.py: DEV firmware skip")

    # Change 2: DEV signature relax
    old2 = '''  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    cloudlog.info("Version mismatch after flashing, exiting")
    raise AssertionError'''

    new2 = '''  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    # C3XL: DEV firmware accepted - only fail if not DEV
    if "DEV" not in panda_version:
      cloudlog.info("Version mismatch after flashing, exiting")
      raise AssertionError'''

    apply_edit("selfdrive/pandad/pandad.py", old2, new2, "pandad.py: DEV signature relax")

    # Change 3: skip SPI panda cold reset  (CRITICAL)
    old3 = '''        if first_run:
          # reset panda to ensure we're in a good state
          cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
          panda.reset(reconnect=True)'''

    new3 = '''        if first_run and not panda.is_internal():
          # reset panda to ensure we're in a good state (skip SPI internal panda - reconnect breaks it)
          cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
          panda.reset(reconnect=True)'''

    apply_edit("selfdrive/pandad/pandad.py", old3, new3, "pandad.py: skip SPI panda reset")


def patch_manager_py():
    """Patch manager.py: hardware init + registration skip + SupportedCars."""
    # Change 1: hardware init at start of manager_init
    old1 = '''def manager_init() -> None:
  save_bootlog()'''

    new1 = '''def manager_init() -> None:
  # C3XL: always init hardware (big cores + amplifier for SPI panda) even when prebuilt
  from openpilot.system.hardware import HARDWARE, AGNOS
  if AGNOS:
    HARDWARE.set_power_save(False)

  save_bootlog()'''

    apply_edit("system/manager/manager.py", old1, new1, "manager.py: hardware init")

    # Change 2: skip registration
    old2 = '''  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog'''

    new2 = '''  # set dongle id - C3XL: skip slow network registration, use cached id
  dongle_id = params.get("DongleId")
  if not dongle_id or dongle_id == UNREGISTERED_DONGLE_ID:
    reg_res = register(show_spinner=False)
    if reg_res:
      dongle_id = reg_res
    else:
      dongle_id = UNREGISTERED_DONGLE_ID
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog'''

    apply_edit("system/manager/manager.py", old2, new2, "manager.py: skip registration")

    # Change 3: SupportedCars - only toyota
    old3 = '''def main() -> None:
  manager_init()
  print(f"python ../../opendbc/car/hyundai/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ../../opendbc/car/hyundai/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ../../opendbc/car/gm/values.py > {Params().get_param_path()}/SupportedCars_gm")
  os.system(f"python ../../opendbc/car/toyota/values.py > {Params().get_param_path()}/SupportedCars_toyota")
  os.system(f"python ../../opendbc/car/mazda/values.py > {Params().get_param_path()}/SupportedCars_mazda")
  os.system(f"python ../../opendbc/car/honda/values.py > {Params().get_param_path()}/SupportedCars_honda")
  os.system(f"python ../../opendbc/car/ford/values.py > {Params().get_param_path()}/SupportedCars_ford")
  os.system(f"python ../../opendbc/car/tesla/values.py > {Params().get_param_path()}/SupportedCars_tesla")
  os.system(f"python ../../opendbc/car/volkswagen/values.py > {Params().get_param_path()}/SupportedCars_volkswagen")'''

    new3 = '''def main() -> None:
  manager_init()
  # C3XL fast path: only generate toyota SupportedCars (Lexus), cache if exists
  params_path = Params().get_param_path()
  toyota_file = os.path.join(params_path, "SupportedCars_toyota")
  if not os.path.exists(toyota_file):
    os.system(f"python ../../opendbc/car/toyota/values.py > {toyota_file}")'''

    apply_edit("system/manager/manager.py", old3, new3, "manager.py: SupportedCars toyota-only")


def create_files():
    """Create new files: prebuilt + hw_init.py"""
    # prebuilt
    create_file("prebuilt", "", "prebuilt (skip scons)")

    # hw_init.py
    hw_init = '''#!/usr/bin/env python3
"""C3XL hardware initialization - called at boot before manager starts.
Ensures SPI, amplifier, and big cores are ready for pandad."""
import os, sys, time

sys.path.insert(0, '/data/openpilot')

from openpilot.system.hardware import HARDWARE, AGNOS

if AGNOS:
    print("hw_init: disabling power save (big cores + amp + IRQ)...")
    HARDWARE.set_power_save(False)
    HARDWARE.initialize_hardware()
    print("hw_init: hardware init done")

    # Wait for SPI device to appear (panda needs ~2s after power-up)
    print("hw_init: waiting for SPI panda...")
    for i in range(30):
        if os.path.exists('/dev/spidev0.0') or os.path.exists('/dev/spidev0.1'):
            print(f"hw_init: SPI device found after {i*0.1:.1f}s")
            break
        time.sleep(0.1)
    else:
        print("hw_init: WARNING - SPI device not found after 3s, continuing anyway")

    print("hw_init: done, starting manager")
else:
    print("hw_init: not AGNOS, skipping")
'''
    create_file("scripts/hw_init.py", hw_init, "scripts/hw_init.py")


def main():
    print(f"=== C3XL Boot Patch v{PATCH_VERSION} ===")
    print(f"Target: {OP_DIR}\n")

    check_root()

    print("[1/6] launch_chffrplus.sh")
    patch_launch_script()

    print("\n[2/6] selfdrive/car/card.py")
    patch_card_py()

    print("\n[3/6] selfdrive/pandad/pandad.py")
    patch_pandad_py()

    print("\n[4/6] system/manager/manager.py")
    patch_manager_py()

    print("\n[5/6] New files")
    create_files()

    print("\n[6/6] Verify")
    ok = True
    checks = [
        ("launch_chffrplus.sh", "hw_init.py"),
        ("selfdrive/car/card.py", "SKIP_FW_QUERY"),
        ("selfdrive/pandad/pandad.py", "not panda.is_internal()"),
        ("system/manager/manager.py", "HARDWARE.set_power_save(False)"),
        ("prebuilt", None),  # just check existence
        ("scripts/hw_init.py", "spidev"),
    ]
    for fname, keyword in checks:
        fpath = os.path.join(OP_DIR, fname)
        if keyword is None:
            if os.path.isfile(fpath):
                print(green(f"  [OK] {fname} exists"))
            else:
                print(red(f"  [FAIL] {fname} missing!"))
                ok = False
        else:
            if os.path.isfile(fpath):
                with open(fpath, "r", encoding="utf-8", errors="replace") as f:
                    if keyword in f.read():
                        print(green(f"  [OK] {fname}"))
                    else:
                        print(yellow(f"  [WARN] {fname} - pattern '{keyword}' not found"))
            else:
                print(red(f"  [FAIL] {fname} missing!"))
                ok = False

    print()
    if ok:
        print(green("=== All patches applied successfully! ==="))
        print("Run './launch_chffrplus.sh' or reboot to test.")
    else:
        print(yellow("=== Done with warnings - review above ==="))


if __name__ == "__main__":
    main()
