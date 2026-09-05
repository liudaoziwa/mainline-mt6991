#!/usr/bin/env python3
"""Diff the reserved memory map of the built DTB against the vendor FDT.

A reservation that is missing, short, or at the wrong address means the kernel
will happily allocate out of memory that BL31, the TEE, GenieZone or the modem
is still executing from.  The symptom is not a clean failure, so the map is
compared mechanically rather than eyeballed.
"""
import os
import re
import subprocess
import sys

# Self-locating: script lives at the kernel tree root; the built DTB is the
# O=out artifact produced by build.sh (run ./build.sh first).
HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(HERE, "fdt")
BUILT = os.path.join(HERE, "out/arch/arm64/boot/dts/mediatek/",
                     "mt6991-realme-rmx6688.dtb")

# Deliberate differences, asserted rather than ignored.
VENDOR_PSTORE = "mblock-2-pstore"
OURS_RAMOOPS = "ramoops"
FB = "mblock-16-framebuffer"


def sh(args):
    r = subprocess.run(args, capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else None


def regions(dtb):
    """name -> (addr, size, no_map), keyed on the vendor's node name."""
    out = {}
    for child in (sh(["fdtget", "-l", dtb, "/reserved-memory"]) or "").split():
        node = "/reserved-memory/" + child
        props = (sh(["fdtget", "-p", dtb, node]) or "").split()
        if "reg" not in props:
            continue
        w = (sh(["fdtget", "-t", "x", dtb, node, "reg"]) or "").split()
        if len(w) != 4:
            continue
        hi, lo, shi, slo = (int(x, 16) for x in w)
        # Our generator appends @<addr>; strip it to compare by vendor name.
        name = re.sub(r"@[0-9a-fA-F]+$", "", child)
        out[name] = ((hi << 32) | lo, (shi << 32) | slo, "no-map" in props)
    return out


def main():
    van, our = regions(VENDOR), regions(BUILT)
    if not van or not our:
        sys.exit("could not read reserved-memory from both trees")

    # The vendor pstore mblock is re-expressed as a mainline ramoops node.
    if OURS_RAMOOPS in our:
        our[VENDOR_PSTORE] = our.pop(OURS_RAMOOPS)

    problems = []

    for name, (addr, size, nomap) in sorted(van.items()):
        if name not in our:
            problems.append("MISSING   %s @ 0x%x (0x%x bytes)" % (name, addr, size))
            continue
        oaddr, osize, onomap = our[name]
        if (oaddr, osize) != (addr, size):
            problems.append(
                "MISMATCH  %s vendor 0x%x/0x%x vs ours 0x%x/0x%x"
                % (name, addr, size, oaddr, osize))
        if onomap != nomap:
            if name == FB and onomap and not nomap:
                pass  # intended: simplefb needs it out of the linear map
            else:
                problems.append("NO-MAP    %s vendor=%s ours=%s"
                                % (name, nomap, onomap))

    for name in sorted(set(our) - set(van)):
        problems.append("EXTRA     %s (not in vendor tree)" % name)

    print("vendor fixed reservations : %d" % len(van))
    print("ours                      : %d" % len(our))

    if FB in our:
        print("framebuffer no-map added  : %s" % our[FB][2])
    if VENDOR_PSTORE in our and VENDOR_PSTORE in van:
        print("ramoops covers pstore     : 0x%x/0x%x == 0x%x/0x%x"
              % (our[VENDOR_PSTORE][0], our[VENDOR_PSTORE][1],
                 van[VENDOR_PSTORE][0], van[VENDOR_PSTORE][1]))

    if problems:
        print("\n%d PROBLEM(S):" % len(problems))
        for p in problems:
            print("  " + p)
        return 1

    print("\nreserved memory map matches the vendor tree exactly")
    print("(only intended difference: no-map added on the framebuffer)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
