#!/usr/bin/env bash
# Compare the ported 1G decoder (./test_decode) against the upstream reference
# (moricef dec406_hex) on a set of known FGB frames. Exit non-zero on any
# mismatch.
set -u

UPSTREAM="${1:-${UPSTREAM:-$HOME/Decode_sarsat_406_v1g_v2g}}"
REF="$UPSTREAM/build/dec406_hex"

if [[ ! -x "$REF" ]]; then
    echo "!! upstream reference not built: $REF"
    echo "   run 'make dec406_hex' (or 'make') in $UPSTREAM first"
    exit 2
fi

# Known-good 144-bit FGB frames (36 hex, sync included).
VECTORS=(
    FFFED08E39048D158AC01E3AA482856824CE   # ELT-DT, France, composite position
    FFFE2F8E390000000AE018A81700EDA84498   # ELT-DT, France, offsets
    FFFE2F8E390000003F5FD2B4ED8F1E0F01EE   # ELT-DT, invalid-coordinate path
    FFFED090FD15E0059FEFFC28BEB861F0FABE   # RLS Location, Turkiye, default position
)

# Normalize away lines that are environment/timestamp/diagnostic noise.
norm() { grep -vE '^<[0-9]>|decoding completed|Hexadecimal content|^\[[0-9]{2}:'; }

fail=0
for v in "${VECTORS[@]}"; do
    a=$("$REF" "$v" 2>&1 | norm)
    b=$(./test_decode "$v" 2>&1 | norm)
    if [[ "$a" == "$b" ]]; then
        echo "PASS  $v"
    else
        echo "FAIL  $v"
        diff <(printf '%s\n' "$a") <(printf '%s\n' "$b") | sed 's/^/      /'
        fail=1
    fi
done

echo
if [[ $fail -eq 0 ]]; then echo "parity: ALL PASS"; else echo "parity: MISMATCH"; fi
exit $fail
