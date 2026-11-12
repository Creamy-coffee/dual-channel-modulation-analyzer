#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
work=$(mktemp -d)
trap 'rm -f "$work/commands"; rmdir "$work"' EXIT
iverilog -g2012 -s tb_harmonic_commands -o "$work/commands" \
  "$root/src/fpga/rtl/ab5_uart_commands.v" "$root/sim/fpga/tests/tb_harmonic_commands.sv"
vvp "$work/commands"
