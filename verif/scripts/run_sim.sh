#!/bin/sh
#-----------------------------------------------------------------------------
# BFA - Beamforming Accelerator
# File        : run_sim.sh
# Description : Regression driver for Icarus Verilog.
#
# Usage:
#   ./run_sim.sh              default configuration only (8 elements, 4 beams)
#   ./run_sim.sh sweep        sweep 12 parameter configurations
#   ./run_sim.sh mutate       fault-injection self-check of the testbench
#   ./run_sim.sh all          all of the above
#
# Requires: iverilog and vvp on PATH (tested with Icarus Verilog 12.0).
#-----------------------------------------------------------------------------

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)
RTL="$ROOT/logical/bfa/verilog"
TB="$ROOT/verif/tb/verilog"
LOGS="$ROOT/verif/logs"

mkdir -p "$LOGS"

SRC="$RTL/bfa.v $RTL/bfa_apb_regs.v $RTL/bfa_core.v \
     $RTL/bfa_cmac.v $RTL/bfa_rnd_sat.v $RTL/bfa_wgt_ram.v $TB/bfa_tb.v"

MODE=${1:-default}

#-----------------------------------------------------------------------------
compile_and_run () {
  NE=$1
  NB=$2
  OUT="$LOGS/bfa_tb_${NE}x${NB}"
  iverilog -g2005 -DBFA_TB_ELEMENTS="$NE" -DBFA_TB_BEAMS="$NB" \
           -o "$OUT.vvp" -I "$RTL" $SRC 2>"$OUT.compile.log"
  vvp "$OUT.vvp" > "$OUT.log" 2>&1
  RESULT=$(grep -E "TEST (PASSED|FAILED)|timeout" "$OUT.log" | head -1)
  CHECKS=$(grep -oE "checks run : [0-9]+" "$OUT.log" | grep -oE "[0-9]+" || echo "-")
  printf "  %-16s %-8s %s\n" "${NE}elem x ${NB}beam" "$CHECKS" "$RESULT"
  case "$RESULT" in
    *PASSED*) return 0 ;;
    *)        return 1 ;;
  esac
}

FAILED=0

#-----------------------------------------------------------------------------
if [ "$MODE" = "default" ] || [ "$MODE" = "all" ]; then
  echo ""
  echo "== default configuration =="
  printf "  %-16s %-8s %s\n" "CONFIG" "CHECKS" "RESULT"
  compile_and_run 8 4 || FAILED=1
fi

#-----------------------------------------------------------------------------
if [ "$MODE" = "sweep" ] || [ "$MODE" = "all" ]; then
  echo ""
  echo "== parameter sweep =="
  printf "  %-16s %-8s %s\n" "CONFIG" "CHECKS" "RESULT"
  for CFG in "1 1" "1 4" "2 2" "4 2" "8 4" "16 4" "6 3" "5 5" "2 8" "16 2" "8 8" "3 7"; do
    # shellcheck disable=SC2086
    set -- $CFG
    compile_and_run "$1" "$2" || FAILED=1
  done
fi

#-----------------------------------------------------------------------------
# Fault injection. Each mutation is a known bug; the testbench MUST report a
# failure for every one of them. A mutation that still passes means the
# testbench is not actually checking that behaviour.
#-----------------------------------------------------------------------------
if [ "$MODE" = "mutate" ] || [ "$MODE" = "all" ]; then
  echo ""
  echo "== fault injection (every mutant must be caught) =="
  MUTDIR=$(mktemp -d)
  cp "$RTL"/*.v "$RTL"/*.vh "$MUTDIR"/

  run_mutant () {
    DESC=$1
    FILE=$2
    OLD=$3
    NEW=$4
    rm -rf "$MUTDIR/work"; mkdir -p "$MUTDIR/work"
    cp "$MUTDIR"/*.v "$MUTDIR"/*.vh "$MUTDIR/work"/
    python3 - "$MUTDIR/work/$FILE" "$OLD" "$NEW" <<'PYEOF'
import sys
path, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(path).read()
if old not in s:
    sys.stderr.write("MUTATION PATTERN NOT FOUND: %s\n" % old)
    sys.exit(2)
open(path, "w").write(s.replace(old, new, 1))
PYEOF
    iverilog -g2005 -o "$MUTDIR/work/m.vvp" -I "$MUTDIR/work" \
             "$MUTDIR"/work/bfa.v "$MUTDIR"/work/bfa_apb_regs.v \
             "$MUTDIR"/work/bfa_core.v "$MUTDIR"/work/bfa_cmac.v \
             "$MUTDIR"/work/bfa_rnd_sat.v "$MUTDIR"/work/bfa_wgt_ram.v \
             "$TB/bfa_tb.v" 2>/dev/null
    R=$(vvp "$MUTDIR/work/m.vvp" 2>&1 | grep -E "TEST (PASSED|FAILED)|timeout" | head -1)
    case "$R" in
      *PASSED*) printf "  NOT CAUGHT  %s\n" "$DESC"; FAILED=1 ;;
      *)        printf "  caught      %s\n" "$DESC" ;;
    esac
  }

  run_mutant "complex multiply sign (ac-bd becomes ac+bd)" bfa_cmac.v \
    "acc_re_q <= acc_re_q + p_rr - p_ii;" "acc_re_q <= acc_re_q + p_rr + p_ii;"

  run_mutant "dropped cross term in imaginary path" bfa_cmac.v \
    "acc_im_q <= acc_im_q + p_ri + p_ir;" "acc_im_q <= acc_im_q + p_ri;"

  run_mutant "accumulator never cleared between snapshots" bfa_cmac.v \
    "end else if (acc_clr) begin" "end else if (1'b0) begin"

  run_mutant "coefficient RAM address off by one" bfa_core.v \
    ".raddr (elem_cnt)" ".raddr (elem_cnt + 1'b1)"

  run_mutant "results captured before pipeline drains" bfa_core.v \
    "assign pipe_busy = s1_v | v_m | v_a;" "assign pipe_busy = s1_v;"

  run_mutant "TLAST wrong on a single beam packet" bfa_core.v \
    "tx_last_q  <= (cfg_nbeam_m1 == {BM_AW{1'b0}});" "tx_last_q  <= 1'b0;"

  run_mutant "early TLAST not flagged as an error" bfa_core.v \
    "if (elem_cnt == cfg_nelem_m1) begin
                discard <= 1'b0;" "if (1'b1) begin
                discard <= 1'b0;"

  run_mutant "overflow detection stuck low" bfa_rnd_sat.v \
    "assign ovf  = (shifted > max_v) || (shifted < min_v);" "assign ovf  = 1'b0;"

  run_mutant "coefficient aperture not locked while enabled" bfa_apb_regs.v \
    "else if (cfg_en)       err_c = 1'b1;   // locked while enabled" \
    "else if (1'b0)         err_c = 1'b1;"

  run_mutant "interrupt events never latched" bfa_apb_regs.v \
    "irq_stat <= (irq_stat & ~w1c_mask) | evt_vec;" \
    "irq_stat <= (irq_stat & ~w1c_mask);"

  rm -rf "$MUTDIR"
fi

#-----------------------------------------------------------------------------
echo ""
if [ "$FAILED" -eq 0 ]; then
  echo "REGRESSION PASSED"
  exit 0
else
  echo "REGRESSION FAILED"
  exit 1
fi
