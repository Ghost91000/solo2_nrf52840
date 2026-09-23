#!/usr/bin/env python3
"""Rectangular NFC antenna calculator (nRF52840 NFCT, 13.56 MHz).

Geometry: single-layer planar rectangular spiral, round copper wire,
outer envelope measured over the wire.

Method: Rosa/Grover segment summation - the loop inductance is the sum of the
self inductances of all straight segments plus the mutual inductances of every
pair of parallel segments (perpendicular pairs contribute nothing). Inductance
is computed at low frequency; skin effect is ignored in L but accounted for in
R_ac and Q.

Validated against:
  * the classical closed form for two parallel equal filaments,
  * a square loop vs an equal-perimeter circle (400.6 nH vs 394.1 nH),
  * Nordic's own example "Lant = 2 uH -> ~130 pF per pin" (gives 134 pF).

Usage:  python nfc_antenna.py
"""
import math

MU0 = 4 * math.pi * 1e-7
F0_NFC = 13.56e6
CINT_DEFAULT = 4e-12           # PS: CPAD_NFC (pad capacitance on NFC pads)
E12 = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2]
RHO_CU = 1.68e-8


def asinh(x: float) -> float:
    """Inverse hyperbolic sine (explicit, so old interpreters still work)."""
    return math.log(x + math.sqrt(x * x + 1.0))


def simpson(func, a: float, b: float, n: int = 4000) -> float:
    """Composite Simpson rule with an even number of intervals."""
    n += n % 2
    h = (b - a) / n
    total = func(a) + func(b)
    for i in range(1, n):
        total += (4 if i % 2 else 2) * func(a + i * h)
    return total * h / 3.0


def mut_parallel(l1: float, l2: float, d: float, s: float = 0.0) -> float:
    """Mutual inductance of two parallel filaments.

    Filament 1 spans x in [0, l1] at y=0, filament 2 spans [s, s+l2] at y=d.
    Inner Neumann integral is taken in closed form (asinh), outer by Simpson.
    """
    def outer(x1: float) -> float:
        return asinh((s + l2 - x1) / d) - asinh((s - x1) / d)
    return (MU0 / (4 * math.pi)) * simpson(outer, 0.0, l1)


def self_segment(length: float, wire_radius: float) -> float:
    """Self inductance of a straight round wire: (mu0*l/2pi)(ln(2l/a) - 3/4)."""
    return (MU0 * length / (2 * math.pi)) * (math.log(2 * length / wire_radius) - 0.75)


def loop_inductance(side_a: float, side_b: float, wire_radius: float) -> float:
    """Single rectangular turn (a = long side, b = short side, centre lines)."""
    own = 2 * self_segment(side_a, wire_radius) + 2 * self_segment(side_b, wire_radius)
    opposite = mut_parallel(side_a, side_a, side_b) + mut_parallel(side_b, side_b, side_a)
    return own - opposite


def coupling(turn_i: tuple, turn_j: tuple) -> float:
    """Mutual inductance between two nested rectangular turns (same current dir)."""
    ai, bi = turn_i
    aj, bj = turn_j
    long_pair = 2 * mut_parallel(ai, aj, abs(bi - bj) / 2, -(aj - ai) / 2)
    short_pair = 2 * mut_parallel(bi, bj, abs(ai - aj) / 2, -(bj - bi) / 2)
    return long_pair + short_pair


def spiral(coil: dict) -> tuple:
    """Inductance and wire length of a planar rectangular spiral.

    coil keys: w_out, l_out (outer envelope), d_wire, pitch, turns.
    """
    dw, pitch, n = coil["d_wire"], coil["pitch"], coil["turns"]
    radius = dw / 2
    turns = [(coil["w_out"] - dw - 2 * pitch * i, coil["l_out"] - dw - 2 * pitch * i)
             for i in range(n)]
    inductance = sum(loop_inductance(a, b, radius) for a, b in turns)
    inductance += 2 * sum(coupling(turns[i], turns[j])
                          for i in range(n) for j in range(i + 1, n))
    return inductance, sum(2 * (a + b) for a, b in turns)


def ac_resistance(wire_length: float, d_wire: float, f: float = F0_NFC) -> tuple:
    """DC resistance, skin-effect AC resistance and skin depth."""
    area = math.pi * (d_wire / 2) ** 2
    r_dc = RHO_CU * wire_length / area
    delta = math.sqrt(RHO_CU / (math.pi * f * MU0))
    r_ac = r_dc * (d_wire / (4 * delta)) if d_wire > 3 * delta else r_dc
    return r_dc, r_ac, delta


def tuning_cap(inductance: float, cint: float = CINT_DEFAULT, cp: float = 0.0) -> float:
    """Ctune per pin from the PS formula: Ctune = 2/((2pi f)^2 L) - Cp - Cint."""
    return 2 / ((2 * math.pi * F0_NFC) ** 2 * inductance) - cp - cint


def resonant_frequency(inductance: float, ctune: float, cint: float = CINT_DEFAULT) -> float:
    """f0 of the symmetric network: f0 = 1/(2 pi sqrt(L * C')), C' = (Cint+Ctune)/2."""
    return 1 / (2 * math.pi * math.sqrt(inductance * 0.5 * (cint + ctune)))


def cap_values() -> list:
    """E12 capacitor values from 4.7 pF to 470 pF."""
    vals = {round(e * m, 12) for m in (1e-10, 1e-11) for e in E12}
    return sorted(v for v in vals if 4.7e-12 <= v <= 4.7e-10)


def nearest_single(target: float) -> float:
    """Closest single E12 capacitor."""
    return min(cap_values(), key=lambda v: abs(v - target))


def best_pair(target: float) -> tuple:
    """Closest sum of two E12 capacitors: returns (label, value)."""
    vals = cap_values()
    best = min(((abs(c1 + c2 - target), c1 + c2, c1, c2)
                for i, c1 in enumerate(vals) for c2 in vals[i:]), key=lambda x: x[0])
    return f"{best[2] * 1e12:.0f}+{best[3] * 1e12:.0f}", best[1]


def report(geometry: dict) -> None:
    """Print one table for the given geometry, 2..8 turns."""
    print(f"\n=== {geometry['w_out']*1e3:.0f}x{geometry['l_out']*1e3:.0f} мм, провод "
          f"{geometry['d_wire']*1e3:.1f} мм, шаг {geometry['pitch']*1e3:.1f} мм ===")
    print(f"{'N':>2} {'L,мкГн':>8} {'провод,мм':>9} {'Rac,Ом':>7} {'Q':>5} "
          f"{'Ctune/пин,пФ':>13} {'E12':>7} {'f0,МГц':>7} {'пара':>10} {'f0,МГц':>7}")
    for n in range(2, 9):
        geo = dict(geometry, turns=n)
        inductance, wire_len = spiral(geo)
        _, r_ac, _ = ac_resistance(wire_len, geo["d_wire"])
        ct = tuning_cap(inductance)
        single = nearest_single(ct)
        pair_label, pair_value = best_pair(ct)
        print(f"{n:>2} {inductance*1e6:>8.2f} {wire_len*1e3:>9.0f} {r_ac:>7.2f} "
              f"{2*math.pi*F0_NFC*inductance/r_ac:>5.0f} {ct*1e12:>13.0f} {single*1e12:>5.0f}пФ "
              f"{resonant_frequency(inductance, single)/1e6:>7.2f} {pair_label+' пФ':>10} "
              f"{resonant_frequency(inductance, pair_value)/1e6:>7.2f}")


def self_test() -> None:
    """Cross-checks of the mutual-inductance and loop formulas."""
    closed = (MU0 / (2 * math.pi)) * (0.1 * math.log((0.1 + math.hypot(0.1, 0.1)) / 0.1)
                                      - math.hypot(0.1, 0.1) + 0.1)
    print(f"mut(0.1,0.1,d=0.1): {mut_parallel(0.1, 0.1, 0.1)*1e9:.4f} нГн "
          f"vs формула {closed*1e9:.4f} нГн")
    square = loop_inductance(0.1, 0.1, 0.5e-3)
    a_eq = 0.4 / (2 * math.pi)
    circle = MU0 * a_eq * (math.log(8 * a_eq / 0.5e-3) - 2)
    print(f"квадрат 100x100 мм: {square*1e9:.1f} нГн vs круг равного периметра: {circle*1e9:.1f} нГн")
    print(f"Nordic-проверка: 2 мкГн -> Ctune = {tuning_cap(2e-6)*1e12:.0f} пФ (в PS ~130 пФ)")


def main() -> None:
    self_test()
    report({"w_out": 33e-3, "l_out": 18e-3, "d_wire": 0.6e-3, "pitch": 0.6e-3})
    report({"w_out": 33e-3, "l_out": 18e-3, "d_wire": 0.6e-3, "pitch": 0.8e-3})
    report({"w_out": 31e-3, "l_out": 16e-3, "d_wire": 0.6e-3, "pitch": 0.6e-3})


if __name__ == "__main__":
    main()
