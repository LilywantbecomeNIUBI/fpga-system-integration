# Mecanum navigation: pre-RTL cycle and ownership review

Design gate: this document is written before the new grid/sweep RTL. Clock is
50 MHz, all cycle counts below include explicit synchronous RAM latency. No
RAM reset, combinational RAM read, full-array clear, or inferred extra read
port is permitted. Each engine issues at most one multiplication OR one RAM
read in a cycle. Comparisons and commits use registered intermediate values.

## Reuse ledger

* `nav_math`, `nav_vfh`, `nav_histogram`: existing Task4 RTL and independent
  `tools/task4/model.py`, math/VFH fixture regressions. Keep original modules
  intact. VFH is advisory only; its circular footprint must not veto a safe
  rectangular candidate. Differential wheel/pose equations are not reused.
* `mc_cordic_sincos`, mecanum wheel ordering/mixer, 1 kHz loop and V3 motion
  protocol: current integrated, board-tested baseline. No additional IMU owner.
* RAM: Gowin SUG550-2.4E (installed IDE), section BSRAM inference; existing
  `lidar_scan_360_source.sv` synchronous SDP template. One write + registered
  read, no same-address mixed-port dependency. No vendor source copied.
* Nav2 footprint checker / line iterator (Apache-2.0, repository URL in
  `MECANUM_NAVIGATION.md`): algorithm reference, not copied RTL. Filled footprint
  and step padding are required, not just perimeter line tests.

## Map organization / transaction states

### VFH reused distance table: device-compatible synchronous SDP correction

The first whole-system implementation rejected the old inferred SP RAM's
WRITE_MODE=10 (PA2122, GW5AT-138B). Keep the algorithm and use explicit separate
clocked write/read ports, without RAM reset. The complete72-word fill precedes
all reads, so no same-address read/write result is used. State23 writes
`ranges[i] = input_distance`; state3 issues the synchronous read at address i
and captures observed/binary flags; new state30 captures range_q into d; state24
consumes d and saved flags. This adds exactly72 clocks per VFH run. Review gate:
unit synthesis must infer SDP instead of the unsupported SP write mode, and all
existing226 reference cases must retain the same selected headings/masks/costs.

Logical banks: two 1024x32 memories packed in one 2048x32 synchronous SDP array.
Cell codes: UNKNOWN=0, FREE=1, OCCUPIED=2, RESERVED=3 (blocked).
`cell={y[6:0],x[6:0]}`, `word={bank,cell[13:4]}`, shift=`{cell[3:0],1'b0}`.
One physical registered read is arbitrated: updater READ wins one cycle;
planner can read on other cycles. Planner receives an explicit grant and
response valid; it must not assume a fixed grant interval. Foreground is
immutable while `planner_lock` is high. Background is not externally readable.

| State | Action / next state | Multiplication | RAM action |
|---|---|---|---|
| BOOT | counter 0..2047; ready only after final write | none | one zero write |
| CLEAR | clear background word 0..1023; then IDLE | none | one zero write |
| IDLE | accept one cell transaction or publish request | none | planner may read |
| ADDRESS | latch packed address, slot, requested code | none | planner may read |
| READ | issue background word address | none | one updater read |
| LATCH | capture registered RAM result / extract old cell | none | planner may read |
| CLASSIFY | compare old/requested values, occupied wins | none | planner may read |
| MERGE | construct one complete 32-bit updated word | none | planner may read |
| WRITE | commit updated word, acknowledge cell, IDLE | none | one background write |
| PUBLISH | wait for no planner lock/response, swap and increment epoch; old foreground becomes next background | none | no access |

The first four cell phases are ADDRESS, READ, LATCH, CLASSIFY. A subsequent
update (including another cell in the same word) cannot be accepted before
WRITE completes. Reset clears valid/ownership only, then BOOT clears storage.
Map stale threshold 250 ms; malformed/overflow scans cannot publish. Code 3
is never cleared by FREE. Boot needs 2048 writes; each later clear 1024 writes.

## Ray update engine

FIFO contains point angle/distance/intensity plus arrival pose and error epoch.
FIFO data is synchronous RAM, pointers/valid only are reset. Overflow poisons
the scan. Twelve points arriving on adjacent cycles must fit without backpressure.
Use the existing iterative CORDIC for trigonometry; no floating point or `/50`
combinational divider. Cell conversion is signed constant division with explicit
floor for negative coordinates, implemented as a 33-cycle restoring divider.

| Phase | Registered action | Resource |
|---|---|---|
| POINT | capture point, pose/epoch, compare angle wrap in following phase | none |
| ANGLE | compute sensor angle + pose angle - map angle | additions |
| TRIG | start/wait existing CORDIC | no multiplier/read |
| PX | range * cosine | one multiply |
| PY | range * sine | one multiply |
| OFFSET | add sensor translation and pose displacement | additions |
| BOUNDS | retain signed endpoint; compare next ray cell with bounds | comparisons |
| RAY_INIT | initialize Bresenham dx/dy/error and endpoint marker | additions |
| CELL | submit FREE ray cell, OCCUPIED endpoint; wait for grid ack | grid transaction |
| RAY_FLAGS | compare doubled error / endpoint / bounds | comparisons |
| RAY_STEP | update x/y/error using saved flags; CELL or POINT | additions |

Zero/low-quality readings are not out-of-range evidence. Range clearing is
disabled until the sensor provides a separately validated status. Known self
mask is applied before geometric hazards, not a generic near-distance discard.
Current-body FREE initialization precedes all observed obstacle writes.

## Filled rectangle sweep engine

Input is a bounded candidate trajectory, immutable foreground epoch/pose and
calibrated footprint. Read all covered cells, not only rectangle edges. Raster
bounds include half-cell diagonal and inter-step displacement/rotation padding.
Outside-grid or arithmetic overflow rejects a candidate. UNKNOWN accumulates a
finite cost and enforces the low-speed limit; codes 2/3 reject immediately.

| Phase | Action | Resource / address |
|---|---|---|
| CAPTURE | latch candidate, map epoch, trajectory step | none |
| DIFF | cell/map/trajectory coordinate differences | additions |
| MUL_XC | dx * cos(theta) | one multiply |
| MUL_YS | dy * sin(theta) | one multiply |
| SUM_X | local_x = registered products sum | no multiply/read |
| MUL_XS | dx * sin(theta) | one multiply |
| MUL_YC | dy * cos(theta) | one multiply |
| SUM_Y | local_y = registered products difference | no multiply/read |
| FLAGS | compare footprint extents / grid bounds | comparisons |
| SELECT | skip outside footprint or request cell | saved flags only |
| READ | request `{foreground,y[6:0],x[6:0]}` packed word; hold until grant | one RAM read |
| WAIT | wait response-valid, capture cell code | none |
| COST_FLAGS | occupied/unknown/overflow flags | comparisons |
| COST | saturating unknown/clearance cost arithmetic | additions |
| ADVANCE | commit flags, next cell/step/candidate, or DONE | no multiply/read |

First four arithmetic phases are DIFF, MUL_XC, MUL_YS, SUM_X; the other product
pair is deliberately NOT combined into those cycles. Shape setup uses the
same serialized arithmetic. Circular overlap means 'needs rectangle test',
never 'rectangle definitely collides'. Top-eight misses return SEARCH_LIMITED,
not NO_PATH. Remaining candidates rotate across planning periods.

| Sweep control state | Entry / register writes / exit |
|---|---|
| IDLE(0) | start captures all inputs/epoch, initializes count/cost/safe/cycles; invalid steps/geometry -> FINISH, otherwise TRIG_START |
| TRIG_START(1) | pulse trig_start -> TRIG_WAIT(2) |
| TRIG_WAIT(2) | trig_done captures s32 Q16 cs/sn; bound_index=0 -> BOUND(3) |
| BOUND(3) | index_mm=px-radius-pad, px+radius+pad, py-radius-pad, or py+radius+pad according to bound_index -> INDEX_START(4) |
| INDEX_START(4) | pulse index_start -> INDEX_WAIT(5) |
| INDEX_WAIT(5) | done+outside -> safe=0/FINISH; otherwise latch lo/hi_x/y; next bound -> BOUND; fourth result initializes cx/cy -> DIFF(6) |
| DIFF..ADVANCE(6..19) | exact per-clock arithmetic/read table above; registered footprint flag selects read/skip; OCCUPIED -> safe=0/FINISH; last raster cell -> NEXT_STEP |
| NEXT_STEP(20) | count+1>=total -> FINISH; otherwise count++, px+=sx, py+=sy, next_angle=s33(angle)+step_theta -> ANGLE_FLAGS |
| ANGLE_FLAGS(22) | compare next_angle against +/-pi, register high/low flags -> ANGLE_COMMIT |
| ANGLE_COMMIT(23) | saved high/low flags select +/-2pi normalization and commit s32 angle -> TRIG_START |
| FINISH(21) | one-cycle done=1, busy=0 -> IDLE; retained results valid only with done |

All busy states except FINISH have a common highest-priority exit: abort or
epoch mismatch sets safe=0 -> FINISH; cycle guard additionally sets timed_out.
The guard also covers CORDIC, divider and stalled RAM waits. Arithmetic cannot
escape the module budget by remaining indefinitely in a wait state.

## Bounds and failure behavior (review checklist)

* Planner budget: 2,500,000 clocks (50 ms), including ranking, 8 precise tests
  and commit. No path requiring unbounded while loops is synthesizable.
* Sweeper has an independent cycle guard and explicit abort; timeout, epoch
  mismatch or lost map lock rejects output before motor target commit.
* Rays stop at their first exit from the 128x128 grid, after at most 128 cells;
  source FIFO depth >=32, point engine must prove max service rate against
  12-point/921600-baud bursts. Any overflow invalidates that revolution.
* No restart releases an old permit. Global STOP/lease/sensor faults override
  directional constraints. Projected escape velocity is rechecked, including
  actual velocity and stopping tail; rotation is not exempt from collision.
* CDC source holds complete snapshot until acknowledgment; sink timeout does
  not depend on source clock. Each new path needs explicit SDC evidence.
* Unit synthesis must confirm expected BSRAM mapping before whole-system PnR.
  Existing full system: 224/340 BSRAM; camera's 129 must remain unchanged.

Pre-RTL review: checked port count, nonblocking read latency, serialized RMW,
bank-lock lifetime, signed floor/bounds, and separation of arithmetic/flags/
commit. Detailed implemented FSM cycle bounds and regression evidence are
recorded alongside each kernel's tests; this document is not timing signoff.

## Implemented width / valid / read-latency contract

The tables above were the pre-RTL gate. This appendix records the reviewed
implementation, including the later slope-preserving ray fix. All positions
below are map-anchor millimetres unless marked otherwise. `sN` means signed N
bits; integers have no fractional bits. Angles/trig values use s32 Q16. Products
are s64 Q16 and are shifted only after both products have been registered.

| Register/interface | Width / format | Lifetime / valid |
|---|---|---|
| point FIFO | 32 x 168 bits | synchronous read, no data reset |
| FIFO record | angle u16, distance u16, intensity u8, pose x/y s16 integer, yaw s32 Q16, error u32, loss u32 | FIFO_WAIT captures response; POINT consumes |
| FIFO pointers/count | u5/u5/u6 | push requires count<32; conservative full+pop drops/poisons rather than overwriting |
| map anchors / ray origin and endpoint | s32 integer; yaw s32 Q16 | anchor immutable during scan; separate published anchor + metadata_epoch |
| cell conversion | numerator/quotient u33, remainder u7, iterations u6 | start then 33 busy cycles; done tags index/outside/extended_index together |
| extended cell / Bresenham x,y | s13 integer | supports physical 16-bit range + calibrated offsets; not the full s32 mm domain |
| Bresenham dx,dy,error | s15 integer; doubled error s16 | compare flags before next-cell commit |
| grid cell / word / slot | u14 / u11 / u5 | `{y,x}` / `{bank,cell[13:4]}` / `{cell[3:0],0}` |
| grid read result | u32 word then u2 cell | response valid required, never sample combinational array data |
| sweep x/y,dx/dy,local_x/y | s32 integer | captured start; DIFF and SUM states |
| sweep sx/sy | s16 integer per 100 ms | 20 samples production; total u6 <=32 |
| sweep angle/step | s32 Q16 | production heading/omega bounds keep every angle inside CORDIC [-pi,pi] |
| sweep products | four s64 Q16 registers | one shared s32 x s32 multiplier, XC/YS/XS/YC only |
| bounds/cell counters | u7 x4/u7 x2 | each four bounds waits for its own division done |
| sweep cost/cycles | u32/u32 | UNKNOWN saturating cost; deadline overrides safe |

### Complete mapper transition / write table

State numbers are the RTL constants. Each listed arrow costs one clock unless
it explicitly waits. Any reset returns IDLE, drops FIFO ownership and scan
validity. `!enable` does the same without clearing RAM. Default state aborts.
`scan_abort` poisons the background; only a complete subsequent scan recovers.

| States | Register work / next condition | Multiply / RAM request |
|---|---|---|
| IDLE(0) | nonempty -> FIFO_WAIT; advance rp/count | FIFO read A=rp only |
| FIFO_WAIT(1) | record=fifo_q -> POINT | none |
| POINT(2) | save previous angle; first wrap -> BEGIN; later valid wrap -> SEALED; discontinuity -> abort/IDLE; invalid return -> IDLE; else ANGLE | none |
| BEGIN(3) | scan_begin, capture pose anchor/error/loss, clear danger_work -> SEAL_BEGIN | none |
| SEAL_BEGIN(22) | -> CLEAR_WAIT | none |
| CLEAR_WAIT(4) | grid clear done -> anchor CORDIC | none |
| TRIG_START(9) / TRIG_WAIT(10) | start then wait for 16-iteration CORDIC; mode2 -> self index; mode0 -> LXC; mode1 -> PX; mode3 -> BODY_PX | none |
| INDEX_START(14) / INDEX_WAIT(15) | one coordinate division at a time, coordinates0..3 ray origin/endpoint;4..7 self rectangle | none |
| SELF(5) / SELF_WAIT(6) | submit FREE at `{init_y,init_x}`, wait cell_done, increment x then y; final -> ANGLE or IDLE | grid RMW, no mapper read/multiply |
| ANGLE(7) | angle_product=(angle-36000 if >18000)*11993645 -> ANGLE_SUM | one multiplication |
| ANGLE_SUM(8) | normalize body minus anchor angle first; save in ray_angle and trig_angle, mode0 -> TRIG_START | none |
| LXC(25), LYS(26), LXS(27), LYC(28) | laser x*cos, y*sin, x*sin, y*cos; separate s64 registers | one multiplication each |
| TXC(29), TYS(30), TXS(31), TYC(32) | displacement x*anchor_cos, y*anchor_sin, x*anchor_sin, y*anchor_cos | one multiplication each |
| ORIGIN_SUM(33) | sums/shifts -> ox/oy -> RAY_TRIG | none |
| RAY_TRIG(34) | add sensor yaw and saved ray angle to already normalized relative heading; normalize once more, mode1 -> TRIG_START | none |
| PX(11), PY(12) | distance*cos then distance*sin | one multiplication each |
| SUM(13) | ex/ey=origin+shifted products; mode3 body angle -> TRIG_START | none |
| BODY_PX(35), BODY_PY(36) | distance*body cos then sin | one multiplication each |
| BODY_SUM(37) | hazard_x/y=laser offset+shifted products -> CLASSIFY | none |
| CLASSIFY(23) | self_hit flag only if inside body AND calibrated mask -> SELF_MASK_DECIDE | none |
| SELF_MASK_DECIDE(38) | self_hit -> IDLE without clearing ray; otherwise -> HAZARD_FLAGS | none |
| HAZARD_FLAGS(41) | register interior-body flag, expanded danger-band flag and four axial normal flags | comparisons only, no multiply/read |
| HAZARD_COMMIT(42) | interior unmasked return -> all normals latched; else saved band/normal flags update danger; start origin indices | registered flags commit, no multiply/read |
| RAY_INIT(16) | cx/cy=origin; dx/dy/step signs -> HAZARD | none |
| HAZARD(24) | err=dx+dy -> CELL | none |
| CELL(17) / CELL_WAIT(18) | submit `{cy,cx}` FREE or valid endpoint OCCUPIED; wait ack; doubled error/at_end -> RAY_FLAGS | grid RMW only |
| RAY_FLAGS(19) | registered move_x/move_y -> RAY_STEP | none |
| RAY_STEP(20) | endpoint -> IDLE; else next_cx/next_cy/error -> RAY_BOUNDS | none |
| RAY_BOUNDS(39) | register inside-grid flags -> RAY_COMMIT | none |
| RAY_COMMIT(40) | flags true: commit cx/cy -> CELL; else IDLE | none |
| SEALED(21) | wait grid epoch change; publish anchor/metadata_epoch and danger snapshot -> BEGIN | none |

SEALED cannot exchange banks until planner_lock and pending read responses
drain. Planner deadline bounds this wait. Stopped point input cannot publish a
partial scan; foreground expires independently. Timeout of stale map removes
motor permission even while the mapper waits. No overflow can publish the
affected revolution: FIFO loss is captured in each record and checked at wrap.

### Physical SDP clock table

| Edge/state before edge | Read port | Write port | Valid/tag after edge |
|---|---|---|---|
| BOOT/CLEAR | planner read allowed during CLEAR only, foreground bank | zero at counter address | initialization remains asserted through last write |
| cell accept IDLE | optional foreground read | none | updater latches background word address/slot/code |
| ADDRESS | optional foreground read | none | -> READ |
| READ | background update_address -> ram_q | none | planner_ready=0 |
| LATCH | optional foreground read | none | old_word/old_code capture previous ram_q |
| FLAGS / MERGE | optional foreground read | none | resolved then merged full word |
| WRITE | optional foreground read | merged word at background update_address | cell_done=1; next transaction starts later |
| any granted planner request | `{foreground,read_address[13:4]}` -> ram_q | background only | read_tag=1, read_slot latched |
| following edge | arbitrated next address | background only | read_done=old read_tag; read_code=previous ram_q slot |
| PUBLISH | no request accepted | none | exchange only !lock && !read_tag && !read_done |

Reads and writes never depend on same-address collision mode: simultaneous
planner read and writer use different banks; the updater's own read and write
are separate states. RAM output registers/data arrays have no reset.

### Hard bounds and throughput

Grid cell handshake costs at most 10 clocks including request/ack observation;
128 ray cells take at most 128*17=2176 clocks including Bresenham states. Four
33-cycle divisions, three CORDIC handshakes and serialized geometry keep a
normal point below 2500 clocks. Scan initialization adds <=1024+325*10+300
=4574 clocks at maximum calibrated 600/600/300 mm footprint. FIFO absorbs 32
points (two full 12-point bursts plus8); physical 47-byte/921600baud packets
take >=25499 clocks, and the tested 12-adjacent-point/25000-clock-gap stream
has zero loss. Sustained pathological longest-ray throughput is NOT promised:
2500*12>25499, so overflow must poison a revolution and stop on expired map.
No finite FIFO is claimed to repair a sustained rate mismatch.

Sweep bounding side <=ceil(2*(800+464)/50)+2=53 cells. Conservative per-candidate
WCET <=32*(53*53*18+180)=1,623,744 clocks without updater arbitration; production
20 samples <=1,014,840 clocks. Each grid updater collision adds at most one wait
clock to a read request. Ranking 81 candidates and insertion <=12,000 clocks;
eight maximum-sized sweeps can exceed50ms. The **whole planner hard guard at
2,480,000 clocks** then rejects the whole round, zeroes its output and releases
the lock. It does not commit an earlier partial winner. A 20,000-clock reserve
covers service transforms/divisions/commit. Normal tested planner rounds use
174,377..251,026 clocks (3.49..5.03 ms). Timeout is a tested stop behavior, not
an assertion that every footprint can check eight trajectories in50ms.

Reference comparison: `tools/mecanum/navigation_model.py` independently uses
rectangle/square SAT and 5ms dense poses. One hundred translation/rotation/interior
scenes have no missed collision; five conservative false positives are allowed
and counted. Additional RTL tests cover UNKNOWN, reserved code3, bank exchange,
same-word updates, sparse long rays, self masks, reset, timeout, later candidates,
momentum and old-map abort. This is functional simulation, not board acceptance.

Final continuous-sweep budget: user pad includes >=36mm half-cell diagonal
and measured braking distance. The additional64mm covers <=29mm accumulated
integer step error plus <=22mm nearest-sample displacement (150mm/s translation
and radius800mm *0.35rad/s rotation, at half of100ms), plus trig rounding.
Planner rejects measured speed beyond its calibrated limit and angular step
>0.035rad, so this envelope cannot be bypassed by clipping a candidate.

## Coordinate service resource serialization (implementation review)

The first hardware snapshot maps the service's mutually exclusive multiply
statements into31 MULT27X36 macros. Replace those statements with one shared
s32 x s32 multiplier, preserving all product widths and output rounding:

| Clock/state | Work / register writes | Resource / tag |
|---|---|---|
| Original multiply state3..6,8..11,19..22,24,30..33,45..48 | latch mul_a/mul_b s32, mul_slot u2 and mul_return u6; next54 | operand setup only, no RAM read |
| 54 | mul_product s64 = registered mul_a * mul_b; next55 | one multiplier, no input-state mux in multiplier path |
| 55 | write p0/p1/p2/p3 selected by registered slot; state=registered return | commit only |
| 50 (native atan done) | capture desired_heading s16; normalize low16 bearing to0..35999; start existing nav_math op1 with divisor500; next52 | no combinational divide |
| 52 | wait math_done, commit quotient[6:0] as histogram index; start VFH | existing bounded divider, no new multiplier |

Each product adds2 clocks; all21 products add42 clocks. Sector division adds
the existing bounded divider latency (under100 clocks). The20,000-clock
service reserve remains ample. Reset clears the slot/return/product registers;
global fault/origin checks retain priority. Compare every product group against
the frozen original service at checkpoints7/12/23/25/34/49, with signed/boundary
and seeded inputs, before a fresh whole-system synthesis and route.

## Generated map reset: intermediate revision (superseded below)

The timestamp age is a multi-bit combinational subtraction/comparison. It must
not directly drive asynchronous RAM-controller resets. A single clk50 register
map_enable_q samples `!origin && imu_fresh && !odom_overflow`; only its Q and the
common reset drive map_reset_n. Root reset assertion stays asynchronous; pose
fault/origin map invalidation takes at most one extra clock, while independent
global motion permission still revokes immediately. No RAM data reset is added.
Test a skewed65535->65536 timer transition via an intermediate131071 bus value:
the old combinational predicate pulses reset, the registered predicate must not.
Also test legitimate expiry, fresh-sample recovery and common asynchronous reset.

## Compile-time specialization of the reused arithmetic unit

The generic nav_math originally retained every operation in both navigation
instances despite disjoint call sets. Add OP_MASK, default11111 for legacy
callers. The path instance uses00111 (multiply/divide/sqrt); the coordinate
service uses11010 (divide/atan2/sincos). Constant guards cover both startup and
busy datapaths, so unused CORDIC or integer multiply/sqrt logic can be removed
at elaboration rather than relying on unreachable-state inference.

No enabled state, register format, result rounding or latency changes: multiply
32 iterations, divide64, sqrt32, CORDIC20 iterations plus final output cycle.
Disabled operations return result0/error/done at the first accepting edge;
invalid busy mode also terminates with error. Start while busy stays ignored.
Reset and the50ms planner/service budget are unchanged; no RAM is added.

An immutable pre-change full unit is stored in tests/navigation/reference.
The specialized/full instances match its outputs and handshakes for1734 cases
and58248 checked cycles, including signed extremes, disabled operations and
mid-transaction reset. This is an area simplification; its physical routing
benefit must be checked in a new whole-system frozen build.


## STA-driven revision reviewed before RTL (2026-11-03)

No clock periods, interfaces or timing exceptions are relaxed. Packed map data
RAM stays unchanged. Map fault/origin clearing becomes synchronous control
clearing driven by map_enable_q; only common rst_n drives asynchronous pins.
Grid clear overrides all controller state/valid tags at an edge, restarts BOOT
at address0, and forbids publication until2048 sequential writes complete.
The RAM process has no reset/soft-clear branch. Histogram FIFO/sector RAMs
have separate unreset synchronous ports; known bits alone mask invalid data.

| Module / state | Arithmetic / request | Next / commit |
|---|---|---|
| service56 | latch signed omega magnitude |57 |
| service57 ->54 ->55 | shared s32 multiply magnitude*26215; no RAM |58 |
| service58 | slice registered product >>18, restore sign |30 |
| planner TEST_SCORE | latch signed16 magnitudes | STEP_X |
| STEP_X / STEP_Y | one unsigned17x18 product per state | STEP_COMMIT |
| STEP_COMMIT | slice products >>18, sign restore, start sweep | TEST_WAIT |
| TEST_WAIT | on sweep_done latch sum and eligibility/unknown flags | TEST_COMMIT |
| TEST_COMMIT | compare registered total to best | commit winner / next |
| histogram FIFO_REQ | one FIFO synchronous read | FIFO_LATCH |
| FIFO_LATCH | capture point; no RAM read | BIN |
| BIN | repeated subtraction500, no multiplier | OLD_REQ |
| OLD_REQ | one background-sector synchronous read | OLD_LATCH |
| OLD_LATCH | capture old distance/stamp/known | OLD_FLAGS |
| OLD_FLAGS | timestamp compare | UPDATE |
| UPDATE | prepare one merged stamp/range word, known flags | WRITE |
| WRITE | mirrored identical writes, no read of that address | IDLE |

External histogram read is synchronous, tagged, and bypasses pending commits
for same-word hazards. Callers receive an explicit extra wait state as needed;
no stale index is interpreted as the new index. Disabled-map clearing discards
FIFO/control state and clears only known bits, never RAM arrays. Worst FIFO
point service remains <100 clocks;32 entries cover native12-point bursts.
VFH signed18 coordinates still require signed19 intermediate turn shift:
turn_radius is unsigned16, and subtract/add must be evaluated before narrowing.
All full16-bit turn/safe radius interfaces retain their mathematical range.

Frame stream retains exactly the same four banks and packed memory depth.
New local write stage extends frame publication delay from2 to3 clocks.
New local read-address stage extends fill-valid/index and bank tags from4 to5.
Only data/address/control pipeline registers change; no camera RAM reset or
new frame buffer is introduced. LK adds PREP_U/PREP_V before existing DIV_U/V;
registered numerator magnitude/sign feeds shift-only dividend load.


Final reset topology supersedes the intermediate map_reset_n above: grid,
histogram and advisory receive only common rst_n on asynchronous reset pins;
optional internal SYNC_CLEAR defaults0 for legacy callers and is enabled by
the navigation service. map_clear=!map_enable_q synchronously clears control
ownership/valid tags. Raymap receives common rst_n and enable=calibrated &&
map_enable_q. Disable cancels publish/scan_begin and queued metadata and asserts
scan_abort. Faults still revoke motion immediately through the independent gate;
control invalidation takes the registered predicate plus one synchronous edge.
Reset releases need no new false path. Map RAM never gets reset or bulk-cleared.

Final observed planner rounds174572..251221 clocks vs174377..251026 previously:
195 extra clocks from score/step/commit stages;50ms guard unchanged. VFH max6446
clocks after histogram latency alignment, vs6374 before. All remain within the
existing service reserve. New reference/gate tests cover18 kernel benches,
2560 settled histogram reads and111289 signed reciprocal inputs plus65536 accel
values. Histogram inferred4 SDPB and347 FF in the standalone unit; final whole
system inference is separately audited.
