# FPGA build policy

- User instruction (2026-11-03): future hardware placement/routing uses whole-system incremental builds. Do not start whole-system placement/routing from scratch or silently fall back to a clean build.
- Use `tools/task4/build_ethernet.py --system --mecanum --raw-points --clock-cleanup --route-option 1`. It freezes a copy of the complete `.pr` reference from `config/pnr.json` and applies Gowin `set_option -inc_pnr`.
- Preserve each completed whole-system `.pr`, its manifest and timing evidence. Missing or incompatible references must be reported before doing placement/routing.
- An explicit user override may authorize `--full-rebuild`; do not infer it from an incremental failure. On 2026-11-03 the user explicitly authorized a full build of the three pipeline optimizations after PA2136 rejected the old reference, requesting the fastest practical implementation settings. Use Place 0 / Route 2 for this authorized attempt; retain final checks. The default for later builds remains incremental.
- Recompile changed RTL as necessary, and retain module and full-system functional tests plus final timing audits. Incremental compilation does not waive checks or imply timing closure.
- Keep the camera frame buffer architecture and its 129 BSRAM primitives unchanged. PR1014 and the known eth125 MPW -0.849 ns remain recorded as the user's accepted experimental hardware-test boundary; do not conceal them or relabel full STA signoff as passed.
