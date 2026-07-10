# Underwater ChArUco Calibration — Procedure Flowchart

Setup: waterproofed ZED camera fixed to an underwater robot, ChArUco board moved
by an operator, controlled over an Ethernet tether via VNC.

Render this diagram in any Mermaid-aware viewer (GitHub, VS Code Markdown preview,
mermaid.live).

```mermaid
flowchart TD
    START([Start: calibrate in water]) --> PREP

    subgraph PREP_STAGE [1 - Prep]
        PREP[Submerge camera in operating water/port] --> PORT{Port type?}
        PORT -->|Flat port| RADTAN[Use RadTan default<br/>8-coeff rational model<br/>expect RMS floor ~1px]
        PORT -->|Dome port| PIN[Near-pinhole<br/>default model fine]
        RADTAN --> BOARD
        PIN --> BOARD
        BOARD[Board: rigid, flat, matte,<br/>neutral buoyancy, high contrast<br/>angle dive lights off glare]
    end

    BOARD --> PARAMS[Set params:<br/>--charuco --squares_x/y<br/>--square_size --marker_size --dict<br/>verify against numbers on board]

    PARAMS --> PATH{Capture path?}
    PATH -->|Best: no VNC lag| SVO[Record SVO on dive]
    PATH -->|Live over VNC| LIVE[Move board slowly,<br/>pause 1-2s per pose,<br/>focus window before keys]
    SVO --> TOPSIDE[Copy SVO topside<br/>run tool with --svo<br/>on fast SDK machine]

    TOPSIDE --> LAUNCH
    LIVE --> LAUNCH
    LAUNCH[Launch tool<br/>./build_and_run.sh stereo_calib --charuco] --> HUD

    HUD{HUD: N/140 corners<br/>L and R both green?}
    HUD -->|N = 0 / very low| FIXDET[Get closer / clearer water<br/>kill glare<br/>try --charuco_legacy<br/>recheck dict + squares]
    FIXDET --> HUD
    HUD -->|Green, 6+ corners| ACQ

    subgraph ACQ_STAGE [2 - Acquisition loop]
        ACQ[Pose board for coverage:<br/>X/Y edges+corners, Size near+far,<br/>Skew tilt/rotate] --> CAP[Press S / spacebar to capture]
        CAP --> GATE{Frame accepted?}
        GATE -->|Blurry| BLUR[Slow board / add light /<br/>lower min_sharpness]
        GATE -->|Too similar / too small| VARY[New position + orientation]
        GATE -->|No target| FIXDET
        BLUR --> ACQ
        VARY --> ACQ
        GATE -->|Accepted| SCORES{All 4 scores 100%<br/>AND 25+ samples?<br/>OR 35 max reached?}
        SCORES -->|No| ACQ
    end

    SCORES -->|Yes| CALC[Auto-calibrate:<br/>per-camera + stereo solve<br/>outlier frames removed + re-run]

    CALC --> RMS{RMS acceptable?<br/>aim under 0.5px<br/>flat-port floor ~1px}
    RMS -->|Too high| DIAG[Flatten/re-mount board,<br/>kill glare, clean lens,<br/>more edge/skew/size coverage]
    DIAG --> ACQ
    RMS -->|Good| SAVE[Save zed_calibration_SN*.yml<br/>+ SN*.conf]

    SAVE --> VALIDATE[Validate in water:<br/>./build_and_run.sh stereo_check<br/>-- --charuco --calib_opencv &lt;file&gt;]
    VALIDATE --> CHECK{Left / Right / Stereo reproj<br/>hold low across full FOV?}
    CHECK -->|Red anywhere| ACQ
    CHECK -->|Consistent + low| DONE([Done: calibration valid])
```

## Notes keyed to the flow

- **Calibrate in water** — refraction changes focal length + distortion; a dry
  calibration is invalid underwater.
- **Capture path** — the SVO-record-then-replay-topside path avoids VNC lag and the
  moving-board timing problem entirely; strongly preferred.
- **HUD** — full board = (squares_x-1) x (squares_y-1) = 14 x 10 = **140** corners for the
  AndyMark 15x11 board. Underwater turbidity lowers N; ChArUco still uses partial views, so
  do not chase 140 — green (6+ corners) is enough to capture.
- **Keys** — capture: `S` / `s` / spacebar (use `S` if VNC eats spacebar); quit: `q` / `Esc`.
  Give the window focus first.
- **Sharpness gate** — `min_sharpness = 100.0` may reject good-looking underwater frames;
  can be exposed as a `--min_sharpness` flag if needed.
