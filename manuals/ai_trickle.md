# AI Trickle

AI Trickle replaces the fixed PID tuning with a controller that learns how your powder behaves in your trickler and plans every charge from that knowledge. It learns per profile, keeps learning while you reload, and lets you set the tolerance and the speed independently, or let it find the fastest safe speed by itself. A web page shows everything live and can optionally ask Claude to analyse your throws and propose settings.

## Why throws are sometimes over, sometimes under

When a tube stops, powder is still falling and the scale only reacts with a delay. How much is "in flight" depends on the powder, the speed and on chance: kernels come in bursts, stick powders form clumps, and the coarse tube keeps dribbling for a moment after it stops. A controller with fixed values cannot know any of this, so it lands above the target at one time and below it at another.

## How AI Trickle works

1. **Powder model.** For each profile the trickler learns
   `weight(t) = kc · coarse revs(t − latency) + kf · fine revs(t − latency) + tail`:
   powder per revolution of each tube, the delay between motor and scale reading, the tail of the coarse tube after it stops, and how strongly the flow fluctuates.
2. **Prediction instead of reaction.** During the charge it computes how much powder is still in flight and stops *before* the scale shows the target. Close to the target it slows down exactly enough that a flow surge cannot carry it past the target.
3. **Settle and top up.** After stopping it waits until the scale is settled. If the charge is short, it tops up with slow pulses of the fine tube. In precision mode it deliberately aims slightly low, because an over charge cannot be undone and a top up can.
4. **Continuous learning.** Every throw is recorded and fitted against the model afterwards, so the model follows changes in powder, humidity or hopper level. Each throw only counts as much as it actually reveals about each parameter. A strongly deviating throw is flagged as drift.
5. **Probe pulses.** A learning session starts with short, separate pulses from both tubes. Their sharp edges make the delay, flow and tail clearly measurable, independent of how well the old profile was tuned.
6. **Safety.** If the motors run but no powder arrives (empty hopper), the trickler stops and waits for a button press. Learned values are always limited to safe ranges.

## Usage

### On the display

Main menu → **Settings** → **AI Trickle**

* **Mode:** speed Precision / Balanced / Speed / Auto (or *Custom* from the web page), tolerance (±0.01 … ±0.20 gr), adaptive controller on/off, keep learning on/off. Confirm with **OK**.
* **Learn Profile:** choose a profile, enter the charge weight, start. The trickler throws 8 charges by default (adjustable on the web page). Empty the pan after each throw. The display shows `Learn 3/8`. It stops early as soon as the throws are within the tolerance.
* **Model Info:** learned values and achieved accuracy.
* **Reset Model:** forget what was learned for the current profile.

During a normal charge the title shows `AI: 42.00` and the line below the weight shows the phase, the speed and the tolerance (`Fine  Spd 70  Tol 0.02`). After the charge that line shows the result: `OK  +0.00`, `Over  +0.04` or `Under  -0.04`. The LEDs light green when the charge is within the tolerance.

If a profile has not been learned yet, the first charge automatically starts with probe pulses.

### Web page

Settings live where all other settings are, in the web portal under **Settings**:

* **Settings → AI Trickle:** on/off, tolerance, auto speed, speed, keep learning, throws per learning session
* **Settings → Profiles / Charge Mode:** a note says which values the AI uses (the min/max flow speeds) and which only the classic PID controller uses (Kp/Ki/Kd, stop thresholds)
* **Settings → Scale:** scale diagnostics
* **Settings → System Control:** firmware update

The **AI** button in the bottom bar (or `http://<trickler-ip>/ai`) opens the analysis page with three tabs: **Overview**, **Learn** (learning, series & ladder, model and backup) and **Assistant** (Claude).

* Live status with predicted final weight
* Current tolerance and speed (with auto speed: learned speed, over charge rate, top ups) and an estimated time, calibrated with your actual times
* Start and track a learning session
* Chart of the last throw: measured weight, model curve, coarse stop
* History with error chart, statistics (mean, spread, % within tolerance, average time); stored in the browser, per profile
* Learned model with explanations
* **AI assistant (Claude):** see below

### Claude assistant (optional)

1. Create an API key in the Anthropic Console (console.anthropic.com).
2. On the AI page under *API key and model*, paste the key and save. The key is stored **only in this browser** and sent directly to the Anthropic API, never to the trickler.
3. The device you use the page on needs internet access, so the trickler must be in WiFi station mode (your home network), not in access point mode.

Claude receives the settings, the learned model, the last throws and the trace of the last throw. It explains what is going on and proposes concrete changes as a card with **Apply** buttons. Nothing changes until you click one. Claude tunes the machine only and gives no load data.

Default model: Claude Opus 5.5. For lower cost choose Sonnet 5.5 or Haiku 5.5.

### Series and ladder

*Learn* tab → **Series & ladder**.

* **Series:** a fixed number of charges at one weight. During the series the display shows `Series 3/20 42.00`. Afterwards the trickler leaves charge mode and the page shows the result: mean, spread, share within tolerance, total time, as a table and as CSV.
* **Ladder:** start weight, step, number of charges and charges per rung. The trickler raises the charge weight after each rung by itself (display: `Ladder 4/10 42.90`). Before starting, the page lists every charge weight for confirmation. The result table shows the actual weights per rung. All values come from you; the trickler only checks for obvious typos (at most 100 charges, step at most 2 gr).

### Model backup

*Learn* tab → **Learned powder model** → **Backup**:

* **Export:** saves the models of all 8 profiles plus the AI settings as a JSON file.
* **Import:** restores such a file, for example after an EEPROM erase or on a new board.
* **Copy:** takes over the model of another profile into the current one, as a start for a similar powder. It keeps learning from there.

### Scale diagnostics

*Settings* → **Scale** → **Scale Diagnostics**. Outside charge mode, leave the scale untouched for 10/20/60 s. Result: readings per second, noise (after removing drift), drift per minute, largest jump, resolution, each with a plain verdict, e.g. "draft or vibration" or "let the scale warm up". The results are also passed to Claude.

### Firmware update over WiFi (Pico 2 W)

The Pico 2 W has two firmware slots (A/B). An update is written into the slot that is not running, checked with SHA-256 and only then started. The chip also checks the image hash itself at every start and falls back to the other slot if the image is damaged. Settings and learned models are stored in the EEPROM and survive every update.

If a new firmware does not work, hold BOOTSEL, plug in USB and copy a working UF2.

**One-time setup (USB):**

1. Hold BOOTSEL and plug in USB, the Pico shows up as a drive.
2. Drag `pico2_w_partition_table.uf2` onto it. The Pico restarts and shows up as a drive again.
3. Drag `pico2_w.uf2` onto it.

Both files are offered under *Settings* → **System Control** as long as the setup is missing. They are also in the `firmware` branch of your repository.

**After that:** *Settings* → **System Control** → **Check for Updates** → **Install** → **Restart & Activate**. Your phone or PC downloads the firmware from GitHub, checks it and sends it to the trickler, so the trickler itself does not need internet access. Alternatively, upload a `.bin` file from your computer with **Upload file**.

Charge mode must be off during an update.

**Prerequisite on GitHub:** in your fork under *Actions*, enable the workflows once. After that, every push to `main` builds the firmware and publishes it to the `firmware` branch together with a `manifest.json`.

The Pico W (RP2040) has no A/B support. For it, the page offers the UF2 file for the USB method.

## Tolerance and speed

The **tolerance** decides when a charge is good: the scale reading must be within target ± tolerance. The **speed** (0 … 100) decides how aggressively the controller gets there. They are independent, so ±0.02 gr at full speed is a valid setting.

| Speed | Behaviour |
|---|---|
| Precision (0) | slow final approach, aims slightly low, tops up anything in the lower half of the band |
| Balanced (50) | middle ground |
| Speed (100) | fast approach, hands over from coarse to fine as early as that is faster, only tops up readings outside the tolerance |

**Auto speed** (default) tunes the speed per profile while you reload: after 12 charges without an over charge it goes up by 10, as long as the charges actually get faster. Two over charges within about ten charges cost 25 (never below 30). If a higher speed made the charges slower (more top ups), it goes back to the fastest speed seen. The web page shows the learned speed, the over charge rate and the top ups per charge.

Note: a single kernel of extruded powder weighs about 0.01-0.04 gr. Tolerances below one kernel weight are physically impossible for that powder, and with a 0.02 gr tolerance one stray kernel decides.

### Does the AI change the Charge Mode settings?

No. The adaptive controller does not use the PID gains (Kp/Ki/Kd) or the coarse/fine stop thresholds at all; it plans every charge from the learned model. It does use from the profile and charge mode settings: the **min/max speeds** of both tubes, the coarse backoff, the servo gate, precharge, the LED colours, the decimal places and the zero check.

### Getting fast

The charge time is mostly decided by three things:

1. **Max speeds of the profile** (coarse and fine). With the default "AR2209" limits of 8 / 2 rev/s a ball powder flows about 13 gr/s from the coarse tube and 0.5 gr/s from the fine tube. Raising them to 15 / 5 rev/s cuts several seconds. The motor configuration has its own max speed; the lower one wins.
2. **The scale.** Set the A&D FX to stream at the fastest rate with the fast response setting. Every top up costs about one scale delay plus a settle (1-2 s); scale diagnostics under *Settings* → **Scale** shows the rate and noise.
3. **The powder.** Ball and fine extruded powders can be thrown fast and tight. With coarse stick powders the odd over charge comes from single kernels and clumps, at any speed.

Simulated, 44 gr, ±0.02 gr tolerance, after a learning session (100 charges each, "scale" = reading within ±0.02 gr, "true" = actual powder within ±0.02 gr):

| Powder | Limits coarse / fine | Speed | Ø time | Scale in tol. | True in tol. | Over |
|---|---|---|---|---|---|---|
| Ball | 8 / 2 rev/s | 0 | 19.6 s | 99 % | 99 % | 1 % |
| Ball | 8 / 2 rev/s | 100 | 11.0 s | 100 % | 62 % | 3 % |
| Ball | 8 / 2 rev/s | Auto (→100) | 10.6 s | 100 % | 72 % | 2 % |
| Ball | 15 / 5 rev/s | 0 | 16.9 s | 100 % | 99 % | 1 % |
| Ball | 15 / 5 rev/s | 50 | 12.5 s | 98 % | 97 % | 3 % |
| Ball | 15 / 5 rev/s | 100 | **9.7 s** | 100 % | 72 % | 0 % |
| Short extruded | 15 / 5 rev/s | 100 | 13.9 s | 88 % | 75 % | 9 % |

At high speed the controller accepts a reading of exactly target − 0.02 gr, so the true weight is often 0.02-0.03 gr low while the scale shows a hit. If you need the true weight inside ±0.02 gr, use speed 50 or a tolerance of ±0.01 gr.

Reproduce with `make -C tests/sim && tests/sim/trickle_bench study`.

## Simulation results

The controller was tested on the host with a physical simulator (`tests/sim`): kernels, flow fluctuation, clumps, flight time, coarse tail, and a scale with delay, filter, noise and 0.02 gr resolution (similar to the A&D FX-120i). Per configuration: a learning session, then 100 charges of 42 gr, compared with the classic PID controller and the default profile "AR2209".

| Powder | Controller | within ±0.02 gr | within ±0.04 gr | Ø time |
|---|---|---|---|---|
| Ball | Classic PID | 61 % | 99 % | 44 s |
| Ball | AI precision | **99 %** | 100 % | **20 s** |
| Ball | AI speed 100, ±0.10 | 47 % | 85 % | **9 s** |
| Short extruded | Classic PID | 64 % | 99 % | 56 s |
| Short extruded | AI precision | **85 %** | 92 % | **29 s** |
| Flake | Classic PID | 49 % | 97 % | 54 s |
| Flake | AI precision | **94 %** | 99 % | **25 s** |
| Long stick (clumpy) | Classic PID | 29 % | 82 % | 64 s |
| Long stick (clumpy) | AI precision | **46 %** | 80 % | **40 s** |

With long, clumpy stick powders a single clump (≈0.25 gr) decides the outcome. The AI is still about twice as fast there, but not more accurate in every respect. These are simulation values: real powders and tricklers differ, which is exactly why the system learns on your machine.

Reproduce:

```
make -C tests/sim run
```

## Development

* `src/trickle_model.c/.h` – model, controller, learning (pure C, also builds on the PC)
* `src/powder_sim.c/.h` – powder and scale simulator (also used by the *Simulated* scale driver: select it under Settings → Scale → Driver to try everything without powder)
* `src/ai_trickle.c/.h` – firmware integration: EEPROM (block at 11k), learning session, history, REST
* `src/html/ai_trickle.html` – web page
* `tests/sim/` – unit tests, benchmark, data export
* `tests/ai_web_mock.py` – REST mock for developing the web page

* `src/fw_update.c/.h`, `src/sha256.c/.h` – firmware update (HTTP POST into the free A/B partition)
* `src/scale_diag.c/.h` – scale diagnostics
* `targets/pico2_w_ab_partitions.json` – partition table, the build creates `pt.uf2` from it
* `scripts/make_firmware_manifest.py` – collects images + manifest for the `firmware` branch (CI)

REST endpoints: `/rest/ai_config`, `/rest/ai_model`, `/rest/ai_status`, `/rest/ai_history`, `/rest/ai_trace`, `/rest/ai_learn`, `/rest/ai_series`, `/rest/scale_diag`, `/rest/fw_info`, `/rest/fw_apply`, `POST /upload/firmware?size=&sha=`.
