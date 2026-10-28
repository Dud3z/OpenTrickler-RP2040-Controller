# Safety, Watchdog, Display and Updates

All functions here can be switched off. Their settings are stored in their own EEPROM block, so the older settings blocks stay untouched (no reset after the update).

## Safety while charging

*Settings* → **Charging** → *Safety*:

* **Stop when the pan is lifted** (default on): if the reading falls more than 2 gr below zero during a charge, both motors stop at once, the servo gate (if used) closes and the charge is aborted. The display shows *Pan removed!*, the web page a message. Empty the pan and put it back, the trickler zeroes and carries on with the next charge. The aborted charge is logged as *aborted* in the statistics.
* **Pause when the table is bumped** (default on): powder can only make the reading go up. If it drops by more than the set amount (default 0.5 gr) compared with the last 0.4 s, the motors pause and the display shows *Bump! Waiting for scale*. As soon as the scale is stable again (5 readings within the set point accuracy) the charge carries on from the stable weight; after a minute without calm it carries on anyway.
  * While the coarse trickler runs, and for 2 s after it stopped, only drops of more than 5 gr count. Reason: the falling coarse stream pushes on the pan, and when it stops the reading falls by that push (about 1 gr at high flow) without anything being wrong.
  * Raise the amount if charges pause without a reason (e.g. a noisy scale), lower it to be more sensitive.

The logic is tested on the PC (`tests/sim/guard_test.c`): 50 normal charges with 5 to 30 gr/s coarse flow and the stream push give no false alarm; bumps in the fine phase, big bumps in the coarse phase and a lifted pan are found.

## Motors

*Settings* → **Motors** → *Both Motors*:

* **Reduced standstill current** (default on, 30 %): the TMC2209 drivers lower the current about 0.4 s after a motor stopped and raise it to the full current with the first step. Motors and drivers stay cooler while the trickler waits for the pan. Switched off, the driver default of 50 % is used (as before). If a tube does not start reliably after a pause, raise the value.

## Display and LEDs

*Settings* → **Display & LEDs**:

* **Dim after a while without use** (default on, 300 s, 10 %): backlight and LEDs go down. A turn or press of the knob, a charge, the web page or something put on or taken off the scale wakes them up. Never while charging.
* **Sound on the web page** (default on): the Mini 12864 of the OpenTrickler board has no beeper connected (EXP1 pin 1 is not wired) and all free GPIOs are in use, so the web page beeps instead: two short beeps when the charge is within the tolerance, a low tone for over and a rising tone for under, an alarm when the pan was lifted. Phones play sound only after one tap on the page.
* **QR code**: main menu → *Wireless* → *Wifi Info*, turn the knob: a QR code opens the web page (address of the trickler). In hotspot mode a second one joins the hotspot (phones offer to connect), then the first one opens the page.

## Watchdog and error log

*Settings* → **System**:

* **Watchdog** (default on): restarts the trickler when the firmware hangs for 8 s. This is also a safety function: the motors are stepped by the PIO hardware, which keeps turning at the last speed when the CPU hangs; the restart stops them.
* **Restarts and errors**: shows the start number, the reason of the last start and a log of the last 10 errors with how long the trickler ran before:
  * *Hang* – the watchdog restarted the trickler
  * *Internal check failed* – with file and line
  * *Firmware error (panic)*, *Stack overflow* (with the task), *Out of memory*, *Crash (hard fault)* (with the address)

  Before, such errors simply froze the trickler. If it crashes three times in a row right after the start, it stays stopped instead of restarting endlessly.

## Unsaved settings

*Apply* without *Save to EEPROM* keeps a setting only until the next restart. The settings pages show which pages have such changes, with a **Save now** button.

## Firmware updates

* **Notice** (*Settings* → **Firmware Update**, default on): the web page looks for a new build at most once a day and marks *Settings* and *Firmware Update*. The Pico 2 W also looks itself (2 minutes after the start, then daily, never while charging); the main menu shows *Update!*.
* **Update from the display** (Pico 2 W): *Settings* → **Firmware Update** → OK. The trickler reads the manifest of the `firmware` branch on GitHub, shows the installed and the available build, downloads `pico2_w.bin` into the free A/B partition (progress bar), checks the SHA-256 and restarts into it on a press. Settings and learned models stay (EEPROM).
  * The connection is HTTPS; the GitHub certificate has to chain to one of the root certificates built into the firmware (`src/fw_ca_bundle.h`: USERTrust/Sectigo, DigiCert G2/G3, GlobalSign, ISRG). If GitHub ever switches to another CA, the display says that the connection or certificate check failed: update over the web page then, which always works, and regenerate the list with `scripts/make_ca_bundle.py`.
  * The Pico W has no room for two firmware images: there the display says so; update over USB.

## Other

* **Light / dark design**: button at the top right of the settings (automatic → light → dark), the AI page follows.
* **Icons** for the browser tab and the iOS home screen come from the trickler itself, they work without internet (hotspot).
* **Session evaluation**: *Evaluate* in the Session block of the Trickler page (or *Evaluate session* on the AI page, Assistant) sends the statistics of the session per load and all its charges to the assistant.
