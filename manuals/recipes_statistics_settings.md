# Recipes, Session Statistics and the Settings Pages

## Recipes

A recipe holds a load: a **name** (e.g. `.308 Match`), a **note** (bullet, primer, case …), the **charge weight** and the **powder profile**. Choosing a recipe selects the profile and sets the charge weight in one step, so a load is never charged with the wrong powder profile.

* **Manage:** *Settings* → **Recipes**: New Recipe, Edit, Up (order) and Delete. Up to 16 recipes, stored in the EEPROM. A new recipe starts with the charge weight from the Trickler page and the selected profile.
* **Web page:** on the Trickler page, pick the recipe in the list above the charge weight. The weight is filled in and the profile is selected; the line below shows the profile and the note. A recipe cannot be applied while a charge is running.
* **Display:** main menu → **Recipes**, pick the recipe (the line below shows weight, profile and note) and press **Next**. The trickler goes straight to "Put pan on the scale".
* **Backup:** *Settings* → **System** → Export/Import Config includes the recipes.

## Session statistics and CSV export

The trickler logs the last 150 charges (in RAM, they are cleared on a restart): charge weight, result, tolerance of the profile, time, top ups, controller and recipe. A charge topped up with **Tap** is logged with the weight after the taps.

The **Session** block on the Trickler page shows the charges since the session started:

* number of charges, share within the tolerance, over/under charges and the mean charge time
* per load (recipe, or charge weight and profile without a recipe): number, mean, standard deviation (SD), extreme spread (ES), share within the tolerance and mean time

**Export CSV** downloads all charges of the session (`;` separated with decimal commas, opens directly in a German Excel or LibreOffice). Columns: charge, time since start of the trickler, profile, recipe, target, result, error, tolerance, in tolerance, charge time, top ups, controller (AI/PID), type (charge, learning, probe, series, aborted).

**New Session** starts the statistics from zero (it offers to export the old session first). Without it, the session runs from the start of the trickler.

## Settings pages

The settings menu is grouped by what you do:

| Group | Page | Contents |
|---|---|---|
| Charging | **Charging** | Controller (AI Trickle or Classic PID), stop thresholds (Classic PID only), set point accuracy, decimal places; *After the Stop*: coarse backoff, fine tube reverse, tap angle; *Servo Gate Sequence*: pre-charge and gate position (only with the servo gate); *Safety*: stop when the pan is lifted, pause when the table is bumped |
| | **Recipes** | Loads: name, note, charge weight, profile |
| Powder | **Profiles** | Profile selection, name, speeds, Kp/Ki/Kd (Classic PID only), **tolerance**, what AI Trickle has learned for the profile |
| AI | **AI Trickle** | Auto speed, speed, fine reverse learned by Auto, limits in use, keep learning, throws per learning session |
| Hardware | **Scale** | Scale driver and diagnostics |
| | **Motors** | Coarse and fine trickler motor, reduced standstill current |
| | **Servo Gate** | Servo gate |
| | **Display & LEDs** | Mini 12864 display, Neopixel LEDs, charge colours, dimming, sound on the web page |
| System | **Wireless** | WiFi |
| | **System** | Info, save to EEPROM, reboot, erase, backup (export/import), restarts and errors, watchdog |
| | **Firmware Update** | Online update, update notice |

Each page has one **Apply** button for all its fields; pages with changes that are applied but not saved to the EEPROM are listed at the top with a **Save now** button. The light/dark design is switched at the top right. Safety, watchdog, dimming and the update notice: [safety_and_system.md](safety_and_system.md). The stored settings did not change with the new pages; old links like `/#settings-fine-motor` open the new page.
