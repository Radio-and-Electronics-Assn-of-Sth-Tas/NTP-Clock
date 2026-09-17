# NTP Clock (VK2ARH V2.1) — Firmware Change Summary

Board: Lolin S2 Mini (ESP32-S2FN4R2) — note the PCB schematic (rev 1.1) only shows footprints for a WEMOS D1 mini or ESP32-C3 Zero, so this board's wiring to the S2 Mini is custom/hand-wired and not reflected in the schematic.

---

## 1. Display driver (`tft_setup.h`)

**Problem:** Screen showed correct text/UI on the left portion, with the right portion showing static/noise.

**Root cause:** The display panel is actually an **ST7789**, not the ILI9341 the sketch was configured for.

**Fix applied (by you, in `tft_setup.h`):**
```cpp
//#define ILI9341_DRIVER
#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_BGR      // was TFT_BRG
#define TFT_INVERSION_OFF          // new
```
Pin assignments, HSPI port, and orientation were unchanged and confirmed correct via a GPIO identification test sketch (ruling out a pin-mapping issue on this un-labelled clone S2 Mini board).

---

## 2. WiFi Access Point auto-disable (`netInit()`)

**Problem:** The device's own WiFi Access Point stayed active permanently, even after successfully joining the home WiFi network.

**Fix:** Once `wifiConnect()` succeeds, the AP is now shut down and the radio drops into plain Station mode:
```cpp
if (wifiConnect())
{
   ...
   WiFi.softAPdisconnect(true);
   WiFi.mode(WIFI_STA);
}
```
If the STA connection later drops, `netTickTime()` / `netInit()` bring the AP back automatically, so reconfiguration is never permanently locked out.

---

## 3. Weather & solar data fetched over plain HTTP instead of HTTPS

**Problem:** Weather fetches (OpenWeatherMap) were intermittently failing with HTTP error `-1` (`HTTPC_ERROR_CONNECTION_REFUSED`), which reflects a TLS/connection-level failure on the ESP32's constrained mbedTLS stack, not a real server response — this was the underlying cause of the display periodically reverting to "(Wx updating)".

**Fix:** Both external data fetches now use plain `WiFiClient` + `http://` URLs instead of `WiFiClientSecure` + `https://`:
- `SOLAR_URL` → `http://www.hamqsl.com/solarxml.php`
- Weather `baseURL` → `http://api.openweathermap.org/data/2.5/weather`

This removes the TLS handshake and its associated memory/timing overhead from both fetches. As a follow-on, the now-unused `#include "Certificate.h"` was also removed (that file can be deleted from the project).

---

## 4. Preference save/load bugs

A full audit of every `prefs.put*`/`prefs.get*`/`prefs.isKey*` call turned up four real bugs, all in `forceDefaults()` (factory reset) and `readSettings()` (boot-time loader) — the web page's own save handler was correct throughout.

| # | Location | Bug | Fix |
|---|----------|-----|-----|
| 1 | `forceDefaults()` | Wrote the "MST only" default under the wrong key (`"showmst"` instead of `"showmstonly"`) | Corrected key name |
| 2 | `readSettings()` | Checked `isKey("showestonly")` — a key that's never written — instead of `"showmstonly"`, so the real "MST only" setting was silently reset to default on **every boot** | Corrected key name |
| 3 | `readSettings()` | When initializing `"showgmt"` for the first time, saved the wrong variable (`showESTEDT` instead of `showGMTBST`) | Corrected variable |
| 4 | `readSettings()` | Typo'd key when creating the humidity offset default (`"hunoffset"` instead of `"humoffset"`) | Corrected key name |

Bug #2 was the most user-visible — the "Show MST (only)" checkbox could never actually persist across a reboot.

---

## 5. Hardcoded personal WiFi credentials removed

`forceDefaults()` had several real WiFi SSIDs/passwords, a login username, and what looked like a home address (used as an SSID) hardcoded as the "factory defaults." These have all been replaced with empty strings, so a factory reset now clears network credentials entirely rather than reinstating personal ones.

---

## 6. Altitude display now respects the metric/imperial toggle

**Problem:** Switching the display to metric converted every other unit except altitude, which stayed in feet.

**Fix:** `showAltitude()` now checks `useMetric` (as every other unit-aware display function already did) and converts feet → metres when set, updating both the label (`"Alt (ft):"` / `"Alt (m):"`) and the value.

---

## 7. NTP server is now configurable

**Problem:** The NTP server was hardcoded to `pool.ntp.org` in two places — ezTime's implicit default, and a `configTime()` call originally added for HTTPS certificate validation (no longer needed after change #3).

**Fix:** New `ntpServer` setting, defaulting to `pool.ntp.org`:
- New **"NTP Server"** field on the `/network` web page (under a new "NTP Time Sync" heading).
- Saved to flash, reloaded on boot, reset to default on factory reset.
- Takes effect **immediately** on save (calls `setServer()` and `configTime()` directly) — no reboot required.

---

## 8. Force WiFi AP mode via button (new feature)

**Problem:** No way to force the config Access Point back on while already connected to a home network — you had to wait for a genuine disconnect.

**Fix:** An **extra-long press on the right button** (previously used to clear the stored web login password) now forces `WiFi.mode(WIFI_AP_STA)` + `wifiInitAP()` instead.

**Usage note:** This still falls through into the existing "reboot the clock?" prompt afterward. Answer **No** to that prompt to stay in forced AP mode with the WiFi info screen shown; answering **Yes** reboots the device, which will reconnect to STA and immediately disable the AP again per change #2. Let me know if you'd rather this skip the reboot prompt entirely.

---

## 9. Timezone table — issues

- **`ACST-9:30ACDT,...,M4.1.0/2:00:00`** and **`AEST-10AEDT,...,M4.1.0/2:00:00`** — the end-of-DST time should be `3:00:00`, not `2:00:00` (POSIX TZ end-transition times are expressed in daylight-time terms, and Australia's clocks fall back from 3am ACDT/AEDT to 2am ACST/AEST).
- **`GMT1BST,...`** — should be `GMT0BST,...`. `GMT1` incorrectly means UTC−1 year-round; GMT is UTC+0.
