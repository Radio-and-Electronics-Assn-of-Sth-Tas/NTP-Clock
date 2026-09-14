Title:         NTP Dual Clock
Author:        (original) Bruce E. Hall, w8bh.net (Original Version)
               (current) RAHinsley (VK2ARH), MJCulross (KD5RXT)
Date:          (original) 13 Feb 2021
               (current) 25 Jun 2026
Hardware:      ESP32-S2-mini, with ILI9341 TFT display
Software:      Arduino IDE (currently 1.8.19) with additional board
               packages & libraries (details below)
Legal:         Copyright (c) 2021 Bruce E. Hall.
               Open Source under the terms of the MIT License.

Description:   Dual UTC/Local NTP Clock with TFT display. Time is refreshed via
               NTP (both) and/or GPS. Status indicator for time freshness &
               WiFi strength.

Rev History:   11/27/20   Initial GitHub commit
               11/28/20   Added code to handle dropped WiFi connection
               11/30/20   showTimeDate() mod by John Price (WA2FZW)
               12/01/20   showAMPM() added by John Price (WA2FZW)
               02/05/21   Added support for ESP8266 modules
               02/07/21   Added day-above-month option
               02/10/21   Added date-leading-zero option
               04/13/25   Added solar weather data to display - rkincaid/CalQRP
                          Club. Thank you N0NBH and hamqsl.com for providing
                          this service.

               05/06/25   Modified by AI6P & WA2FZW to enable full secure connection
                          to the 'hamqsl' server and made the code work on both
                          the ESP32 and ESP8266 processors.

               05/11/25   Complete overhaul by WA2FZW and AI6P. Added the ability
                          to show a user defined sequence of items from the solar
                          data XML file in the UTC time header block.

                          Eliminated some unused functions and moved some code
                          around that was not in the best places.

                          Allow the user to define more than one WiFi network
                          to try connecting to.

               11/01/25   Version 3.1 added the capability to display a repeating
                          sequence of local timezones.

               11/25/25   Renamed to MWRS v 2.5 - Modified by VK2ARH to merge all
                          functionality developed for Manly Warringah Radio
                          Society's (MWRS) NTP Clock project with AI6P & WA2FZW's
                          code.

                          This included use of WiFi Manager, Display Dimming
                          (additonal LDR, resistor and push button switch needed),
                          and night mode operation developed by KD5RXT.

                          See: https://github.com/VK2ARH/NTP-Clock/tree/main for
                          details and files for new boards to support the MWRS
                          functionality.

                          Changes to start sequence and status panels to indicate
                          progress with joining WiFi network.

                          Restructured some of the code to include greater resiliance
                          when WiFi signal lost WiFi status indicator to show WiFi
                          signal lost.

                          This firmware can be used without the additional hardware;
                          you just lose the Dimming and night mode capability. All
                          other features are available.

                          Modified the size of the AM/PM indicator on the clock.

                          Use is made of EEPROM to remember all clock settings
                          Dimming Level, Night Mode etc.) so that the clock reverts
                          to these settings when rebooted.

               12/12/25   Renamed to MWRS V2.5.33 and incorporated use of Button2
                          library to handle button processing. Tidied up minor
                          areas of code.

               12/19/25   Renamed to MWRS V2.5.34 incorporates local temp, humidity
                          and pressure (calculated as QNH - Sea Level Pressure)
                          using BME280.

               01/10/26   Renamed to MWRS V2.5.4 by WA2FZW. a lot of general
                          cleanup and handling of some error conditions that
                          weren't being handled previously.

                          Changed the WiFi and brightness button designations
                          to allow the user to force a reset of the WiFi
                          credentials on startup 'D4' could not be used
                          for that.

               01/14/26   Changed BME280 initialization to load the temperature
                          offset into the device. The BME280 uses that in
                          computing the other readings, so simply doing the
                          compensation in the code here doesn't guarantee that
                          the pressure and humidity readings are accurate.

               01/16/26   Added the variable 'showMode' to indicate whether
                          or not the solar data items should be shown ot not.
                          The option can be flip-flopped by a double click
                          of the WiFi button. When 'false', only the weather
                          data will be displayed if 'BME280_INSTALLED' and
                          'SHOW_DATA_ENV' are both 'true'. The value of 'showMode'
                          is saved in the EEPROM whenever it changes.

               04/13/26   Revamped many aspects, always working towards a single set
                          of source that can be built/targeted for either the ESP8266
                          (Wemos D1) or the ESP32-S2-mini.  This is now accomplished
                          just by setting one "#define" line in the tft_setup.h file.
                          Note that some of the other contents (e.g. specifying the
                          TFT pins in use) in the tft_setup.h file are not actually
                          used for the ESP8266, but that the file must still be present
                          in order to provide/specify the target processor board.
                          If/when the ESP32-S2-mini is substituted for the ESP8266,
                          additional functionality (e.g. internet weather & WiFi
                          scanning) is automatically added into the build using the
                          "#define" processor selection.

               05/11/26   Added voltage monitoring & display, almost all settings can
                          now be controlled via the AP hosted webpage, added optional
                          DEBUG capability to the AP hosted webpage, removed the
                          ESP32-C3-zero as a processor target, made the WiFi scan
                          a separate webpage & made it (re)scan automatically.  Laid
                          the groundwork for eventually adding GPS capability.

               06/01/26   Made GPS startup user-initiated (either by button press, or
                          from the AP hosted webpage).  Reworked the items displayed
                          in the top section to allow a full 8-character title & avoid
                          having either side of it appear to be cut-off.  Made the two
                          supply voltage measurements each a running average of 16
                          values (reducing thrashing/blinking display).  Added a
                          button functionality definition screen, which is accessed
                          using the combination of both buttons (PRESS, PRESS,
                          RELEASE, RELEASE)...this screen is also displayed at
                          startup.  Changed the button functionality definitions to
                          implement Richard's (VK2ARH) excellent suggestions.  GPS
                          controls (enable/disable & factory reset) are now initiated
                          by a long-click on SW2 (BRIGHT).  Added two additional
                          Australian time zone definitions (AWST & ACST).  Changed
                          the supply voltage measurement correction factor to be a
                          multiplier.  Automatically handle when the RX & TX pins are
                          swapped on the GPS.  Allow data mode display selections to
                          be made from the AP hosted webpage, in addition to being
                          made from a button press.  Allow the GPS fix state to be
                          determined either from the fix state reported by the GPS
                          (only works for some GPS models) or from the number of
                          satellites in use.  Allow the Title/Banner to be updated
                          from the AP hosted webpage.
               07/12/26   Added GPS data display, changed from "Restart WiFi" to
                          "Reboot" with long-press on SW1 (WIFI/CONFIG), added a
                          thread to manage the button timing & detection, which
                          made button operations more reliable & responsive, enable
                          DEBUG mode with an extra-long press (more than 8 seconds)
                          of the BRIGHT button, clear any previously set CONFIG
                          page password with an extra-long press (more than 8
                          seconds) of the WIFI/CONFIG button.
               07/25/26   Added support for the ESP32-S3-mini processor module (which
                          can successfully be used in place of the ESP32-S2-mini, and
                          includes 200k more RAM).  Using the ESP32-S3-mini allows all
                          capabilities (solar, environmental, weather, MLS, & GPS) to
                          be used simultaneously, with no restrictions. Fixed GPS-only
                          to show all data (lat/lon/alt).
	       14/09/26   First version of REAST firmware. Caters for different screen driver.
