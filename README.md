# Windows Crazyflie Setup

This guide explains how to set up a Windows machine to run a Crazyflie Python flight script using the Crazyradio dongle.

This setup is for **Windows PowerShell / Miniconda**, not WSL/Ubuntu.

## Important Notes

- `pip` installs the Python library: `cflib`.
- **libusbK** is the USB driver Windows needs for the Crazyradio dongle.
- **Zadig** is the tool used to install the `libusbK` driver onto the Crazyradio dongle.
- **Homebrew is not needed on Windows.**

## 1. Install Python Dependencies

Open PowerShell or Anaconda Prompt and run:

```powershell
pip install --upgrade pip
pip install --upgrade cflib pyusb libusb-package
```

## 2. Download libusbK

Download libusbK for Windows:

```text
https://sourceforge.net/projects/libusbk/
```

Install libusbK on your computer.

This gives Windows the USB driver package needed for Crazyradio access.

## 3. Install libusbK onto the Crazyradio Dongle Using Zadig

Plug in the **Crazyradio dongle**.

Download and open Zadig:

```text
https://zadig.akeo.ie/
```

Use Zadig to assign the **libusbK** driver to the Crazyradio dongle:

1. Click `Options`.
2. Enable `List All Devices`.
3. Select the Crazyradio device. It may appear as:
   - `Crazyradio PA USB Dongle`
   - `Crazyradio 2.0`
   - `Bitcraze Crazyradio`
4. In the driver selection box, choose:

```text
libusbK(v3.1.0.0) **versions may differ**
```

5. Click `Install Driver` or `Replace Driver`.
6. Wait for the driver installation to finish.
7. Unplug and replug the Crazyradio dongle.

After this step, Windows should allow Python/cflib to access the Crazyradio.

## 4. Verify Crazyradio Detection

Run this in PowerShell:

```powershell
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

If the driver is not working, the output may look like:

```text
Cannot find a Crazyradio Dongle
[]
```

If the driver is working, the output should include a radio interface, for example:

```text
[['radio://0/80/2M', '']]
```

## 5. Run the Test Flight

From PowerShell:

```powershell
cd C:\Users\pelek\Downloads\crazyflie2-client\crazyflie2-client-updated
python cf2flight.py
```

Before running:

- Put the Crazyflie on the floor.
- Make sure the props are clear.
- Make sure the battery is connected.
- Keep `Ctrl+C` ready to stop the script.
- Watch the terminal for connection and preflight messages.

The test script attempts a short low hover at about `0.25 m`, then stops, disconnects, and writes a timestamped `.json` flight log.

## Troubleshooting

### Error: `Cannot find a Crazyradio Dongle`

This usually means Windows/Python cannot access the Crazyradio USB dongle.

Try:

1. Unplug and replug the Crazyradio.
2. Reopen Zadig and confirm the selected device is the **Crazyradio dongle**, not the drone.
3. Confirm the installed driver is:

```text
libusbK
```

4. If needed, click `Replace Driver` in Zadig and reinstall `libusbK`.
5. Try a different USB port.
6. Avoid USB hubs.
7. Close any other Crazyflie client or program that might already be using the dongle.
8. Run the detection command again:

```powershell
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

### Output is `[]`

If the output is:

```text
[]
```

then no Crazyradio interface was detected.

The expected working output should look something like:

```text
[['radio://0/80/2M', '']]
```

## Sources

Bitcraze Windows USB driver docs:

```text
https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/
```

Bitcraze cflib installation docs:

```text
https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/
```
