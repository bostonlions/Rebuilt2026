# Elastic Dashboard – Trimmer Autonomous Selector

The robot publishes the autonomous routine selection to NetworkTables under **Trimmer/Autonomous** for Elastic Dashboard.

## Adding the widget

1. Open Elastic Dashboard and connect to the robot (or run in simulation).
2. Click **Add Widget**.
3. In the **NetworkTables** tab, browse to **Trimmer** → **Autonomous**.
4. Add a **ComboBox Chooser** or **Split Button Chooser** widget for the autonomous selector.
5. The widget will show the current routine and allow selecting a different one.

## NetworkTables layout

- `Trimmer/Autonomous/selected` – current routine (read/write)
- `Trimmer/Autonomous/options` – list of routines
- `Trimmer/Autonomous/type` – `"String Chooser"` (for widget compatibility)

## Syncing with D-pad

The D-pad on the operator controller (POV 0/180 for up/down when on the Autonomous subsystem) and the Elastic Dashboard widget both control the same selection. Changes in either place are reflected in both.
