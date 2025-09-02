#!/bin/bash
set -e

# === Runtime Parameters ===
SIM_TIME=${SIM_TIME:-3600}
ALTITUDE_THRESHOLD=${ALTITUDE_THRESHOLD:-10.0}
TEAM_ID=${TEAM_ID:-default}

echo "=== ADCS Fault Simulation Startup ==="
echo "Team ID: $TEAM_ID"
echo "Simulation Duration: ${SIM_TIME}s"
echo "Altitude Threshold: ${ALTITUDE_THRESHOLD}km"
echo ""

# === Inject duration into 42 config ===
sed -i "s/DURATION_PLACEHOLDER/${SIM_TIME}.0/" /home/osk/42/InOut/Sim.txt

# === Start 42 Simulator ===
echo "[42] Launching headless spacecraft simulation..."
cd /home/osk/42
nohup ./42 InOut > /tmp/42.log 2>&1 &
sleep 2

# === Start cFS Core ===
echo "[cFS] Booting core flight system..."
cd /home/osk/cFS/build/exe/cpu1
nohup ./core-cpu1 > /tmp/cfs.log 2>&1 &
# sleep 3

# === Start ADCS Telemetry Bridge ===
echo "[Bridge] Starting telemetry bridge..."
cd /home/osk
python3 telemetry_bridge.py

echo ""
echo "=== Simulation Complete ==="
echo "Logs:"
echo " - 42: /tmp/42.log"
echo " - cFS: /tmp/cfs.log"
echo " - Telemetry: /tmp/telemetry.log"

