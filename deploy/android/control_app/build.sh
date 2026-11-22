#!/bin/bash
# Build the AMR 仿真控制 APK with Android build-tools 34 + platform android-34 (cloud / any Linux):
#   ASDK=/opt/asdk bash build.sh   → bin/agv-control.apk   (signed with a debug keystore in ~/.agv-control.keystore)
set -e
cd "$(dirname "$0")"
ASDK=${ASDK:-/opt/asdk}; BT=${BT:-$ASDK/android-15}; JAR=$ASDK/android-34/android.jar
rm -rf obj bin; mkdir -p obj bin
javac --release 8 -nowarn -g:none -d obj -cp "$JAR" src/com/agvsim/control/*.java 2>&1 | grep -v "^warning\|^Note\|bootstrap" || true
$BT/d8 --min-api 26 --output bin --lib "$JAR" $(find obj -name '*.class')
$BT/aapt package -f -M AndroidManifest.xml -S res -A assets -I "$JAR" -F bin/unsigned.apk
(cd bin && $BT/aapt add unsigned.apk classes.dex >/dev/null)
$BT/zipalign -f 4 bin/unsigned.apk bin/aligned.apk
KS=${KS:-$HOME/.agv-control.keystore}
[ -f "$KS" ] || keytool -genkeypair -validity 10000 -dname "CN=agv-sim" -keystore "$KS" -storepass android -keypass android -alias agv -keyalg RSA -keysize 2048 2>/dev/null
$BT/apksigner sign --ks "$KS" --ks-pass pass:android --ks-key-alias agv --key-pass pass:android --out bin/agv-control.apk bin/aligned.apk
ls -la bin/agv-control.apk
