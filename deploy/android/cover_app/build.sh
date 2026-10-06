#!/data/data/com.termux/files/usr/bin/bash
# 在 Termux 里编译外屏面板 APK (需要: pkg install aapt ecj dx apksigner openjdk-17；android.jar 随 aapt 包安装)
#   bash build.sh            → ./bin/agv-cover.apk
# 安装要用电脑上的 adb: adb install -r agv-cover.apk ; adb shell am start -n com.agvsim.cover/.MainActivity
set -e
cd "$(dirname "$0")"
JAR=$PREFIX/share/java/android.jar
rm -rf obj bin; mkdir -p obj bin
ecj -d obj -cp "$JAR" src/com/agvsim/cover/*.java
d8 --output bin --lib "$JAR" $(find obj -name '*.class')
aapt package -f -M AndroidManifest.xml -S res -I /system/framework/framework-res.apk -F bin/unsigned.apk
(cd bin && aapt add unsigned.apk classes.dex >/dev/null)
[ -f ~/.agv-cover.keystore ] || keytool -genkeypair -validity 10000 -dname "CN=agv-sim" -keystore ~/.agv-cover.keystore \
    -storepass android -keypass android -alias cover -keyalg RSA -keysize 2048 2>/dev/null
apksigner sign --ks ~/.agv-cover.keystore --ks-pass pass:android --ks-key-alias cover --key-pass pass:android \
    --out bin/agv-cover.apk bin/unsigned.apk
ls -la bin/agv-cover.apk
