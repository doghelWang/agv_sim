#!/data/data/com.termux/files/usr/bin/sh
set -e

export PATH=/data/data/com.termux/files/usr/bin:$PATH
APP_DIR=/data/data/com.termux/files/home/CoverMonitorApp
ANDROID_JAR=/data/data/com.termux/files/usr/share/java/android.jar
FRAMEWORK_RES=/system/framework/framework-res.apk

cd $APP_DIR

echo "[1/6] Cleaning and preparing build directories..."
rm -rf gen obj bin
mkdir -p gen obj bin

echo "[2/6] Generating R.java via aapt..."
aapt package -m -J gen/ -M AndroidManifest.xml -S res/ -I $FRAMEWORK_RES

echo "[3/6] Compiling Java sources via ecj..."
ecj -d obj/ -cp $ANDROID_JAR src/com/cloudai/covermonitor/MainActivity.java gen/com/cloudai/covermonitor/R.java

echo "[4/6] Converting bytecode to classes.dex via d8..."
CLASS_FILES=$(find obj/ -name "*.class")
d8 --output bin/ --lib $ANDROID_JAR $CLASS_FILES

echo "[5/6] Packaging APK via aapt..."
aapt package -f -M AndroidManifest.xml -S res/ -I $FRAMEWORK_RES -F bin/unsigned.apk
cd bin
aapt add unsigned.apk classes.dex
cd ..

echo "[6/6] Signing APK with apksigner..."
if [ ! -f bin/debug.keystore ]; then
    keytool -genkeypair -validity 10000 -dname "CN=CloudAI,O=Monitor,C=CN" \
        -keystore bin/debug.keystore -storepass android -keypass android \
        -alias androiddebugkey -keyalg RSA -keysize 2048
fi

apksigner sign --ks bin/debug.keystore --ks-pass pass:android \
    --ks-key-alias androiddebugkey --key-pass pass:android \
    --out bin/CoverMonitor.apk bin/unsigned.apk

echo "SUCCESS! CoverMonitor.apk created at $APP_DIR/bin/CoverMonitor.apk"
ls -lh bin/CoverMonitor.apk
