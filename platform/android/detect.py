import os
import platform
import subprocess
import sys
from typing import TYPE_CHECKING

from methods import print_error, print_warning
from platform_methods import validate_arch

if TYPE_CHECKING:
    from SCons.Script.SConscript import SConsEnvironment


def get_name():
    return "Android"


def can_build():
    return os.path.exists(get_env_android_sdk_root())


def get_tools(env: "SConsEnvironment"):
    return ["clang", "clang++", "as", "ar", "link"]


def get_opts():
    from SCons.Variables import BoolVariable

    return [
        ("ANDROID_HOME", "Path to the Android SDK", get_env_android_sdk_root()),
        (
            "ndk_platform",
            'Target platform (android-<api>, e.g. "android-' + str(get_min_target_api()) + '")',
            "android-" + str(get_min_target_api()),
        ),
        BoolVariable("store_release", "Editor build for Google Play Store (for official builds only)", False),
        BoolVariable(
            ("generate_android_binaries", "generate_apk"),
            "Generate APK, AAB & AAR binaries after building Android library by calling Gradle",
            False,
        ),
        BoolVariable("swappy", "Use Swappy Frame Pacing library", False),
    ]


def get_doc_classes():
    return [
        "EditorExportPlatformAndroid",
    ]


def get_doc_path():
    return "doc_classes"


# Return the ANDROID_HOME environment variable.
def get_env_android_sdk_root():
    return os.environ.get("ANDROID_HOME", os.environ.get("ANDROID_SDK_ROOT", ""))


def get_min_sdk_version(platform):
    return int(platform.split("-")[1])


def get_android_ndk_root(env: "SConsEnvironment"):
    return os.path.join(env["ANDROID_HOME"], "ndk", get_ndk_version())


# This is kept in sync with the value in 'platform/android/java/app/config.gradle'.
def get_ndk_version():
    return "28.1.13356709"


# This is kept in sync with the value in 'platform/android/java/app/config.gradle'.
def get_min_target_api():
    return 24


def get_flags():
    return {
        "arch": "arm64",
        "target": "template_debug",
        "supported": ["mono"],
    }


# Check if Android NDK version is installed
# If not, install it.
def install_ndk_if_needed(env: "SConsEnvironment"):
    sdk_root = env["ANDROID_HOME"]
    if not os.path.exists(get_android_ndk_root(env)):
        extension = ".bat" if os.name == "nt" else ""
        sdkmanager = os.path.join(sdk_root, "cmdline-tools", "latest", "bin", "sdkmanager" + extension)
        if os.path.exists(sdkmanager):
            # Install the Android NDK
            print("Installing Android NDK...")
            ndk_download_args = "ndk;" + get_ndk_version()
            subprocess.check_call([sdkmanager, ndk_download_args])
        else:
            print_error(
                f'Cannot find "{sdkmanager}". Please ensure ANDROID_HOME is correct and cmdline-tools'
                f' are installed, or install NDK version "{get_ndk_version()}" manually.'
            )
            sys.exit(255)
    env["ANDROID_NDK_ROOT"] = get_android_ndk_root(env)


def detect_swappy():
    archs = ["arm64-v8a", "armeabi-v7a", "x86", "x86_64"]
    has_swappy = True
    for arch in archs:
        if not os.path.isfile(f"thirdparty/swappy-frame-pacing/{arch}/libswappy_static.a"):
            has_swappy = False
    return has_swappy


def configure(env: "SConsEnvironment"):
    # Validate arch.
    supported_arches = ["x86_32", "x86_64", "arm32", "arm64"]
    validate_arch(env["arch"], get_name(), supported_arches)

    if get_min_sdk_version(env["ndk_platform"]) < get_min_target_api():
        print_warning(
            "Minimum supported Android target api is %d. Forcing target api %d."
            % (get_min_target_api(), get_min_target_api())
        )
        env["ndk_platform"] = "android-" + str(get_min_target_api())

    install_ndk_if_needed(env)
    ndk_root = env["ANDROID_NDK_ROOT"]

    # Architecture

    if env["arch"] == "arm32":
        target_triple = "armv7a-linux-androideabi"
    elif env["arch"] == "arm64":
        target_triple = "aarch64-linux-android"
    elif env["arch"] == "x86_32":
        target_triple = "i686-linux-android"
    elif env["arch"] == "x86_64":
        target_triple = "x86_64-linux-android"

    target_option = ["-target", target_triple + str(get_min_sdk_version(env["ndk_platform"]))]
    env.Append(ASFLAGS=[target_option, "-c"])
    env.Append(CCFLAGS=target_option)
    env.Append(LINKFLAGS=target_option)

    # LTO

    if env["lto"] == "auto":  # LTO benefits for Android (size, performance) haven't been clearly established yet.
        env["lto"] = "none"

    if env["lto"] != "none":
        if env["lto"] == "thin":
            env.Append(CCFLAGS=["-flto=thin"])
            env.Append(LINKFLAGS=["-flto=thin"])
        else:
            env.Append(CCFLAGS=["-flto"])
            env.Append(LINKFLAGS=["-flto"])

    # Compiler configuration

    env["SHLIBSUFFIX"] = ".so"

    if env["PLATFORM"] == "win32":
        env.use_windows_spawn_fix()

    if sys.platform.startswith("linux"):
        host_subpath = "linux-x86_64"
    elif sys.platform.startswith("darwin"):
        host_subpath = "darwin-x86_64"
    elif sys.platform.startswith("win"):
        if platform.machine().endswith("64"):
            host_subpath = "windows-x86_64"
        else:
            host_subpath = "windows"

    toolchain_path = os.path.join(ndk_root, "toolchains", "llvm", "prebuilt", host_subpath)
    compiler_path = os.path.join(toolchain_path, "bin")

    env["CC"] = os.path.join(compiler_path, "clang")
    env["CXX"] = os.path.join(compiler_path, "clang++")
    env["AR"] = os.path.join(compiler_path, "llvm-ar")
    env["RANLIB"] = os.path.join(compiler_path, "llvm-ranlib")
    env["AS"] = os.path.join(compiler_path, "clang")

    # Use TempFileMunge since some AR invocations are too long otherwise
    env["ARCOM_ORIG"] = env["ARCOM"]
    env["ARCOM"] = "${TEMPFILE('$ARCOM_ORIG', '$ARCOMSTR')}"

    env.Append(
        CCFLAGS=(["-fpic", "-ffunction-sections", "-funwind-tables", "-fstack-protector-strong", "-fvisibility=hidden"])
    )

    has_swappy = detect_swappy()
    if not has_swappy:
        print_warning(
            "Swappy Frame Pacing not detected! It is strongly recommended you download it from https://github.com/godotengine/godot-swappy/releases and extract it so that the following files can be found:\n"
            + " thirdparty/swappy-frame-pacing/arm64-v8a/libswappy_static.a\n"
            + " thirdparty/swappy-frame-pacing/armeabi-v7a/libswappy_static.a\n"
            + " thirdparty/swappy-frame-pacing/x86/libswappy_static.a\n"
            + " thirdparty/swappy-frame-pacing/x86_64/libswappy_static.a\n"
            + "Without Swappy, Godot apps on Android will inevitable suffer stutter and struggle to keep consistent 30/60/90/120 fps. Though Swappy cannot guarantee your app will be stutter-free, not having Swappy will guarantee there will be stutter even on the best phones and the most simple of scenes."
        )
        if env["swappy"]:
            print_error("Use build option `swappy=no` to ignore missing Swappy dependency and build without it.")
            sys.exit(255)

    if get_min_sdk_version(env["ndk_platform"]) >= 24:
        env.Append(CPPDEFINES=[("_FILE_OFFSET_BITS", 64)])

    if env["arch"] == "x86_32":
        if has_swappy:
            env.Append(LIBPATH=["#thirdparty/swappy-frame-pacing/x86"])
    elif env["arch"] == "x86_64":
        if has_swappy:
            env.Append(LIBPATH=["#thirdparty/swappy-frame-pacing/x86_64"])
    elif env["arch"] == "arm32":
        env.Append(CCFLAGS=["-march=armv7-a", "-mfloat-abi=softfp"])
        env.Append(CPPDEFINES=["__ARM_ARCH_7__", "__ARM_ARCH_7A__"])
        env.Append(CPPDEFINES=["__ARM_NEON__"])
        if has_swappy:
            env.Append(LIBPATH=["#thirdparty/swappy-frame-pacing/armeabi-v7a"])
    elif env["arch"] == "arm64":
        env.Append(CCFLAGS=["-mfix-cortex-a53-835769"])
        env.Append(CPPDEFINES=["__ARM_ARCH_8A__"])
        if has_swappy:
            env.Append(LIBPATH=["#thirdparty/swappy-frame-pacing/arm64-v8a"])

    env.Append(CCFLAGS=["-ffp-contract=off"])

    # Link flags

    env.Append(LINKFLAGS=["-Wl,--gc-sections", "-Wl,--no-undefined", "-Wl,-z,now"])
    env.Append(LINKFLAGS=["-Wl,-soname,libgodot_android.so"])

    env.Prepend(CPPPATH=["#platform/android"])
    env.Append(CPPDEFINES=["ANDROID_ENABLED", "UNIX_ENABLED"])
    env.Append(LIBS=["OpenSLES", "EGL", "android", "log", "z", "dl"])

    if env["vulkan"]:
        env.Append(CPPDEFINES=["VULKAN_ENABLED", "RD_ENABLED"])
        if has_swappy:
            env.Append(CPPDEFINES=["SWAPPY_FRAME_PACING_ENABLED"])
            env.Append(LIBS=["swappy_static"])
        if not env["use_volk"]:
            env.Append(LIBS=["vulkan"])

    if env["opengl3"]:
        env.Append(CPPDEFINES=["GLES3_ENABLED"])
        env.Append(LIBS=["GLESv3"])
