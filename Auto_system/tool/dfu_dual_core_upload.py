import os

Import("env")


DFU_VID_PID = "2341:0366"
UPLOAD_LAYOUT = {
    "giga_r1_m7": ("0x08040000", False),
    "giga_r1_m4": ("0x08100000", True),
}


def dfu_util_path():
    platform = env.PioPlatform()
    package_dir = platform.get_package_dir("tool-dfuutil-arduino")
    executable = "dfu-util.exe" if os.name == "nt" else "dfu-util"
    return os.path.join(package_dir, executable)


board = env.subst("$BOARD")
address, leave_after_upload = UPLOAD_LAYOUT[board]
leave_suffix = ":leave" if leave_after_upload else ""

board_config = env.BoardConfig()
board_config.update("upload.disable_flushing", True)
board_config.update("upload.use_1200bps_touch", False)
board_config.update("upload.wait_for_upload_port", False)
board_config.update("upload.require_upload_port", False)

env.Replace(
    UPLOADER=dfu_util_path(),
    UPLOADERFLAGS=[
        "-d",
        DFU_VID_PID,
        "-a",
        "0",
        "-s",
        f"{address}{leave_suffix}",
        "-D",
    ],
    UPLOADCMD='"$UPLOADER" $UPLOADERFLAGS "${SOURCE.get_abspath()}"',
)
