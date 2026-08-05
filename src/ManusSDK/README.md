# MANUS SDK runtime files

This directory is the only vendor SDK boundary used by `src/manus_ros2`.

- `include/` contains the MANUS SDK headers required at build time.
- `lib/libManusSDK_Integrated.so` is local-only and can be restored with
  `scripts/fetch_manus_sdk.sh`.
- `License` and `NOTICE.txt` contain the vendor and third-party notices.

Do not copy MANUS example clients, ImGui sources, generated calibration files,
or build outputs into this directory.
