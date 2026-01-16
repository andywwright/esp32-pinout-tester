import os

from SCons.Script import Import

Import("env")

method = os.environ.get("TEST_METHOD", "").strip().lower()
if method == "morse":
    env.Append(CPPDEFINES=["MODE_MORSE"])

if os.environ.get("TEST_MODE", "").strip() in ("1", "true", "yes", "on"):
    env.Append(CPPDEFINES=["TEST_MODE"])
