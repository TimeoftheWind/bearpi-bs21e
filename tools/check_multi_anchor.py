"""Host checks using real SDK headers; does not produce flashable firmware."""
import pathlib
import subprocess
import tempfile
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SAMPLE = ROOT / "application/samples/products/sle_measure_dis"

def run(args):
    result = subprocess.run(args, cwd=ROOT)
    if result.returncode:
        raise SystemExit(result.returncode)

headers = subprocess.check_output(["rg", "--files", "-g", "*.h"], cwd=ROOT, text=True).splitlines()
dirs = sorted({str(pathlib.Path(n).parent) for n in headers
               if "/musl/" not in n and "/libc/" not in n})
flags = ["clang", "-std=gnu99", "-D__LITEOS__", "-DCHIP_BS21E", "-DSLEM_CARKEY",
         "-Werror=incompatible-pointer-types", "-Werror=implicit-function-declaration"]
for d in dirs:
    flags += ["-idirafter", d]
files = [SAMPLE / "sle_measure_dis_client/sle_measure_dis_client.c",
         SAMPLE / "sle_measure_dis_client/sle_measure_dis_client_slem.c",
         SAMPLE / "imu/imu_client.c", SAMPLE / "imu/imu_sensor.c",
         SAMPLE / "sle_measure_dis_server/sle_measure_dis_server.c",
         SAMPLE / "sle_measure_dis_server/sle_measure_dis_server_alg.c"]
run(flags + ["-fsyntax-only"] + [str(p) for p in files])
for role in ("CLIENT", "SERVER"):
    run(flags + ["-fsyntax-only", "-DCONFIG_SAMPLE_SUPPORT_SLE_MEASURE_DIS_" + role,
                 "-include", str(ROOT / "tests/host_app_init.h"),
                 str(SAMPLE / "sle_measure_dis.c")])
run(flags + ["-fsyntax-only", "-DCONFIG_SAMPLE_SUPPORT_SLE_MEASURE_DIS_SERVER", "-DMEASURE_SERVER_SUFFIX=6",
             "-include", str(ROOT / "tests/host_app_init.h"), str(SAMPLE / "sle_measure_dis.c")])
with tempfile.TemporaryDirectory(prefix="multi-anchor-check-") as temp:
    for name in ("client", "server"):
        source = ROOT / ("tests/multi_anchor_" + name + "_test.c")
        if not source.exists():
            continue
        exe = str(pathlib.Path(temp) / name)
        # Hardware-only functions are stripped; the invoked paths have explicit mocks.
        linker = ["-Wl,-dead_strip"] if sys.platform == "darwin" else ["-Wl,--gc-sections"]
        run(flags + ["-O1", "-ffunction-sections", "-fdata-sections"] + linker +
            [str(source), "-o", exe])
        run([exe])
        if name == "server":
            run(flags + ["-DMEASURE_SERVER_SUFFIX=6", "-O1", "-ffunction-sections", "-fdata-sections"] +
                linker + [str(source), "-o", exe])
            run([exe])
print("SDK-header syntax and host behavior checks passed; target build/hardware still required.")
