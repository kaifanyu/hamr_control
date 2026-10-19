"""Exercise the real shell launcher against bounded, fake graphics tools."""
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/graphics_env.sh"
OVERRIDES = ("GALLIUM_DRIVER", "LIBGL_ALWAYS_SOFTWARE", "LIBGL_ALWAYS_INDIRECT",
             "MESA_D3D12_DEFAULT_ADAPTER_NAME", "MESA_LOADER_DRIVER_OVERRIDE", "DRI_PRIME",
             "__NV_PRIME_RENDER_OFFLOAD", "__NV_PRIME_RENDER_OFFLOAD_PROVIDER",
             "__GLX_VENDOR_LIBRARY_NAME", "__EGL_VENDOR_LIBRARY_FILENAMES",
             "__EGL_VENDOR_LIBRARY_DIRS", "EGL_PLATFORM")
SOFTWARE = "llvmpipe (LLVM 20.1.2, 256 bits)"


@pytest.fixture
def graphics_probe(tmp_path):
    binaries = tmp_path / "bin"
    binaries.mkdir()
    for command in ("env", "timeout"):
        (binaries / command).symlink_to(Path("/usr/bin") / command)
    probe = '''#!/bin/bash
printf '%s|%s|%s|%s|%s\\n' "${0##*/}" "$*" "${DRI_PRIME:-}" "${__NV_PRIME_RENDER_OFFLOAD:-}" "${GALLIUM_DRIVER:-}" >> "$FAKE_CALLS"
renderer="${FAKE_DEFAULT:-}"
if [[ "${DRI_PRIME:-}" == 1 ]]; then renderer="${FAKE_PRIME:-}"; fi
if [[ "${__NV_PRIME_RENDER_OFFLOAD:-}" == 1 ]]; then renderer="${FAKE_NVIDIA:-}"; fi
if [[ "${GALLIUM_DRIVER:-}" == d3d12 ]]; then renderer="${FAKE_D3D12:-}"; fi
[[ -n "$renderer" ]] || exit 1
# Software reports direct rendering too; it must never be accepted as a GPU.
printf 'direct rendering: Yes\\n'
if [[ "${0##*/}" == eglinfo ]]; then
  printf 'OpenGL core profile renderer: %s\\n' "$renderer"
else
  printf 'OpenGL renderer string: %s\\n' "$renderer"
fi
'''
    for command, body in {"glxinfo": probe, "eglinfo": probe,
                          "uname": '#!/bin/bash\nprintf "%s\\n" "${FAKE_KERNEL:-6.8.0-generic}"\n',
                          "xrandr": '#!/bin/bash\nprintf "%s\\n" "${FAKE_PROVIDERS:-modesetting}"\n'}.items():
        path = binaries / command
        path.write_text(body)
        path.chmod(0o755)

    def run(*, missing=(), **values):
        calls = tmp_path / "calls"
        calls.write_text("")
        environment = {key: value for key, value in os.environ.items() if key not in OVERRIDES}
        environment.update(PATH=str(binaries), DISPLAY=":mock", FAKE_CALLS=str(calls))
        environment.update(values)
        for command in missing:
            (binaries / command).unlink()
        result = subprocess.run(["/bin/bash", "-eu", "-c", '''source "$1"
printf 'RESULT|%s|%s|%s|%s|%s|%s\\n' "${HAMR_GRAPHICS_RENDERER:-}" "${GALLIUM_DRIVER:-}" "${DRI_PRIME:-}" "${__NV_PRIME_RENDER_OFFLOAD:-}" "${__GLX_VENDOR_LIBRARY_NAME:-}" "${MESA_D3D12_DEFAULT_ADAPTER_NAME:-}"
''', "graphics-test", str(SCRIPT)], env=environment, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
        assert result.returncode == 0, result.stdout
        return result.stdout, calls.read_text()
    return run


def test_software_direct_rendering_is_not_a_gpu(graphics_probe):
    output, calls = graphics_probe(FAKE_DEFAULT=SOFTWARE, FAKE_PRIME=SOFTWARE)
    assert "CPU/software" in output
    assert "GPU renderer" not in output
    assert "RESULT|" + SOFTWARE + "|||||" in output
    assert "|-B|1||" in calls


@pytest.mark.parametrize("renderer", ["Mesa Intel(R) UHD Graphics 770", "AMD Radeon RX 7600", "NVIDIA RTX 4060/PCIe/SSE2"])
def test_native_default_gpu_is_preserved_when_offload_fails(graphics_probe, renderer):
    output, _ = graphics_probe(FAKE_DEFAULT=renderer)
    assert "GPU renderer (GLX, driver default / fallback): " + renderer in output
    assert "RESULT|" + renderer + "|||||" in output


def test_mesa_hybrid_selects_working_second_gpu(graphics_probe):
    output, calls = graphics_probe(FAKE_DEFAULT="Mesa Intel(R) Graphics", FAKE_PRIME="AMD Radeon RX 7600")
    assert "Mesa PRIME offload" in output
    assert "RESULT|AMD Radeon RX 7600||1|||" in output
    assert "|-B|1||" in calls


def test_nvidia_offload_needs_provider_and_verified_renderer(graphics_probe):
    output, calls = graphics_probe(FAKE_DEFAULT="Mesa Intel(R) Graphics", FAKE_PROVIDERS="name:NVIDIA-G0",
                                  FAKE_NVIDIA="NVIDIA RTX 4060/PCIe/SSE2")
    assert "NVIDIA PRIME offload" in output
    assert "RESULT|NVIDIA RTX 4060/PCIe/SSE2|||1|nvidia|" in output
    assert "|-B||1|" in calls


def test_failed_nvidia_candidate_does_not_poison_fallback(graphics_probe):
    output, _ = graphics_probe(FAKE_DEFAULT="Mesa Intel(R) Graphics", FAKE_PROVIDERS="name:NVIDIA-G0",
                              FAKE_NVIDIA=SOFTWARE, FAKE_PRIME="AMD Radeon RX 7600")
    assert "Mesa PRIME offload" in output
    assert "RESULT|AMD Radeon RX 7600||1|||" in output


@pytest.mark.parametrize("override,value", [("LIBGL_ALWAYS_SOFTWARE", "true"), ("GALLIUM_DRIVER", "llvmpipe"),
                                           ("MESA_D3D12_DEFAULT_ADAPTER_NAME", "Intel"), ("DRI_PRIME", "0"),
                                           ("__NV_PRIME_RENDER_OFFLOAD", "0"), ("__GLX_VENDOR_LIBRARY_NAME", "mesa")])
def test_explicit_choices_disable_auto_selection(graphics_probe, override, value):
    output, calls = graphics_probe(FAKE_DEFAULT=SOFTWARE, FAKE_PRIME="AMD Radeon RX 7600", **{override: value})
    assert "explicit environment override" in output
    assert "GPU renderer" not in output
    assert len(calls.splitlines()) == 1


def test_missing_glxinfo_reports_unverified_and_install_instruction(graphics_probe):
    output, calls = graphics_probe(missing=("glxinfo",))
    assert "unverified" in output and "sudo apt install mesa-utils" in output
    assert calls == ""


def test_headless_probes_egl_without_glx(graphics_probe):
    output, calls = graphics_probe(DISPLAY="", FAKE_DEFAULT="AMD Radeon RX 7600")
    assert "GPU renderer (EGL" in output
    assert "glxinfo" not in calls
    assert "eglinfo|-B -p surfaceless" in calls
    assert "d3d12" not in calls


def test_headless_without_eglinfo_keeps_driver_default(graphics_probe):
    output, calls = graphics_probe(DISPLAY="", missing=("eglinfo",))
    assert "no DISPLAY / no EGL probe" in output
    assert "unverified" in output
    assert "GPU renderer" not in output
    assert calls == ""


def test_unknown_renderer_is_not_claimed_as_accelerated(graphics_probe):
    output, _ = graphics_probe(FAKE_DEFAULT="Unknown GPU renderer")
    assert "Hardware acceleration was not verified" in output
    assert "GPU renderer (" not in output
