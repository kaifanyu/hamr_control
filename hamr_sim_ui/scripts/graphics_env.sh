#!/usr/bin/env bash
# Source before Qt/Gazebo. Probe actual OpenGL renderers, never infer GPU use
# from "direct rendering: Yes" (llvmpipe reports that too). No system changes.
# Driver references: docs.mesa3d.org/envvars.html and drivers/d3d12.html;
# NVIDIA's Linux README, "PRIME Render Offload".

_hamr_graphics_probe() {
  local output line lower
  _hamr_probe_renderer=""
  _hamr_probe_hardware=false
  if [[ "$_hamr_probe_api" == GLX ]]; then
    output="$(env "$@" timeout 5 glxinfo -B 2>/dev/null || true)"
  else
    # Restrict to the platform used for headless rendering; eglinfo's default
    # enumerates unrelated devices, which cannot establish the chosen renderer.
    output="$(env "$@" timeout 5 eglinfo -B -p "${EGL_PLATFORM:-surfaceless}" 2>/dev/null || true)"
  fi
  while IFS= read -r line; do
    case "$line" in
      *"OpenGL renderer string:"*|*"OpenGL core profile renderer:"*|*"OpenGL compatibility profile renderer:"*|*"OpenGL ES profile renderer:"*)
        _hamr_probe_renderer="${line#*:}"
        _hamr_probe_renderer="${_hamr_probe_renderer#"${_hamr_probe_renderer%%[![:space:]]*}"}"
        break ;;
    esac
  done <<< "$output"
  lower="${_hamr_probe_renderer,,}"
  case "$lower" in
    ""|*llvmpipe*|*softpipe*|*swrast*|*swiftshader*|*lavapipe*|*"software rasterizer"*|*"basic render"*) return 0 ;;
  esac
  if [[ "$output" == *"Accelerated: no"* ]]; then
    return 0
  fi
  case "$lower" in
    *nvidia*|*amd*|*radeon*|*intel*|*iris*|*d3d12*|*adreno*|*mali*|*apple*|*virgl*|*svga3d*) _hamr_probe_hardware=true ;;
    *) [[ "$output" != *"Accelerated: yes"* ]] || _hamr_probe_hardware=true ;;
  esac
  return 0
}

_hamr_graphics_report() {
  export HAMR_GRAPHICS_RENDERER="${_hamr_probe_renderer:-unverified (driver default)}"
  export HAMR_GRAPHICS_BACKEND="$_hamr_probe_api"
  if [[ "$_hamr_probe_hardware" == true ]]; then
    printf '[HAMR] GPU renderer (%s, %s): %s\n' "$_hamr_probe_api" "$1" "$HAMR_GRAPHICS_RENDERER"
  elif [[ -n "$_hamr_probe_renderer" ]]; then
    printf '[HAMR] CPU/software or unrecognized renderer (%s, %s): %s. Hardware acceleration was not verified.\n' \
      "$_hamr_probe_api" "$1" "$HAMR_GRAPHICS_RENDERER" >&2
  else
    printf '[HAMR] Renderer (%s, %s): unverified; keeping the driver default or explicit overrides.\n' \
      "$_hamr_probe_api" "$1" >&2
  fi
}

_hamr_graphics_select() {
  local variable explicit=false kernel default_renderer default_hardware adapter names providers
  local _hamr_probe_api=GLX _hamr_probe_renderer="" _hamr_probe_hardware=false
  # Respect choices made by users, launchers, PRIME wrappers, and containers.
  for variable in GALLIUM_DRIVER LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT \
      MESA_D3D12_DEFAULT_ADAPTER_NAME MESA_LOADER_DRIVER_OVERRIDE DRI_PRIME \
      __NV_PRIME_RENDER_OFFLOAD __NV_PRIME_RENDER_OFFLOAD_PROVIDER \
      __GLX_VENDOR_LIBRARY_NAME __EGL_VENDOR_LIBRARY_FILENAMES __EGL_VENDOR_LIBRARY_DIRS EGL_PLATFORM; do
    if [[ -n "${!variable:-}" ]]; then explicit=true; fi
  done
  if [[ -z "${DISPLAY:-}" ]]; then
    _hamr_probe_api=EGL
    if ! command -v eglinfo >/dev/null; then
      _hamr_graphics_report 'no DISPLAY / no EGL probe'
      printf '[HAMR] Headless GPU preflight needs eglinfo (Ubuntu: sudo apt install mesa-utils); Gazebo will select its renderer at runtime.\n' >&2
      return 0
    fi
  elif ! command -v glxinfo >/dev/null; then
    _hamr_graphics_report 'glxinfo unavailable'
    printf '[HAMR] Install the optional graphics probe (Ubuntu: sudo apt install mesa-utils) to verify GPU selection.\n' >&2
    return 0
  fi
  if ! command -v timeout >/dev/null; then
    _hamr_graphics_report 'timeout unavailable'
    printf '[HAMR] Install coreutils for a bounded graphics preflight; keeping renderer settings.\n' >&2
    return 0
  fi
  _hamr_graphics_probe
  if [[ "$explicit" == true ]]; then
    _hamr_graphics_report 'explicit environment override'
    return 0
  fi
  default_renderer="$_hamr_probe_renderer"
  default_hardware="$_hamr_probe_hardware"
  kernel="$(uname -r 2>/dev/null || true)"
  if [[ -e /dev/dxg && "${kernel,,}" == *microsoft* ]]; then
    # WSL's D3D12 desktop path requires DISPLAY. Never force it for headless
    # EGL: its device selection/support depends on the installed Mesa version.
    if [[ "$_hamr_probe_api" == GLX ]]; then
      adapter=""
      if [[ -x /usr/lib/wsl/lib/nvidia-smi ]]; then
        names="$(timeout 3 /usr/lib/wsl/lib/nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || true)"
        if [[ "$names" == *NVIDIA* ]]; then adapter=NVIDIA; fi
      fi
      _hamr_graphics_probe GALLIUM_DRIVER=d3d12 "MESA_D3D12_DEFAULT_ADAPTER_NAME=$adapter"
      if [[ "$_hamr_probe_hardware" != true && -n "$adapter" ]]; then
        adapter=""
        _hamr_graphics_probe GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=
      fi
      if [[ "$_hamr_probe_hardware" == true && "$_hamr_probe_renderer" == *D3D12* ]]; then
        export GALLIUM_DRIVER=d3d12
        if [[ -n "$adapter" ]]; then export MESA_D3D12_DEFAULT_ADAPTER_NAME="$adapter"; fi
        _hamr_graphics_report 'WSLg D3D12'
        return 0
      fi
    fi
  else
    # Only try proprietary NVIDIA offload when its driver/provider exists.
    # A failed candidate never leaks environment changes into the next probe.
    providers=""
    if [[ "$_hamr_probe_api" == GLX ]] && command -v xrandr >/dev/null; then
      providers="$(timeout 3 xrandr --listproviders 2>/dev/null || true)"
    fi
    if [[ -e /proc/driver/nvidia/version || "$providers" == *NVIDIA* ]]; then
      if [[ "$_hamr_probe_api" == GLX ]]; then
        _hamr_graphics_probe __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
      else
        _hamr_graphics_probe __NV_PRIME_RENDER_OFFLOAD=1
      fi
      if [[ "$_hamr_probe_hardware" == true && "${_hamr_probe_renderer,,}" == *nvidia* ]]; then
        export __NV_PRIME_RENDER_OFFLOAD=1
        if [[ "$_hamr_probe_api" == GLX ]]; then export __GLX_VENDOR_LIBRARY_NAME=nvidia; fi
        _hamr_graphics_report 'NVIDIA PRIME offload'
        return 0
      fi
    fi
    # Mesa chooses the first non-default GPU with DRI_PRIME=1. On a one-GPU
    # system it may return the same renderer, so keep the default in that case.
    _hamr_graphics_probe DRI_PRIME=1
    if [[ "$_hamr_probe_hardware" == true && "$_hamr_probe_renderer" != "$default_renderer" ]]; then
      export DRI_PRIME=1
      _hamr_graphics_report 'Mesa PRIME offload'
      return 0
    fi
  fi
  _hamr_probe_renderer="$default_renderer"
  _hamr_probe_hardware="$default_hardware"
  _hamr_graphics_report 'driver default / fallback'
  return 0
}

_hamr_graphics_select
unset -f _hamr_graphics_probe _hamr_graphics_report _hamr_graphics_select
