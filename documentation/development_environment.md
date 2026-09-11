# Development environment

For development I use VSCode with `clangd` rather than `intelliSenseEngine` because it reads `compile_commands.json` directly — correct ARM toolchain headers, exact build flags, no manual config.

Make sure clangd is installed on your system:

```bash
sudo apt-get install -y clangd && clangd --version
```

# Read SWO debug text stream

SWO is not used by the project (USART is more reliable).
Here is how to use it for reference.

Characters written to `ITM_SendChar` show up in a view named
`SWV ITM Data Console`, but its configuration button does not work, so the view
falls short.

```
st-trace --clock=16m
```

Warning: This requires a patch of st-trace(0001-fix-st-trace-fix-SWO-trace-on-STLINK-V3-HS-bulk-endp.patch) otherwise it will fail with `2026-06-08T23:00:36 ERROR usb.c: read_trace insufficient buffer length`

