# tf2agw experimental prototype

Experimental Linux PTY DED/TF-ish to AGWPE/Dire Wolf bridge.

This is a clean prototype inspired by LU7DID's tf2agw design notes. It is **not** a port of Pedro's recovered Delphi DLL source.

## Build

```sh
make
```

## Run

```sh
./tf2agw -s /tmp/tf2agw -a 127.0.0.1 -p 8000 -c N0CALL -v
```

Then point LinFBB's TF/WA8DED device at `/tmp/tf2agw`.

## Current status

Protocol spike only. Implemented enough plumbing to test:

- create PTY and symlink `/tmp/tf2agw`
- connect to AGW/Dire Wolf TCP port
- send AGW version/port/register/connect/disconnect/data frames
- maintain a small hostmode-channel to AGW-session table
- return channel-prefixed text/data back to the PTY

Likely still needs adjustment against real LinFBB TF-driver byte behavior and Dire Wolf AGW command semantics.
