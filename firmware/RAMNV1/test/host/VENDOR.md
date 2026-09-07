# Vendored files

## `ramn_test_vectors.h`

Generated, not written. Do not hand-edit it — a refresh overwrites it.

| | |
|---|---|
| Source repo | https://github.com/ramn-telematics/ramn-protocol |
| Pinned revision | `58695de7ba887733405f68e9f20a4036b4258082` |
| Path in source | `c_conformance/ramn_test_vectors.h` |

### Refreshing

```sh
cd ../ramn-protocol && make generate && make test
cp c_conformance/ramn_test_vectors.h <this directory>/
```

Update the pinned revision above in the same commit. ramn-protocol's
`conformance` workflow diffs this file against a freshly generated one, so a
stale copy fails a run there even when nobody touches this repository.

The ESP32 repository has this as a shared component with a `refresh.sh` that
does the whole job in one run. This copy is deliberately plainer — one test
app, one include path — but the same rule applies: the header and the pinned
revision are committed together, or the pin is a lie.

### What it is for

The vectors were produced by an implementation written from the spec,
independently of this firmware. `test_conformance.c` asserts that
`ProcessESP32Response` agrees with them. That is a different question from
whether it agrees with a fixture written next to it in the same file: a
shared misreading of the wire format survives the second and dies against the
first.
