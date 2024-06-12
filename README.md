# Arcane Node

This repository contains an implementation of an ARCANE node.

## Dependencies

#### 1. `flip-link`:

```console
$ cargo install flip-link
```

#### 2. `probe-rs`:

```console
$ cargo install probe-rs --features cli
```

### Run

```console
$ DEFMT_LOG=trace cargo rb main
```

If you're running out of memory (`flip-link` bails with an overflow error), you can decrease the size of the device memory buffer by setting the `DEFMT_BRTT_BUFFER_SIZE` environment variable. The default value is 1024 bytes, and powers of two should be used for optimal performance:

```console
$ DEFMT_BRTT_BUFFER_SIZE=64 cargo rb minimal
```
