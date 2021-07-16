# `stm32f303` playground

[`probe-run`] + [`defmt`] + [`flip-link`] + [`rtic`] Rust embedded playground

[`probe-run`]: https://crates.io/crates/probe-run
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link
[`rtic`]: https://github.com/rtic-rs/cortex-m-rtic

## Dependencies

#### 1. `flip-link`:

```console
$ cargo install flip-link
```

#### 2. `probe-run`:

```console
$ cargo install probe-run
```

## Run!

Start by `cargo run`-ning `src/bin/serial.rs`:

```console
$ # `rb` is an alias for `run --bin`
$ cargo rb serial
  Finished dev [optimized + debuginfo] target(s) in 0.3s
  Running `probe-run --chip STM32F303VCTx target/thumbv7em-none-eabihf/debug/serial`
  (HOST) INFO  flashing program (13.39 KiB)
  (HOST) INFO  success!
───────────────────────────────────────────────────────────────────────────────
```
