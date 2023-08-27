cargo build --release
cargo objcopy --release -- -O ihex app.hex
nrfutil pkg generate --debug-mode --application app.hex app.zip
