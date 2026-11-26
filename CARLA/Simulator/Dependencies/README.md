# WSL graphics dependency

The Windows bootstrap pins the ASURT Mesa Dozen kit to [release `v0.1.0`](https://github.com/hassanradwannn/Mesa-Dozen-Custom-WSL-driver-for-Carla/releases/tag/v0.1.0). Its `asurt-mesa-dozen-wsl-team-kit-20261024.tar.gz` asset has SHA-256 `f729e89f8e069b4e717a3b38d5ce89311789a38b81e450d4b6014e08b3204200`.

The installer places the driver under `~/.local/opt/asurt-dozen-20261024` in Ubuntu 22.04 on WSL2. The WSLg CARLA launcher selects the `descriptor-reset-candidate` variant through the kit's per-process wrapper. See the [driver repository](https://github.com/hassanradwannn/Mesa-Dozen-Custom-WSL-driver-for-Carla) for dependencies, source, licenses, and rollback.
