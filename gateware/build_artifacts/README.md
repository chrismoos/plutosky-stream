# Build artifacts

Files committed so that consumers do not need to run a build.

- `csr.json`, `csr.csv`: the register map exported by
  `soc.targets.plutosky_stream` for the shipped build (two channels, one
  receive stream, one transmit stream). The daemon resolves register names from
  the JSON. Both files must be regenerated whenever the CSR layout changes, and
  `SOC_VERSION` must change with it.
- `vivado-2r2t/`: the Vivado build that is the shipped bitstream. It contains
  the SD boot image, the `fpga_manager` bitstream, a matching `csr.json` and the
  routed reports. `vivado-2r2t/README.md` lists its files and the fabric
  clocks it requires.
