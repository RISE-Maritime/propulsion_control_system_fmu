# propulsion-control-system-fmu

This FMU implements a very simplistic propulsion control system for ships.

## For development

This is an example setup of using rust for implementing a model that is also cross-compiled and packaged as a multi-architecture FMU compatible with FMI 3.0.

Use the devcontainer. Use cargo as you would in any other project, i.e. `cargo build`, `cargo test`.

## For generating a release

Run `build_fmu.sh`, it will:
* Install `cross` is not already installed.
* Compile the project for:
  * x86_64-unknown-linux-gnu
  * x86_64-pc-windows-gnu
* Package the resulting shred libraries and the `modelDescription.xml` into a temporary folder structure adhearing to FMI 3.0.
* Zip the temporary folder structure into an FMU.

