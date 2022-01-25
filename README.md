[![gcc](https://github.com/ToyotaResearchInstitute/maliput_integration_tests/actions/workflows/build.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/maliput_integration_tests/actions/workflows/build.yml)

# Maliput Integration Tests

Integration tests for road network runtime interface and reference backends:
1. maliput_dragway
1. maliput_multilane

## Build

1. Setup a development workspace as described [here](https://github.com/ToyotaResearchInstitute/maliput_documentation/blob/main/docs/installation_quickstart.rst).

2. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

3. Build maliput packages and their dependencies:

  - If not building drake from source:

   ```sh
   colcon build --packages-up-to maliput_integration_tests
   ```

  - If building drake from source:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to maliput_integration_tests
   ```
