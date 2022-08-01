[![GCC](https://github.com/maliput/maliput_integration_tests/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput_integration_tests/actions/workflows/build.yml)

# maliput_integration_tests

## Description

This package contains integration tests for [`Maliput`](https://github.com/maliput/maliput), while relying on reference backends and utility packages:
1. [maliput_dragway](https://github.com/maliput/maliput_dragway)
1. [maliput_multilane](https://github.com/maliput/maliput_multilane)
1. [maliput_malidrive](https://github.com/maliput/maliput_malidrive)
1. [maliput_object](https://github.com/maliput/maliput_object)


**Note**: For full information about Maliput please visit [Maliput Documentation](https://maliput.readthedocs.io/en/latest/index.html).

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput_integration_tests.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_integration_tests
    ```

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)

#### Tests

```sh
colcon test --packages-select maliput_integration_tests
```

### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput_integration_tests/blob/main/LICENSE)
