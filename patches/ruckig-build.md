# ruckig on macOS (i2rt / YAM bench only)

ruckig 0.15.3's sdist fails to build on macOS because its `pyproject.toml`
uses the key `cmake.targets`; rename it to `build.targets` and build a wheel:

    pip download --no-binary :all: ruckig==0.15.3
    tar xzf ruckig-0.15.3.tar.gz && cd ruckig-0.15.3
    sed -i '' 's/cmake.targets/build.targets/' pyproject.toml
    pip wheel . -w ../wheels && pip install ../wheels/ruckig-*.whl

`fastgripper-dm` does not depend on ruckig; only the i2rt adapter path does.
