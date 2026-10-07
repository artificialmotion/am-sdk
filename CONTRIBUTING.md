# Contributing

Thanks for looking at am-sdk.

- **Issues are welcome now.** Bugs, unclear docs, API friction, missing features: open an issue.
- **Outside pull requests are closed until 1.0.** Until then the API moves with the hardware, and we keep the tree in step with it ourselves. Pull requests from people outside Artificial Motion are closed without review. Issues are welcome: if you have a fix, open an issue with the patch or a description and we fold it in.
- **Security problems:** see [SECURITY.md](SECURITY.md), do not open an issue.

Before you open an issue with a patch, run the tests:

```sh
cmake -S c -B c/build -DAM_SDK_WERROR=ON && cmake --build c/build && ctest --test-dir c/build
cd python && python -m pytest
```

Rules for code in this repository:
- C stays C99, warning-free with `-Wall -Wextra -pedantic`, and compiles as C++.
- The C and Python simulators stay one model. A change to one goes into the other, and the cross-check test proves it.
- Unit data lives in `python/am_sdk/data`. After a change, run `python tools/gen_unit_data.py`.

Contributions we take in are licensed under Apache-2.0 (inbound = outbound). There is no CLA.
