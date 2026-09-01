# Third-Party Notices

This repository's own deployment & integration code is licensed under the [Apache License 2.0](LICENSE). However, it bundles third-party components that are distributed under their own licenses — **some of which are NON-COMMERCIAL (CC BY-NC-SA 4.0)** and prohibit commercial use.

## 1. FAST-LIO-Localization-SC-QN (localization & mapping stack)

- **Directory**: `src/fast-lio-mapping_and_localization-sc-qn/`
- **Upstream author**: Lee Eungchang (KAIST), GitHub [`engcang`](https://github.com/engcang)
- **Upstream projects**: [FAST-LIO-Localization-SC-QN](https://github.com/engcang/FAST-LIO-Localization-SC-QN), [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN)
- **License**: **CC BY-NC-SA 4.0** — Attribution-NonCommercial-ShareAlike 4.0 — **NON-COMMERCIAL ONLY**
- **Copyright**: EungChang-Mason-Lee; includes components © KAIST, Naver Labs

This directory compiles/links the following sub-components into its executables:

| Component | Directory | Upstream | License |
|---|---|---|---|
| FAST_LIO (LIO odometry) | `third_party/FAST_LIO` | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) | GPL-2.0 |
| Quatro (global registration) | `third_party/Quatro` | [engcang/quatro](https://github.com/engcang/quatro) | conflicting: `package.xml` = GPL-3.0, `README.md` = CC BY-NC-SA 4.0 |
| ScanContext / scancontext_tro (loop candidate) | `third_party/scancontext`, `third_party/scancontext_tro` | [gisbi-kim/scancontext_tro](https://github.com/gisbi-kim/scancontext_tro) | CC BY-NC-SA 4.0 |
| nano_gicp (ICP) | `third_party/nano_gicp` | [engcang/nano_gicp](https://github.com/engcang/nano_gicp) | MIT |

## 2. Other bundled third-party packages

| Component | Directory | Upstream | License |
|---|---|---|---|
| sentry_navigation | `src/sentry_navigation` | [IRobot-Algorithm/sentry_navigation](https://github.com/IRobot-Algorithm/sentry_navigation) | MIT |
| livox_ros_driver2 | `src/livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | MIT |

## Important: commercial-use warning

The **CC BY-NC-SA 4.0** license prohibits commercial use. If you intend to use `tron1-ss` — or any part that bundles the localization & mapping stack above — in a commercial product or service, you MUST either:

1. obtain a commercial license from the original authors, or
2. replace the non-commercial components with commercially-licensed alternatives.

The Apache-2.0 license in this repository applies **only** to LimX Dynamics' own deployment & integration code, **not** to the bundled third-party components listed above.
