# `pack-scenario-image`

Build a container image for a scenario package generated with `scenario-new`,
with the CARLA client version fixed at build time.

This directory is self-contained — `action.yml`, the `Dockerfile`, the
build-context assembler and the smoke test all live here — so the action is
meant to be referenced from other repositories:

```yaml
- uses: actions/checkout@v4

- uses: tier4/autoware_lanelet2_to_opendrive/.github/actions/pack-scenario-image@v2.62.0
  with:
    scenario-package-path: my_scenario_package
    image: ghcr.io/my-org/my-scenario
    carla-version: "0.10.0"
    push: "true"
```

The ref you pin is both the action version **and** the framework version that
ends up in the image: the runner checks the whole repository out next to the
action, and `autoware-carla-scenario` / `autoware-lanelet2-to-opendrive` are
built from that copy. Your repository needs to contain only your scenario
package.

Push to a registry with `docker/login-action` first; this action does not
handle credentials.

Inputs, outputs, sizing, and how to build the same image by hand are documented
in [`autoware_carla_scenario/docs/docker.md`](../../../autoware_carla_scenario/docs/docker.md).
