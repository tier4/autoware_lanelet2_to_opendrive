# `pack-scenario-image`

Build a container image for a scenario package generated with `scenario-new`,
with the CARLA client version fixed at build time.

This directory is self-contained — `action.yml`, the `Dockerfile`, the
build-context assembler and the smoke test all live here — so the action is
meant to be referenced from other repositories:

```yaml
- uses: actions/checkout@v4

- uses: tier4/autoware_lanelet2_to_opendrive/.github/actions/pack-scenario-image@main
  with:
    scenario-package-path: my_scenario_package
    image: ghcr.io/my-org/my-scenario
    carla-version: "0.10.0"
    push: "true"
```

Your repository needs to contain only your scenario package: the framework
comes from the ref you reference the action by. Push to a registry with
`docker/login-action` first; this action does not handle credentials.

The virtualenv ships as four layers -- the CARLA client, the framework's
dependency closure, the framework wheels, the scenario -- ordered so that
rebuilding a scenario against an unchanged client and framework leaves the
first three untouched, and pulling the new image transfers only the last.

Inputs, outputs, sizing, and how to build the same image by hand are documented
in [`autoware_carla_scenario/docs/docker.md`](../../../autoware_carla_scenario/docs/docker.md).
