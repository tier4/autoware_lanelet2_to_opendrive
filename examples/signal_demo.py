#!/usr/bin/env python3
"""
Demo script showing how to use signals with OpenDRIVE roads.

This script demonstrates:
1. Creating traffic light signals
2. Adding signals to roads
3. Generating OpenDRIVE XML output with signals
"""

import lxml.etree as ET
from autoware_lanelet2_to_opendrive.opendrive import (
    Road,
    Signals,
    Signal,
    SignalType,
    Validity,
    create_traffic_light,
    PlanView,
    Line,
)


def main():
    """Demonstrate signal creation and XML output."""
    print("=" * 80)
    print("OpenDRIVE Signal Demo")
    print("=" * 80)

    # Create a simple road
    road = Road(
        id=1,
        name="Main Street",
        length=200.0,
        junction=-1,
    )

    # Create plan view with a simple line geometry
    line = Line()
    line.s = 0.0
    line.x = 0.0
    line.y = 0.0
    line.hdg = 0.0
    line.length = 200.0
    plan_view = PlanView(geometries=[line])
    road.plan_view = plan_view

    # Create signals container
    signals = Signals()

    # Example 1: Create a traffic light using the convenience function
    print("\n1. Creating traffic light using convenience function...")
    traffic_light_1 = create_traffic_light(
        signal_id=100,
        name="TrafficLight_Intersection_1",
        s=50.0,  # Position at 50m along the road
        t=-4.5,  # 4.5m to the left of the road reference line
        orientation="-",
        z_offset=0.0,
        height=1.2,
        width=0.6,
        traffic_light_type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        from_lane=-1,  # Apply to right lane
        to_lane=-1,
    )
    signals.add_signal(traffic_light_1)
    print(f"   Added: {traffic_light_1}")

    # Example 2: Create a traffic light manually with more control
    print("\n2. Creating traffic light manually with custom attributes...")
    traffic_light_2 = Signal(
        id=101,
        name="TrafficLight_Intersection_2",
        s=150.0,  # Position at 150m along the road
        t=-4.5,
        z_offset=0.0,
        h_offset=0.0,
        roll=0.0,
        pitch=0.0,
        orientation="-",
        dynamic="yes",  # Traffic lights are dynamic (change state during simulation)
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
        value=-1.0,
        text="",
        height=1.2,
        width=0.6,
        validities=[
            Validity(from_lane=-2, to_lane=-1)  # Apply to both right lanes
        ],
    )
    signals.add_signal(traffic_light_2)
    print(f"   Added: {traffic_light_2}")

    # Example 3: Create a pedestrian traffic light
    print("\n3. Creating pedestrian traffic light...")
    pedestrian_light = Signal(
        id=102,
        name="Pedestrian_Light_Crosswalk_1",
        s=75.0,
        t=-6.0,  # Further to the side for pedestrian crossing
        z_offset=0.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_PEDESTRIAN,
        subtype=-1,
        height=0.8,
        width=0.4,
        validities=[Validity(from_lane=0, to_lane=0)],  # Center lane
    )
    signals.add_signal(pedestrian_light)
    print(f"   Added: {pedestrian_light}")

    # Attach signals to the road
    road.signals = signals

    print(f"\n4. Total signals added to road: {len(signals)}")

    # Generate XML output
    print("\n5. Generating OpenDRIVE XML...")
    road_xml = road.to_xml()
    xml_string = ET.tostring(road_xml, encoding="unicode", pretty_print=True)

    print("\nGenerated OpenDRIVE XML (Road with Signals):")
    print("-" * 80)
    print(xml_string)
    print("-" * 80)

    # Extract and display just the signals section
    signals_xml = road_xml.find("signals")
    if signals_xml is not None:
        signals_string = ET.tostring(signals_xml, encoding="unicode", pretty_print=True)
        print("\nSignals Section Detail:")
        print("-" * 80)
        print(signals_string)
        print("-" * 80)

    print("\nDemo completed successfully!")
    print("\nYou can now use Signal, Signals, and create_traffic_light to add")
    print("traffic signals to your OpenDRIVE roads.")


if __name__ == "__main__":
    main()
