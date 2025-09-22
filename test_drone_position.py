import genesis as gs
import torch
import numpy as np

def test_drone_position():
    """Test drone position tracking for both URDF and MJCF versions"""
    gs.init(backend=gs.cpu)  # Use CPU for debugging

    scene = gs.Scene(
        show_viewer=False,
        sim_options=gs.options.SimOptions(dt=0.01)
    )

    # Test URDF version
    print("=== Testing URDF Version ===")
    plane = scene.add_entity(morph=gs.morphs.Plane())
    drone_urdf = scene.add_entity(morph=gs.morphs.Drone(file="urdf/drones/cf2x.urdf", pos=(0, 0, 0.2)))

    # Test MJCF version
    print("\n=== Testing MJCF Version ===")
    drone_mjcf = scene.add_entity(morph=gs.morphs.DroneMJCF(file="xml/cf2.xml", pos=(0, 0, 0.2)))

    scene.build()

    print("URDF Drone initial position:", drone_urdf.get_pos().cpu().numpy())
    print("MJCF Drone initial position:", drone_mjcf.get_pos().cpu().numpy())

    # Step simulation
    scene.step()

    print("\nAfter first step:")
    print("URDF Drone position:", drone_urdf.get_pos().cpu().numpy())
    print("MJCF Drone position:", drone_mjcf.get_pos().cpu().numpy())

    # Apply some force to move the drones
    print("\n=== Applying force to move drones ===")

    # For URDF drone
    drone_urdf.set_propellels_rpm([1000, 1000, 1000, 1000])

    # For MJCF drone (should work the same way)
    drone_mjcf.set_propellels_rpm([1000, 1000, 1000, 1000])

    # Step simulation multiple times
    for i in range(10):
        scene.step()
        print(f"Step {i+1}:")
        print(f"  URDF position: {drone_urdf.get_pos().cpu().numpy()}")
        print(f"  MJCF position: {drone_mjcf.get_pos().cpu().numpy()}")

    # Test the hover function
    print("\n=== Testing hover function ===")
    hover(drone_urdf)
    hover(drone_mjcf)

    for i in range(20):
        scene.step()
        if i % 5 == 0:
            print(f"Step {i+11}:")
            print(f"  URDF position: {drone_urdf.get_pos().cpu().numpy()}")
            print(f"  MJCF position: {drone_mjcf.get_pos().cpu().numpy()}")

if __name__ == "__main__":
    test_drone_position()
