import genesis as gs
import torch

def test_drone_debug():
    """Test drone entity properties for both URDF and MJCF versions"""
    gs.init(backend=gs.cpu)

    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))

    # Test URDF version
    print("=== Testing URDF Version ===")
    plane = scene.add_entity(morph=gs.morphs.Plane())
    drone_urdf = scene.add_entity(morph=gs.morphs.Drone(file="urdf/drones/cf2x.urdf", pos=(0, 0, 0.2)))

    # Test MJCF version
    print("\n=== Testing MJCF Version ===")
    drone_mjcf = scene.add_entity(morph=gs.morphs.DroneMJCF(file="xml/cf2.xml", pos=(0, 0, 0.2)))

    scene.build()

    print("URDF Drone properties:")
    print(f"  base_link_idx: {drone_urdf.base_link_idx}")
    print(f"  n_links: {drone_urdf.n_links}")
    print(f"  _link_start: {drone_urdf._link_start}")
    print(f"  _base_links_idx: {drone_urdf._base_links_idx}")

    print("\nMJCF Drone properties:")
    print(f"  base_link_idx: {drone_mjcf.base_link_idx}")
    print(f"  n_links: {drone_mjcf.n_links}")
    print(f"  _link_start: {drone_mjcf._link_start}")
    print(f"  _base_links_idx: {drone_mjcf._base_links_idx}")

    # Test get_pos
    print("\nTesting get_pos:")
    print(f"URDF position: {drone_urdf.get_pos().cpu().numpy()}")
    print(f"MJCF position: {drone_mjcf.get_pos().cpu().numpy()}")

    # Test other methods
    print("\nTesting other methods:")
    try:
        print(f"URDF quat: {drone_urdf.get_quat().cpu().numpy()}")
        print(f"MJCF quat: {drone_mjcf.get_quat().cpu().numpy()}")
    except Exception as e:
        print(f"Error getting quat: {e}")

    try:
        print(f"URDF vel: {drone_urdf.get_vel().cpu().numpy()}")
        print(f"MJCF vel: {drone_mjcf.get_vel().cpu().numpy()}")
    except Exception as e:
        print(f"Error getting vel: {e}")

    # Test propeller links
    print("\n=== Testing Propeller Links ===")
    try:
        propellers_link_name = ["prop0_link", "prop1_link", "prop2_link", "prop3_link"]
        print("URDF propeller links:")
        for name in propellers_link_name:
            try:
                link = drone_urdf.get_link(name)
                print(f"  {name}: idx={link.idx}, pos={link.get_pos().cpu().numpy()}")
            except Exception as e:
                print(f"  {name}: Error - {e}")

        print("MJCF propeller links:")
        for name in propellers_link_name:
            try:
                link = drone_mjcf.get_link(name)
                print(f"  {name}: idx={link.idx}, pos={link.get_pos().cpu().numpy()}")
            except Exception as e:
                print(f"  {name}: Error - {e}")
    except Exception as e:
        print(f"Error testing propeller links: {e}")

    # Test drone properties
    print("\n=== Testing Drone Properties ===")
    try:
        print(f"URDF n_propellers: {drone_urdf.n_propellers}")
        print(f"URDF _propellers_link_idxs: {drone_urdf._propellers_link_idxs}")
        print(f"URDF KF: {drone_urdf.KF}, KM: {drone_urdf.KM}")
        print(f"URDF propellers_spin: {drone_urdf.propellers_spin}")
    except Exception as e:
        print(f"Error getting URDF drone properties: {e}")

    try:
        print(f"MJCF n_propellers: {drone_mjcf.n_propellers}")
        print(f"MJCF _propellers_link_idxs: {drone_mjcf._propellers_link_idxs}")
        print(f"MJCF KF: {drone_mjcf.KF}, KM: {drone_mjcf.KM}")
        print(f"MJCF propellers_spin: {drone_mjcf.propellers_spin}")
    except Exception as e:
        print(f"Error getting MJCF drone properties: {e}")

    # Test physics simulation
    print("\n=== Testing Physics Simulation ===")

    # Apply force
    drone_urdf.set_propellels_rpm([1000, 1000, 1000, 1000])
    drone_mjcf.set_propellels_rpm([1000, 1000, 1000, 1000])

    print("Initial positions:")
    print(f"URDF position: {drone_urdf.get_pos().cpu().numpy()}")
    print(f"MJCF position: {drone_mjcf.get_pos().cpu().numpy()}")

    # Step simulation
    for i in range(5):
        scene.step()
        print(f"Step {i+1}:")
        print(f"  URDF position: {drone_urdf.get_pos().cpu().numpy()}")
        print(f"  MJCF position: {drone_mjcf.get_pos().cpu().numpy()}")
        print(f"  URDF vel: {drone_urdf.get_vel().cpu().numpy()}")
        print(f"  MJCF vel: {drone_mjcf.get_vel().cpu().numpy()}")

    # Test RPM retrieval
    print("\n=== Testing RPM Retrieval ===")
    urdf_rpm = drone_urdf.get_propellels_rpm().cpu().numpy()
    mjcf_rpm = drone_mjcf.get_propellels_rpm().cpu().numpy()
    print(f"URDF RPM: {urdf_rpm}")
    print(f"MJCF RPM: {mjcf_rpm}")

    # Test individual propeller positions after simulation
    print("\n=== Testing Individual Propeller Positions ===")
    propellers_link_name = ["prop0_link", "prop1_link", "prop2_link", "prop3_link"]
    print("URDF propeller positions after simulation:")
    for name in propellers_link_name:
        try:
            link = drone_urdf.get_link(name)
            print(f"  {name}: pos={link.get_pos().cpu().numpy()}")
        except Exception as e:
            print(f"  {name}: Error - {e}")

    print("MJCF propeller positions after simulation:")
    for name in propellers_link_name:
        try:
            link = drone_mjcf.get_link(name)
            print(f"  {name}: pos={link.get_pos().cpu().numpy()}")
        except Exception as e:
            print(f"  {name}: Error - {e}")

    # Test base link position
    print("\n=== Testing Base Link Positions ===")
    try:
        base_link_urdf = drone_urdf.get_link("base_link")
        print(f"URDF base_link pos: {base_link_urdf.get_pos().cpu().numpy()}")
        print(f"URDF base_link vel: {base_link_urdf.get_vel().cpu().numpy()}")
    except Exception as e:
        print(f"URDF base_link error: {e}")

    # Test correct MJCF base link
    try:
        base_link_mjcf = drone_mjcf.get_link("cf2")
        print(f"MJCF base_link 'cf2' pos: {base_link_mjcf.get_pos().cpu().numpy()}")
        print(f"MJCF base_link 'cf2' vel: {base_link_mjcf.get_vel().cpu().numpy()}")
        print(f"MJCF base_link 'cf2' idx: {base_link_mjcf.idx}")
    except Exception as e:
        print(f"MJCF base_link 'cf2' error: {e}")

    # Compare with DroneEntity.get_pos()
    print("\n=== Comparing get_pos() with direct link access ===")
    print(f"URDF get_pos(): {drone_urdf.get_pos().cpu().numpy()}")
    print(f"URDF base_link pos: {drone_urdf.get_link('base_link').get_pos().cpu().numpy()}")

    print(f"MJCF get_pos(): {drone_mjcf.get_pos().cpu().numpy()}")
    print(f"MJCF base_link 'cf2' pos: {drone_mjcf.get_link('cf2').get_pos().cpu().numpy()}")

    # Check if base_links_idx matches the actual base link
    print("\n=== Checking base_links_idx ===")
    print(f"URDF base_links_idx: {drone_urdf._base_links_idx}")
    print(f"URDF base_link idx: {drone_urdf.get_link('base_link').idx}")
    print(f"URDF match: {drone_urdf._base_links_idx[0].item() == drone_urdf.get_link('base_link').idx}")

    print(f"MJCF base_links_idx: {drone_mjcf._base_links_idx}")
    print(f"MJCF base_link 'cf2' idx: {drone_mjcf.get_link('cf2').idx}")
    print(f"MJCF match: {drone_mjcf._base_links_idx[0].item() == drone_mjcf.get_link('cf2').idx}")

    # Test all links for MJCF
    print("\n=== Testing All MJCF Links ===")
    try:
        for i in range(drone_mjcf.n_links):
            try:
                link = drone_mjcf.get_link(i)
                print(f"MJCF link {i} ({link.name}): pos={link.get_pos().cpu().numpy()}, vel={link.get_vel().cpu().numpy()}")
            except Exception as e:
                print(f"MJCF link {i}: Error - {e}")
    except Exception as e:
        print(f"Error getting MJCF links: {e}")

    # Test by name instead of index
    print("\n=== Testing MJCF Links by Name ===")
    try:
        # Try to find the correct base link name
        for i in range(drone_mjcf.n_links):
            try:
                link = drone_mjcf.get_link(i)
                print(f"MJCF link {i}: name='{link.name}', idx={link.idx}")
            except Exception as e:
                print(f"MJCF link {i}: Error - {e}")

        # Try to find the correct base link by name
        base_link_names = ["cf2", "base_link", "body"]
        for name in base_link_names:
            try:
                link = drone_mjcf.get_link(name)
                print(f"MJCF base link '{name}': pos={link.get_pos().cpu().numpy()}, vel={link.get_vel().cpu().numpy()}")
                break
            except Exception as e:
                print(f"MJCF base link '{name}': Error - {e}")
    except Exception as e:
        print(f"Error testing MJCF links by name: {e}")

def test_mjcf_parsing():
    """Test MJCF parsing to understand body order"""
    import genesis.utils.mjcf as mju

    # Create a simple morph for testing
    class SimpleMorph:
        def __init__(self):
            self.file = "xml/cf2.xml"
            self.pos = (0, 0, 0.2)
            self.quat = None
            self.scale = 1.0
            self.visualization = True
            self.collision = True
            self.requires_jac_and_IK = True
            self.default_armature = 0.1

    morph = SimpleMorph()

    # Parse MJCF
    print("=== Testing MJCF Parsing ===")
    try:
        from genesis.surfaces import Default
        surface = Default()
        l_infos, links_j_infos, links_g_infos, eqs_info = mju.parse_xml(morph, surface)

        print(f"Number of bodies: {len(l_infos)}")
        for i, l_info in enumerate(l_infos):
            print(f"Body {i}: name={l_info.get('name', 'unknown')}, parent_idx={l_info['parent_idx']}, pos={l_info.get('pos', 'unknown')}")

    except Exception as e:
        print(f"Error parsing MJCF: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_mjcf_parsing()
    test_drone_debug()
