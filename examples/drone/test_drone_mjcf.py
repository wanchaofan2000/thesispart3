#!/usr/bin/env python3
"""
Test script to verify DroneMJCF functionality
"""

import genesis as gs

def test_drone_mjcf_creation():
    """Test that DroneMJCF can be created and used with DroneEntity"""
    
    print("Testing DroneMJCF creation...")
    
    # Initialize Genesis
    gs.init(backend=gs.cpu)  # Use CPU for testing
    
    # Create scene
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))
    
    # Add ground plane
    plane = scene.add_entity(morph=gs.morphs.Plane())
    
    try:
        # Test DroneMJCF creation
        drone = scene.add_entity(morph=gs.morphs.DroneMJCF(
            file="xml/cf2_with_payload_connect.xml", 
            pos=(0, 0, 0.5),
            kf=3.16e-10,  # Manual KF
            km=7.94e-12,  # Manual KM
            propellers_link_name=("cf2", "cf2", "cf2", "cf2"),  # Use main body as propeller links
            propellers_spin=(-1, 1, -1, 1)
        ))
        
        print("✓ DroneMJCF created successfully")
        print(f"  - Drone type: {type(drone)}")
        print(f"  - Number of propellers: {drone.n_propellers}")
        print(f"  - KF coefficient: {drone.KF}")
        print(f"  - KM coefficient: {drone.KM}")
        print(f"  - Propeller indices: {drone.propellers_idx}")
        print(f"  - Propeller spin directions: {drone.propellers_spin}")
        
        # Build scene
        scene.build()
        print("✓ Scene built successfully")
        
        # Test propeller control
        base_rpm = 14468.429183500699
        drone.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])
        print("✓ Propeller RPM set successfully")
        
        # Test simulation step
        scene.step()
        print("✓ Simulation step completed")
        
        print("\n🎉 All tests passed! DroneMJCF is working correctly.")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

def test_drone_mjcf_with_manual_params():
    """Test DroneMJCF with manually specified KF and KM parameters"""
    
    print("\nTesting DroneMJCF with manual parameters...")
    
    # Genesis is already initialized from previous test
    
    # Create scene
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))
    
    # Add ground plane
    plane = scene.add_entity(morph=gs.morphs.Plane())
    
    try:
        # Test DroneMJCF with manual KF and KM
        drone = scene.add_entity(morph=gs.morphs.DroneMJCF(
            file="xml/cf2_with_payload_connect.xml", 
            pos=(0, 0, 0.5),
            kf=3.16e-10,  # Manual KF
            km=7.94e-12,  # Manual KM
            propellers_link_name=("cf2", "cf2", "cf2", "cf2"),  # Use main body as propeller links
            propellers_spin=(-1, 1, -1, 1)
        ))
        
        print("✓ DroneMJCF with manual parameters created successfully")
        print(f"  - KF coefficient: {drone.KF}")
        print(f"  - KM coefficient: {drone.KM}")
        
        # Build scene
        scene.build()
        print("✓ Scene built successfully")
        
        print("🎉 Manual parameter test passed!")
        
    except Exception as e:
        print(f"❌ Manual parameter test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    print("=" * 60)
    print("DroneMJCF Test Suite")
    print("=" * 60)
    
    success1 = test_drone_mjcf_creation()
    success2 = test_drone_mjcf_with_manual_params()
    
    if success1 and success2:
        print("\n🎉 All tests passed! DroneMJCF is ready to use.")
    else:
        print("\n❌ Some tests failed. Please check the implementation.")
