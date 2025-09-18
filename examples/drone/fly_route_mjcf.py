import genesis as gs
import math
import time
from quadcopter_controller import DronePIDController
from genesis.engine.entities.drone_entity import DroneEntity
from genesis.vis.camera import Camera

base_rpm = 14468.429183500699
min_rpm = 0.9 * base_rpm
max_rpm = 1.5 * base_rpm


def hover(drone: DroneEntity):
    drone.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])


def clamp(rpm):
    return max(min_rpm, min(int(rpm), max_rpm))


def fly_to_point(target, controller: DronePIDController, scene: gs.Scene):
    drone = controller.drone
    step = 0
    x = target[0] - drone.get_pos()[0]
    y = target[1] - drone.get_pos()[1]
    z = target[2] - drone.get_pos()[2]

    distance = math.sqrt(x**2 + y**2 + z**2)

    while distance > 0.1 and step < 1000:
        [M1, M2, M3, M4] = controller.update(target)
        M1 = clamp(M1)
        M2 = clamp(M2)
        M3 = clamp(M3)
        M4 = clamp(M4)
        drone.set_propellels_rpm([M1, M2, M3, M4])
        scene.step()
        
        # Update distance calculation
        drone_pos = drone.get_pos()
        drone_pos = drone_pos.cpu().numpy()
        x = target[0] - drone_pos[0]
        y = target[1] - drone_pos[1]
        z = target[2] - drone_pos[2]
        distance = math.sqrt(x**2 + y**2 + z**2)
        step += 1
        
        if step % 50 == 0:  # Print progress every 50 steps
            print(f"Step {step}: Distance to target: {distance:.3f}m, Position: {drone_pos}")


def main():
    gs.init(backend=gs.cuda)

    ##### scene #####
    # Create scene with visualization enabled
    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -4.0, 2.0),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=45,
        max_FPS=60,
    )
    
    scene = gs.Scene(
        show_viewer=True, 
        sim_options=gs.options.SimOptions(dt=0.01),
        viewer_options=viewer_options
    )

    ##### entities #####
    plane = scene.add_entity(morph=gs.morphs.Plane())

    # Use DroneMJCF with cf2.xml for basic drone without payload
    # Match URDF parameters: kf=3.16e-10, km=7.94e-12, mass=0.027kg
    drone = scene.add_entity(morph=gs.morphs.DroneMJCF(
        file="xml/cf2.xml", 
        pos=(0, 0, 0.2),  # Match URDF starting position
        # Manually specify KF and KM for MJCF files (from URDF properties)
        kf=3.16e-10,
        km=7.94e-12,
        propellers_link_name=("cf2", "cf2", "cf2", "cf2"),  # Use main body as propeller links
        propellers_spin=(-1, 1, -1, 1)
    ))

    # parameters are tuned such that the
    # drone can fly, not optimized (from original fly_route.py)
    pid_params = [
        [2.0, 0.0, 0.0],   # pos_x
        [2.0, 0.0, 0.0],   # pos_y
        [2.0, 0.0, 0.0],   # pos_z
        [20.0, 0.0, 20.0], # vel_x
        [20.0, 0.0, 20.0], # vel_y
        [25.0, 0.0, 20.0], # vel_z
        [10.0, 0.0, 1.0],  # att_roll
        [10.0, 0.0, 1.0],  # att_pitch
        [2.0, 0.0, 0.2],   # att_yaw
    ]

    controller = DronePIDController(drone, dt=0.01, base_rpm=base_rpm, pid_params=pid_params)

    ##### build #####
    scene.build()
    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -4.0, 2.0),  # Now behind the drone (negative Y)
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=45,
        max_FPS=60,
    )

    # Hover for a bit
    hover(drone)
    for _ in range(100):
        scene.step()

    # Fly to different points
    targets = [
        (1, 0, 1),
        (0, 1, 1),
        (-1, 0, 1),
        (0, -1, 1),
        (0, 0, 1),
    ]

    for target in targets:
        print(f"Flying to target: {target}")
        fly_to_point(target, controller, scene)

    # Hover at final position
    hover(drone)
    for _ in range(100):
        scene.step()

    print("Flight completed!")
    print("Press ESC to exit the viewer...")
    
    # Keep the viewer running until user closes it
    try:
        while scene.viewer.is_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Simulation interrupted by user")


if __name__ == "__main__":
    main()
