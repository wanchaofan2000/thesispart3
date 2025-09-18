import numpy as np
import genesis as gs
import time
import math
from quadcopter_controller import DronePIDController
from genesis.engine.entities.drone_entity import DroneEntity


class CircleController:
    def __init__(self, drone: DroneEntity):
        self.drone = drone
        self.radius = 1  # 写死圆半径
        self.height = 1.0  # 写死飞行高度
        self.angular_speed = 0.2  # 写死角速度
        self.angle = 0.0
        self.running = True
        
        # PID parameters for circle flight
        pid_params = [
            [2.0, 0.0, 0.0],  # pos_x
            [2.0, 0.0, 0.0],  # pos_y
            [2.0, 0.0, 0.0],  # pos_z
            [20.0, 0.0, 20.0],  # vel_x
            [20.0, 0.0, 20.0],  # vel_y
            [25.0, 0.0, 20.0],  # vel_z
            [10.0, 0.0, 1.0],  # att_roll
            [10.0, 0.0, 1.0],  # att_pitch
            [2.0, 0.0, 0.2],   # att_yaw
        ]
        
        # 根据负载质量调整base_rpm
        # 升力 ∝ RPM²，质量 ∝ 升力
        # 无人机质量: 0.027 kg, 负载质量: 0.005 kg
        # 总质量增加比例: (0.027 + 0.005) / 0.027 = 1.185
        # RPM需要增加: √1.185 ≈ 1.089
        # 原base_rpm: 14468.43, 调整后: 14468.43 * 1.089 ≈ 15760
        base_rpm = 15760
        self.pid_controller = DronePIDController(
            drone=drone, 
            dt=0.01, 
            base_rpm=base_rpm, 
            pid_params=pid_params
        )
        
        self.min_rpm = 0.9 * base_rpm
        self.max_rpm = 1.5 * base_rpm

    def clamp(self, rpm):
        return max(self.min_rpm, min(int(rpm), self.max_rpm))

    def get_circle_target(self):
        """Calculate the next target point on the circle"""
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        z = self.height
        return (x, y, z)

    def update(self):
        """Update drone to follow circular path"""
        if not self.running:
            return None
            
        # Get current target point on circle
        target = self.get_circle_target()
        
        # Update PID controller
        rpms = self.pid_controller.update(target)
        
        # Clamp RPMs to valid range
        rpms = [self.clamp(rpm) for rpm in rpms]
        
        # Set propeller RPMs
        self.drone.set_propellels_rpm(rpms)
        
        # Update angle for next iteration
        self.angle += self.angular_speed * 0.01  # dt = 0.01
        
        return rpms


def run_sim(scene, controller, max_steps=10000):
    """Run simulation with circle controller"""
    step = 0
    while controller.running and step < max_steps:
        try:
            # Update drone with circle controller
            rpms = controller.update()
            
            if rpms is not None:
                # Print progress every 100 steps
                if step % 100 == 0:
                    target = controller.get_circle_target()
                    drone_pos = controller.drone.get_pos()
                    print(f"Step {step}: Target {target}, Drone pos {drone_pos.cpu().numpy()}, RPMs {[int(r) for r in rpms]}")

            # Update physics
            scene.step()
            step += 1

            time.sleep(1 / 60)  # Limit simulation rate
        except Exception as e:
            print(f"Error in simulation loop: {e}")
            break

    print(f"Simulation completed after {step} steps")
    if scene.viewer:
        scene.viewer.stop()


def main():
    # Initialize Genesis
    gs.init(backend=gs.gpu)

    # Create scene with fixed camera view (no following)
    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -3.0, 2.0),  # Fixed camera position to observe circle motion
        camera_lookat=(0.0, 0.0, 1.0),  # Look at center of circle
        camera_fov=45,
        max_FPS=60,
    )

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=viewer_options,
        show_viewer=True,
    )

    # Add entities
    plane = scene.add_entity(gs.morphs.Plane())
    
    # 使用cf2xpayload_simple.urdf模型
    model_file = "urdf/drones/cf2xpayload_simple.urdf"
    
    print(f"Loading URDF model from: {model_file}")
    print("Circle flight simulation!")
    
    # 添加无人机
    drone = scene.add_entity(
        morph=gs.morphs.Drone(
            file=model_file,
            pos=(1.0, 0, 1.0),  # 起始高度1.0m
        ),
    )
    
    # 添加一些额外的球体来测试碰撞
    ball1 = scene.add_entity(
        gs.morphs.Sphere(
            pos=(1.0, 0.0, 0.3),
            radius=0.05,
        )
    )
    
    ball2 = scene.add_entity(
        gs.morphs.Sphere(
            pos=(-1.0, 0.0, 0.3),
            radius=0.05,
        )
    )

    # Build scene
    scene.build()

    # Initialize circle controller
    controller = CircleController(drone=drone)

    # Print simulation info
    print(f"\nCircle Flight Simulation:")
    print(f"Circle radius: 1.0 m")
    print(f"Flight height: 1.0 m")
    print(f"Angular speed: 0.2 rad/s")
    print(f"Model loaded: {model_file}")
    print("Starting simulation...")

    # Run simulation
    run_sim(scene, controller)


if __name__ == "__main__":
    main()