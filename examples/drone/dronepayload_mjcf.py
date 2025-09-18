import numpy as np
import genesis as gs
import time


class State:
    def __init__(self, pos, vel, quat_xyzw, omega):
        self.position = pos
        self.velocity = vel
        self.quaternion = quat_xyzw  # x y z w
        self.omega = omega


class SE3Controller:
    def __init__(self):
        # SE3控制器参数 (来自mujoco_ref)
        self.kx = 0.6   # 位置控制反馈系数
        self.kv = 0.4   # 速度控制反馈系数
        self.kR = 6.0   # SO3控制反馈系数
        self.kw = 1.0   # 角速度控制反馈系数
        self.gravity_vec = np.array([0.0, 0.0, -1.0])
        # 无人机转动惯量 (diag: Ixx, Iyy, Izz) - 待从XML读取
        self.inertia = np.array([1.395e-5, 1.395e-5, 2.173e-5])

    @staticmethod
    def quat_to_rot(q):
        x, y, z, w = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    @staticmethod
    def vee(R):
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    def control_update(self, current: State, goal: State, dt, forward):
        # linear
        e_x = current.position - goal.position
        e_v = current.velocity - goal.velocity
        trans = np.array([
            -self.kx * e_x[0] - self.kv * e_v[0] + goal.velocity[0],
            -self.kx * e_x[1] - self.kv * e_v[1] + goal.velocity[1],
            -self.kx * e_x[2] - self.kv * e_v[2] + goal.velocity[2],
        ])

        # attitude
        R_curr = self.quat_to_rot(current.quaternion)
        goal_z = trans - self.gravity_vec
        nz = np.linalg.norm(goal_z)
        if nz > 1e-6:
            goal_z = goal_z / nz
        else:
            goal_z = R_curr[:, 2]
        right = np.cross(forward, goal_z)
        right_norm = np.linalg.norm(right)
        right = right / (right_norm + 1e-9)
        fwd = np.cross(goal_z, right)
        R_goal = np.stack([right, fwd, goal_z], axis=1)
        e_R = 0.5 * self.vee(R_goal.T @ R_curr - R_curr.T @ R_goal)
        e_w = current.omega - R_curr.T @ (R_goal @ goal.omega)

        # 力矩域PD + 科氏项 (机体系)
        tau_pd = np.array([
            -self.kR * e_R[0] - self.kw * e_w[0],
            -self.kR * e_R[1] - self.kw * e_w[1],
            -self.kR * e_R[2] - self.kw * e_w[2],
        ])
        coriolis = np.cross(current.omega, self.inertia * current.omega)
        torque_body = tau_pd + coriolis

        return nz, torque_body


class Mixer:
    def __init__(self):
        # 电机参数 (来自mujoco_ref)
        self.Ct = 3.25e-4       # 电机推力系数 (N/krpm^2)
        self.Cd = 7.9379e-6     # 电机反扭系数 (Nm/krpm^2)
        self.L = 0.065 / 2.0    # 电机力臂长度 (m)
        self.max_thrust = 0.1573    # 单个电机最大推力 (N)
        self.max_torque = 3.842e-03 # 单个电机最大扭矩 (Nm)
        self.max_speed = 22.0   # 电机最大转速 (krpm)
        
        # 动力分配矩阵 (来自mujoco_ref)
        self.mat = np.array([
            [self.Ct, self.Ct, self.Ct, self.Ct],                                   # F total
            [self.Ct*self.L, -self.Ct*self.L, -self.Ct*self.L, self.Ct*self.L],     # Mx + - - +
            [-self.Ct*self.L, -self.Ct*self.L, self.Ct*self.L, self.Ct*self.L],     # My - - + +
            [-self.Cd, self.Cd, -self.Cd, self.Cd]                                  # Mz - + - +
        ])
        self.inv_mat = np.linalg.inv(self.mat)
        
        # 螺旋桨位置 (相对于质心)
        self.prop_positions = np.array([
            [self.L, self.L, 0],    # motor1 (前右)
            [self.L, -self.L, 0],   # motor2 (前左) 
            [-self.L, -self.L, 0],  # motor3 (后左)
            [-self.L, self.L, 0],   # motor4 (后右)
        ])

    def calculate(self, thrust, mx, my, mz):
        """动力分配函数 (来自mujoco_ref) - 带饱和处理"""
        Mx, My = mx, my  # Copy
        Mz = 0  # 首先进行X Y轴分配
        control_input = np.array([thrust, Mx, My, Mz])
        motor_speed_squ = self.inv_mat @ control_input
        
        # X Y Z三轴动力分配的顺序决定最终取舍的不同
        # 一般情况下 首先对X Y轴动力进行分配 余量用于分配Z轴
        max_value = np.max(motor_speed_squ)
        min_value = np.min(motor_speed_squ)
        ref_value = np.sum(motor_speed_squ) / 4.0  # 参考转速(不施加扭矩时的转速平方)
        
        max_trim_scale = 1.0
        min_trim_scale = 1.0
        if max_value > self.max_speed **2:  # 存在电机动力饱和 计算缩放因子进行缩放
            max_trim_scale = (self.max_speed ** 2 - ref_value)/(max_value - ref_value)
        if min_value < 0:  # 存在电机动力负饱和 计算缩放因子进行缩放
            min_trim_scale = (ref_value)/(ref_value - min_value)
        scale = min(max_trim_scale, min_trim_scale)
        
        # 对X Y扭矩施加缩放因子
        Mx = Mx * scale  
        My = My * scale
        # 重新计算电机转速平方
        control_input = np.array([thrust, Mx, My, Mz])
        motor_speed_squ = self.inv_mat @ control_input
        
        if scale < 1.0:  # 存在Trim 不进行Z轴扭矩分配 直接返回
            motor_speed_squ = np.abs(motor_speed_squ)
            return np.sqrt(motor_speed_squ)  # 返回电机转速
        else:  # 仍然有余量 可以进行Z轴扭矩分配
            Mz = mz
            control_input_withz = np.array([thrust, Mx, My, Mz])  # 添加Z轴扭矩重新计算
            motor_speed_squ_withz = self.inv_mat @ control_input_withz
            
            # 判断是否饱和
            max_value = np.max(motor_speed_squ_withz)
            min_value = np.min(motor_speed_squ_withz)
            max_index = np.argmax(motor_speed_squ_withz)
            min_index = np.argmin(motor_speed_squ_withz)
            max_trim_scale_z = 1.0
            min_trim_scale_z = 1.0
            if max_value > self.max_speed **2:  # 存在电机动力饱和 计算缩放因子进行缩放
                max_trim_scale_z = (self.max_speed ** 2 - motor_speed_squ[max_index])/(max_value - motor_speed_squ[max_index])
            if min_value < 0:  # 存在电机动力负饱和 计算缩放因子进行缩放
                min_trim_scale_z = (motor_speed_squ[min_index])/(motor_speed_squ[min_index] - min_value)
            scale_z = min(max_trim_scale_z, min_trim_scale_z)
            
            # 对Z轴扭矩施加缩放因子
            Mz = Mz * scale_z
            # 重新计算电机转速平方
            control_input_withz = np.array([thrust, Mx, My, Mz])
            motor_speed_squ_withz = self.inv_mat @ control_input_withz
            motor_speed_squ_withz = np.abs(motor_speed_squ_withz)
            return np.sqrt(motor_speed_squ_withz)  # 返回电机转速

    def allocate(self, thrust_N, torque_Nm):
        """简化版分配函数 (向后兼容)"""
        return self.calculate(thrust_N, torque_Nm[0], torque_Nm[1], torque_Nm[2])
    
    def rpm_to_forces_torques(self, motor_krpm, R_body_to_world):
        """将电机转速转换为各个螺旋桨的推力和力矩
        
        Args:
            motor_krpm: 电机转速 (krpm)
            R_body_to_world: 机体到世界坐标系的旋转矩阵 (3x3)
        
        Returns:
            forces_world: 各桨推力在世界坐标系 (N)
            torques_world: 各桨力矩在世界坐标系 (Nm)
            positions_world: 各桨位置在世界坐标系 (m)
        """
        thrusts = motor_krpm**2 * self.Ct  # 各桨推力 (N)
        forces_body = []
        torques_body = []
        
        for i, (thrust, pos_body) in enumerate(zip(thrusts, self.prop_positions)):
            # 机体坐标系：推力沿机体+z方向
            force_body = np.array([0.0, 0.0, thrust])
            forces_body.append(force_body)
            
            # 机体坐标系：推力产生的力矩
            torque_from_thrust_body = np.cross(pos_body, force_body)
            
            # 机体坐标系：反扭矩绕机体z轴
            # 与Mixer.mat第四行 [-Cd, +Cd, -Cd, +Cd] 保持一致的旋向符号
            spin_direction = -1 if i in [0, 2] else 1  # 1,3为-; 2,4为+
            reaction_torque_body = np.array([0, 0, motor_krpm[i]**2 * self.Cd * spin_direction])
            
            # 机体坐标系总力矩
            total_torque_body = torque_from_thrust_body + reaction_torque_body
            torques_body.append(total_torque_body)
        
        # 转换到世界坐标系
        forces_world = [R_body_to_world @ f for f in forces_body]
        torques_world = [R_body_to_world @ t for t in torques_body]
        positions_world = [R_body_to_world @ p for p in self.prop_positions]
        
        return forces_world, torques_world, positions_world


def main():
    # Initialize Genesis
    gs.init(backend=gs.cpu)

    # Create scene with initial camera view
    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -2.0, 1.0),  # 相机位置
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=45,
        max_FPS=60,
    )

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,  # 10ms步长，和interactive_drone.py一致
            gravity=(0, 0, -9.8066),
        ),
        viewer_options=viewer_options,
        show_viewer=True,
    )

    # Add ground plane
    plane = scene.add_entity(gs.morphs.Plane())

    # Add drone with CONNECT constraint model
    drone = scene.add_entity(
        morph=gs.morphs.MJCF(
            file="xml/cf2_with_payload_connect.xml",
            pos=(0.0, 0, 0.5),
        )
    )

    # Build scene
    scene.build()

    # Set camera to follow drone
    scene.viewer.follow_entity(drone)

    # Initialize virtual actuator chain (SE3 + Mixer)
    se3 = SE3Controller()
    mixer = Mixer()
    mass = 0.033        # 无人机质量 (kg) - 来自XML
    g = 9.8066          # 重力加速度 (m/s^2)
    # 力矩域控制：不使用额外缩放
    circle_radius = 1.6
    circle_height = 0.7
    circle_speed = 0.05
    wait_time = 1.5
    
    # 控制模式选择
    use_distributed_forces = True  # True: 分解到四桨施力, False: 净力/净矩施力
    
    mode_desc = "分布式四桨施力" if use_distributed_forces else "净力净矩施力"
    print(f"Starting SE3+Mixer virtual actuator with CONNECT rope... 模式: {mode_desc}")

    prev_pos = None
    last_log_time = -1.0
    start_time = time.time()

    try:
        while True:
            sim_time = time.time() - start_time

            # Current state - 读取真实状态
            qpos = drone.get_qpos()
            if hasattr(qpos, 'numpy'):
                qpos = qpos.numpy()
            pos = qpos[:3]
            # MJCF qpos is [pos(3), quat(wxyz)], convert to xyzw
            quat_wxyz = qpos[3:7]
            quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
            
            # 计算线速度 (有限差分)
            if prev_pos is not None:
                vel = (pos - prev_pos) / scene.sim.dt
            else:
                vel = np.zeros(3)
            prev_pos = pos.copy()
            
            # 读取真实角速度 (世界坐标系)
            base_link = drone.get_link("cf2")
            omega_world = base_link.get_ang()  # 角速度 (世界坐标系)
            if hasattr(omega_world, 'numpy'):
                omega_world = omega_world.numpy()
            
            # 转换到机体坐标系
            R = se3.quat_to_rot(quat_xyzw)
            omega = R.T @ omega_world  # 机体坐标系角速度
            
            current = State(pos, vel, quat_xyzw, omega)

            # Circle trajectory (with initial hover to start point)
            if sim_time < wait_time:
                goal_p = np.array([circle_radius, 0.0, circle_height])
                fwd = np.array([0.0, 1.0, 0.0])
            else:
                ang = 2 * np.pi * circle_speed * (sim_time - wait_time)
                goal_p = np.array([circle_radius * np.cos(ang), circle_radius * np.sin(ang), circle_height])
                fwd = np.array([-np.sin(ang), np.cos(ang), 0.0])
            goal = State(goal_p, np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3))

            # Controller → thrust (in g) + body torque (Nm)
            thrust_g, torque_body = se3.control_update(current, goal, scene.sim.dt, fwd)
            thrust_N = thrust_g * g * mass

            # Mixer (motor rpm)
            motor_krpm = mixer.calculate(thrust_N, torque_body[0], torque_body[1], torque_body[2])

            if use_distributed_forces:
                # 分布式四桨施力：分解到各桨位置施力
                R_body_to_world = se3.quat_to_rot(quat_xyzw)
                forces_world, torques_world, positions_world = mixer.rpm_to_forces_torques(motor_krpm, R_body_to_world)
                base_link_idx = [drone.get_link("cf2").idx]
                
                # 合并所有推力和力矩 (避免多次API调用)
                total_force_world = np.sum(forces_world, axis=0)
                total_torque_world = np.sum(torques_world, axis=0)
                
                force_tensor = [[float(total_force_world[0]), float(total_force_world[1]), float(total_force_world[2])]]
                torque_tensor = [[float(total_torque_world[0]), float(total_torque_world[1]), float(total_torque_world[2])]]
                
                scene.sim.rigid_solver.apply_links_external_force(force=force_tensor, links_idx=base_link_idx)
                scene.sim.rigid_solver.apply_links_external_torque(torque=torque_tensor, links_idx=base_link_idx)
            else:
                # 净力/净矩施力模式 (原方法)
                base_link_idx = [drone.get_link("cf2").idx]
                force_tensor = [[0.0, 0.0, float(thrust_N)]]
                # 机体系 -> 世界系
                R_body_to_world = se3.quat_to_rot(quat_xyzw)
                torque_world = R_body_to_world @ torque_body
                torque_tensor = [[float(torque_world[0]), float(torque_world[1]), float(torque_world[2])]]
                scene.sim.rigid_solver.apply_links_external_force(force=force_tensor, links_idx=base_link_idx)
                scene.sim.rigid_solver.apply_links_external_torque(torque=torque_tensor, links_idx=base_link_idx)

            # Step simulation
            scene.step()

            # Log (2 Hz)
            if sim_time - last_log_time >= 0.5 and sim_time > 0:
                last_log_time = sim_time
                pos_err = np.linalg.norm(goal_p - pos)
                omega_norm = np.linalg.norm(omega)
                print(
                    f"Time: {sim_time:.1f}s | Pos: [{pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}] | "
                    f"Target: [{goal_p[0]:.3f} {goal_p[1]:.3f} {goal_p[2]:.3f}] | Err: {pos_err:.3f} | "
                    f"Omega: [{omega[0]:.3f} {omega[1]:.3f} {omega[2]:.3f}] (|ω|={omega_norm:.3f}) | "
                    f"Thrust: {thrust_N:.4f}N | Tau_body: [{torque_body[0]:.6f} {torque_body[1]:.6f} {torque_body[2]:.6f}] | "
                    f"Motors(krpm): [{motor_krpm[0]:.2f} {motor_krpm[1]:.2f} {motor_krpm[2]:.2f} {motor_krpm[3]:.2f}]"
                )

            # Sim pacing: 60 FPS like interactive_drone.py
            time.sleep(1 / 60)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        print("Simulation finished")


if __name__ == "__main__":
    main()
