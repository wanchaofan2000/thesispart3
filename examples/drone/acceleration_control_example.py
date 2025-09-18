import argparse
import numpy as np
import genesis as gs
import time
import threading
from pynput import keyboard


class AccelerationDroneController:
    def __init__(self, drone):
        self.drone = drone
        self.running = True
        self.pressed_keys = set()
        
        # 加速度控制参数
        self.acceleration_gains = [2.0, 2.0, 2.0]  # x, y, z轴增益
        self.target_acceleration = np.array([0.0, 0.0, 0.0])  # 目标加速度
        
        # 设置加速度控制
        self.drone.set_acceleration_gains(self.acceleration_gains)
        self.drone.enable_acceleration_control(True)

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                self.running = False
                return False
            self.pressed_keys.add(key)
            print(f"Key pressed: {key}")
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key)
        except KeyError:
            pass

    def update_acceleration(self):
        """根据按键更新目标加速度"""
        # 重置加速度
        self.target_acceleration = np.array([0.0, 0.0, 0.0])
        
        # 根据按键设置加速度
        if keyboard.Key.up in self.pressed_keys:
            self.target_acceleration[1] = 2.0  # 向前加速
        if keyboard.Key.down in self.pressed_keys:
            self.target_acceleration[1] = -2.0  # 向后加速
        if keyboard.Key.left in self.pressed_keys:
            self.target_acceleration[0] = -2.0  # 向左加速
        if keyboard.Key.right in self.pressed_keys:
            self.target_acceleration[0] = 2.0   # 向右加速
        if keyboard.Key.space in self.pressed_keys:
            self.target_acceleration[2] = 2.0   # 向上加速
        if keyboard.Key.shift in self.pressed_keys:
            self.target_acceleration[2] = -2.0  # 向下加速
            
        # 设置目标加速度
        self.drone.set_target_acceleration(self.target_acceleration)
        
        return self.target_acceleration


def run_sim(scene, drone, controller):
    while controller.running:
        try:
            # 更新加速度控制
            target_acc = controller.update_acceleration()
            
            # 计算所需的螺旋桨转速
            rpms = drone.compute_acceleration_control()
            
            if rpms is not None:
                # 应用螺旋桨转速
                drone.set_propellels_rpm(rpms)
            
            # 更新物理仿真
            scene.step()
            
            time.sleep(1 / 60)  # 限制仿真速率
        except Exception as e:
            print(f"仿真循环中的错误: {e}")

    if scene.viewer:
        scene.viewer.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", default=True, help="启用可视化 (默认: True)")
    parser.add_argument("-m", "--mac", action="store_true", default=False, help="在MacOS上运行 (默认: False)")
    args = parser.parse_args()

    # 初始化Genesis
    gs.init(backend=gs.cpu)

    # 创建场景
    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -4.0, 2.0),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=45,
        max_FPS=60,
    )

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=viewer_options,
        show_viewer=args.vis,
    )

    # 添加实体
    plane = scene.add_entity(gs.morphs.Plane())
    drone = scene.add_entity(
        morph=gs.morphs.Drone(
            file="urdf/drones/cf2xpayload.urdf",
            pos=(0.0, 0, 0.5),
        ),
    )

    scene.viewer.follow_entity(drone)

    # 构建场景
    scene.build()

    # 初始化加速度控制器
    controller = AccelerationDroneController(drone)

    # 打印控制说明
    print("\n无人机加速度控制:")
    print("↑ - 向前加速")
    print("↓ - 向后加速")
    print("← - 向左加速")
    print("→ - 向右加速")
    print("空格 - 向上加速")
    print("Shift - 向下加速")
    print("ESC - 退出\n")

    # 启动键盘监听器
    listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()

    if args.mac:
        # 在另一个线程中运行仿真
        sim_thread = threading.Thread(target=run_sim, args=(scene, drone, controller))
        sim_thread.start()
        
        # 主线程处理可视化
        while controller.running:
            scene.viewer.update()
            time.sleep(1 / 60)
    else:
        # 直接运行仿真
        run_sim(scene, drone, controller)

    listener.stop()


if __name__ == "__main__":
    main()

