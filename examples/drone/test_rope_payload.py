#!/usr/bin/env python3
"""
测试柔性吊索载荷URDF模型的简单脚本
"""

import genesis as gs
import time

def test_rope_payload_model():
    """测试柔性吊索载荷模型"""
    
    print("正在初始化Genesis引擎...")
    gs.init(backend=gs.cpu)
    
    print("创建场景...")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.0, -6.0, 3.0),
            camera_lookat=(0.0, 0.0, 0.0),
            camera_fov=45,
            max_FPS=60,
        ),
        show_viewer=True,
    )
    
    # 添加地面
    plane = scene.add_entity(gs.morphs.Plane())
    
    print("加载柔性吊索载荷模型...")
    try:
        # 加载新的URDF模型 - 调整初始位置，让小球在0z平面以上
        # 无人机起始高度2.0米，绳索长度1.0米，小球半径0.025米
        # 所以小球最低点应该在 2.0 - 0.05 - 1.0 - 0.025 = 0.925米 > 0
        drone = scene.add_entity(
            morph=gs.morphs.Drone(
                file="urdf/drones/cf2xpayload.urdf",
                pos=(0.0, 0, 2.0),  # 起始高度2.0米，确保小球在0z平面以上
            ),
        )
        print("✓ 柔性吊索载荷模型加载成功！")
        print("✓ 无人机起始高度：2.0米")
        print("✓ 绳索总长度：1.0米")
        print("✓ 小球最低点：约0.925米（在0z平面以上）")
        
        # 添加一些测试球体
        ball1 = scene.add_entity(
            gs.morphs.Sphere(
                pos=(0.5, 0.0, 0.3),
                radius=0.05,
            )
        )
        
        ball2 = scene.add_entity(
            gs.morphs.Sphere(
                pos=(-0.5, 0.0, 0.3),
                radius=0.05,
            )
        )
        
        print("✓ 测试球体添加成功！")
        
    except Exception as e:
        print(f"✗ 模型加载失败: {e}")
        return False
    
    # 设置相机跟随无人机
    scene.viewer.follow_entity(drone)
    
    print("构建场景...")
    scene.build()
    
    print("开始仿真...")
    print("按ESC键退出")
    print("观察柔性吊索的摆动行为...")
    print("注意：现在所有关节都添加了阻尼，应该减少高频抖动")
    
    # 设置一些初始推力
    initial_rpms = [17475.8, 17475.8, 17475.8, 17475.8]
    drone.set_propellels_rpm(initial_rpms)
    
    # 运行仿真
    try:
        while True:
            # 更新物理
            scene.step()
            
            # 控制循环
            time.sleep(1/60)
            
    except KeyboardInterrupt:
        print("\n仿真被用户中断")
    except Exception as e:
        print(f"仿真出错: {e}")
    
    print("仿真结束")
    return True

if __name__ == "__main__":
    print("=" * 50)
    print("柔性吊索载荷模型测试（改进版）")
    print("=" * 50)
    print("改进内容：")
    print("1. 所有revolute关节添加了阻尼（damping=0.01, friction=0.001）")
    print("2. 绳索段质量从0改为0.0005kg（相对于球质量的1/10）")
    print("3. 无人机起始高度调整为2.0米，确保小球在0z平面以上")
    print("=" * 50)
    
    success = test_rope_payload_model()
    
    if success:
        print("✓ 测试完成！")
    else:
        print("✗ 测试失败！")
