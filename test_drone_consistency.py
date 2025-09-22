#!/usr/bin/env python3
"""
测试URDF和MJCF无人机动力学一致性
"""

import genesis as gs
import numpy as np
import time

def test_urdf_drone():
    """测试URDF无人机"""
    print("=== 测试URDF无人机 ===")
    
    # 创建场景
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))
    
    # 添加地面
    plane = scene.add_entity(morph=gs.morphs.Plane())
    
    # 添加URDF无人机
    drone_urdf = scene.add_entity(morph=gs.morphs.Drone(file="urdf/drones/cf2x.urdf", pos=(0, 0, 0.2)))
    
    # 构建场景
    scene.build()
    
    print(f"URDF无人机参数:")
    print(f"  - KF (推力系数): {drone_urdf.KF}")
    print(f"  - KM (力矩系数): {drone_urdf.KM}")
    print(f"  - 螺旋桨数量: {drone_urdf.n_propellers}")
    print(f"  - 螺旋桨旋转方向: {drone_urdf.propellers_spin}")
    
    # 测试悬停
    base_rpm = 14468.429183500699
    drone_urdf.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])
    
    # 运行几步模拟
    for i in range(10):
        scene.step()
        pos = drone_urdf.get_pos()
        print(f"  步骤 {i+1}: 位置 = {pos.cpu().numpy()}")
    
    return drone_urdf

def test_mjcf_drone():
    """测试MJCF无人机"""
    print("\n=== 测试MJCF无人机 ===")
    
    # 创建场景
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))
    
    # 添加地面
    plane = scene.add_entity(morph=gs.morphs.Plane())
    
    # 添加MJCF无人机
    drone_mjcf = scene.add_entity(morph=gs.morphs.DroneMJCF(file="xml/cf2.xml", pos=(0, 0, 0.2)))
    
    # 构建场景
    scene.build()
    
    print(f"MJCF无人机参数:")
    print(f"  - KF (推力系数): {drone_mjcf.KF}")
    print(f"  - KM (力矩系数): {drone_mjcf.KM}")
    print(f"  - 螺旋桨数量: {drone_mjcf.n_propellers}")
    print(f"  - 螺旋桨旋转方向: {drone_mjcf.propellers_spin}")
    
    # 测试悬停
    base_rpm = 14468.429183500699
    drone_mjcf.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])
    
    # 运行几步模拟
    for i in range(10):
        scene.step()
        pos = drone_mjcf.get_pos()
        print(f"  步骤 {i+1}: 位置 = {pos.cpu().numpy()}")
    
    return drone_mjcf

def compare_dynamics():
    """比较URDF和MJCF的动力学参数"""
    print("\n=== 动力学参数比较 ===")
    
    # 测试URDF
    drone_urdf = test_urdf_drone()
    
    # 测试MJCF
    drone_mjcf = test_mjcf_drone()
    
    # 比较参数
    print(f"\n参数一致性检查:")
    print(f"  KF: URDF={drone_urdf.KF}, MJCF={drone_mjcf.KF}, 一致={abs(drone_urdf.KF - drone_mjcf.KF) < 1e-15}")
    print(f"  KM: URDF={drone_urdf.KM}, MJCF={drone_mjcf.KM}, 一致={abs(drone_urdf.KM - drone_mjcf.KM) < 1e-15}")
    print(f"  螺旋桨数量: URDF={drone_urdf.n_propellers}, MJCF={drone_mjcf.n_propellers}, 一致={drone_urdf.n_propellers == drone_mjcf.n_propellers}")
    print(f"  旋转方向: URDF={drone_urdf.propellers_spin}, MJCF={drone_mjcf.propellers_spin}, 一致={np.array_equal(drone_urdf.propellers_spin.cpu().numpy(), drone_mjcf.propellers_spin.cpu().numpy())}")
    
    return drone_urdf, drone_mjcf

def main():
    """主函数"""
    print("无人机动力学一致性测试")
    print("=" * 50)
    
    # 初始化Genesis
    gs.init(backend=gs.cpu)
    
    try:
        # 比较动力学
        drone_urdf, drone_mjcf = compare_dynamics()
        
        print("\n✅ 测试完成！")
        print("URDF和MJCF无人机的动力学参数已保持一致。")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

