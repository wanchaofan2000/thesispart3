# Genesis URDF解析规则详解

## 1. 概述

Genesis项目使用自定义的URDF解析器来将URDF文件转换为可仿真的实体。解析过程主要涉及链接(links)、关节(joints)、几何体(geometries)和惯性属性(inertial properties)的处理。

## 2. URDF文件结构

### 2.1 基本元素

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- 链接定义 -->
  <link name="link_name">
    <!-- 惯性属性 -->
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    
    <!-- 视觉几何体 -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!-- 几何体类型 -->
      </geometry>
      <material name="material_name"/>
    </visual>
    
    <!-- 碰撞几何体 -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!-- 几何体类型 -->
      </geometry>
    </collision>
  </link>
  
  <!-- 关节定义 -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>
  
  <!-- 材料定义 -->
  <material name="material_name">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
</robot>
```

## 3. 解析规则详解

### 3.1 链接解析

#### 3.1.1 惯性属性解析
```python
# 惯性属性处理
if link.inertial is None:
    # 默认值
    l_info["inertial_pos"] = gu.zero_pos()
    l_info["inertial_quat"] = gu.identity_quat()
    l_info["inertial_i"] = None
    l_info["inertial_mass"] = None
else:
    # 解析惯性属性
    l_info["inertial_pos"] = link.inertial.origin[:3, 3]
    l_info["inertial_quat"] = gu.R_to_quat(link.inertial.origin[:3, :3])
    l_info["inertial_i"] = link.inertial.inertia
    l_info["inertial_mass"] = link.inertial.mass
```

#### 3.1.2 几何体解析
支持的几何体类型：
- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="path/to/mesh.obj"/>`

### 3.2 关节解析

#### 3.2.1 支持的关节类型

1. **fixed** - 固定关节
   - 自由度：0
   - 用途：连接两个链接，无相对运动

2. **revolute** - 旋转关节
   - 自由度：1
   - 用途：绕指定轴旋转
   - 限制：位置、速度、力矩

3. **continuous** - 连续旋转关节
   - 自由度：1
   - 用途：无限制的旋转
   - 限制：无位置限制

4. **prismatic** - 移动关节
   - 自由度：1
   - 用途：沿指定轴移动
   - 限制：位置、速度、力矩

5. **floating** - 自由关节
   - 自由度：6
   - 用途：完全自由的运动（位置+姿态）

#### 3.2.2 关节限制解析
```python
# 关节限制处理
if joint.limit is not None:
    j_info["dofs_limit"] = np.array([
        [
            joint.limit.lower if joint.limit.lower is not None else -np.inf,
            joint.limit.upper if joint.limit.upper is not None else np.inf,
        ]
    ])
```

### 3.3 坐标系转换

#### 3.3.1 位置和姿态
- URDF使用右手坐标系
- 位置：`xyz="x y z"` (米)
- 姿态：`rpy="roll pitch yaw"` (弧度)

#### 3.3.2 缩放处理
```python
# 应用缩放因子
l_info["pos"] *= morph.scale
l_info["inertial_pos"] *= morph.scale
l_info["inertial_mass"] *= morph.scale**3
l_info["inertial_i"] *= morph.scale**5
```

## 4. 无人机特殊处理

### 4.1 螺旋桨识别
```python
# 螺旋桨链接识别
propellers_link = gs.List([self.get_link(name) for name in morph.propellers_link_name])
self._propellers_link_idxs = torch.tensor(
    [link.idx for link in propellers_link], dtype=gs.tc_int, device=gs.device
)
```

### 4.2 动力学参数
```xml
<!-- 从URDF文件读取动力学参数 -->
<gazebo>
  <plugin name="drone_dynamics" filename="libgazebo_ros_force.so">
    <kf>1.0</kf>  <!-- 推力系数 -->
    <km>0.1</km>  <!-- 力矩系数 -->
  </plugin>
</gazebo>
```

## 5. 性能优化

### 5.1 固定关节合并
```python
# 合并固定关节连接的链接
if morph.merge_fixed_links:
    robot = merge_fixed_links(robot, morph.links_to_keep)
```

### 5.2 链接排序
```python
# 按运动学树深度排序链接
l_infos, links_j_infos, links_g_infos, _ = _order_links(l_infos, links_j_infos, links_g_infos)
```

## 6. 约束处理

### 6.1 等式约束
```python
# 解析等式约束（如mimic关节）
eqs_info = parse_equalities(robot, morph)
```

### 6.2 关节限制
- 位置限制：`lower` 和 `upper`
- 速度限制：`velocity`
- 力矩限制：`effort`

## 7. 材料处理

### 7.1 颜色定义
```xml
<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>
```

### 7.2 材质优先级
```python
# 材质优先级处理
if morph.prioritize_urdf_material or not tmesh.visual.defined:
    if geom.material is not None and geom.material.color is not None:
        mesh.set_color(geom.material.color)
```

## 8. 错误处理

### 8.1 常见错误
1. **文件不存在**：检查URDF文件路径
2. **几何体不支持**：确保使用支持的几何体类型
3. **关节类型不支持**：检查关节类型是否在支持列表中
4. **链接引用错误**：确保关节中的parent和child链接存在

### 8.2 调试信息
```python
# 启用调试日志
gs.logger.debug(f"Parsing joint: {joint.name}")
gs.logger.debug(f"Parsing link: {link.name}")
```

## 9. 最佳实践

### 9.1 URDF文件编写
1. 使用有意义的链接和关节名称
2. 正确设置惯性属性
3. 合理设置关节限制
4. 使用适当的几何体类型

### 9.2 性能优化
1. 合并固定关节连接的链接
2. 使用简化的碰撞几何体
3. 合理设置缩放因子

### 9.3 调试技巧
1. 检查链接和关节的层次结构
2. 验证惯性属性设置
3. 确认几何体正确加载
4. 测试关节限制设置

## 10. 示例：无人机绳索负载模型

参考 `urdf/drones/drone_with_payload.urdf` 文件，展示了如何创建包含多个链接和关节的复杂模型。

关键要点：
1. 使用固定关节连接螺旋桨
2. 使用旋转关节模拟绳索连接
3. 正确设置质量和惯性属性
4. 合理配置关节限制

