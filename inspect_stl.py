#!/usr/bin/env python3
"""
解析 STL 二进制文件，打印包围盒和中心位置
直接在项目根目录跑：python inspect_stl.py
"""
import struct
import os
import sys

def read_stl_info(filepath):
    size = os.path.getsize(filepath)
    with open(filepath, 'rb') as f:
        # 读取 80 字节头部
        header = f.read(80)
        print(f"文件头: {header.strip()}")
        
        # 读取三角面数量
        num_triangles = struct.unpack('<I', f.read(4))[0]
        print(f"三角面数: {num_triangles:,}")
        print(f"实际文件大小: {size:,} 字节 (预期: {84 + num_triangles * 50:,})")
        
        # 读所有三角形的顶点，计算包围盒
        min_x, min_y, min_z = 1e9, 1e9, 1e9
        max_x, max_y, max_z = -1e9, -1e9, -1e9
        
        for i in range(num_triangles):
            # 跳过法向量 (12 bytes)
            f.read(12)
            # 3 个顶点，每个 (3 floats × 4 bytes = 12 bytes)
            for _ in range(3):
                vx, vy, vz = struct.unpack('<fff', f.read(12))
                min_x = min(min_x, vx); max_x = max(max_x, vx)
                min_y = min(min_y, vy); max_y = max(max_y, vy)
                min_z = min(min_z, vz); max_z = max(max_z, vz)
            # 跳过属性字节
            f.read(2)
        
        print("\n" + "=" * 60)
        print("📐 包围盒信息 (单位: 米)")
        print("=" * 60)
        print(f"  X 范围: [{min_x:.4f}, {max_x:.4f}]   尺寸: {max_x - min_x:.4f} m = {max_x - min_x:.1f} mm")
        print(f"  Y 范围: [{min_y:.4f}, {max_y:.4f}]   尺寸: {max_y - min_y:.4f} m = {max_y - min_y:.1f} mm")
        print(f"  Z 范围: [{min_z:.4f}, {max_z:.4f}]   尺寸: {max_z - min_z:.4f} m = {max_z - min_z:.1f} mm")
        
        cx = (min_x + max_x) / 2
        cy = (min_y + max_y) / 2
        cz = (min_z + max_z) / 2
        print(f"\n  包围盒中心: ({cx:.4f}, {cy:.4f}, {cz:.4f}) m")
        
        print("\n" + "=" * 60)
        print("📍 坐标系原点在探头中的位置 (STL (0,0,0) 相对于包围盒)")
        print("=" * 60)
        print(f"  X 轴向原点偏移: {0 - cx:.4f} m")
        print(f"  Y 轴向原点偏移: {0 - cy:.4f} m")
        print(f"  Z 轴向原点偏移: {0 - cz:.4f} m")
        
        print("\n" + "=" * 60)
        print("🧭 结论")
        print("=" * 60)
        # 判断哪个轴是探头的长度轴
        ranges = {
            'X': max_x - min_x,
            'Y': max_y - min_y,
            'Z': max_z - min_z
        }
        long_axis = max(ranges, key=ranges.get)
        print(f"  最长轴是 {long_axis} 轴 (长度 {ranges[long_axis]*1000:.1f} mm) → 这是探头的'长度方向'")
        
        # 原点相对包围盒的位置
        for axis, mn, mx in [('X', min_x, max_x), ('Y', min_y, max_y), ('Z', min_z, max_z)]:
            if abs(mn) < 0.001:
                print(f"  {axis} 轴: STL 原点 ≈ 包围盒最小值 (一端)")
            elif abs(mx) < 0.001:
                print(f"  {axis} 轴: STL 原点 ≈ 包围盒最大值 (一端)")
            else:
                center = (mn + mx) / 2
                if abs(center) < 0.005:
                    print(f"  {axis} 轴: STL 原点 ≈ 包围盒中心")
                else:
                    print(f"  {axis} 轴: STL 原点在包围盒内部 ({mn:.3f}~{mx:.3f})")

if __name__ == "__main__":
    stl_path = sys.argv[1] if len(sys.argv) > 1 else "dobot_rviz/meshes/cr5/ultrasound_probe.stl"
    if not os.path.exists(stl_path):
        # 也试试根目录下的原始文件
        alt = "Linear USG probe.stl"
        if os.path.exists(alt):
            stl_path = alt
        else:
            print(f"找不到 STL 文件: {stl_path}")
            sys.exit(1)
    
    print(f"解析文件: {stl_path}\n")
    read_stl_info(stl_path)