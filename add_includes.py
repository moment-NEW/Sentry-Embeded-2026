import os
import xml.etree.ElementTree as ET

# 此脚本现在会遍历您在 search_dir_name 变量中指定的文件夹下的所有子文件夹，并将它们的相对路径添加到 .uvprojx 文件的 <IncludePath> 标签中
def main():
    """
    主函数，用于扫描指定目录下的所有子文件夹，
    并将它们的相对路径添加到 Keil MDK 项目的 IncludePath 中。
    """
    try:
        # --- 配置 ---
        # 将此变量更改为您想要扫描的文件夹名称
        search_dir_name = 'Sentry_libs'
        # Keil 项目文件相对于此脚本的位置
        keil_proj_relative_path = os.path.join('MDK-ARM', 'frame.uvprojx')
        # ----------------

        # 获取项目根目录（此脚本所在的目录）
        root_dir = os.path.dirname(os.path.abspath(__file__))
        # 要扫描的完整路径
        search_dir_full_path = os.path.join(root_dir, search_dir_name)
        # Keil 项目文件的完整路径
        proj_file_path = os.path.join(root_dir, keil_proj_relative_path)
        # MDK-ARM 文件夹的路径，用于计算相对路径
        mdk_arm_dir = os.path.dirname(proj_file_path)

        if not os.path.isdir(search_dir_full_path):
            print(f"错误：找不到目录 '{search_dir_full_path}'。请检查 'search_dir_name' 变量。")
            return

        # 收集所有子文件夹的路径
        include_paths_to_add = []
        for dirpath, _, _ in os.walk(search_dir_full_path):
            # 计算相对于 MDK-ARM 文件夹的路径
            rel_path = os.path.relpath(dirpath, mdk_arm_dir).replace('\\', '/')
            include_paths_to_add.append(rel_path)

        if not include_paths_to_add:
            print(f"在 '{search_dir_name}' 中没有找到要添加的子文件夹。")
            return

        print(f"找到的包含路径: {include_paths_to_add}")

        # 解析 .uvprojx 文件
        tree = ET.parse(proj_file_path)
        xml_root = tree.getroot()

        # 找到所有的 <IncludePath> 标签并更新它们
        # Keil 项目文件可能为 C/C++ 和汇编定义不同的包含路径
        paths_updated = False
        for elem in xml_root.iter('IncludePath'):
            # 分割现有路径，处理空标签的情况
            current_paths = elem.text.split(';') if elem.text else []
            # 去除空白条目
            current_paths_set = set(filter(None, current_paths))

            newly_added_paths = []
            for path in include_paths_to_add:
                if path not in current_paths_set:
                    current_paths_set.add(path)
                    newly_added_paths.append(path)

            if newly_added_paths:
                paths_updated = True
                # 按字母顺序排序以保持一致性
                elem.text = ';'.join(sorted(list(current_paths_set)))

        # 保存修改后的 .uvprojx 文件
        if paths_updated:
            # 注册命名空间以保留它
            ET.register_namespace('', "http://www.keil.com/pack/")
            tree.write(proj_file_path, encoding='UTF-8', xml_declaration=True)
            print("成功将包含路径添加到项目中。")
        else:
            print("所有路径均已存在，无需更新。")

    except FileNotFoundError:
        print(f"错误：项目文件 '{proj_file_path}' 未找到。")
    except ET.ParseError:
        print(f"错误：解析项目文件 '{proj_file_path}' 失败。请检查文件是否为有效的 XML。")
    except Exception as e:
        print(f"发生未知错误: {e}")


if __name__ == "__main__":
    main()
