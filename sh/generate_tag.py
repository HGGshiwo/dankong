#!/usr/bin/env python3

import argparse
import math
import os
import re
import urllib.request
from urllib.error import HTTPError, URLError

from PIL import Image, ImageDraw


def parse_size(size_str):
    """解析尺寸，支持 m 自动转为 mm (如 '1.2m' -> 1200.0, 'mm')"""
    match = re.match(r"^([0-9.]+)([a-zA-Z]+)$", size_str.strip())
    if match:
        val = float(match.group(1))
        unit = match.group(2).lower()
        if unit == "m":
            return val * 1000.0, "mm"
        return val, unit
    raise argparse.ArgumentTypeError(
        f'Invalid size format: "{size_str}". Example: "1.2m" or "200mm"'
    )


def fetch_tag_image(family, prefix, tag_id, cache_root):
    """从本地缓存获取 AprilTag 图片，如果不存在则自动去 GitHub 下载"""
    cache_dir = os.path.join(os.path.expanduser(cache_root), family)
    os.makedirs(cache_dir, exist_ok=True)

    filename = f"{prefix}{tag_id:05d}.png"
    filepath = os.path.join(cache_dir, filename)

    if os.path.exists(filepath):
        return filepath

    url = f"https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/{family}/{filename}"
    print(f"  -> 本地未缓存，正在下载 {filename}...")

    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req) as response, open(filepath, "wb") as out_file:
            out_file.write(response.read())
        return filepath
    except HTTPError as e:
        if e.code == 404:
            print(f"  [错误] 找不到图片 (404): {url}")
        else:
            print(f"  [错误] HTTP {e.code}: 下载失败 {url}")
        return None
    except URLError as e:
        print(f"  [错误] 网络连接失败: {e.reason}")
        return None


def gen_svg_rects_payload(outer_im, inner_im, outer_tag_size, inner_tag_size, family):
    """专门针对 10像素（外2内3分界线定位）定制的嵌套生成算法"""
    out_w, out_h = outer_im.size  # 此时外层图片应为 10x10 像素
    in_w, in_h = inner_im.size

    # 严格按照用户定义的布局：标称尺寸跨度为 6 个像素
    out_measure_px = 6

    # 内层根据标准 AprilTag 3 灵活布局计算（去掉最外圈黑边）
    if "Custom" in family or "Standard" in family or "Circle" in family:
        in_measure_px = in_w - 2
    else:
        in_measure_px = in_w

    outer_pix = outer_im.convert("RGBA").load()
    inner_pix = inner_im.convert("RGBA").load()

    # 1. 计算物理单元大小 (mm)
    outer_cell_size = outer_tag_size / out_measure_px
    inner_cell_size = inner_tag_size / in_measure_px

    # 2. 计算最终整体画布尺寸
    outer_full_size = outer_cell_size * out_w  # 10 * outer_cell_size
    inner_full_size = inner_cell_size * in_w

    # 内层 Tag 整体居中时的物理偏移量
    inner_offset = (outer_full_size - inner_full_size) / 2.0

    payload = ""

    # 3. 绘制外层 Tag 像素
    payload += "  \n"
    for y in range(out_h):
        for x in range(out_w):
            r, g, b, a = outer_pix[x, y]
            if 4 <= x <= 5 and 4 <= y <= 5:
                continue
            if r > 200 and g > 200 and b > 200:
                continue
            payload += f'  <rect x="{x * outer_cell_size:.2f}" y="{y * outer_cell_size:.2f}" width="{outer_cell_size:.2f}" height="{outer_cell_size:.2f}" fill="rgba({r},{g},{b},{a/255})"/>\n'

    # 4. 绘制内层 Tag 的白色静默区（Quiet Zone）
    quiet_zone_offset = inner_offset - inner_cell_size
    quiet_zone_size = inner_full_size + (2 * inner_cell_size)

    payload += "  \n"
    payload += f'  <rect x="{quiet_zone_offset:.2f}" y="{quiet_zone_offset:.2f}" width="{quiet_zone_size:.2f}" height="{quiet_zone_size:.2f}" fill="white"/>\n'

    # 5. 绘制内层 Tag 像素
    payload += "  \n"
    for y in range(in_h):
        for x in range(in_w):
            r, g, b, a = inner_pix[x, y]
            if r > 200 and g > 200 and b > 200:
                continue
            rect_x = inner_offset + x * inner_cell_size
            rect_y = inner_offset + y * inner_cell_size
            payload += f'  <rect x="{rect_x:.2f}" y="{rect_y:.2f}" width="{inner_cell_size:.2f}" height="{inner_cell_size:.2f}" fill="rgba({r},{g},{b},{a/255})"/>\n'

    return payload, outer_full_size


def gen_png_image(
    outer_im, inner_im, outer_tag_size, inner_tag_size, family, dpi, view_box=None
):
    """
    使用 PIL 直接渲染高分辨率的 PNG 图像。
    view_box: 如果为 (vx, vy, vw, vh) 则只裁剪并输出该物理区域（用于分片）
    """
    # 毫米转英寸再乘 DPI 得到像素
    mm_to_inch = 1.0 / 25.4
    scale = dpi * mm_to_inch

    out_w, out_h = outer_im.size
    in_w, in_h = inner_im.size

    out_measure_px = 6
    if "Custom" in family or "Standard" in family or "Circle" in family:
        in_measure_px = in_w - 2
    else:
        in_measure_px = in_w

    outer_cell_size = outer_tag_size / out_measure_px
    inner_cell_size = inner_tag_size / in_measure_px
    outer_full_size = outer_cell_size * out_w
    inner_full_size = inner_cell_size * in_w
    inner_offset = (outer_full_size - inner_full_size) / 2.0

    # 确定画布的物理大小和像素大小
    if view_box is not None:
        vx, vy, vw, vh = view_box
        img_w_px = int(round(vw * scale))
        img_h_px = int(round(vh * scale))
        offset_x_mm = -vx
        offset_y_mm = -vy
    else:
        img_w_px = int(round(outer_full_size * scale))
        img_h_px = int(round(outer_full_size * scale))
        offset_x_mm = 0
        offset_y_mm = 0

    # 创建白色背景画布
    img = Image.new("RGBA", (img_w_px, img_h_px), "white")
    draw = ImageDraw.Draw(img)

    outer_pix = outer_im.convert("RGBA").load()
    inner_pix = inner_im.convert("RGBA").load()

    # 辅助闭包：快速计算物理毫米对应的像素坐标
    def to_px(mm_x, mm_y):
        px_x = int(round((mm_x + offset_x_mm) * scale))
        px_y = int(round((mm_y + offset_y_mm) * scale))
        return px_x, px_y

    # 1. 绘制外层像素
    for y in range(out_h):
        for x in range(out_w):
            if 4 <= x <= 5 and 4 <= y <= 5:
                continue
            r, g, b, a = outer_pix[x, y]
            if r > 200 and g > 200 and b > 200:
                continue

            x0, y0 = to_px(x * outer_cell_size, y * outer_cell_size)
            x1, y1 = to_px((x + 1) * outer_cell_size, (y + 1) * outer_cell_size)
            draw.rectangle([x0, y0, x1 - 1, y1 - 1], fill=(r, g, b, a))

    # 2. 绘制内层白色静默区
    quiet_zone_offset = inner_offset - inner_cell_size
    quiet_zone_size = inner_full_size + (2 * inner_cell_size)
    qx0, qy0 = to_px(quiet_zone_offset, quiet_zone_offset)
    qx1, qy1 = to_px(
        quiet_zone_offset + quiet_zone_size, quiet_zone_offset + quiet_zone_size
    )
    draw.rectangle([qx0, qy0, qx1 - 1, qy1 - 1], fill="white")

    # 3. 绘制内层像素
    for y in range(in_h):
        for x in range(in_w):
            r, g, b, a = inner_pix[x, y]
            if r > 200 and g > 200 and b > 200:
                continue
            rect_x = inner_offset + x * inner_cell_size
            rect_y = inner_offset + y * inner_cell_size

            x0, y0 = to_px(rect_x, rect_y)
            x1, y1 = to_px(rect_x + inner_cell_size, rect_y + inner_cell_size)
            draw.rectangle([x0, y0, x1 - 1, y1 - 1], fill=(r, g, b, a))

    # 4. 如果是分片模式，模拟 SVG 绘制一条虚线边界裁剪线 (可选)
    if view_box is not None:
        # 在边缘画一圈淡淡的灰色裁剪指引线
        draw.rectangle([0, 0, img_w_px - 1, img_h_px - 1], outline=(153, 153, 153, 255))

    return img


def wrap_svg(
    payload, phys_w, phys_h, unit, view_x, view_y, view_w, view_h, add_cut_line=False
):
    """将 payload 包装为完整的 SVG，利用 viewBox 裁剪特定区域"""
    svg_text = '<?xml version="1.0" standalone="yes"?>\n'
    svg_text += f'<svg width="{phys_w}{unit}" height="{phys_h}{unit}" viewBox="{view_x} {view_y} {view_w} {view_h}" xmlns="http://www.w3.org/2000/svg">\n'

    svg_text += " <g>\n"
    svg_text += payload
    svg_text += " </g>\n"

    if add_cut_line:
        svg_text += f"  \n"
        svg_text += f'  <rect x="{view_x}" y="{view_y}" width="{view_w}" height="{view_h}" fill="none" stroke="#999999" stroke-width="0.5" stroke-dasharray="5,5" />\n'

    svg_text += "</svg>\n"
    return svg_text


def main():
    parser = argparse.ArgumentParser(
        description="自动下载并生成支持 A4 分片打印的嵌套 AprilTag (同时输出 SVG 和 PNG)。"
    )
    parser.add_argument(
        "--family", type=str, default="tagCustom48h12", help="AprilTag 族类"
    )
    parser.add_argument("--prefix", type=str, default="tag48_12_", help="图片前缀")
    parser.add_argument(
        "--cache-dir",
        type=str,
        default="~/.cache/apriltag-imgs",
        help="图片自动下载的缓存目录",
    )

    parser.add_argument("--start-id", type=int, default=0, help="起始 ID (默认 0)")
    parser.add_argument("--count", type=int, default=1, help="生成数量 (默认 1)")
    parser.add_argument(
        "--outer-size", type=str, default="0.8m", help='外层标称尺寸 (如 "0.8m")'
    )
    parser.add_argument(
        "--inner-size", type=str, default="0.15m", help='内层标称尺寸 (如 "0.15m")'
    )
    parser.add_argument("--out-dir", type=str, default=".", help="输出文件夹路径")

    parser.add_argument(
        "--split", action="store_true", help="是否分片为多张小图以便 A4 打印"
    )
    parser.add_argument(
        "--slice-w",
        type=float,
        default=190.0,
        help="分片宽度，默认 190 (适配A4安全打印区)",
    )
    parser.add_argument(
        "--slice-h",
        type=float,
        default=270.0,
        help="分片高度，默认 270 (适配A4安全打印区)",
    )
    parser.add_argument(
        "--dpi", type=int, default=300, help="生成 PNG 时的 DPI 清晰度精度 (默认 300)"
    )

    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    out_tag_val, out_unit = parse_size(args.outer_size)
    in_tag_val, in_unit = parse_size(args.inner_size)

    if out_unit != in_unit:
        print("错误: 外层和内层的尺寸单位必须一致 (计算已将其统一为 mm)")
        return

    print(f"缓存目录: {os.path.expanduser(args.cache_dir)}")

    for i in range(args.count):
        outer_id = args.start_id + i * 2
        inner_id = args.start_id + i * 2 + 1
        msg = f"\n正在处理组合 [{i+1}/{args.count}]: 外层 ID {outer_id} + 内层 ID {inner_id}"
        print(msg)

        outer_path = fetch_tag_image(args.family, args.prefix, outer_id, args.cache_dir)
        inner_path = fetch_tag_image(args.family, args.prefix, inner_id, args.cache_dir)

        if not outer_path or not inner_path:
            print("  [跳过] 由于图片获取失败，跳过此标签生成。")
            continue

        with Image.open(outer_path, "r") as outer_im, Image.open(
            inner_path, "r"
        ) as inner_im:
            # 1. 生成基础 SVG 载荷
            payload, outer_full_size = gen_svg_rects_payload(
                outer_im, inner_im, out_tag_val, in_tag_val, args.family
            )

            # 2. 根据是否分片，分别导出 SVG 和 PNG
            if not args.split:
                # ----------------- 完整模式 -----------------
                # 保存 SVG
                svg_content = wrap_svg(
                    payload,
                    outer_full_size,
                    outer_full_size,
                    out_unit,
                    0,
                    0,
                    outer_full_size,
                    outer_full_size,
                )
                base_name = f"tag_{args.family}_out{outer_id:05d}_in{inner_id:05d}"
                svg_out_name = os.path.join(args.out_dir, f"{base_name}.svg")
                with open(svg_out_name, "w") as fp:
                    fp.write(svg_content)

                # 保存 PNG
                png_out_name = os.path.join(args.out_dir, f"{base_name}.png")
                png_img = gen_png_image(
                    outer_im, inner_im, out_tag_val, in_tag_val, args.family, args.dpi
                )
                png_img.save(png_out_name, dpi=(args.dpi, args.dpi))

                print(f"  -> 已生成完整 SVG: {svg_out_name}")
                print(f"  -> 已生成完整 PNG: {png_out_name} (分辨率 DPI: {args.dpi})")
                print(f"     (含黑边实际物理尺寸: {outer_full_size:.2f}{out_unit})")

            else:
                # ----------------- 分片模式 -----------------
                cols = math.ceil(outer_full_size / args.slice_w)
                rows = math.ceil(outer_full_size / args.slice_h)

                split_dir = os.path.join(args.out_dir, f"tag_{outer_id}_split")
                os.makedirs(split_dir, exist_ok=True)

                for r in range(rows):
                    for c in range(cols):
                        view_x = c * args.slice_w
                        view_y = r * args.slice_h

                        # 分片渲染 SVG
                        svg_content = wrap_svg(
                            payload=payload,
                            phys_w=args.slice_w,
                            phys_h=args.slice_h,
                            unit=out_unit,
                            view_x=view_x,
                            view_y=view_y,
                            view_w=args.slice_w,
                            view_h=args.slice_h,
                            add_cut_line=True,
                        )
                        svg_part_name = os.path.join(
                            split_dir, f"row{r+1}_col{c+1}.svg"
                        )
                        with open(svg_part_name, "w") as fp:
                            fp.write(svg_content)

                        # 分片渲染 PNG
                        png_part_name = os.path.join(
                            split_dir, f"row{r+1}_col{c+1}.png"
                        )
                        view_box = (view_x, view_y, args.slice_w, args.slice_h)
                        png_img = gen_png_image(
                            outer_im,
                            inner_im,
                            out_tag_val,
                            in_tag_val,
                            args.family,
                            args.dpi,
                            view_box=view_box,
                        )
                        png_img.save(png_part_name, dpi=(args.dpi, args.dpi))

                print(
                    f"  -> 分片已完成，共 {rows * cols} 组图 (每组含 SVG + PNG)，存入目录: {split_dir}"
                )
                print(
                    f"  -> 提示：飞控中输入的 tagSize 请写 {args.outer_size}，但拼出的物理尺寸会是 {outer_full_size:.2f}{out_unit}。"
                )


if __name__ == "__main__":
    main()
