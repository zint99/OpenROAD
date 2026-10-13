"""Generate the cell physical view diagrams with Python's standard library."""

from html import escape
from pathlib import Path


OUT = Path(__file__).resolve().parent / "images" / "cell_physical_view"
BLUE = "#2878bd"
GREEN = "#18866a"
RED = "#cb454e"
INK = "#23354b"


class SVG:
    def __init__(self, width, height, title):
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}" role="img">',
            f"<title>{escape(title)}</title>",
            '<defs><pattern id="hatch" width="8" height="8" '
            'patternUnits="userSpaceOnUse"><rect width="8" height="8" '
            'fill="#e9edf2"/><path d="M-2,2 L2,-2 M0,8 L8,0 M6,10 L10,6" '
            'stroke="#a0acbb" stroke-width="1.3"/></pattern></defs>',
            '<style>text{font-family:"Noto Sans CJK SC",sans-serif;'
            f'fill:{INK};font-size:17px}}</style>',
            f'<rect width="{width}" height="{height}" fill="white"/>',
        ]

    def text(self, x, y, value, size=17, color=INK, anchor="start"):
        self.parts.append(
            f'<text x="{x}" y="{y}" text-anchor="{anchor}" '
            f'style="font-size:{size}px;fill:{color}">{escape(value)}</text>'
        )

    def rect(self, x, y, width, height, fill, stroke="none", dash=False):
        pattern = ' stroke-dasharray="7 5"' if dash else ""
        self.parts.append(
            f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="2"{pattern}/>'
        )

    def line(self, x1, y1, x2, y2, color="#a0acbb", dash=False):
        pattern = ' stroke-dasharray="5 4"' if dash else ""
        self.parts.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            f'stroke="{color}" stroke-width="1.5"{pattern}/>'
        )

    def dot(self, x, y, color=RED):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="5" fill="{color}"/>')

    def polygon(self, pts, fill, stroke=INK):
        coords = " ".join(f"{x},{y}" for x, y in pts)
        self.parts.append(
            f'<polygon points="{coords}" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="1.8"/>'
        )

    def save(self, name):
        (OUT / name).write_text("\n".join(self.parts + ["</svg>"]) + "\n")


def map_rect(svg, box, ox, oy, scale, fill, stroke="none", dash=False):
    x1, y1, x2, y2 = box
    svg.rect(ox + x1 * scale, oy - y2 * scale,
             (x2 - x1) * scale, (y2 - y1) * scale, fill, stroke, dash)


def cell_layers():
    s = SVG(980, 660, "INV_X1 的分层物理抽象")
    s.text(35, 43, "同一个 cell，在不同层上提供不同的物理接口", 25)
    for ox, title in [(135, "M1"), (595, "M2")]:
        s.text(ox + 96, 97, title, 23, anchor="middle")
        map_rect(s, (0, 0, 1.2, 2.4), ox, 525, 160, "#f8fafc", INK, True)
        s.text(ox + 96, 558, "宽 1.2 μm", anchor="middle")
        s.dot(ox, 525)
        s.text(ox - 8, 588, "局部原点 (0, 0)", 16)
    for box in [(0.1, 0.8, 0.25, 1.1), (0.1, 1.3, 0.25, 1.6)]:
        map_rect(s, box, 135, 525, 160, BLUE)
    map_rect(s, (0.45, 0.4, 0.75, 1.9), 135, 525, 160, "url(#hatch)", "#8795a7")
    for box in [(0, 0.05, 1.2, 0.15), (0, 2.25, 1.2, 2.35)]:
        map_rect(s, box, 135, 525, 160, RED)
    s.text(77, 312, "A", 22, BLUE)
    s.line(100, 305, 148, 297, BLUE)
    s.text(340, 160, "VDD", 18, RED)
    s.text(340, 513, "VSS", 18, RED)
    s.text(246, 337, "OBS", 17)
    s.text(64, 440, "高", 16)
    s.text(40, 466, "2.4 μm", 16)
    map_rect(s, (0.9, 1.65, 1.08, 2.0), 595, 525, 160, GREEN)
    s.text(807, 238, "Y", 22, GREEN)
    s.line(792, 236, 768, 236, GREEN)
    s.text(621, 372, "此处未声明 OBS", 16)
    s.text(35, 635, "虚线：placement boundary   蓝 / 绿：信号 pin   红：电源 pin   斜线：OBS", 17)
    s.save("cell_layers.svg")


def pin_bbox():
    s = SVG(960, 490, "引脚金属与包围盒的区别")
    s.text(35, 43, "A 的 bounding box 包含两块金属之间的空隙", 25)
    ox, oy, scale = 70, 770, 420
    for box in [(0.1, 0.8, 0.25, 1.1), (0.1, 1.3, 0.25, 1.6)]:
        map_rect(s, box, ox, oy, scale, BLUE)
    map_rect(s, (0.1, 0.8, 0.25, 1.6), ox, oy, scale, "none", INK, True)
    s.text(228, 132, "M1 金属：y = 1.30 … 1.60 μm", 20)
    s.line(177, 150, 213, 125, BLUE)
    s.text(228, 266, "空隙：y = 1.10 … 1.30 μm", 20)
    s.line(177, 265, 213, 259, INK)
    s.text(228, 299, "在包围盒内，但不在这两块 pin 金属内", 17, "#64748b")
    s.text(228, 389, "M1 金属：y = 0.80 … 1.10 μm", 20)
    s.line(177, 388, 213, 382, BLUE)
    s.text(80, 472, "两块矩形的 x 范围相同：0.10 … 0.25 μm；此图按比例绘制。", 18)
    s.save("pin_bbox.svg")


TRANSFORMS = {
    "R0": lambda x, y: (x, y),
    "R90": lambda x, y: (-y, x),
    "R180": lambda x, y: (-x, -y),
    "R270": lambda x, y: (y, -x),
    "MX": lambda x, y: (x, -y),
    "MY": lambda x, y: (-x, y),
    "MXR90": lambda x, y: (y, x),
    "MYR90": lambda x, y: (-y, -x),
}


def transformed_box(s, box, fn, ox, oy, scale, fill, stroke):
    x1, y1, x2, y2 = box
    pts = [fn(x, y) for x, y in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]]
    s.polygon([(ox + scale * x, oy - scale * y) for x, y in pts], fill, stroke)


def orientations():
    s = SVG(1160, 750, "八种朝向的几何变换")
    s.text(30, 42, "固定局部原点，观察形状朝向变化", 25)
    for i, (name, fn) in enumerate(TRANSFORMS.items()):
        col, row = i % 4, i // 4
        left, top = 15 + col * 287, 72 + row * 315
        ox, oy = left + 140, top + 165
        s.rect(left, top, 275, 303, "#f8fafc", "#e4eaf0")
        s.text(left + 17, top + 30, name, 22)
        s.line(ox - 117, oy, ox + 117, oy)
        s.line(ox, oy + 117, ox, oy - 117)
        s.text(ox + 119, oy - 7, "x", 14)
        s.text(ox + 6, oy - 116, "y", 14)
        transformed_box(s, (0, 0, 1.2, 2.4), fn, ox, oy, 41, "#e6edf5", INK)
        transformed_box(s, (0.1, 0.8, 0.25, 1.1), fn, ox, oy, 41, BLUE, BLUE)
        transformed_box(s, (0.9, 1.65, 1.08, 2.0), fn, ox, oy, 41, GREEN, GREEN)
        s.dot(ox, oy)
    s.text(30, 733, "红点：原点 (0, 0)     蓝色：A 的一块矩形     绿色：Y     所有面板比例相同，未加平移", 17)
    s.save("orientations.svg")


def origin_location():
    s = SVG(1080, 515, "R90 变换中的实例原点与左下角")
    s.text(30, 43, "旋转后，实例原点与包围盒左下角可以不同", 25)
    for left, title in [(25, "固定 origin"), (555, "固定 location")]:
        s.rect(left, 75, 500, 390, "#f8fafc", "#e4eaf0")
        s.text(left + 20, 113, title, 23)
        # Both final rectangles have width H = 2.4 and height W = 1.2.
        x, bottom, scale = left + 90, 315, 130
        s.rect(x, bottom - 1.2 * scale, 2.4 * scale, 1.2 * scale, "#e6edf5", INK)
        # R90 maps the local origin to the bottom-right corner.
        origin_x = x + 2.4 * scale
        transformed_box(s, (0.1, 0.8, 0.25, 1.1), TRANSFORMS["R90"],
                        origin_x, bottom, scale, BLUE, BLUE)
        s.dot(origin_x, bottom)
        s.rect(x - 5, bottom - 5, 10, 10, GREEN)
        s.text(x + 156, 145, "2.4 μm", 17, anchor="middle")
        s.text(left + 15, 235, "1.2 μm", 15)
        s.text(x + 156, 224, "R90", 27, anchor="middle")
        if left == 25:
            loc, origin = "(97.6, 200)", "(100, 200)"
        else:
            loc, origin = "(100, 200)", "(102.4, 200)"
        s.text(left + 20, 365, f"绿色方点  location = {loc}", 19, GREEN)
        s.text(left + 20, 405, f"红色圆点  origin = {origin}", 19, RED)
    s.text(30, 497, "坐标单位：μm。两图各用独立画布；单元的 master 原点为 (0, 0)。", 17)
    s.save("origin_location.svg")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    cell_layers()
    pin_bbox()
    orientations()
    origin_location()
