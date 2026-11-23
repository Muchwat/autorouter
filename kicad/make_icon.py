"""Draw the plugin's icon: a capital A made of tracks, pads and a via.

    python3 kicad/make_icon.py              icon.png (24 px, the toolbar) and
                                            icon-64.png (the package list)
    python3 kicad/make_icon.py --preview    preview.png: every design, at toolbar
                                            size on light and dark, and enlarged

The shapes are laid out on a 24 x 24 grid and drawn with 8 x 8 samples a
pixel, so the edges come out smooth at any size. No libraries needed.
"""

import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN = os.path.join(HERE, "autorouter_plugin")

# colours (KiCad's own, roughly: front copper red, back copper blue, pads gold)
FRONT = (200, 60, 52)
BACK = (66, 110, 200)
COPPER = (205, 122, 52)
PAD = (226, 176, 64)
VIA = (190, 190, 190)


# ---------------------------------------------------------------- shapes
# each shape: (kind, colour, ...) in 24-unit coordinates; later ones on top


def track(a, b, w, colour):
    return ("track", colour, a, b, w / 2.0)


def pad(c, w, h, colour=PAD, r=0.6):
    return ("pad", colour, c, w / 2.0, h / 2.0, r)


def via(c, r=2.5, hole=1.0, colour=VIA):
    return ("via", colour, c, r, hole)


def path(points, w, colour):
    return [track(points[i], points[i + 1], w, colour) for i in range(len(points) - 1)]


DESIGNS = {
    # the letter as it is written: two slanted legs from a via at the top, a crossbar
    "slanted": (
        path([(5.5, 19.5), (12, 4.5), (18.5, 19.5)], 2.2, COPPER)
        + [track((8.2, 13.6), (15.8, 13.6), 2.0, COPPER)]
        + [pad((5.5, 20.2), 4.6, 3.0), pad((18.5, 20.2), 4.6, 3.0), via((12, 4.6))]
    ),
    # the letter as a router would lay it: straight up, then 45 degrees to the via
    "routed": (
        path([(5, 20), (5, 11.5), (12, 4.5), (19, 11.5), (19, 20)], 2.2, COPPER)
        + [track((5, 14), (19, 14), 2.0, COPPER)]
        + [pad((5, 20.3), 4.2, 3.0), pad((19, 20.3), 4.2, 3.0), via((12, 4.6))]
    ),
    # as "routed", on two layers: the left side on the front, the right on the
    # back, changing layer at the via on top; the crossbar dips to the back too
    "two layers": (
        path([(5, 20), (5, 11.5), (12, 4.5)], 2.2, FRONT)
        + path([(12, 4.5), (19, 11.5), (19, 20)], 2.2, BACK)
        + [track((5, 14.5), (12, 14.5), 2.0, FRONT), track((12, 14.5), (19, 14.5), 2.0, BACK)]
        + [pad((5, 20.3), 4.2, 3.0), pad((19, 20.3), 4.2, 3.0), via((12, 4.6)), via((12, 14.5), 1.7, 0.7)]
    ),
}


# ---------------------------------------------------------------- drawing


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    px, py = p[0] - a[0], p[1] - a[1]
    l2 = ax * ax + ay * ay
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, (px * ax + py * ay) / l2))
    dx, dy = px - t * ax, py - t * ay
    return (dx * dx + dy * dy) ** 0.5


def _inside(shape, p):
    """(covered, colour) of a point: a via's hole is see-through."""
    kind, colour = shape[0], shape[1]
    if kind == "track":
        _, _, a, b, r = shape
        return _seg_dist(p, a, b) <= r
    if kind == "pad":
        _, _, c, hw, hh, r = shape
        dx, dy = max(abs(p[0] - c[0]) - (hw - r), 0.0), max(abs(p[1] - c[1]) - (hh - r), 0.0)
        return (dx * dx + dy * dy) ** 0.5 <= r
    if kind == "via":
        _, _, c, r, hole = shape
        d = ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5
        return d <= r
    return False


def _hole(shape, p):
    if shape[0] != "via":
        return False
    _, _, c, r, hole = shape
    return ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5 <= hole


def render(shapes, size, ss=8, background=None):
    """RGBA rows of the design at `size` pixels a side."""
    rows = []
    k = 24.0 / size
    for y in range(size):
        row = []
        for x in range(size):
            r = g = b = a = 0.0
            for sy in range(ss):
                for sx in range(ss):
                    p = ((x + (sx + 0.5) / ss) * k, (y + (sy + 0.5) / ss) * k)
                    colour = None
                    for s in shapes:
                        if _inside(s, p):
                            colour = s[1]
                        if _hole(s, p):
                            colour = None  # drilled through: nothing there
                    if colour is not None:
                        r, g, b, a = r + colour[0], g + colour[1], b + colour[2], a + 1
            n = ss * ss
            if a:
                col = (r / a, g / a, b / a)
            else:
                col = (0, 0, 0)
            alpha = a / n
            if background is not None:
                col = tuple(c * alpha + bg * (1 - alpha) for c, bg in zip(col, background))
                alpha = 1.0
            row.append((int(round(col[0])), int(round(col[1])), int(round(col[2])), int(round(alpha * 255))))
        rows.append(row)
    return rows


def png(rows):
    h, w = len(rows), len(rows[0])
    raw = b"".join(b"\x00" + b"".join(bytes(px) for px in row) for row in rows)

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def paste(sheet, rows, x0, y0):
    for y, row in enumerate(rows):
        for x, px in enumerate(row):
            sheet[y0 + y][x0 + x] = px


def preview(path):
    """Each design: at 24 px on KiCad's light and dark toolbars, at 48 px, and at 192 px."""
    light, dark, paper = (236, 236, 236), (52, 52, 56), (255, 255, 255)
    names = list(DESIGNS)
    cell_w, cell_h = 260, 330
    sheet = [[paper + (255,) for _ in range(cell_w * len(names))] for _ in range(cell_h)]
    for i, name in enumerate(names):
        x0 = i * cell_w + 34
        paste(sheet, render(DESIGNS[name], 192, 4, background=paper), x0, 16)
        # toolbar size, light and dark, with a little room round it as on a toolbar
        for j, bg in enumerate((light, dark)):
            tile = [[bg + (255,) for _ in range(40)] for _ in range(40)]
            paste(tile, render(DESIGNS[name], 24, 8, background=bg), 8, 8)
            paste(sheet, tile, x0 + j * 48, 230)
        paste(sheet, render(DESIGNS[name], 48, 8, background=light), x0 + 104, 222)
    with open(path, "wb") as f:
        f.write(png(sheet))
    return names


def main():
    if "--preview" in sys.argv:
        out = os.path.join(HERE, "preview.png") if len(sys.argv) < 3 else sys.argv[-1]
        print(out, preview(out))
        return
    name = os.environ.get("ICON", "two layers")
    with open(os.path.join(PLUGIN, "icon.png"), "wb") as f:
        f.write(png(render(DESIGNS[name], 24)))
    with open(os.path.join(PLUGIN, "icon-64.png"), "wb") as f:
        f.write(png(render(DESIGNS[name], 64)))
    print("icon.png and icon-64.png:", name)


if __name__ == "__main__":
    main()
