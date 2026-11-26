"""Render matching Windows and XFCE shortcut icons."""
from pathlib import Path

from PIL import Image, ImageDraw


def make_icon(name, top, bottom, symbol):
    size = 256
    image = Image.new("RGBA", (size, size))
    mask = Image.new("L", (size, size))
    ImageDraw.Draw(mask).rounded_rectangle((10, 10, 245, 245), radius=47, fill=255)
    fill = Image.new("RGB", (size, size))
    pixels = ImageDraw.Draw(fill)
    colors = [tuple(bytes.fromhex(color)) for color in (top, bottom)]
    for y in range(size):
        amount = y / (size - 1)
        tone = tuple(round(a + (b - a) * amount) for a, b in zip(*colors))
        pixels.line((0, y, 255, y), fill=tone)
    image.paste(fill, mask=mask)
    draw = ImageDraw.Draw(image)
    white = "#f6fbff"
    accent = "#ffdb91"

    def box(x1, y1, x2, y2):
        return x1*4, y1*4, x2*4, y2*4

    def line(points, color=white, width=2.8):
        draw.line([(round(x*4), round(y*4)) for x, y in points], fill=color,
                  width=round(width*4), joint="curve")

    def circle(x, y, radius, color):
        draw.ellipse(box(x-radius, y-radius, x+radius, y+radius), fill=color)

    def rectangle(bounds, radius=2, fill=None, outline=None, width=3):
        draw.rounded_rectangle(box(*bounds), radius=radius*4, fill=fill, outline=outline, width=width*4)

    if symbol in ("wslg", "novnc"):
        rectangle((13, 13, 51, 46), radius=4, outline=white)
        if symbol == "wslg":
            line([(32, 46), (32, 52)])
            line([(23, 52), (41, 52)])
            line([(20, 24), (26, 30), (20, 36)])
            line([(31, 36), (44, 36)], accent)
        else:
            line([(13, 22), (51, 22)], width=2)
            for x in (19, 24, 29):
                circle(x, 18, 1.3, accent)
            line([(30, 28), (36, 34), (30, 40)])
            line([(20, 34), (36, 34)])
    elif symbol == "windows":
        for x in (16, 33):
            for y in (16, 33):
                rectangle((x, y, x + 14, y + 14), radius=1, fill=white)
    elif symbol == "tools":
        rectangle((14, 24, 50, 47), radius=4, fill=white)
        rectangle((24, 17, 40, 29), radius=3, outline=white)
        line([(15, 33), (49, 33)], "#8b4a17", width=2)
        rectangle((29, 31, 35, 37), radius=1, fill=accent)
    elif symbol == "weather":
        circle(25, 24, 8.5, accent)
        for angled in (((25, 10), (25, 13)), ((11, 24), (14, 24)), ((15, 14), (18, 17)), ((33, 15), (36, 12))):
            line(angled, accent, width=2.2)
        circle(25, 38, 7, white)
        circle(34, 34, 9, white)
        circle(44, 39, 7, white)
        rectangle((25, 39, 44, 46), radius=3, fill=white)
    elif symbol == "track":
        draw.polygon([(32*4, 13*4), (45*4, 45*4), (19*4, 45*4)], fill=accent)
        line([(25, 31), (39, 31)], white, width=4)
        rectangle((15, 45, 49, 50), radius=2, fill=white)
    elif symbol == "rviz":
        for span in (13, 20):
            draw.arc(box(32-span, 32-span, 32+span, 32+span), 18, 311, fill=white, width=9)
        line([(32, 32), (48, 20)], accent)
        circle(32, 32, 2.8, accent)
    elif symbol == "trackdraw":
        line([(19, 19), (43, 19), (49, 26), (49, 39), (42, 47), (24, 47),
              (15, 40), (15, 30), (24, 30), (29, 36), (39, 36), (42, 31)], width=4)
        circle(19, 19, 2.8, accent)
        circle(42, 31, 2.8, accent)
    elif symbol == "stop":
        rectangle((19, 19, 45, 45), radius=4, fill=white)
    elif symbol == "start":
        draw.polygon([(x*4, y*4) for x, y in ((12, 49), (25, 15), (39, 15), (52, 49))], fill=white)
        for top, bottom in ((19, 24), (28, 34), (39, 46)):
            line([(32, top), (32, bottom)], "#21435b", width=2.5)
    elif symbol == "silverstone-split":
        line([(14, 46), (17, 27), (27, 16), (45, 16), (51, 21), (46, 31),
              (34, 30), (29, 35), (40, 43), (35, 50), (21, 50), (14, 46)], width=3.5)
        line([(19, 23), (25, 29)], accent, width=3.5)
    elif symbol == "map-packs":
        line([(13, 47), (13, 20), (25, 14), (39, 19), (51, 14), (51, 43),
              (39, 49), (25, 44), (13, 47)], width=3.1)
        line([(25, 14), (25, 44)], accent, width=2.8)
        line([(39, 19), (39, 49)], width=2.8)
    elif symbol == "manager":
        for y, x in ((20, 24), (32, 42), (44, 30)):
            line([(14, y), (50, y)], width=2.7)
            circle(x, y, 4, accent)
    elif symbol == "zed-rviz":
        rectangle((12, 21, 52, 44), radius=5, outline=white)
        for x in (23, 41):
            circle(x, 32.5, 4.6, accent)
    elif symbol == "slam-rviz":
        for x, y in ((16, 17), (44, 15), (29, 29), (48, 43), (18, 48)):
            line([(32, 34), (x, y)], width=2.0)
            circle(x, y, 3.3, white)
        circle(32, 34, 4.4, accent)
    elif symbol == "slam-diagnostics":
        rectangle((12, 17, 52, 47), radius=4, outline=white)
        line([(17, 34), (24, 34), (28, 25), (34, 40), (38, 30), (47, 30)], accent, width=2.5)
    elif symbol == "planning-rviz":
        line([(17, 47), (27, 37), (42, 37), (46, 31), (42, 25), (28, 25), (24, 18), (40, 15)], width=3.4)
        circle(17, 47, 4.6, accent)
        circle(40, 15, 4.6, white)
    elif symbol == "control-rviz":
        draw.ellipse(box(13, 13, 51, 51), outline=white, width=13)
        line([(15, 28), (25, 28), (32, 34), (39, 28), (49, 28)], width=3.4)
        line([(32, 33), (32, 50)], width=3.4)
        circle(32, 33, 4, accent)
    elif symbol == "track-control-rviz":
        draw.arc(box(14, 14, 50, 52), 20, 295, fill=white, width=13)
        line([(49, 35), (48, 24), (39, 28)], width=3.3)
        line([(26, 36), (31, 41), (39, 30)], accent, width=3.0)

    folder = Path(__file__).parent / "Icons"
    folder.mkdir(exist_ok=True)
    image.save(folder / f"{name}.png")
    image.save(folder / f"{name}.ico", sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])


for spec in (
    ("wslg", "078c87", "064850"),
    ("novnc", "7852cb", "402078"),
    ("windows", "087bd0", "193b87"),
    ("tools", "bb621c", "78360e"),
    ("weather", "298cc2", "174772"),
    ("track", "c4582b", "7e2534"),
    ("rviz", "348653", "174e3c"),
    ("trackdraw", "a44284", "652a69"),
    ("stop", "cb4541", "8d2929"),
    ("start", "43758f", "20364d"),
    ("silverstone-split", "846ccc", "44418e"),
    ("map-packs", "136d61", "0c3f36"),
    ("manager", "445eae", "252b69"),
    ("zed-rviz", "0d879e", "104657"),
    ("slam-rviz", "6e5ab4", "3e2877"),
    ("slam-diagnostics", "be426e", "70243e"),
    ("planning-rviz", "2774ac", "183d77"),
    ("control-rviz", "927331", "5a401a"),
    ("track-control-rviz", "3c8d72", "194e4e"),
):
    make_icon(*spec, spec[0])
